use crate::config::DroneConfig;
use crate::mavlink_client::{
    arm_vehicle, connect_mavlink, monitor_mission, other_err, set_px4_mission_mode,
    start_heartbeat_thread, start_mission, upload_mission, wait_for_position, wait_for_vehicle,
    AnyResult,
};
use crate::mission::{build_relative_mission_items, hash_mission_items, hash_mission_specs};
use std::sync::atomic::Ordering;
use std::sync::mpsc as std_mpsc;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::task::{self, JoinHandle};

#[derive(Debug)]
pub enum VehicleCommand {
    /// Phase 3 split: prepare means connect, discover, build/upload the mission,
    /// optionally arm, then wait for an explicit supervisor start/abort decision.
    PrepareMission { phase: String },

    /// Start a mission that has already emitted a readiness vote.
    StartPreparedMission { phase: String },

    /// Abort a prepared mission before it starts.
    AbortPreparedMission { phase: String, reason: String },

    Shutdown,
}

#[derive(Debug, Clone)]
pub struct VehicleEvent {
    pub vehicle: String,
    pub phase: String,
    pub kind: VehicleEventKind,
}

impl VehicleEvent {
    pub fn new(
        vehicle: impl Into<String>,
        phase: impl Into<String>,
        kind: VehicleEventKind,
    ) -> Self {
        Self {
            vehicle: vehicle.into(),
            phase: phase.into(),
            kind,
        }
    }
}

#[derive(Debug, Clone)]
pub enum VehicleEventKind {
    CommandAccepted,
    Connecting { endpoint: String },
    Discovered { system_id: u8, component_id: u8 },
    StartPosition { lat_deg: f64, lon_deg: f64 },
    MissionBuilt {
        item_count: usize,
        spec_hash: String,
        upload_hash: String,
    },
    MissionUploaded,
    Armed,
    ReadinessVote {
        mission_hash: String,
        item_count: usize,
    },
    WaitingForStart {
        mission_hash: String,
    },
    StartApproved,
    StartAborted {
        reason: String,
    },
    MissionStarted,
    MissionMonitorStarted,
    Completed,
    Failed { error: String },
    ShutdownAck,
}

pub struct VehicleActorHandle {
    pub name: String,
    pub commands: mpsc::Sender<VehicleCommand>,
    pub task: JoinHandle<()>,
}

pub fn spawn_vehicle_actor(
    cfg: DroneConfig,
    events: mpsc::Sender<VehicleEvent>,
) -> VehicleActorHandle {
    let name = cfg.name.clone();
    let (commands, command_rx) = mpsc::channel(8);

    let actor = VehicleActor {
        cfg,
        command_rx,
        events,
        start_signal_tx: None,
    };

    let task = tokio::spawn(actor.run());

    VehicleActorHandle {
        name,
        commands,
        task,
    }
}

struct VehicleActor {
    cfg: DroneConfig,
    command_rx: mpsc::Receiver<VehicleCommand>,
    events: mpsc::Sender<VehicleEvent>,
    start_signal_tx: Option<std_mpsc::Sender<StartSignal>>,
}

#[derive(Debug)]
enum StartSignal {
    Start,
    Abort { reason: String },
}

impl VehicleActor {
    async fn run(mut self) {
        while let Some(command) = self.command_rx.recv().await {
            match command {
                VehicleCommand::PrepareMission { phase } => {
                    self.emit(&phase, VehicleEventKind::CommandAccepted).await;

                    if self.start_signal_tx.is_some() {
                        self.emit(
                            &phase,
                            VehicleEventKind::Failed {
                                error: "vehicle already has a prepared mission in progress"
                                    .to_string(),
                            },
                        )
                        .await;
                        continue;
                    }

                    let (start_tx, start_rx) = std_mpsc::channel::<StartSignal>();
                    self.start_signal_tx = Some(start_tx);

                    let cfg = self.cfg.clone();
                    let phase_for_task = phase.clone();
                    let events = self.events.clone();
                    let vehicle = self.cfg.name.clone();

                    let blocking_task = task::spawn_blocking(move || {
                        run_drone_blocking_until_supervisor_decision(
                            cfg,
                            phase_for_task,
                            events.clone(),
                            start_rx,
                        )
                    });

                    let completion_events = self.events.clone();
                    tokio::spawn(async move {
                        let result = blocking_task.await;
                        match result {
                            Ok(Ok(())) => {
                                emit_async(
                                    &completion_events,
                                    &vehicle,
                                    &phase,
                                    VehicleEventKind::Completed,
                                )
                                .await;
                            }
                            Ok(Err(e)) => {
                                emit_async(
                                    &completion_events,
                                    &vehicle,
                                    &phase,
                                    VehicleEventKind::Failed {
                                        error: e.to_string(),
                                    },
                                )
                                .await;
                            }
                            Err(e) => {
                                emit_async(
                                    &completion_events,
                                    &vehicle,
                                    &phase,
                                    VehicleEventKind::Failed {
                                        error: format!(
                                            "blocking vehicle task panicked or was cancelled: {e}"
                                        ),
                                    },
                                )
                                .await;
                            }
                        }
                    });
                }
                VehicleCommand::StartPreparedMission { phase } => {
                    self.send_start_signal(&phase, StartSignal::Start).await;
                }
                VehicleCommand::AbortPreparedMission { phase, reason } => {
                    self.send_start_signal(&phase, StartSignal::Abort { reason }).await;
                }
                VehicleCommand::Shutdown => {
                    if let Some(start_tx) = self.start_signal_tx.take() {
                        let _ = start_tx.send(StartSignal::Abort {
                            reason: "vehicle actor shutting down".to_string(),
                        });
                    }
                    self.emit("supervisor", VehicleEventKind::ShutdownAck).await;
                    break;
                }
            }
        }
    }

    async fn send_start_signal(&mut self, phase: &str, signal: StartSignal) {
        let Some(start_signal_tx) = self.start_signal_tx.take() else {
            self.emit(
                phase,
                VehicleEventKind::Failed {
                    error: "no prepared mission is waiting for a start decision".to_string(),
                },
            )
            .await;
            return;
        };

        if let Err(e) = start_signal_tx.send(signal) {
            self.emit(
                phase,
                VehicleEventKind::Failed {
                    error: format!("failed to send start signal to blocking worker: {e}"),
                },
            )
            .await;
        }
    }

    async fn emit(&self, phase: &str, kind: VehicleEventKind) {
        emit_async(&self.events, &self.cfg.name, phase, kind).await;
    }
}

async fn emit_async(
    events: &mpsc::Sender<VehicleEvent>,
    vehicle: &str,
    phase: &str,
    kind: VehicleEventKind,
) {
    let event = VehicleEvent::new(vehicle.to_string(), phase.to_string(), kind);
    let _ = events.send(event).await;
}

fn run_drone_blocking_until_supervisor_decision(
    cfg: DroneConfig,
    phase: String,
    events: mpsc::Sender<VehicleEvent>,
    start_rx: std_mpsc::Receiver<StartSignal>,
) -> AnyResult<()> {
    let vehicle = cfg.name.clone();
    emit_blocking(
        &events,
        &vehicle,
        &phase,
        VehicleEventKind::Connecting {
            endpoint: cfg.endpoint.clone(),
        },
    );
    println!("[{}] connecting on {}", cfg.name, cfg.endpoint);

    let conn = connect_mavlink(&cfg.endpoint)?;

    let (target_system, target_component) = wait_for_vehicle(&conn, Duration::from_secs(20))?;
    emit_blocking(
        &events,
        &vehicle,
        &phase,
        VehicleEventKind::Discovered {
            system_id: target_system,
            component_id: target_component,
        },
    );
    println!(
        "[{}] discovered target sys={}, comp={}",
        cfg.name, target_system, target_component
    );

    if let Some(expected) = cfg.expected_system_id
        && expected != target_system
    {
        return Err(format!(
            "[{}] expected system_id {}, got {}",
            cfg.name, expected, target_system
        )
        .into());
    }

    let (heartbeat_running, heartbeat_handle) = start_heartbeat_thread(conn.clone());

    let result = (|| -> AnyResult<()> {
        let (start_lat_deg, start_lon_deg) = wait_for_position(&conn, Duration::from_secs(20))?;
        emit_blocking(
            &events,
            &vehicle,
            &phase,
            VehicleEventKind::StartPosition {
                lat_deg: start_lat_deg,
                lon_deg: start_lon_deg,
            },
        );
        println!(
            "[{}] start position lat={:.7}, lon={:.7}",
            cfg.name, start_lat_deg, start_lon_deg
        );

        let mission_items = build_relative_mission_items(
            target_system,
            target_component,
            start_lat_deg,
            start_lon_deg,
            &cfg.mission,
        )?;

        let spec_hash = hash_mission_specs(&cfg.mission)?;
        let upload_hash = hash_mission_items(&mission_items);

        emit_blocking(
            &events,
            &vehicle,
            &phase,
            VehicleEventKind::MissionBuilt {
                item_count: mission_items.len(),
                spec_hash: spec_hash.clone(),
                upload_hash,
            },
        );

        upload_mission(&conn, target_system, target_component, &mission_items)?;
        emit_blocking(&events, &vehicle, &phase, VehicleEventKind::MissionUploaded);
        println!("[{}] mission uploaded", cfg.name);

        if cfg.arm.unwrap_or(true) {
            arm_vehicle(&conn, target_system, target_component)?;
            emit_blocking(&events, &vehicle, &phase, VehicleEventKind::Armed);
            println!("[{}] armed", cfg.name);
        }

        emit_blocking(
            &events,
            &vehicle,
            &phase,
            VehicleEventKind::ReadinessVote {
                mission_hash: spec_hash.clone(),
                item_count: mission_items.len(),
            },
        );
        emit_blocking(
            &events,
            &vehicle,
            &phase,
            VehicleEventKind::WaitingForStart {
                mission_hash: spec_hash,
            },
        );

        match start_rx.recv() {
            Ok(StartSignal::Start) => {
                emit_blocking(&events, &vehicle, &phase, VehicleEventKind::StartApproved);
            }
            Ok(StartSignal::Abort { reason }) => {
                emit_blocking(
                    &events,
                    &vehicle,
                    &phase,
                    VehicleEventKind::StartAborted {
                        reason: reason.clone(),
                    },
                );
                return Err(other_err(format!(
                    "mission start aborted by supervisor: {reason}"
                )));
            }
            Err(e) => {
                return Err(other_err(format!(
                    "mission start channel closed before supervisor decision: {e}"
                )));
            }
        }

        if cfg.auto_start.unwrap_or(true) {
            set_px4_mission_mode(&conn, target_system, target_component)?;
            start_mission(
                &conn,
                target_system,
                target_component,
                mission_items.len() as u16,
            )?;
            emit_blocking(&events, &vehicle, &phase, VehicleEventKind::MissionStarted);
            println!("[{}] mission started", cfg.name);

            emit_blocking(
                &events,
                &vehicle,
                &phase,
                VehicleEventKind::MissionMonitorStarted,
            );
            monitor_mission(
                &conn,
                (mission_items.len() - 1) as u16,
                Duration::from_secs(300),
            )?;
        } else {
            println!(
                "[{}] auto_start=false; mission prepared but not started",
                cfg.name
            );
        }

        println!("TERMINATING -- {}", cfg.name);
        Ok(())
    })();

    heartbeat_running.store(false, Ordering::Relaxed);
    let _ = heartbeat_handle.join();
    println!("TERMINATED -- {}", cfg.name);

    result
}

fn emit_blocking(
    events: &mpsc::Sender<VehicleEvent>,
    vehicle: &str,
    phase: &str,
    kind: VehicleEventKind,
) {
    let event = VehicleEvent::new(vehicle.to_string(), phase.to_string(), kind);
    let _ = events.blocking_send(event);
}

pub async fn send_command(
    commands: &mpsc::Sender<VehicleCommand>,
    command: VehicleCommand,
    vehicle_name: &str,
) -> AnyResult<()> {
    commands
        .send(command)
        .await
        .map_err(|e| other_err(format!("failed to send command to {vehicle_name}: {e}")))
}
