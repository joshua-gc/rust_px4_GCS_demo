use crate::config::DroneConfig;
use crate::mavlink_client::{
    arm_vehicle, connect_mavlink, monitor_mission, other_err, set_px4_mission_mode, start_heartbeat_thread,
    start_mission, upload_mission, wait_for_position, wait_for_vehicle, AnyResult,
};
use crate::mission::build_relative_mission_items;
use std::sync::atomic::Ordering;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::task::{self, JoinHandle};

#[derive(Debug)]
pub enum VehicleCommand {
    RunMission { phase: String },
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
    MissionBuilt { item_count: usize },
    MissionUploaded,
    Armed,
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
}

impl VehicleActor {
    async fn run(mut self) {
        while let Some(command) = self.command_rx.recv().await {
            match command {
                VehicleCommand::RunMission { phase } => {
                    self.emit(&phase, VehicleEventKind::CommandAccepted).await;

                    let cfg = self.cfg.clone();
                    let phase_for_task = phase.clone();
                    let events = self.events.clone();

                    let result = task::spawn_blocking(move || {
                        run_drone_blocking(cfg, phase_for_task, events)
                    })
                    .await;

                    match result {
                        Ok(Ok(())) => self.emit(&phase, VehicleEventKind::Completed).await,
                        Ok(Err(e)) => {
                            self.emit(
                                &phase,
                                VehicleEventKind::Failed {
                                    error: e.to_string(),
                                },
                            )
                            .await;
                        }
                        Err(e) => {
                            self.emit(
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
                }
                VehicleCommand::Shutdown => {
                    self.emit("supervisor", VehicleEventKind::ShutdownAck).await;
                    break;
                }
            }
        }
    }

    async fn emit(&self, phase: &str, kind: VehicleEventKind) {
        let event = VehicleEvent::new(self.cfg.name.clone(), phase.to_string(), kind);
        let _ = self.events.send(event).await;
    }
}

fn run_drone_blocking(
    cfg: DroneConfig,
    phase: String,
    events: mpsc::Sender<VehicleEvent>,
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
        emit_blocking(
            &events,
            &vehicle,
            &phase,
            VehicleEventKind::MissionBuilt {
                item_count: mission_items.len(),
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
        }

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
