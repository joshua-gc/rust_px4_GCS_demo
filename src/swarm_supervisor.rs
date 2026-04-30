use crate::config::{load_config, DroneConfig, SwarmConfig};
use crate::consensus::{ConsensusTracker, ReadinessVote, SwarmDecision, VoteOutcome};
use crate::mavlink_client::{other_err, AnyResult};
use crate::mission::hash_mission_specs;
use crate::vehicle_actor::{
    send_command, spawn_vehicle_actor, VehicleActorHandle, VehicleCommand, VehicleEvent,
    VehicleEventKind,
};
use std::collections::{HashMap, HashSet};
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time::{timeout, Instant};

pub async fn run_mission_files(paths: &[String]) -> AnyResult<()> {
    for path in paths {
        run_mission_file(path).await?;
    }

    Ok(())
}

async fn run_mission_file(path: &str) -> AnyResult<()> {
    println!("=== mission phase: {path} ===");

    let config = load_config(path)?;
    run_swarm(path.to_string(), config).await
}

async fn run_swarm(phase: String, config: SwarmConfig) -> AnyResult<()> {
    let drone_count = config.drones.len();
    if drone_count == 0 {
        return Err(other_err("mission file contains no drones"));
    }

    let expected_hashes = expected_mission_hashes(&config.drones)?;
    let consensus_enabled = config.consensus.enabled;
    let quorum_policy = config.consensus.quorum_policy()?;
    let readiness_timeout = Duration::from_secs(config.consensus.readiness_timeout_secs);

    let (event_tx, mut event_rx) = mpsc::channel::<VehicleEvent>(256);

    let mut actors = Vec::with_capacity(drone_count);
    for drone in config.drones {
        actors.push(spawn_vehicle_actor(drone, event_tx.clone()));
    }
    drop(event_tx);

    let actor_names: HashSet<String> = actors.iter().map(|actor| actor.name.clone()).collect();
    let mut tracker = ConsensusTracker::new(expected_hashes, quorum_policy);

    if consensus_enabled {
        println!(
            "[{}][supervisor] readiness gate enabled: {}, timeout={}s",
            phase,
            tracker.policy_label(),
            readiness_timeout.as_secs()
        );
    } else {
        println!(
            "[{}][supervisor] readiness gate disabled; vehicles start as soon as they report ready",
            phase
        );
    }

    for actor in &actors {
        send_command(
            &actor.commands,
            VehicleCommand::PrepareMission {
                phase: phase.clone(),
            },
            &actor.name,
        )
        .await?;
    }

    let readiness_deadline = Instant::now() + readiness_timeout;
    let mut terminal = HashSet::new();
    let mut started = HashSet::new();
    let mut planned_aborts: HashMap<String, String> = HashMap::new();
    let mut prestart_failures: HashMap<String, String> = HashMap::new();
    let mut errors = Vec::new();
    let mut phase_abort_reason: Option<String> = None;
    let mut consensus_decided = !consensus_enabled;

    while terminal.len() < drone_count {
        let event = if consensus_enabled && !consensus_decided {
            let remaining = readiness_deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                None
            } else {
                match timeout(remaining, event_rx.recv()).await {
                    Ok(event) => event,
                    Err(_) => None,
                }
            }
        } else {
            event_rx.recv().await
        };

        let Some(event) = event else {
            if consensus_enabled && !consensus_decided {
                let reason = format!(
                    "readiness timeout after {}s; {}",
                    readiness_timeout.as_secs(),
                    match tracker.decide() {
                        SwarmDecision::Hold { reason } => reason,
                        SwarmDecision::Start { reason, .. } => reason,
                        SwarmDecision::Abort { reason } => reason,
                    }
                );
                println!("[{}][supervisor] aborting phase: {}", phase, reason);
                phase_abort_reason = Some(reason.clone());
                abort_vehicles(
                    &actors,
                    &actor_names,
                    &terminal,
                    &mut planned_aborts,
                    &phase,
                    &reason,
                )
                .await;
                consensus_decided = true;
                continue;
            }

            break;
        };

        print_event(&event);

        match &event.kind {
            VehicleEventKind::ReadinessVote {
                mission_hash,
                item_count,
            } => {
                if consensus_enabled && !consensus_decided {
                    let vote = ReadinessVote {
                        vehicle: event.vehicle.clone(),
                        phase: event.phase.clone(),
                        mission_hash: mission_hash.clone(),
                        item_count: *item_count,
                    };

                    match tracker.observe_vote(vote) {
                        VoteOutcome::Accepted {
                            ready_count,
                            threshold,
                        } => println!(
                            "[{}][supervisor] readiness accepted from {}; {}/{} votes",
                            phase, event.vehicle, ready_count, threshold
                        ),
                        VoteOutcome::Rejected { reason } => println!(
                            "[{}][supervisor] readiness rejected from {}: {}",
                            phase, event.vehicle, reason
                        ),
                        VoteOutcome::Duplicate => println!(
                            "[{}][supervisor] duplicate readiness vote ignored from {}",
                            phase, event.vehicle
                        ),
                    }

                    match tracker.decide() {
                        SwarmDecision::Start {
                            participants,
                            reason,
                        } => {
                            println!("[{}][supervisor] start decision: {}", phase, reason);
                            let participant_set: HashSet<String> =
                                participants.iter().cloned().collect();
                            start_vehicles(&actors, &participant_set, &phase, &mut started).await;

                            let non_participants: HashSet<String> = actor_names
                                .difference(&participant_set)
                                .cloned()
                                .collect();
                            abort_vehicles(
                                &actors,
                                &non_participants,
                                &terminal,
                                &mut planned_aborts,
                                &phase,
                                "excluded by quorum start decision",
                            )
                            .await;
                            consensus_decided = true;
                        }
                        SwarmDecision::Abort { reason } => {
                            println!("[{}][supervisor] abort decision: {}", phase, reason);
                            phase_abort_reason = Some(reason.clone());
                            abort_vehicles(
                                &actors,
                                &actor_names,
                                &terminal,
                                &mut planned_aborts,
                                &phase,
                                &reason,
                            )
                            .await;
                            consensus_decided = true;
                        }
                        SwarmDecision::Hold { reason } => {
                            println!("[{}][supervisor] holding: {}", phase, reason);
                        }
                    }
                } else if !consensus_enabled && !started.contains(&event.vehicle) {
                    let mut vehicle_to_start = HashSet::new();
                    vehicle_to_start.insert(event.vehicle.clone());
                    start_vehicles(&actors, &vehicle_to_start, &phase, &mut started).await;
                }
            }
            VehicleEventKind::Completed => {
                terminal.insert(event.vehicle.clone());
            }
            VehicleEventKind::Failed { error } => {
                terminal.insert(event.vehicle.clone());

                if planned_aborts.contains_key(&event.vehicle) {
                    println!(
                        "[{}][supervisor] {} stopped as planned: {}",
                        phase, event.vehicle, error
                    );
                } else if consensus_enabled && !consensus_decided {
                    println!(
                        "[{}][supervisor] {} failed before readiness decision: {}",
                        phase, event.vehicle, error
                    );
                    prestart_failures.insert(event.vehicle.clone(), error.clone());
                    tracker.observe_failure(&event.vehicle, error.clone());
                    match tracker.decide() {
                        SwarmDecision::Abort { reason } => {
                            println!("[{}][supervisor] abort decision: {}", phase, reason);
                            phase_abort_reason = Some(reason.clone());
                            abort_vehicles(
                                &actors,
                                &actor_names,
                                &terminal,
                                &mut planned_aborts,
                                &phase,
                                &reason,
                            )
                            .await;
                            consensus_decided = true;
                        }
                        SwarmDecision::Start {
                            participants,
                            reason,
                        } => {
                            println!("[{}][supervisor] start decision: {}", phase, reason);
                            let participant_set: HashSet<String> =
                                participants.iter().cloned().collect();
                            start_vehicles(&actors, &participant_set, &phase, &mut started).await;

                            let non_participants: HashSet<String> = actor_names
                                .difference(&participant_set)
                                .cloned()
                                .collect();
                            abort_vehicles(
                                &actors,
                                &non_participants,
                                &terminal,
                                &mut planned_aborts,
                                &phase,
                                "excluded by quorum start decision",
                            )
                            .await;
                            consensus_decided = true;
                        }
                        SwarmDecision::Hold { reason } => {
                            println!("[{}][supervisor] holding: {}", phase, reason);
                        }
                    }
                } else if prestart_failures.contains_key(&event.vehicle) {
                    println!(
                        "[{}][supervisor] {} remains excluded from degraded start",
                        phase, event.vehicle
                    );
                } else {
                    errors.push(format!("{} failed: {}", event.vehicle, error));
                }
            }
            _ => {}
        }
    }

    for actor in &actors {
        let _ = send_command(&actor.commands, VehicleCommand::Shutdown, &actor.name).await;
    }

    for actor in actors {
        let name = actor.name;
        actor
            .task
            .await
            .map_err(|e| other_err(format!("vehicle actor {name} join failed: {e}")))?;
    }

    if terminal.len() < drone_count {
        errors.push(format!(
            "supervisor stopped after receiving terminal events from {}/{} vehicles",
            terminal.len(),
            drone_count
        ));
    }

    if let Some(reason) = phase_abort_reason {
        errors.push(format!("phase aborted by supervisor: {reason}"));
    }

    if errors.is_empty() {
        println!("=== mission phase complete: {phase} ===");
        Ok(())
    } else {
        Err(other_err(format!(
            "mission phase {phase} completed with errors: {}",
            errors.join("; ")
        )))
    }
}

fn expected_mission_hashes(drones: &[DroneConfig]) -> AnyResult<HashMap<String, String>> {
    let mut expected = HashMap::new();

    for drone in drones {
        if expected.contains_key(&drone.name) {
            return Err(other_err(format!(
                "duplicate drone name in mission config: {}",
                drone.name
            )));
        }

        expected.insert(drone.name.clone(), hash_mission_specs(&drone.mission)?);
    }

    Ok(expected)
}

async fn start_vehicles(
    actors: &[VehicleActorHandle],
    vehicles: &HashSet<String>,
    phase: &str,
    started: &mut HashSet<String>,
) {
    for actor in actors {
        if vehicles.contains(&actor.name) && !started.contains(&actor.name) {
            println!("[{}][supervisor] starting {}", phase, actor.name);
            if send_command(
                &actor.commands,
                VehicleCommand::StartPreparedMission {
                    phase: phase.to_string(),
                },
                &actor.name,
            )
            .await
            .is_ok()
            {
                started.insert(actor.name.clone());
            }
        }
    }
}

async fn abort_vehicles(
    actors: &[VehicleActorHandle],
    vehicles: &HashSet<String>,
    terminal: &HashSet<String>,
    planned_aborts: &mut HashMap<String, String>,
    phase: &str,
    reason: &str,
) {
    for actor in actors {
        if vehicles.contains(&actor.name) && !terminal.contains(&actor.name) {
            println!(
                "[{}][supervisor] aborting {} before start: {}",
                phase, actor.name, reason
            );
            planned_aborts.insert(actor.name.clone(), reason.to_string());
            let _ = send_command(
                &actor.commands,
                VehicleCommand::AbortPreparedMission {
                    phase: phase.to_string(),
                    reason: reason.to_string(),
                },
                &actor.name,
            )
            .await;
        }
    }
}

fn short_hash(hash: &str) -> &str {
    hash.get(..12).unwrap_or(hash)
}

fn print_event(event: &VehicleEvent) {
    match &event.kind {
        VehicleEventKind::CommandAccepted => {
            println!("[{}][{}] command accepted", event.phase, event.vehicle);
        }
        VehicleEventKind::Connecting { endpoint } => {
            println!(
                "[{}][{}] connecting: {}",
                event.phase, event.vehicle, endpoint
            );
        }
        VehicleEventKind::Discovered {
            system_id,
            component_id,
        } => {
            println!(
                "[{}][{}] discovered sys={}, comp={}",
                event.phase, event.vehicle, system_id, component_id
            );
        }
        VehicleEventKind::StartPosition { lat_deg, lon_deg } => {
            println!(
                "[{}][{}] start position lat={:.7}, lon={:.7}",
                event.phase, event.vehicle, lat_deg, lon_deg
            );
        }
        VehicleEventKind::MissionBuilt {
            item_count,
            spec_hash,
            upload_hash,
        } => {
            println!(
                "[{}][{}] mission built: {} items, spec_hash={}, upload_hash={}",
                event.phase,
                event.vehicle,
                item_count,
                short_hash(spec_hash),
                short_hash(upload_hash)
            );
        }
        VehicleEventKind::MissionUploaded => {
            println!("[{}][{}] mission uploaded", event.phase, event.vehicle);
        }
        VehicleEventKind::Armed => {
            println!("[{}][{}] armed", event.phase, event.vehicle);
        }
        VehicleEventKind::ReadinessVote {
            mission_hash,
            item_count,
        } => {
            println!(
                "[{}][{}] readiness vote: item_count={}, mission_hash={}",
                event.phase,
                event.vehicle,
                item_count,
                short_hash(mission_hash)
            );
        }
        VehicleEventKind::WaitingForStart { mission_hash } => {
            println!(
                "[{}][{}] waiting for supervisor start decision, mission_hash={}",
                event.phase,
                event.vehicle,
                short_hash(mission_hash)
            );
        }
        VehicleEventKind::StartApproved => {
            println!("[{}][{}] start approved", event.phase, event.vehicle);
        }
        VehicleEventKind::StartAborted { reason } => {
            println!(
                "[{}][{}] start aborted: {}",
                event.phase, event.vehicle, reason
            );
        }
        VehicleEventKind::MissionStarted => {
            println!("[{}][{}] mission started", event.phase, event.vehicle);
        }
        VehicleEventKind::MissionMonitorStarted => {
            println!("[{}][{}] monitoring mission", event.phase, event.vehicle);
        }
        VehicleEventKind::Completed => {
            println!("[{}][{}] completed", event.phase, event.vehicle);
        }
        VehicleEventKind::Failed { error } => {
            println!("[{}][{}] failed: {}", event.phase, event.vehicle, error);
        }
        VehicleEventKind::ShutdownAck => {
            println!("[{}][{}] shutdown", event.phase, event.vehicle);
        }
    }
}
