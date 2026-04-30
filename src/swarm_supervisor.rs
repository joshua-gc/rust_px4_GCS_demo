use crate::config::{load_config, DroneConfig};
use crate::mavlink_client::{other_err, AnyResult};
use crate::vehicle_actor::{
    send_command, spawn_vehicle_actor, VehicleCommand, VehicleEvent, VehicleEventKind,
};
use std::collections::HashSet;
use tokio::sync::mpsc;

pub async fn run_mission_files(paths: &[String]) -> AnyResult<()> {
    for path in paths {
        run_mission_file(path).await?;
    }

    Ok(())
}

async fn run_mission_file(path: &str) -> AnyResult<()> {
    println!("=== mission phase: {path} ===");

    let config = load_config(path)?;
    run_swarm(path.to_string(), config.drones).await
}

async fn run_swarm(phase: String, drones: Vec<DroneConfig>) -> AnyResult<()> {
    let drone_count = drones.len();
    if drone_count == 0 {
        return Err(other_err("mission file contains no drones"));
    }

    let (event_tx, mut event_rx) = mpsc::channel::<VehicleEvent>(256);

    let mut actors = Vec::with_capacity(drone_count);
    for drone in drones {
        actors.push(spawn_vehicle_actor(drone, event_tx.clone()));
    }
    drop(event_tx);

    for actor in &actors {
        send_command(
            &actor.commands,
            VehicleCommand::RunMission {
                phase: phase.clone(),
            },
            &actor.name,
        )
        .await?;
    }

    let mut finished = HashSet::new();
    let mut errors = Vec::new();

    while finished.len() < drone_count {
        let Some(event) = event_rx.recv().await else {
            break;
        };

        print_event(&event);

        match &event.kind {
            VehicleEventKind::Completed => {
                finished.insert(event.vehicle.clone());
            }
            VehicleEventKind::Failed { error } => {
                finished.insert(event.vehicle.clone());
                errors.push(format!("{} failed: {}", event.vehicle, error));
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

    if finished.len() < drone_count {
        errors.push(format!(
            "supervisor stopped after receiving terminal events from {}/{} vehicles",
            finished.len(),
            drone_count
        ));
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
        VehicleEventKind::MissionBuilt { item_count } => {
            println!(
                "[{}][{}] mission built: {} items",
                event.phase, event.vehicle, item_count
            );
        }
        VehicleEventKind::MissionUploaded => {
            println!("[{}][{}] mission uploaded", event.phase, event.vehicle);
        }
        VehicleEventKind::Armed => {
            println!("[{}][{}] armed", event.phase, event.vehicle);
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
