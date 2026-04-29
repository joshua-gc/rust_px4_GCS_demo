use crate::config::{load_config, DroneConfig};
use crate::mavlink_client::{other_err, AnyResult};
use crate::vehicle_actor::run_drone;
use std::thread;

pub fn run_mission_file(path: &str) -> AnyResult<()> {
    println!("=== mission phase: {path} ===");

    let config = load_config(path)?;
    run_swarm(config.drones)
}

fn run_swarm(drones: Vec<DroneConfig>) -> AnyResult<()> {
    let mut handles = Vec::with_capacity(drones.len());

    for drone in drones {
        handles.push(thread::spawn(move || run_drone(drone)));
    }

    for handle in handles {
        match handle.join() {
            Ok(result) => result?,
            Err(_) => return Err(other_err("drone worker thread panicked")),
        }
    }

    Ok(())
}
