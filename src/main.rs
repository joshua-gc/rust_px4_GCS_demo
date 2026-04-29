mod config;
mod consensus;
mod event_log;
mod fault_injection;
mod mavlink_client;
mod mission;
mod swarm_supervisor;
mod vehicle_actor;

use crate::mavlink_client::AnyResult;
use crate::swarm_supervisor::run_mission_file;

const DEFAULT_MISSION_FILES: [&str; 2] = ["m-stage.json", "m-trace.json"];

fn main() -> AnyResult<()> {
    let mission_files: Vec<String> = std::env::args().skip(1).collect();

    if mission_files.is_empty() {
        for path in DEFAULT_MISSION_FILES {
            run_mission_file(path)?;
        }
    } else {
        for path in mission_files {
            run_mission_file(&path)?;
        }
    }

    Ok(())
}
