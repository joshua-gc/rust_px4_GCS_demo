mod config;
mod consensus;
mod event_log;
mod fault_injection;
mod mavlink_client;
mod mission;
mod swarm_supervisor;
mod vehicle_actor;

use crate::mavlink_client::AnyResult;
use crate::swarm_supervisor::run_mission_files;
const DEFAULT_MISSION_FILES: [&str; 2] = ["m-stage.json", "m-trace.json"];

#[tokio::main(flavor = "multi_thread")]
async fn main() -> AnyResult<()> {
    let mission_files: Vec<String> = std::env::args().skip(1).collect();

    if mission_files.is_empty() {
        let defaults = DEFAULT_MISSION_FILES.map(String::from);
        run_mission_files(&defaults).await?;
    } else {
        run_mission_files(&mission_files).await?;
    }

    Ok(())
}
