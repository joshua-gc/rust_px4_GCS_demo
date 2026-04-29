use rust_px4_GCS_demo::mavlink_client::AnyResult;
use rust_px4_GCS_demo::swarm_supervisor::run_mission_file;

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
