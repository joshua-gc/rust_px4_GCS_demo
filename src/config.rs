use crate::mavlink_client::AnyResult;
use crate::mission::MissionSpec;
use serde::Deserialize;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Debug, Deserialize, Clone)]
pub struct SwarmConfig {
    pub drones: Vec<DroneConfig>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct DroneConfig {
    pub name: String,
    pub endpoint: String,
    pub expected_system_id: Option<u8>,
    pub arm: Option<bool>,
    pub auto_start: Option<bool>,
    pub mission: Vec<MissionSpec>,
}

pub fn load_config(path: impl AsRef<Path>) -> AnyResult<SwarmConfig> {
    let file = File::open(path.as_ref())?;
    let reader = BufReader::new(file);
    let cfg: SwarmConfig = serde_json::from_reader(reader)?;
    Ok(cfg)
}
