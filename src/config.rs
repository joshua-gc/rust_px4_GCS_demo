use crate::consensus::QuorumPolicy;
use crate::mavlink_client::{other_err, AnyResult};
use crate::mission::MissionSpec;
use serde::Deserialize;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

#[derive(Debug, Deserialize, Clone)]
pub struct SwarmConfig {
    pub drones: Vec<DroneConfig>,

    /// Optional Phase 3 swarm-level readiness/consensus settings.
    ///
    /// Existing mission JSON files remain valid because this field has a
    /// default. If omitted, the supervisor requires every configured vehicle to
    /// report ready before the mission phase is started.
    #[serde(default)]
    pub consensus: ConsensusConfig,
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

#[derive(Debug, Deserialize, Clone)]
#[serde(default)]
pub struct ConsensusConfig {
    /// Turn the Phase 3 readiness gate on/off.
    pub enabled: bool,

    /// Supported values:
    /// - "require_all" / "all"
    /// - "majority" / "quorum"
    /// - "at_least" / "n_of_m" / "count" together with quorum_count
    pub quorum_policy: String,

    /// Used only when quorum_policy is "at_least".
    pub quorum_count: Option<usize>,

    /// How long the supervisor waits for readiness votes before aborting the
    /// phase. This is intentionally independent from MAVLink command timeouts.
    pub readiness_timeout_secs: u64,
}

impl Default for ConsensusConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            quorum_policy: "require_all".to_string(),
            quorum_count: None,
            readiness_timeout_secs: 90,
        }
    }
}

impl ConsensusConfig {
    pub fn quorum_policy(&self) -> AnyResult<QuorumPolicy> {
        let policy = match self.quorum_policy.trim().to_ascii_lowercase().as_str() {
            "require_all" | "all" => QuorumPolicy::RequireAll,
            "majority" | "quorum" => QuorumPolicy::Majority,
            "at_least" | "n_of_m" | "count" => {
                let count = self.quorum_count.ok_or_else(|| {
                    other_err("consensus.quorum_count is required when quorum_policy is at_least")
                })?;
                QuorumPolicy::AtLeast(count)
            }
            other => {
                return Err(other_err(format!(
                    "unsupported consensus.quorum_policy {other:?}; use require_all, majority, or at_least"
                )));
            }
        };

        Ok(policy)
    }
}

pub fn load_config(path: impl AsRef<Path>) -> AnyResult<SwarmConfig> {
    let file = File::open(path.as_ref())?;
    let reader = BufReader::new(file);
    let cfg: SwarmConfig = serde_json::from_reader(reader)?;
    Ok(cfg)
}
