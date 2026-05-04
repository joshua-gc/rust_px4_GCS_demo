use std::collections::{BTreeSet, HashMap, HashSet};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionPhase {
    Discovering,
    Uploading,
    WaitingForReadiness,
    Executing,
    Complete,
    Failed,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SwarmDecision {
    /// Enough valid readiness votes have arrived. Start these vehicles and
    /// abort any other non-terminal actors for this phase.
    Start {
        participants: Vec<String>,
        reason: String,
    },

    /// Keep waiting for more votes/events.
    Hold { reason: String },

    /// The phase can no longer satisfy the configured quorum policy.
    Abort { reason: String },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuorumPolicy {
    RequireAll,
    Majority,
    AtLeast(usize),
}

impl QuorumPolicy {
    pub fn threshold(self, total: usize) -> usize {
        match self {
            QuorumPolicy::RequireAll => total,
            QuorumPolicy::Majority => (total / 2) + 1,
            QuorumPolicy::AtLeast(n) => n.min(total),
        }
    }

    pub fn label(self, total: usize) -> String {
        match self {
            QuorumPolicy::RequireAll => format!("require_all ({total}/{total})"),
            QuorumPolicy::Majority => format!("majority ({}/{total})", self.threshold(total)),
            QuorumPolicy::AtLeast(n) => format!("at_least ({}/{total})", n.min(total)),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadinessVote {
    pub vehicle: String,
    pub phase: String,
    pub mission_hash: String,
    pub item_count: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VoteOutcome {
    Accepted {
        ready_count: usize,
        threshold: usize,
    },
    Rejected {
        reason: String,
    },
    Duplicate,
}

#[derive(Debug, Clone)]
pub struct ConsensusTracker {
    policy: QuorumPolicy,
    expected_hashes: HashMap<String, String>,
    votes: HashMap<String, ReadinessVote>,
    rejected: HashMap<String, String>,
    failed: HashMap<String, String>,
}

impl ConsensusTracker {
    pub fn new(expected_hashes: HashMap<String, String>, policy: QuorumPolicy) -> ConsensusTracker {
        ConsensusTracker {
            policy,
            expected_hashes,
            votes: HashMap::new(),
            rejected: HashMap::new(),
            failed: HashMap::new(),
        }
    }

    pub fn total(&self) -> usize {
        self.expected_hashes.len()
    }

    pub fn threshold(&self) -> usize {
        self.policy.threshold(self.total())
    }

    pub fn policy_label(&self) -> String {
        self.policy.label(self.total())
    }

    pub fn expected_vehicles(&self) -> BTreeSet<String> {
        self.expected_hashes.keys().cloned().collect()
    }

    pub fn observe_vote(&mut self, vote: ReadinessVote) -> VoteOutcome {
        if self.votes.contains_key(&vote.vehicle) {
            return VoteOutcome::Duplicate;
        }

        let Some(expected_hash) = self.expected_hashes.get(&vote.vehicle) else {
            let reason = format!("{} is not part of this mission phase", vote.vehicle);
            self.rejected.insert(vote.vehicle, reason.clone());
            return VoteOutcome::Rejected { reason };
        };

        if expected_hash != &vote.mission_hash {
            let reason = format!(
                "{} reported mission hash {}, expected {}",
                vote.vehicle, vote.mission_hash, expected_hash
            );
            self.rejected.insert(vote.vehicle, reason.clone());
            return VoteOutcome::Rejected { reason };
        }

        self.votes.insert(vote.vehicle.clone(), vote);
        VoteOutcome::Accepted {
            ready_count: self.votes.len(),
            threshold: self.threshold(),
        }
    }

    pub fn observe_failure(&mut self, vehicle: &str, error: impl Into<String>) {
        self.failed.insert(vehicle.to_string(), error.into());
    }

    pub fn decide(&self) -> SwarmDecision {
        let threshold = self.threshold();
        let mut participants: Vec<String> = self.votes.keys().cloned().collect();
        participants.sort();

        if participants.len() >= threshold {
            return SwarmDecision::Start {
                participants,
                reason: format!(
                    "{} valid readiness votes satisfy {}",
                    self.votes.len(),
                    self.policy_label()
                ),
            };
        }

        let invalid_or_terminal: HashSet<String> = self
            .failed
            .keys()
            .chain(self.rejected.keys())
            .cloned()
            .collect();
        let possible_ready = self.total().saturating_sub(invalid_or_terminal.len());

        if possible_ready < threshold {
            return SwarmDecision::Abort {
                reason: format!(
                    "quorum impossible: {} valid votes, at most {} possible, need {} ({})",
                    self.votes.len(),
                    possible_ready,
                    threshold,
                    self.policy_label()
                ),
            };
        }

        SwarmDecision::Hold {
            reason: format!(
                "waiting for readiness: {} valid votes, need {} ({})",
                self.votes.len(),
                threshold,
                self.policy_label()
            ),
        }
    }
}
