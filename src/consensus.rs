//! Placeholder module for Phase 3.
//!
//! Phase 2 introduced supervisor/vehicle actors and event channels. This file
//! remains the clean boundary for the next step: swarm-level readiness voting,
//! quorum policies, and mission-phase agreement.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionPhase {
    Discovering,
    Uploading,
    Executing,
    Complete,
    Failed,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SwarmDecision {
    Continue,
    Hold { reason: String },
    Abort { reason: String },
}
