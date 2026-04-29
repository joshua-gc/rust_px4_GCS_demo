//! Placeholder module for Phase 2.
//!
//! This file is intentionally small in the Phase 1 refactor. The goal is to
//! reserve a clean boundary for swarm-level readiness voting, quorum policies,
//! and mission-phase agreement without changing the current runtime behaviour.

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
