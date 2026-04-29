//! Placeholder module for Phase 4.
//!
//! Future work can put lossy-link simulation here: dropped packets, delayed
//! ACKs, duplicated messages, blackholed drones, and stale telemetry.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultMode {
    None,
    DropLink,
    DelayAcks,
    StaleTelemetry,
}
