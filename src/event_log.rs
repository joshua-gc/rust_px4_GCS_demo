//! Placeholder module for event-sourced mission audit logging.
//!
//! Future work can append vehicle events and supervisor decisions here, then use
//! the log for replay, debugging, post-mission analysis, or hash-chained audit.

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EventRecord {
    pub seq: u64,
    pub source: String,
    pub event: String,
}
