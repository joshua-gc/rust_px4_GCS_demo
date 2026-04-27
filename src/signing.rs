use crate::{AnyResult, Conn, Msg};
use mavlink::{common, MavConnection, SigningConfig};
use sha2::{Digest, Sha256};
const RUST_LINK_ID: u8 = 42;
const SIGNING_PHRASE: &str = "my-secure-key-123";
fn derive_signing_key(passphrase: &str) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(passphrase.as_bytes());
    let result = hasher.finalize();

    let mut key = [0u8; 32];
    key.copy_from_slice(&result);
    key
}
fn send_setup_signing(
    conn: &Conn,
    target_system: u8,
    target_component: u8,
    key: [u8; 32],
) -> AnyResult<()> {
    let msg = Msg::SETUP_SIGNING(common::SETUP_SIGNING_DATA {
        target_system,
        target_component,
        secret_key: key,
        initial_timestamp: mavlink_signing_timestamp_now(),
    });

    conn.send_default(&msg)?;
    Ok(())
}

// MAVLink signing timestamp:
// 10-microsecond ticks since 2015-01-01T00:00:00Z.
fn mavlink_signing_timestamp_now() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};

    const MAVLINK_EPOCH_UNIX_SECS: u64 = 1_420_070_400;

    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock before unix epoch");

    let secs_since_2015 = now.as_secs().saturating_sub(MAVLINK_EPOCH_UNIX_SECS);
    secs_since_2015 * 100_000 + u64::from(now.subsec_micros() / 10)
}
fn enable_signing(conn: &mut Conn, key: [u8; 32]) {
    let signing = SigningConfig::new(key, RUST_LINK_ID, true, false);

    conn.setup_signing(Some(signing));
}
