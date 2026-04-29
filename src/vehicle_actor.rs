use crate::config::DroneConfig;
use crate::mavlink_client::{
    arm_vehicle, connect_mavlink, monitor_mission, set_px4_mission_mode, start_heartbeat_thread,
    start_mission, upload_mission, wait_for_position, wait_for_vehicle, AnyResult,
};
use crate::mission::build_relative_mission_items;
use std::sync::atomic::Ordering;
use std::time::Duration;

pub fn run_drone(cfg: DroneConfig) -> AnyResult<()> {
    println!("[{}] connecting on {}", cfg.name, cfg.endpoint);

    let conn = connect_mavlink(&cfg.endpoint)?;

    let (target_system, target_component) = wait_for_vehicle(&conn, Duration::from_secs(20))?;
    println!(
        "[{}] discovered target sys={}, comp={}",
        cfg.name, target_system, target_component
    );

    if let Some(expected) = cfg.expected_system_id
        && expected != target_system
    {
        return Err(format!(
            "[{}] expected system_id {}, got {}",
            cfg.name, expected, target_system
        )
        .into());
    }

    let (heartbeat_running, heartbeat_handle) = start_heartbeat_thread(conn.clone());

    let (start_lat_deg, start_lon_deg) = wait_for_position(&conn, Duration::from_secs(20))?;
    println!(
        "[{}] start position lat={:.7}, lon={:.7}",
        cfg.name, start_lat_deg, start_lon_deg
    );

    let mission_items = build_relative_mission_items(
        target_system,
        target_component,
        start_lat_deg,
        start_lon_deg,
        &cfg.mission,
    )?;

    upload_mission(&conn, target_system, target_component, &mission_items)?;
    println!("[{}] mission uploaded", cfg.name);

    if cfg.arm.unwrap_or(true) {
        arm_vehicle(&conn, target_system, target_component)?;
        println!("[{}] armed", cfg.name);
    }

    if cfg.auto_start.unwrap_or(true) {
        set_px4_mission_mode(&conn, target_system, target_component)?;
        start_mission(
            &conn,
            target_system,
            target_component,
            mission_items.len() as u16,
        )?;
        println!("[{}] mission started", cfg.name);
    }

    monitor_mission(
        &conn,
        (mission_items.len() - 1) as u16,
        Duration::from_secs(300),
    )?;

    println!("TERMINATING -- {}", cfg.name);
    heartbeat_running.store(false, Ordering::Relaxed);
    let _ = heartbeat_handle.join();
    println!("TERMINATED -- {}", cfg.name);

    Ok(())
}
