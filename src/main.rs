use mavlink::common;
use mavlink::error::MessageReadError;
use mavlink::types::CharArray;
use mavlink::{connect, MavConnection, MavlinkVersion};
use std::error::Error;
use std::io::{Error as IoError, ErrorKind};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

type Msg = common::MavMessage;
type Conn = mavlink::Connection<Msg>;
type AnyResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

// These come from PX4's px4_custom_mode.h:
// PX4_CUSTOM_MAIN_MODE_AUTO = 4
// PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
const PX4_MAIN_MODE_AUTO: f32 = 4.0;
const PX4_SUB_MODE_AUTO_MISSION: f32 = 4.0;

fn main() -> AnyResult<()> {
    let connect_addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "udpin:0.0.0.0:14540".to_string());

    println!("Connecting on {connect_addr}");
    println!("Expected setup:");
    println!("  - PX4 SITL in Docker published on 14550 and 14540");
    println!("  - QGroundControl connected to PX4 on 14550");
    println!("  - This Rust app listening on 14540");

    let mut raw_conn = connect::<Msg>(&connect_addr)?;
    raw_conn.set_protocol_version(MavlinkVersion::V2);
    let conn = Arc::new(raw_conn);

    let (target_system, target_component) = wait_for_vehicle(&conn, Duration::from_secs(20))?;
    println!(
        "Discovered PX4 vehicle: system_id={}, component_id={}",
        target_system, target_component
    );

    start_heartbeat_thread(conn.clone());

    let (home_lat_deg, home_lon_deg) = wait_for_position(&conn, Duration::from_secs(20))?;
    println!(
        "Initial position: lat={:.7}, lon={:.7}",
        home_lat_deg, home_lon_deg
    );

    let mission = build_demo_mission(
        target_system,
        target_component,
        home_lat_deg,
        home_lon_deg,
        15.0, // relative altitude in meters
        20.0, // leg size in meters
    );

    upload_mission(&conn, target_system, target_component, &mission)?;
    println!("Mission uploaded.");

    arm_vehicle(&conn, target_system, target_component)?;
    println!("Vehicle armed.");

    set_px4_mission_mode(&conn, target_system, target_component)?;
    println!("Vehicle switched to AUTO.MISSION.");

    start_mission(&conn, target_system, target_component, mission.len() as u16)?;
    println!("Mission started.");

    monitor_mission(&conn, (mission.len() - 1) as u16, Duration::from_secs(300))?;
    println!("Mission complete.");

    Ok(())
}

fn wait_for_vehicle(conn: &Arc<Conn>, timeout: Duration) -> AnyResult<(u8, u8)> {
    let deadline = Instant::now() + timeout;

    while Instant::now() < deadline {
        match conn.recv() {
            Ok((header, msg)) => match msg {
                Msg::HEARTBEAT(hb) => {
                    if hb.autopilot != common::MavAutopilot::MAV_AUTOPILOT_INVALID {
                        return Ok((header.system_id, header.component_id));
                    }
                }
                Msg::STATUSTEXT(text) => {
                    println!("PX4: {}", decode_c_string(&text.text));
                }
                _ => {}
            },
            Err(MessageReadError::Io(e)) if e.kind() == ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                return Err(other_err(format!(
                    "recv error while waiting for HEARTBEAT: {e:?}"
                )));
            }
        }
    }

    Err(timeout_err("Timed out waiting for PX4 HEARTBEAT"))
}

fn wait_for_position(conn: &Arc<Conn>, timeout: Duration) -> AnyResult<(f64, f64)> {
    let deadline = Instant::now() + timeout;

    while Instant::now() < deadline {
        match conn.recv() {
            Ok((_header, msg)) => match msg {
                Msg::GLOBAL_POSITION_INT(pos) => {
                    if pos.lat != 0 && pos.lon != 0 {
                        return Ok((pos.lat as f64 / 1e7, pos.lon as f64 / 1e7));
                    }
                }
                Msg::STATUSTEXT(text) => {
                    println!("PX4: {}", decode_c_string(&text.text));
                }
                _ => {}
            },
            Err(MessageReadError::Io(e)) if e.kind() == ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                return Err(other_err(format!(
                    "recv error while waiting for position: {e:?}"
                )));
            }
        }
    }

    Err(timeout_err("Timed out waiting for GLOBAL_POSITION_INT"))
}

fn start_heartbeat_thread(conn: Arc<Conn>) {
    thread::spawn(move || {
        loop {
            let heartbeat = Msg::HEARTBEAT(common::HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: common::MavType::MAV_TYPE_GCS,
                autopilot: common::MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: common::MavModeFlag::empty(),
                system_status: common::MavState::MAV_STATE_ACTIVE,
                mavlink_version: 3,
            });

            if let Err(e) = conn.send_default(&heartbeat) {
                eprintln!("heartbeat send failed: {e:?}");
            }

            thread::sleep(Duration::from_secs(1));
        }
    });
}

fn build_demo_mission(
    target_system: u8,
    target_component: u8,
    home_lat_deg: f64,
    home_lon_deg: f64,
    rel_alt_m: f32,
    leg_m: f64,
) -> Vec<common::MISSION_ITEM_INT_DATA> {
    let (p1_lat, p1_lon) = offset_lat_lon(home_lat_deg, home_lon_deg, leg_m, 0.0);
    let (p2_lat, p2_lon) = offset_lat_lon(home_lat_deg, home_lon_deg, leg_m, leg_m);
    let (p3_lat, p3_lon) = offset_lat_lon(home_lat_deg, home_lon_deg, 0.0, leg_m);

    vec![
        mission_takeoff_item(
            0,
            target_system,
            target_component,
            home_lat_deg,
            home_lon_deg,
            rel_alt_m,
            1,
        ),
        mission_waypoint_item(
            1,
            target_system,
            target_component,
            p1_lat,
            p1_lon,
            rel_alt_m,
            0,
        ),
        mission_waypoint_item(
            2,
            target_system,
            target_component,
            p2_lat,
            p2_lon,
            rel_alt_m,
            0,
        ),
        mission_waypoint_item(
            3,
            target_system,
            target_component,
            p3_lat,
            p3_lon,
            rel_alt_m,
            0,
        ),
        mission_waypoint_item(
            4,
            target_system,
            target_component,
            home_lat_deg,
            home_lon_deg,
            rel_alt_m,
            0,
        ),
    ]
}

fn mission_takeoff_item(
    seq: u16,
    target_system: u8,
    target_component: u8,
    lat_deg: f64,
    lon_deg: f64,
    rel_alt_m: f32,
    current: u8,
) -> common::MISSION_ITEM_INT_DATA {
    common::MISSION_ITEM_INT_DATA {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: f32::NAN,
        x: deg_to_1e7(lat_deg),
        y: deg_to_1e7(lon_deg),
        z: rel_alt_m,
        seq,
        command: common::MavCmd::MAV_CMD_NAV_TAKEOFF,
        target_system,
        target_component,
        frame: common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        current,
        autocontinue: 1,
    }
}

fn mission_waypoint_item(
    seq: u16,
    target_system: u8,
    target_component: u8,
    lat_deg: f64,
    lon_deg: f64,
    rel_alt_m: f32,
    current: u8,
) -> common::MISSION_ITEM_INT_DATA {
    common::MISSION_ITEM_INT_DATA {
        param1: 0.0,      // hold time
        param2: 2.0,      // acceptance radius
        param3: 0.0,      // pass radius
        param4: f32::NAN, // yaw unchanged
        x: deg_to_1e7(lat_deg),
        y: deg_to_1e7(lon_deg),
        z: rel_alt_m,
        seq,
        command: common::MavCmd::MAV_CMD_NAV_WAYPOINT,
        target_system,
        target_component,
        frame: common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        current,
        autocontinue: 1,
    }
}

fn upload_mission(
    conn: &Arc<Conn>,
    target_system: u8,
    target_component: u8,
    mission: &[common::MISSION_ITEM_INT_DATA],
) -> AnyResult<()> {
    let count_msg = Msg::MISSION_COUNT(common::MISSION_COUNT_DATA {
        count: mission.len() as u16,
        target_system,
        target_component,
    });

    conn.send_default(&count_msg)?;
    println!("Sent MISSION_COUNT({})", mission.len());

    let deadline = Instant::now() + Duration::from_secs(30);

    while Instant::now() < deadline {
        match conn.recv() {
            Ok((_header, msg)) => match msg {
                Msg::MISSION_REQUEST_INT(req) => {
                    let idx = req.seq as usize;
                    if idx >= mission.len() {
                        return Err(other_err(format!(
                            "PX4 requested out-of-range mission item {}",
                            idx
                        )));
                    }

                    conn.send_default(&Msg::MISSION_ITEM_INT(mission[idx].clone()))?;
                    println!("Sent MISSION_ITEM_INT seq={}", idx);
                }
                Msg::MISSION_ACK(ack) => match ack.mavtype {
                    common::MavMissionResult::MAV_MISSION_ACCEPTED => return Ok(()),
                    other => {
                        return Err(other_err(format!(
                            "MISSION_ACK was not accepted: {other:?}"
                        )));
                    }
                },
                Msg::STATUSTEXT(text) => {
                    println!("PX4: {}", decode_c_string(&text.text));
                }
                _ => {}
            },
            Err(MessageReadError::Io(e)) if e.kind() == ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                return Err(other_err(format!(
                    "recv error during mission upload: {e:?}"
                )));
            }
        }
    }

    Err(timeout_err("Timed out during mission upload"))
}

fn arm_vehicle(conn: &Arc<Conn>, target_system: u8, target_component: u8) -> AnyResult<()> {
    let msg = Msg::COMMAND_LONG(common::COMMAND_LONG_DATA {
        param1: 1.0, // arm
        param2: 0.0, // no force
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        target_system,
        target_component,
        confirmation: 0,
    });

    send_command_and_wait_ack(
        conn,
        msg,
        common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        Duration::from_secs(10),
    )
}

fn set_px4_mission_mode(
    conn: &Arc<Conn>,
    target_system: u8,
    target_component: u8,
) -> AnyResult<()> {
    let msg = Msg::COMMAND_LONG(common::COMMAND_LONG_DATA {
        param1: 1.0, // custom mode enabled
        param2: PX4_MAIN_MODE_AUTO,
        param3: PX4_SUB_MODE_AUTO_MISSION,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: common::MavCmd::MAV_CMD_DO_SET_MODE,
        target_system,
        target_component,
        confirmation: 0,
    });

    send_command_and_wait_ack(
        conn,
        msg,
        common::MavCmd::MAV_CMD_DO_SET_MODE,
        Duration::from_secs(10),
    )
}

fn start_mission(
    conn: &Arc<Conn>,
    target_system: u8,
    target_component: u8,
    mission_len: u16,
) -> AnyResult<()> {
    let last_item = mission_len.saturating_sub(1) as f32;

    let msg = Msg::COMMAND_LONG(common::COMMAND_LONG_DATA {
        param1: 0.0,       // first item
        param2: last_item, // last item
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: common::MavCmd::MAV_CMD_MISSION_START,
        target_system,
        target_component,
        confirmation: 0,
    });

    send_command_and_wait_ack(
        conn,
        msg,
        common::MavCmd::MAV_CMD_MISSION_START,
        Duration::from_secs(10),
    )
}

fn send_command_and_wait_ack(
    conn: &Arc<Conn>,
    msg: Msg,
    expected_command: common::MavCmd,
    timeout: Duration,
) -> AnyResult<()> {
    conn.send_default(&msg)?;
    let deadline = Instant::now() + timeout;

    while Instant::now() < deadline {
        match conn.recv() {
            Ok((_header, rx_msg)) => match rx_msg {
                Msg::COMMAND_ACK(ack) if ack.command == expected_command => match ack.result {
                    common::MavResult::MAV_RESULT_ACCEPTED
                    | common::MavResult::MAV_RESULT_IN_PROGRESS => return Ok(()),
                    other => {
                        return Err(other_err(format!(
                            "COMMAND_ACK for {expected_command:?} returned {other:?}"
                        )));
                    }
                },
                Msg::STATUSTEXT(text) => {
                    println!("PX4: {}", decode_c_string(&text.text));
                }
                _ => {}
            },
            Err(MessageReadError::Io(e)) if e.kind() == ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                return Err(other_err(format!(
                    "recv error while waiting for COMMAND_ACK: {e:?}"
                )));
            }
        }
    }

    Err(timeout_err("Timed out waiting for COMMAND_ACK"))
}

fn monitor_mission(conn: &Arc<Conn>, final_seq: u16, timeout: Duration) -> AnyResult<()> {
    let deadline = Instant::now() + timeout;

    while Instant::now() < deadline {
        match conn.recv() {
            Ok((_header, msg)) => match msg {
                Msg::MISSION_CURRENT(current) => {
                    println!("MISSION_CURRENT seq={}", current.seq);
                }
                Msg::MISSION_ITEM_REACHED(reached) => {
                    println!("MISSION_ITEM_REACHED seq={}", reached.seq);
                    if reached.seq >= final_seq {
                        return Ok(());
                    }
                }
                Msg::STATUSTEXT(text) => {
                    println!("PX4: {}", decode_c_string(&text.text));
                }
                _ => {}
            },
            Err(MessageReadError::Io(e)) if e.kind() == ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                return Err(other_err(format!(
                    "recv error while monitoring mission: {e:?}"
                )));
            }
        }
    }

    Err(timeout_err("Timed out waiting for mission completion"))
}

fn offset_lat_lon(lat_deg: f64, lon_deg: f64, north_m: f64, east_m: f64) -> (f64, f64) {
    let lat_rad = lat_deg.to_radians();
    let d_lat = north_m / 111_111.0;
    let d_lon = east_m / (111_111.0 * lat_rad.cos().max(0.01));
    (lat_deg + d_lat, lon_deg + d_lon)
}

fn deg_to_1e7(v: f64) -> i32 {
    (v * 1e7).round() as i32
}

fn decode_c_string(bytes: &CharArray<50>) -> String {
    bytes.to_str().unwrap().to_string()
}

fn timeout_err(msg: &str) -> Box<dyn Error + Send + Sync> {
    Box::new(IoError::new(ErrorKind::TimedOut, msg.to_string()))
}

fn other_err(msg: impl Into<String>) -> Box<dyn Error + Send + Sync> {
    Box::new(IoError::new(ErrorKind::Other, msg.into()))
}
