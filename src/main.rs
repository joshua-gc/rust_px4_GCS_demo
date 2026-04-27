mod signing;

use mavlink::error::MessageReadError;
use mavlink::types::CharArray;
use mavlink::{common, Message};
use mavlink::{connect, MavConnection, MavlinkVersion};
use serde::Deserialize;
use std::error::Error;
use std::fs::File;
use std::io::{BufReader, Error as IoError, ErrorKind};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

type Msg = common::MavMessage;
type Conn = mavlink::Connection<Msg>;
type AnyResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

const PX4_MAIN_MODE_AUTO: f32 = 4.0;
const PX4_SUB_MODE_AUTO_MISSION: f32 = 4.0;

#[derive(Debug, Deserialize, Clone)]
pub struct SwarmConfig {
    drones: Vec<DroneConfig>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct DroneConfig {
    name: String,
    endpoint: String,
    expected_system_id: Option<u8>,
    arm: Option<bool>,
    auto_start: Option<bool>,
    mission: Vec<MissionSpec>,
}

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type", rename_all = "snake_case")]
enum MissionSpec {
    Takeoff {
        rel_alt_m: f32,
    },
    Waypoint {
        north_m: f64,
        east_m: f64,
        rel_alt_m: f32,
        #[serde(default = "default_acceptance_radius")]
        acceptance_radius_m: f32,
    },
    Land {
        north_m: f64,
        east_m: f64,
    },
}
fn default_acceptance_radius() -> f32 {
    2.0
}
fn main() -> AnyResult<()> {
    for path in ["m-stage.json", "m-trace.json"] {
        let config = load_config(path)?;

        let mut handles = Vec::new();
        for drone in config.drones {
            handles.push(thread::spawn(move || run_drone(drone)));
        }

        for handle in handles {
            handle.join().expect("thread panicked")?;
        }
    }

    Ok(())
}

fn load_config(path: &str) -> AnyResult<SwarmConfig> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let cfg: SwarmConfig = serde_json::from_reader(reader)?;
    Ok(cfg)
}

fn run_drone(cfg: DroneConfig) -> AnyResult<()> {
    println!("[{}] connecting on {}", cfg.name, cfg.endpoint);

    let mut raw_conn = connect::<Msg>(&cfg.endpoint)?;
    raw_conn.set_protocol_version(MavlinkVersion::V2);
    let conn = Arc::new(raw_conn);

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

    //request_data_streams(&conn, target_system, target_component)?;

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

fn build_relative_mission_items(
    target_system: u8,
    target_component: u8,
    start_lat_deg: f64,
    start_lon_deg: f64,
    mission: &[MissionSpec],
) -> AnyResult<Vec<common::MISSION_ITEM_INT_DATA>> {
    let mut out = Vec::with_capacity(mission.len());

    for (seq, spec) in mission.iter().enumerate() {
        let seq = seq as u16;

        let item = match spec {
            MissionSpec::Takeoff { rel_alt_m } => common::MISSION_ITEM_INT_DATA {
                param1: 0.0,
                param2: 0.0,
                param3: 0.0,
                param4: f32::NAN,
                x: deg_to_1e7(start_lat_deg),
                y: deg_to_1e7(start_lon_deg),
                z: *rel_alt_m,
                seq,
                command: common::MavCmd::MAV_CMD_NAV_TAKEOFF,
                target_system,
                target_component,
                frame: common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                current: if seq == 0 { 1 } else { 0 },
                autocontinue: 1,
            },

            MissionSpec::Waypoint {
                north_m,
                east_m,
                rel_alt_m,
                acceptance_radius_m,
            } => {
                let (lat_deg, lon_deg) =
                    offset_lat_lon(start_lat_deg, start_lon_deg, *north_m, *east_m);

                common::MISSION_ITEM_INT_DATA {
                    param1: 0.0,
                    param2: *acceptance_radius_m,
                    param3: 0.0,
                    param4: f32::NAN,
                    x: deg_to_1e7(lat_deg),
                    y: deg_to_1e7(lon_deg),
                    z: *rel_alt_m,
                    seq,
                    command: common::MavCmd::MAV_CMD_NAV_WAYPOINT,
                    target_system,
                    target_component,
                    frame: common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    current: if seq == 0 { 1 } else { 0 },
                    autocontinue: 1,
                }
            }

            MissionSpec::Land { north_m, east_m } => {
                let (lat_deg, lon_deg) =
                    offset_lat_lon(start_lat_deg, start_lon_deg, *north_m, *east_m);

                common::MISSION_ITEM_INT_DATA {
                    param1: 0.0,
                    param2: 0.0,
                    param3: 0.0,
                    param4: f32::NAN,
                    x: deg_to_1e7(lat_deg),
                    y: deg_to_1e7(lon_deg),
                    z: 0.0,
                    seq,
                    command: common::MavCmd::MAV_CMD_NAV_LAND,
                    target_system,
                    target_component,
                    frame: common::MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    current: if seq == 0 { 1 } else { 0 },
                    autocontinue: 1,
                }
            }
        };

        out.push(item);
    }

    Ok(out)
}

fn wait_for_vehicle(conn: &Conn, timeout: Duration) -> AnyResult<(u8, u8)> {
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
                m => {
                    println!(
                        "Unmanaged Message with name {} and id {}",
                        m.message_name(),
                        m.message_id()
                    )
                }
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

fn start_heartbeat_thread(conn: Arc<Conn>) -> (Arc<AtomicBool>, JoinHandle<()>) {
    let running = Arc::new(AtomicBool::new(true));
    let running_for_thread = running.clone();
    let handle = thread::spawn(move || {
        while running_for_thread.load(Ordering::Relaxed) {
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
    (running, handle)
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
    Box::new(IoError::other(msg.into()))
}
