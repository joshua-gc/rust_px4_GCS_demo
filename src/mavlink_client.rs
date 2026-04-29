use mavlink::error::MessageReadError;
use mavlink::types::CharArray;
use mavlink::{common, connect, MavConnection, MavlinkVersion, Message};
use std::error::Error;
use std::io::{Error as IoError, ErrorKind};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub type Msg = common::MavMessage;
pub type Conn = mavlink::Connection<Msg>;
pub type AnyResult<T> = Result<T, Box<dyn Error + Send + Sync>>;

const PX4_MAIN_MODE_AUTO: f32 = 4.0;
const PX4_SUB_MODE_AUTO_MISSION: f32 = 4.0;

pub fn connect_mavlink(endpoint: &str) -> AnyResult<Arc<Conn>> {
    let mut raw_conn = connect::<Msg>(endpoint)?;
    raw_conn.set_protocol_version(MavlinkVersion::V2);
    Ok(Arc::new(raw_conn))
}

pub fn wait_for_vehicle(conn: &Conn, timeout: Duration) -> AnyResult<(u8, u8)> {
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

pub fn wait_for_position(conn: &Arc<Conn>, timeout: Duration) -> AnyResult<(f64, f64)> {
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
                    );
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

pub fn start_heartbeat_thread(conn: Arc<Conn>) -> (Arc<AtomicBool>, JoinHandle<()>) {
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

pub fn upload_mission(
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

pub fn arm_vehicle(conn: &Arc<Conn>, target_system: u8, target_component: u8) -> AnyResult<()> {
    let msg = Msg::COMMAND_LONG(common::COMMAND_LONG_DATA {
        param1: 1.0,
        param2: 0.0,
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

pub fn set_px4_mission_mode(
    conn: &Arc<Conn>,
    target_system: u8,
    target_component: u8,
) -> AnyResult<()> {
    let msg = Msg::COMMAND_LONG(common::COMMAND_LONG_DATA {
        param1: 1.0,
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

pub fn start_mission(
    conn: &Arc<Conn>,
    target_system: u8,
    target_component: u8,
    mission_len: u16,
) -> AnyResult<()> {
    let last_item = mission_len.saturating_sub(1) as f32;

    let msg = Msg::COMMAND_LONG(common::COMMAND_LONG_DATA {
        param1: 0.0,
        param2: last_item,
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

pub fn send_command_and_wait_ack(
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

pub fn monitor_mission(conn: &Arc<Conn>, final_seq: u16, timeout: Duration) -> AnyResult<()> {
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

fn decode_c_string(bytes: &CharArray<50>) -> String {
    bytes.to_str().unwrap_or("").to_string()
}

fn timeout_err(msg: &str) -> Box<dyn Error + Send + Sync> {
    Box::new(IoError::new(ErrorKind::TimedOut, msg.to_string()))
}

pub fn other_err(msg: impl Into<String>) -> Box<dyn Error + Send + Sync> {
    Box::new(IoError::new(ErrorKind::Other, msg.into()))
}
