use crate::mavlink_client::AnyResult;
use mavlink::common;
use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum MissionSpec {
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

pub fn build_relative_mission_items(
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

pub fn offset_lat_lon(lat_deg: f64, lon_deg: f64, north_m: f64, east_m: f64) -> (f64, f64) {
    let lat_rad = lat_deg.to_radians();
    let d_lat = north_m / 111_111.0;
    let d_lon = east_m / (111_111.0 * lat_rad.cos().max(0.01));
    (lat_deg + d_lat, lon_deg + d_lon)
}

pub fn deg_to_1e7(v: f64) -> i32 {
    (v * 1e7).round() as i32
}
