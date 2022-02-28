use super::*;

use crate::gyro_source::{TimeQuat, Quat64};

#[derive(Clone)]
pub struct Tracing {
    pub max_angle: f64,
    pub peak_tracking: f64,
    pub horizonlock: horizon::HorizonLock
}


impl Default for Tracing {
    fn default() -> Self { Self {
        max_angle: 5.0,
        peak_tracking: 50.0,
        horizonlock: Default::default()
    } }
}

impl SmoothingAlgorithm for Tracing {
    fn get_name(&self) -> String { "Tracing".to_owned() }

    fn set_parameter(&mut self, name: &str, val: f64) {
        match name {
            "max_angle" => self.max_angle = val,
            "peak_tracking" => self.peak_tracking = val,
            _ => log::error!("Invalid parameter name: {}", name)
        }
    }

    fn set_horizon_lock(&mut self, lock_percent: f64, roll: f64) {
        self.horizonlock.set_horizon(lock_percent, roll);
    }
    
    fn get_parameters_json(&self) -> serde_json::Value {
        serde_json::json!([
            {
                "name": "max_angle",
                "description": "Max.Angle",
                "type": "SliderWithField",
                "from": 1.0,
                "to": 20.0,
                "value": self.max_angle,
                "default": 5.0,
                "unit": "deg"
            },
            {
                "name": "peak_tracking",
                "description": "Peak Tracking",
                "type": "SliderWithField",
                "from": 0.0,
                "to": 100.0,
                "value": self.peak_tracking,
                "default": 50.0,
                "unit": "%"
            }
        ])
    }

    fn get_status_json(&self) -> serde_json::Value {
        serde_json::json!([])
    }

    fn get_checksum(&self) -> u64 {
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        hasher.write_u64(self.max_angle.to_bits());
        hasher.write_u64(self.peak_tracking.to_bits());
        hasher.write_u64(self.horizonlock.get_checksum());
        hasher.finish()
    }

    fn smooth(&mut self, quats: &TimeQuat, duration: f64, _stabilization_params: &StabilizationParams) -> TimeQuat {
        if quats.is_empty() || duration <= 0.0 { return quats.clone(); }

        let max_angle_rad = self.max_angle * std::f64::consts::PI / 180.0;
        let looseness = 1.0 - self.peak_tracking * 0.01;

        let first = quats.iter().next().unwrap();
        let last = quats.iter().next_back().unwrap();
        
        let mut pts = TimeQuat::new();
        pts.insert(*first.0, *first.1);
        pts.insert(*last.0, *last.1);


        loop {
            let (ts, peak_quat, line_quat, peak_angle) = max_diff(&quats, &pts);
            if peak_angle > max_angle_rad {
                let new_quat = peak_quat.slerp(&line_quat, max_angle_rad/peak_angle * looseness);

                pts.insert(ts, new_quat);
            } else {
                break;
            }
        }

        let smoothed = quats.iter().map(|(&ts, _)| (ts, interpolate_lin(&pts, ts))).collect();

        self.horizonlock.lock(&smoothed)
    }
}

fn interpolate_lin(quats: &TimeQuat, ts: i64) -> Quat64 {
    if quats.is_empty() {
        return Quat64::identity();
    }
    let first = quats.iter().next().unwrap();
    if quats.len() == 1  {
        return *first.1;
    }

    let last = quats.iter().next_back().unwrap();
    let lookup_ts = ts.min(*last.0).max(*first.0);

    let quat1 = quats.range(..=lookup_ts).next_back().unwrap();
    if *quat1.0 == lookup_ts {
        return *quat1.1;
    }

    let quat2 = quats.range(lookup_ts..).next().unwrap();
    let time_delta = (quat2.0 - quat1.0) as f64;
    let fract = (lookup_ts - quat1.0) as f64 / time_delta;
    quat1.1.slerp(quat2.1, fract)
}

fn max_diff(quats: &TimeQuat, pts: &TimeQuat) -> (i64, Quat64, Quat64, f64) {
    let line = quats.iter().map(|(&ts, _)| (ts, interpolate_lin(&pts, ts)));
    quats
        .iter()
        .zip(line)
        .map(|(org, ln)| {
            (*org.0, *org.1, ln.1, org.1.angle_to(&ln.1))
        })
        .max_by(|a,b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap()
}