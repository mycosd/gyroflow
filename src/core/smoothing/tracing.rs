use super::*;

use crate::gyro_source::{TimeQuat, Quat64};

#[derive(Clone)]
pub struct Tracing {
    pub max_angle: f64,
    pub peak_tracking: f64,
    //pub curve: f64,
    pub horizonlock: horizon::HorizonLock
}


impl Default for Tracing {
    fn default() -> Self { Self {
        max_angle: 5.0,
        peak_tracking: 50.0,
        //curve: 0.5,
        horizonlock: Default::default()
    } }
}

impl SmoothingAlgorithm for Tracing {
    fn get_name(&self) -> String { "Tracing".to_owned() }

    fn set_parameter(&mut self, name: &str, val: f64) {
        match name {
            "max_angle" => self.max_angle = val,
            "peak_tracking" => self.peak_tracking = val,
            //"curve" => self.curve = val,
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
                "unit": "°"
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
            }/*,
            {
                "name": "curve",
                "description": "Curve",
                "type": "SliderWithField",
                "from": 0.0,
                "to": 1.0,
                "value": self.curve,
                "default": 0.5,
                "unit": ""
            }*/
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

        let max_angle_rad = self.max_angle.max(1.0).min(20.0) * std::f64::consts::PI / 180.0;
        let looseness = 1.0 - (self.peak_tracking * 0.01).max(0.0).min(1.0);

let curve = max_angle_rad;

        let first = quats.iter().next().unwrap();
        let last = quats.iter().next_back().unwrap();
        
        let mut pts = TimeQuat::new();
        pts.insert(*first.0, *first.1);
        pts.insert(*last.0, *last.1);


        loop {
            let (ts, peak_quat, line_quat, peak_angle) = max_diff(&quats, &pts, 0.0);
            if peak_angle > max_angle_rad && !pts.contains_key(&ts) {
                let new_quat = peak_quat.slerp(&line_quat, max_angle_rad/peak_angle * looseness);

                pts.insert(ts, new_quat);
            } else {
                break;
            }
        }

        let smoothed: TimeQuat = quats.iter().map(|(&ts, _)| (ts, interpolate(&pts, ts, curve))).collect();
        let smoothed = smooth_plain(&smoothed, duration, curve);
        self.horizonlock.lock(&smoothed)
    }
}

fn smooth_plain(quats: &TimeQuat, duration: f64, time_constant: f64) -> TimeQuat { 
    if quats.is_empty() || duration <= 0.0 { return quats.clone(); }

    let sample_rate: f64 = quats.len() as f64 / (duration / 1000.0);

    let mut alpha = 1.0;
    if time_constant > 0.0 {
        alpha = 1.0 - (-(1.0 / sample_rate) / time_constant).exp();
    }

    let mut q = *quats.iter().next().unwrap().1;
    let smoothed1: TimeQuat = quats.iter().map(|x| {
        q = q.slerp(x.1, alpha);
        (*x.0, q)
    }).collect();

    // Reverse pass
    let mut q = *smoothed1.iter().next_back().unwrap().1;
    smoothed1.iter().rev().map(|x| {
        q = q.slerp(x.1, alpha);
        (*x.0, q)
    }).collect()
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

    let qm1 = quats.range(..=lookup_ts).next_back().unwrap();
    if *qm1.0 == lookup_ts {
        return *qm1.1;
    }

    let qp1 = quats.range(lookup_ts..).next().unwrap();
    let time_delta = (qp1.0 - qm1.0) as f64;
    let fract = (lookup_ts - qm1.0) as f64 / time_delta;
    qm1.1.slerp(qp1.1, fract)
}

/*
fn interpolate_bezier(quats: &TimeQuat, ts: i64, curve: f64) -> Quat64 {
    if quats.is_empty() {
        return Quat64::identity();
    }
    let first = quats.iter().next().unwrap();
    if quats.len() == 1  {
        return *first.1;
    }

    let last = quats.iter().next_back().unwrap();
    let lookup_ts = ts.min(*last.0).max(*first.0);

    let mut bw_range = quats.range(..=lookup_ts);
    let qm1 = bw_range.next_back().unwrap();
    if *qm1.0 == lookup_ts {
        return *qm1.1;
    }
    let qm2 = bw_range.next_back().unwrap_or(qm1);

    let mut fw_range = quats.range(lookup_ts..);
    let qp1 = fw_range.next().unwrap();
    let qp2 = fw_range.next().unwrap_or(qp1);

    let tm = (qm1.0-qm2.0) as f64;
    let t0 = (qp1.0-qm1.0) as f64;
    let tp = (qp2.0-qp1.0) as f64;

    let dm2 = Quat64::identity().slerp(&qm2.1.rotation_to(qm1.1), 1.0/tm);
    let dm1 = Quat64::identity().slerp(&qm1.1.rotation_to(qp1.1), 1.0/t0);
    let dp2 = Quat64::identity().slerp(&qp2.1.rotation_to(qp1.1), 1.0/tp);
    let dp1 = Quat64::identity().slerp(&qp1.1.rotation_to(qm1.1), 1.0/t0);

    let sm1 = qm1.1 * (dm2.slerp(&dm1, 0.5).powf(t0*curve*0.25));
    let sp1 = qp1.1 * (dp2.slerp(&dp1, 0.5).powf(t0*curve*0.25));


    let fract = (lookup_ts - qm1.0) as f64 / t0;

	let lm1 = qm1.1.slerp(&sm1, fract);
	let lp1 = sp1.slerp(qp1.1, fract);
	let l = sm1.slerp(&sp1, fract);
	let lm2 = lm1.slerp(&l, fract);
	let lp2 = l.slerp(&lp1, fract);
	
	lm2.slerp(&lp2, fract)
}*/

fn interpolate(quats: &TimeQuat, ts: i64, curve: f64) -> Quat64 {
    // if curve == 0.0 {
        interpolate_lin(quats, ts)
    // } else {
        // interpolate_bezier(quats, ts, curve)
    // }
}

fn max_diff(quats: &TimeQuat, pts: &TimeQuat, curve: f64) -> (i64, Quat64, Quat64, f64) {
    if quats.is_empty() {
        return (0, Quat64::identity(), Quat64::identity(), 0.0);
    }
    let line = quats.iter().map(|(&ts, _)| (ts, interpolate(&pts, ts, curve)));
    quats
        .iter()
        .zip(line)
        .map(|(org, ln)| {
            (*org.0, *org.1, ln.1, org.1.angle_to(&ln.1))
        })
        .max_by(|a,b| a.3.partial_cmp(&b.3).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap()
}