use nalgebra::Vector3;
use quaternion as quat;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Range for a single axis.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Range {
    pub min: f64,
    pub max: f64,
    pub step: f64,
}

impl Default for Range {
    fn default() -> Self {
        Self {
            min: 0.0,
            max: 0.0,
            step: 0.0,
        }
    }
}

/// Options controlling workspace computation.
#[derive(Clone, Debug)]
pub struct WorkspaceOptions {
    pub payload: f64,
    pub stroke: f64,
    pub frequency: f64,
    pub servo_torque_limit: f64,
    pub ball_joint_limit_deg: f64,
    pub ball_joint_clamp: bool,
}

impl Default for WorkspaceOptions {
    fn default() -> Self {
        Self {
            payload: 0.0,
            stroke: 0.0,
            frequency: 0.0,
            servo_torque_limit: f64::INFINITY,
            ball_joint_limit_deg: 90.0,
            ball_joint_clamp: true,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub rx: f64,
    pub ry: f64,
    pub rz: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Violation {
    pub reason: String,
    pub count: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Failure {
    pub pose: Pose,
    pub reason: String,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct WorkspaceResult {
    pub coverage: f64,
    pub violations: Vec<Violation>,
    pub reachable: Vec<Pose>,
    pub unreachable: Vec<Failure>,
}

/// HashMap based ranges keyed by axis name (e.g. "x", "rx").
pub type Ranges = HashMap<String, Range>;

/// Trait describing the minimal interface the workspace algorithm expects
/// from a Stewart platform model.
pub trait Platform {
    fn update(&mut self, pos: Vector3<f64>, orient: quat::Quaternion<f64>);
    fn compute_angles(&self) -> Option<Vec<f64>>;
    fn servo_range(&self) -> Option<(f64, f64)> {
        None
    }
    fn horn_length(&self) -> Option<f64> {
        None
    }
    fn b_points(&self) -> Option<Vec<Vector3<f64>>> {
        None
    }
    fn h_points(&self) -> Option<Vec<Vector3<f64>>> {
        None
    }
    fn p_points(&self) -> Option<Vec<Vector3<f64>>> {
        None
    }
    fn cos_beta(&self) -> Option<Vec<f64>> {
        None
    }
    fn sin_beta(&self) -> Option<Vec<f64>> {
        None
    }
    fn orientation(&self) -> quat::Quaternion<f64>;
    fn translation(&self) -> Vector3<f64>;
}

fn map_load_to_force(payload: f64, stroke: f64, freq: f64) -> f64 {
    let mass = payload;
    let accel = (2.0 * std::f64::consts::PI * freq).powi(2) * (stroke / 1000.0);
    mass * (9.81 + accel) / 6.0
}

fn list(ranges: &Ranges, axis: &str) -> Vec<f64> {
    if let Some(r) = ranges.get(axis) {
        let default_step = if axis.starts_with('r') { 5.0 } else { 5.0 };
        let step = if r.step > 0.0 { r.step } else { default_step };
        let count = ((r.max - r.min) / step).floor() as usize + 1;
        let mut arr = Vec::with_capacity(count);
        let mut v = r.min;
        for _ in 0..count {
            arr.push(v);
            v += step;
        }
        arr
    } else {
        vec![0.0]
    }
}

fn dist3(a: &Vector3<f64>, b: &Vector3<f64>) -> f64 {
    (a - b).norm()
}

fn rotate_vector(q: &quat::Quaternion<f64>, v: &Vector3<f64>) -> Vector3<f64> {
    let arr = [v[0], v[1], v[2]];
    let r = quat::rotate_vector(*q, arr);
    Vector3::new(r[0], r[1], r[2])
}

/// Compute reachable workspace using a sweep over the provided ranges.
pub fn compute_workspace<P: Platform>(
    platform: &mut P,
    ranges: &Ranges,
    options: WorkspaceOptions,
) -> WorkspaceResult {
    let leg_force = map_load_to_force(options.payload, options.stroke, options.frequency);
    let base_horn_len = platform.horn_length();
    let torque_per_force = base_horn_len.unwrap_or(1.0);

    let xs = list(ranges, "x");
    let ys = list(ranges, "y");
    let zs = list(ranges, "z");
    let rxs = list(ranges, "rx");
    let rys = list(ranges, "ry");
    let rzs = list(ranges, "rz");

    let total = xs.len() * ys.len() * zs.len() * rxs.len() * rys.len() * rzs.len();

    let mut reachable = Vec::with_capacity(total);
    let mut failures = Vec::with_capacity(total);
    let mut violation_counts: HashMap<String, usize> = HashMap::new();

    for &x in &xs {
        for &y in &ys {
            for &z in &zs {
                for &rx in &rxs {
                    for &ry in &rys {
                        for &rz in &rzs {
                            let pos = Vector3::new(x, y, z);
                            let q = quat::euler_angles(
                                rx.to_radians(),
                                ry.to_radians(),
                                rz.to_radians(),
                            );
                            let prev_pos = platform.translation();
                            let prev_q = platform.orientation();
                            let mut ok = true;
                            let mut reason = String::new();
                            platform.update(pos, q);
                            let b_points = platform.b_points();
                            let h_points = platform.h_points();
                            let p_points = platform.p_points();
                            let cos_beta = platform.cos_beta();
                            let sin_beta = platform.sin_beta();

                            match platform.compute_angles() {
                                Some(angles) => {
                                    if angles.iter().any(|a| a.is_nan()) {
                                        ok = false;
                                        reason = "IK".into();
                                    } else if let Some((min, max)) = platform.servo_range() {
                                        if angles.iter().any(|a| *a < min || *a > max) {
                                            ok = false;
                                            reason = "servo range".into();
                                        }
                                    }
                                }
                                None => {
                                    ok = false;
                                    reason = "IK".into();
                                }
                            }

                            if ok {
                                if let (Some(b), Some(h), Some(horn_len)) =
                                    (b_points.as_ref(), h_points.as_ref(), base_horn_len)
                                {
                                    let tol = f64::max(1e-3 * horn_len, 0.5);
                                    for (bi, hi) in b.iter().zip(h.iter()) {
                                        if dist3(hi, bi) > horn_len + tol {
                                            ok = false;
                                            reason = "horn stretch".into();
                                            break;
                                        }
                                    }
                                }
                            }

                            if ok && options.ball_joint_clamp {
                                if let (Some(h), Some(p), Some(cb), Some(sb)) = (
                                    h_points.as_ref(),
                                    p_points.as_ref(),
                                    cos_beta.as_ref(),
                                    sin_beta.as_ref(),
                                ) {
                                    let plat_normal = rotate_vector(
                                        &platform.orientation(),
                                        &Vector3::new(0.0, 0.0, 1.0),
                                    );
                                    for i in 0..p.len() {
                                        let rod_vec = p[i] - h[i];
                                        let mag = rod_vec.norm();
                                        if mag == 0.0 {
                                            continue;
                                        }
                                        let base_axis = Vector3::new(cb[i], sb[i], 0.0);
                                        let cos_base = rod_vec.dot(&base_axis) / mag;
                                        let cos_plat = rod_vec.dot(&plat_normal) / mag;
                                        let ang_base = cos_base.abs().acos().to_degrees();
                                        let ang_plat = cos_plat.abs().acos().to_degrees();
                                        if ang_base > options.ball_joint_limit_deg
                                            || ang_plat > options.ball_joint_limit_deg
                                        {
                                            ok = false;
                                            reason = "ball joint".into();
                                            break;
                                        }
                                    }
                                }
                            }

                            if ok {
                                let torque = leg_force * torque_per_force;
                                if torque > options.servo_torque_limit {
                                    ok = false;
                                    reason = "torque".into();
                                }
                            }

                            platform.update(prev_pos, prev_q);

                            let pose = Pose {
                                x,
                                y,
                                z,
                                rx,
                                ry,
                                rz,
                            };
                            if ok {
                                reachable.push(pose);
                            } else {
                                failures.push(Failure {
                                    pose: pose.clone(),
                                    reason: reason.clone(),
                                });
                                *violation_counts.entry(reason).or_insert(0) += 1;
                            }
                        }
                    }
                }
            }
        }
    }

    let coverage = if total > 0 {
        reachable.len() as f64 / total as f64
    } else {
        0.0
    };
    let violations = violation_counts
        .into_iter()
        .map(|(reason, count)| Violation { reason, count })
        .collect();
    WorkspaceResult {
        coverage,
        violations,
        reachable,
        unreachable: failures,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DummyPlatform {
        pos: Vector3<f64>,
        q: quat::Quaternion<f64>,
    }

    impl DummyPlatform {
        fn new() -> Self {
            Self {
                pos: Vector3::zeros(),
                q: quat::id(),
            }
        }
    }

    impl Platform for DummyPlatform {
        fn update(&mut self, pos: Vector3<f64>, orient: quat::Quaternion<f64>) {
            self.pos = pos;
            self.q = orient;
        }
        fn compute_angles(&self) -> Option<Vec<f64>> {
            Some(vec![0.0; 6])
        }
        fn orientation(&self) -> quat::Quaternion<f64> {
            self.q
        }
        fn translation(&self) -> Vector3<f64> {
            self.pos
        }
    }

    #[test]
    fn simple_workspace() {
        let mut platform = DummyPlatform::new();
        let mut ranges = Ranges::new();
        ranges.insert(
            "x".into(),
            Range {
                min: 0.0,
                max: 0.0,
                step: 1.0,
            },
        );
        ranges.insert(
            "y".into(),
            Range {
                min: 0.0,
                max: 0.0,
                step: 1.0,
            },
        );
        ranges.insert(
            "z".into(),
            Range {
                min: 0.0,
                max: 0.0,
                step: 1.0,
            },
        );
        ranges.insert(
            "rx".into(),
            Range {
                min: 0.0,
                max: 0.0,
                step: 1.0,
            },
        );
        ranges.insert(
            "ry".into(),
            Range {
                min: 0.0,
                max: 0.0,
                step: 1.0,
            },
        );
        ranges.insert(
            "rz".into(),
            Range {
                min: 0.0,
                max: 0.0,
                step: 1.0,
            },
        );
        let result = compute_workspace(&mut platform, &ranges, WorkspaceOptions::default());
        assert_eq!(result.coverage, 1.0);
        assert_eq!(result.reachable.len(), 1);
        assert!(result.unreachable.is_empty());
    }
}
