use std::process::Command;
use stewart_sim::{compute_workspace, WorkspaceOptions, Range, Ranges, Platform};
use serde_json::from_slice;
use nalgebra::Vector3;
use quaternion as quat;

#[derive(Clone)]
struct DummyPlatform {
    pos: Vector3<f64>,
    q: quat::Quaternion<f64>,
}

impl DummyPlatform {
    fn new() -> Self { Self { pos: Vector3::zeros(), q: quat::id() } }
}

impl Platform for DummyPlatform {
    fn update(&mut self, pos: Vector3<f64>, orient: quat::Quaternion<f64>) {
        self.pos = pos; self.q = orient;
    }
    fn compute_angles(&self) -> Option<Vec<f64>> { Some(vec![0.0;6]) }
    fn orientation(&self) -> quat::Quaternion<f64> { self.q }
    fn translation(&self) -> Vector3<f64> { self.pos }
}

#[test]
fn compare_with_js() {
    let output = Command::new("node").arg("tests/js_workspace_call.mjs").output().expect("run js");
    let js_result: stewart_sim::WorkspaceResult = from_slice(&output.stdout).expect("parse js");

    let mut platform = DummyPlatform::new();
    let mut ranges = Ranges::new();
    ranges.insert("x".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("y".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("z".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("rx".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("ry".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("rz".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    let rust_result = compute_workspace(&mut platform, &ranges, WorkspaceOptions::default());
    assert_eq!(js_result.coverage, rust_result.coverage);
    assert_eq!(js_result.reachable.len(), rust_result.reachable.len());
    assert_eq!(js_result.unreachable.len(), rust_result.unreachable.len());
}
