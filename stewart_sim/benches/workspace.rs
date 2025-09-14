use criterion::{Criterion, black_box, criterion_group, criterion_main};
use nalgebra::Vector3;
use quaternion as quat;
use stewart_sim::{Platform, Range, Ranges, WorkspaceOptions, compute_workspace};

#[derive(Clone)]
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

fn bench_compute_workspace(c: &mut Criterion) {
    let mut platform = DummyPlatform::new();
    let mut ranges = Ranges::new();
    ranges.insert(
        "x".into(),
        Range {
            min: -10.0,
            max: 10.0,
            step: 5.0,
        },
    );
    ranges.insert(
        "y".into(),
        Range {
            min: -10.0,
            max: 10.0,
            step: 5.0,
        },
    );
    ranges.insert(
        "z".into(),
        Range {
            min: -10.0,
            max: 10.0,
            step: 5.0,
        },
    );
    ranges.insert(
        "rx".into(),
        Range {
            min: -10.0,
            max: 10.0,
            step: 10.0,
        },
    );
    ranges.insert(
        "ry".into(),
        Range {
            min: -10.0,
            max: 10.0,
            step: 10.0,
        },
    );
    ranges.insert(
        "rz".into(),
        Range {
            min: -10.0,
            max: 10.0,
            step: 10.0,
        },
    );

    c.bench_function("compute_workspace", |b| {
        b.iter(|| {
            compute_workspace(
                black_box(&mut platform),
                black_box(&ranges),
                black_box(WorkspaceOptions::default()),
            )
        })
    });
}

criterion_group!(benches, bench_compute_workspace);
criterion_main!(benches);
