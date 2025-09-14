use criterion::{Criterion, black_box, criterion_group, criterion_main};
use nalgebra::Vector3;
use quaternion as quat;
use stewart_sim::optimizer::{ConfigurablePlatform, Layout, Optimizer};
use stewart_sim::{Platform, Range, Ranges, WorkspaceOptions};

#[derive(Clone)]
struct DummyPlatform {
    layout: Layout,
    pos: Vector3<f64>,
    q: quat::Quaternion<f64>,
}

impl DummyPlatform {
    fn new() -> Self {
        Self {
            layout: Layout {
                horn_length: 10.0,
                rod_length: 20.0,
            },
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
    fn horn_length(&self) -> Option<f64> {
        Some(self.layout.horn_length)
    }
    fn orientation(&self) -> quat::Quaternion<f64> {
        self.q
    }
    fn translation(&self) -> Vector3<f64> {
        self.pos
    }
}

impl ConfigurablePlatform for DummyPlatform {
    fn apply_layout(&mut self, layout: &Layout) {
        self.layout = layout.clone();
    }
    fn layout(&self) -> Layout {
        self.layout.clone()
    }
}

fn bench_optimizer_step(c: &mut Criterion) {
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
    let options = WorkspaceOptions::default();
    let platform = DummyPlatform::new();
    let mut opt = Optimizer::new(platform, ranges, options, 8, 1, 0.1);
    opt.initialize();

    c.bench_function("optimizer_step", |b| b.iter(|| black_box(opt.step())));
}

criterion_group!(benches, bench_optimizer_step);
criterion_main!(benches);
