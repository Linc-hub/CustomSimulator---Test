use fastrand::Rng;
use serde::{Deserialize, Serialize};

use crate::workspace::{compute_workspace, Platform, Ranges, WorkspaceOptions, WorkspaceResult};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Layout {
    pub horn_length: f64,
    pub rod_length: f64,
}

/// A platform that can be reconfigured by applying a layout.
pub trait ConfigurablePlatform: Platform {
    fn apply_layout(&mut self, layout: &Layout);
    fn layout(&self) -> Layout;
}

/// Simple genetic optimizer translating logic from optimizer.js
pub struct Optimizer<P: ConfigurablePlatform> {
    pub platform: P,
    pub ranges: Ranges,
    pub options: WorkspaceOptions,
    pub population_size: usize,
    pub generations: usize,
    pub mutation_rate: f64,
    pub population: Vec<Layout>,
    rng: Rng,
}

impl<P: ConfigurablePlatform> Optimizer<P> {
    pub fn new(platform: P, ranges: Ranges, options: WorkspaceOptions, population_size: usize, generations: usize, mutation_rate: f64) -> Self {
        Self {
            platform,
            ranges,
            options,
            population_size,
            generations,
            mutation_rate,
            population: Vec::new(),
            rng: Rng::new(),
        }
    }

    fn randomize_layout(&mut self, base: &Layout) -> Layout {
        let mut rand_val = |v: f64| v + (self.rng.f64() * 2.0 - 1.0) * 5.0;
        Layout { horn_length: rand_val(base.horn_length), rod_length: rand_val(base.rod_length) }
    }

    pub fn initialize(&mut self) {
        let base = self.platform.layout();
        self.population = (0..self.population_size).map(|_| self.randomize_layout(&base)).collect();
    }

    fn evaluate(&mut self, layout: &Layout) -> WorkspaceResult {
        self.platform.apply_layout(layout);
        compute_workspace(&mut self.platform, &self.ranges, self.options.clone())
    }

    fn mutate(&mut self, layout: &mut Layout) {
        let mut rand_val = |v: &mut f64| {
            if self.rng.f64() < self.mutation_rate {
                *v += (self.rng.f64() * 2.0 - 1.0) * 5.0;
            }
        };
        rand_val(&mut layout.horn_length);
        rand_val(&mut layout.rod_length);
    }

    /// Execute one optimization step and return best layout found.
    pub fn step(&mut self) -> Layout {
        let population = self.population.clone();
        let mut scored: Vec<(WorkspaceResult, Layout)> = population
            .into_iter()
            .map(|layout| (self.evaluate(&layout), layout))
            .collect();
        scored.sort_by(|a, b| b.0.coverage.partial_cmp(&a.0.coverage).unwrap());
        let survivors: Vec<Layout> = scored
            .iter()
            .take(self.population_size / 2)
            .map(|(_, l)| l.clone())
            .collect();
        let mut new_pop = survivors.clone();
        while new_pop.len() < self.population_size {
            let mut child = survivors[self.rng.usize(0..survivors.len())].clone();
            self.mutate(&mut child);
            new_pop.push(child);
        }
        self.population = new_pop;
        scored[0].1.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::workspace::{Platform, WorkspaceOptions};
    use crate::Range;
    use nalgebra::Vector3;
    use quaternion as quat;

    #[derive(Clone)]
    struct DummyPlatform {
        layout: Layout,
        pos: Vector3<f64>,
        q: quat::Quaternion<f64>,
    }

    impl DummyPlatform {
        fn new() -> Self {
            Self { layout: Layout { horn_length: 10.0, rod_length: 20.0 }, pos: Vector3::zeros(), q: quat::id() }
        }
    }

    impl Platform for DummyPlatform {
        fn update(&mut self, pos: Vector3<f64>, orient: quat::Quaternion<f64>) {
            self.pos = pos;
            self.q = orient;
        }
        fn compute_angles(&self) -> Option<Vec<f64>> { Some(vec![0.0; 6]) }
        fn horn_length(&self) -> Option<f64> { Some(self.layout.horn_length) }
        fn orientation(&self) -> quat::Quaternion<f64> { self.q }
        fn translation(&self) -> Vector3<f64> { self.pos }
    }

    impl ConfigurablePlatform for DummyPlatform {
        fn apply_layout(&mut self, layout: &Layout) { self.layout = layout.clone(); }
        fn layout(&self) -> Layout { self.layout.clone() }
    }

    #[test]
    fn optimizer_runs() {
        let mut ranges = Ranges::new();
        ranges.insert("x".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
        ranges.insert("y".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
        ranges.insert("z".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
        ranges.insert("rx".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
        ranges.insert("ry".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
        ranges.insert("rz".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
        let options = WorkspaceOptions::default();
        let platform = DummyPlatform::new();
        let mut opt = Optimizer::new(platform, ranges, options, 4, 1, 0.1);
        opt.initialize();
        let _best = opt.step();
    }
}
