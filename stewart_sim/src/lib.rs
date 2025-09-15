pub mod workspace;
pub mod optimizer;

// re-export commonly used items
pub use workspace::{compute_workspace, WorkspaceOptions, WorkspaceResult, Range, Ranges, Platform};
pub use optimizer::Optimizer;

#[cfg(feature = "wasm")]
use wasm_bindgen::prelude::*;

#[cfg(feature = "wasm")]
use nalgebra::Vector3;
#[cfg(feature = "wasm")]
use quaternion as quat;
#[cfg(feature = "wasm")]
use optimizer::{ConfigurablePlatform, Layout};

#[cfg(feature = "wasm")]
#[derive(Clone)]
struct WasmPlatform {
    layout: Layout,
    pos: Vector3<f64>,
    q: quat::Quaternion<f64>,
}

#[cfg(feature = "wasm")]
impl WasmPlatform {
    fn new() -> Self {
        Self {
            layout: Layout { horn_length: 10.0, rod_length: 20.0 },
            pos: Vector3::zeros(),
            q: quat::id(),
        }
    }
}

#[cfg(feature = "wasm")]
impl Platform for WasmPlatform {
    fn update(&mut self, pos: Vector3<f64>, orient: quat::Quaternion<f64>) {
        self.pos = pos;
        self.q = orient;
    }

    fn compute_angles(&self) -> Option<Vec<f64>> {
        Some(vec![0.0; 6])
    }

    fn horn_length(&self) -> Option<f64> { Some(self.layout.horn_length) }

    fn orientation(&self) -> quat::Quaternion<f64> { self.q }

    fn translation(&self) -> Vector3<f64> { self.pos }
}

#[cfg(feature = "wasm")]
impl ConfigurablePlatform for WasmPlatform {
    fn apply_layout(&mut self, layout: &Layout) { self.layout = layout.clone(); }
    fn layout(&self) -> Layout { self.layout.clone() }
}

#[cfg(feature = "wasm")]
#[wasm_bindgen]
pub fn optimize_demo() -> JsValue {
    let mut ranges = Ranges::new();
    ranges.insert("x".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("y".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("z".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("rx".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("ry".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
    ranges.insert("rz".into(), Range { min: 0.0, max: 0.0, step: 1.0 });

    let options = WorkspaceOptions::default();
    let platform = WasmPlatform::new();
    let mut opt = Optimizer::new(platform, ranges, options, 4, 1, 0.1);
    opt.initialize();
    let best = opt.step();
    JsValue::from_serde(&best).unwrap()
}
