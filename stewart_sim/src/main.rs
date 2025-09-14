use three_d::*;
use stewart_sim::{
    compute_workspace, WorkspaceOptions, Range, Ranges, Platform,
    optimizer::{Layout, ConfigurablePlatform, Optimizer},
};
use nalgebra::Vector3;
use quaternion as quat;

// -----------------------------------------------------------------------------
// Dummy platform implementation
// -----------------------------------------------------------------------------

#[derive(Clone)]
struct DummyPlatform {
    layout: Layout,
    pos: Vector3<f64>,
    q: quat::Quaternion<f64>,
}

impl DummyPlatform {
    fn new() -> Self {
        Self {
            layout: Layout { horn_length: 10.0, rod_length: 20.0 },
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
    fn compute_angles(&self) -> Option<Vec<f64>> { Some(vec![0.0; 6]) }
    fn horn_length(&self) -> Option<f64> { Some(self.layout.horn_length) }
    fn orientation(&self) -> quat::Quaternion<f64> { self.q }
    fn translation(&self) -> Vector3<f64> { self.pos }
}

impl ConfigurablePlatform for DummyPlatform {
    fn apply_layout(&mut self, layout: &Layout) { self.layout = layout.clone(); }
    fn layout(&self) -> Layout { self.layout.clone() }
}

// -----------------------------------------------------------------------------
// Application entry
// -----------------------------------------------------------------------------

fn main() {
    // --- Window / camera setup ------------------------------------------------
    let window = Window::new(WindowSettings {
        title: "Stewart Simulator".into(),
        ..Default::default()
    }).unwrap();
    let context = window.gl();
    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(2.0, 2.0, 2.0),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 0.0, 1.0),
        degrees(45.0),
        0.1,
        100.0,
    );
    let mut control = OrbitControl::new(vec3(0.0, 0.0, 0.0), 0.1, 10.0);
    let mut gui = GUI::new(&context);

    // --- Platform geometry ----------------------------------------------------
    let mut base_cpu = CpuMesh::cube();
    base_cpu.transform(Mat4::from_nonuniform_scale(1.0, 1.0, 0.05)).ok();
    let base = Gm::new(
        Mesh::new(&context, &base_cpu),
        ColorMaterial::new_opaque(
            &context,
            &CpuMaterial {
                albedo: Srgba::new(100, 100, 100, 255),
                ..Default::default()
            },
        ),
    );

    let mut top_cpu = CpuMesh::cube();
    top_cpu
        .transform(
            Mat4::from_nonuniform_scale(0.5, 0.5, 0.05)
                * Mat4::from_translation(vec3(0.0, 0.0, 0.6)),
        )
        .ok();
    let top = Gm::new(
        Mesh::new(&context, &top_cpu),
        ColorMaterial::new_opaque(
            &context,
            &CpuMaterial {
                albedo: Srgba::new(200, 0, 0, 255),
                ..Default::default()
            },
        ),
    );

    // --- State ----------------------------------------------------------------
    let mut platform = DummyPlatform::new();
    let mut workspace_points: Option<Gm<InstancedMesh, ColorMaterial>> = None;
    let mut optimizer: Option<Optimizer<DummyPlatform>> = None;

    // Pre-build small cube used for workspace points
    let mut point_cpu = CpuMesh::cube();
    point_cpu.transform(Mat4::from_scale(0.02)).ok();

    // --- Render loop ----------------------------------------------------------
    window.render_loop(move |mut frame_input| {
        // Handle orbit camera input
        control.handle_events(&mut camera, &mut frame_input.events);

        // UI ------------------------------------------------------------------
        gui.update(
            &mut frame_input.events,
            frame_input.accumulated_time,
            frame_input.viewport,
            frame_input.device_pixel_ratio,
            |egui_ctx| {
                egui::Window::new("Controls").show(egui_ctx, |ui| {
                    ui.add(
                        egui::Slider::new(&mut platform.layout.horn_length, 5.0..=30.0)
                            .text("Horn length"),
                    );
                    ui.add(
                        egui::Slider::new(&mut platform.layout.rod_length, 10.0..=40.0)
                            .text("Rod length"),
                    );
                    if ui.button("Compute Workspace").clicked() {
                        let mut ranges = Ranges::new();
                        ranges.insert("x".into(), Range { min: -50.0, max: 50.0, step: 50.0 });
                        ranges.insert("y".into(), Range { min: -50.0, max: 50.0, step: 50.0 });
                        ranges.insert("z".into(), Range { min: -50.0, max: 50.0, step: 50.0 });
                        ranges.insert("rx".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        ranges.insert("ry".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        ranges.insert("rz".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        let res = compute_workspace(&mut platform, &ranges, WorkspaceOptions::default());
                        let transformations: Vec<Mat4> = res
                            .reachable
                            .iter()
                            .map(|p| {
                                Mat4::from_translation(vec3(
                                    p.x as f32 / 100.0,
                                    p.y as f32 / 100.0,
                                    p.z as f32 / 100.0,
                                ))
                            })
                            .collect();
                        let instances = Instances {
                            transformations,
                            texture_transformations: None,
                            colors: Some(vec![Srgba::new(0, 200, 0, 255); res.reachable.len()]),
                        };
                        workspace_points = Some(Gm::new(
                            InstancedMesh::new(&context, &instances, &point_cpu),
                            ColorMaterial::default(),
                        ));
                    }

                    if ui.button("Run Optimizer").clicked() {
                        let mut ranges = Ranges::new();
                        ranges.insert("x".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        ranges.insert("y".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        ranges.insert("z".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        ranges.insert("rx".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        ranges.insert("ry".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        ranges.insert("rz".into(), Range { min: 0.0, max: 0.0, step: 1.0 });
                        let options = WorkspaceOptions::default();
                        let mut opt = Optimizer::new(platform.clone(), ranges, options, 4, 1, 0.1);
                        opt.initialize();
                        let best = opt.step();
                        platform.apply_layout(&best);
                        optimizer = Some(opt);
                    }
                });
            },
        );

        // Compose objects list
        let mut objects: Vec<&dyn Object> = vec![&base, &top];
        if let Some(ref pts) = workspace_points {
            objects.push(pts);
        }

        // Render scene
        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.9, 0.9, 0.9, 1.0, 1.0))
            .render(&camera, objects, &[]);
        gui.render().ok();

        FrameOutput::default()
    });
}

