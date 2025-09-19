# Stewart Platform Optimizer — Reproduction Manual

This document describes, in exhaustive detail, how to rebuild the Stewart platform layout optimizer contained in this repository. It covers the file layout, execution environment, the geometry and dynamics math, and the full optimization workflow so another AI coding assistant can recreate the project from scratch.

## 1. Repository Topology and Runtime Environment

### 1.1 High-level structure

- `index.html` renders the single-page interface that hosts the optimizer controls and wires user actions to the optimization backend exposed by `optimizer.js` and `workspace.js`. The UI loads reference layouts, accepts JSON definitions, exposes search ranges, and lets the user export the best layout.【F:index.html†L1-L362】
- `optimizer.js` implements the optimization controller that evaluates candidate layouts, computes performance metrics (workspace coverage, dexterity, stiffness, torque), and manages the optimization lifecycle (population initialization, evaluation, Pareto storage, export). Heavy loops are currently stubbed so the numerical kernels can be restored incrementally.【F:optimizer.js†L1-L198】
- `workspace.js` houses the workspace sweep and constraint checking logic. The nested loops over the translational and rotational workspace are commented out at present; the surrounding structure illustrates how payload, stroke, frequency, servo torque limits, and ball-joint constraints are meant to feed into the evaluation routine.【F:workspace.js†L1-L54】
- `Raw Information/` contains original specifications, schemas, and archived layouts from the previous project. These files document the geometry schema, inverse-kinematics math, and UI/feature requirements that the optimizer needs to satisfy.【F:Raw Information/Stewart_Simulator_Consolidated.md†L1-L186】【F:Raw Information/Stewart_Layout_Schema.json†L1-L70】

### 1.2 Dependencies and hosting

1. Serve `index.html` from any static HTTP server (e.g., `python -m http.server 8000`) or open it directly in a browser that supports ES modules.
2. Ensure the browser loads the legacy math/visualization libraries bundled in `Raw Information/Stuff from Old Project/`:
   - `p5.min.js` for drawing utility hooks.
   - `quaternion.min.js` for quaternion math used by the Stewart model.
   - `stewart.min.js` for the legacy platform object that exposes `initCustom` and the forward/inverse kinematics primitives consumed by `optimizer.js` and `workspace.js`.
3. The optimizer module is imported dynamically when the user presses **Run Optimization**; no bundler is required because all modules use native ES module syntax.【F:index.html†L229-L361】

## 2. Geometry and Layout Schema

### 2.1 Coordinate definitions

The consolidated specification defines the canonical Stewart layout schema. A layout is a JSON object with:

- `base_anchors` \(B_k\): six base attachment points expressed as \([x, y, z]\) coordinates in millimetres relative to the global base frame.
- `platform_anchors` \(P_k\): six platform attachment points, also in millimetres, expressed in the platform frame before motion transforms.
- `beta_angles` \(\beta_k\): servo axis orientation angles (radians) that tilt each horn away from its local frame z-axis.
- `horn_length` \(h\) and `rod_length` \(d\): lengths of the servo horn and connecting rod, respectively.
- Optional `servo_range` \([\alpha_{\min}, \alpha_{\max}]\) in degrees and payload properties (`mass_kg`, `stroke_mm`, `frequency_hz`).【F:Raw Information/Stewart_Layout_Schema.json†L1-L70】【F:Raw Information/Stewart_Simulator_Consolidated.md†L12-L144】

### 2.2 Pose parameterization

A target pose of the moving platform is defined by translation \(\mathbf{t} = (x, y, z)\) and intrinsic ZYX Euler rotations \((\psi, \theta, \phi)\) about the global axes (mapped from the `rx`, `ry`, `rz` sweep ranges). The quaternion library converts the Euler triplet into a rotation quaternion \(q\), which rotates each platform anchor before it is shifted by \(\mathbf{t}\).【F:workspace.js†L4-L17】

## 3. Stewart Platform Kinematics and Dynamics

### 3.1 Inverse kinematics for servo angles

For each leg index \(k \in \{1,\ldots,6\}\):

1. Rotate the platform anchor by \(q\) and translate it into world space:
   \[
   \mathbf{p}'_k = \mathbf{t} + R(q) \mathbf{P}_k
   \]
   where \(R(q)\) is the rotation matrix generated from the quaternion.
2. Form the vector from the base anchor to the moved platform anchor:
   \[
   \mathbf{l}_k = \mathbf{p}'_k - \mathbf{B}_k = (l_{kx}, l_{ky}, l_{kz})
   \]
3. Compute the scalar quantities required for the horn geometry:
   \[
   e_k = 2 h\, l_{kz},\qquad f_k = 2 h (\cos\beta_k\, l_{kx} + \sin\beta_k\, l_{ky}),\qquad g_k = \lVert \mathbf{l}_k \rVert^2 - (d^2 - h^2)
   \]
4. The servo angle \(\alpha_k\) that satisfies the planar four-bar geometry is:
   \[
   \alpha_k = \arcsin\!\left(\frac{g_k}{\sqrt{e_k^2 + f_k^2}}\right) - \operatorname{atan2}(f_k, e_k)
   \]

These expressions are directly lifted from the consolidated specification and remain valid for arbitrary anchor placement and servo orientations.【F:Raw Information/Stewart_Simulator_Consolidated.md†L12-L29】

### 3.2 Jacobian and differential kinematics

To quantify dexterity and stiffness, reconstruct the leg Jacobian once the placeholder is replaced. For each leg:

- Define the unit leg direction \(\hat{u}_k = \mathbf{l}_k / \lVert \mathbf{l}_k \rVert\).
- The Plücker line coordinates of the leg are \((\hat{u}_k, \mathbf{p}'_k \times \hat{u}_k)\).
- Assemble the \(6\times6\) Jacobian \(J\) mapping platform twist \(\dot{\mathbf{x}} = (\dot{\mathbf{t}}, \boldsymbol{\omega})\) to leg-length rates \(\dot{\mathbf{l}}\) using:
  \[
  J_k = \begin{bmatrix} \hat{u}_k^\top & (\mathbf{p}'_k \times \hat{u}_k)^\top \end{bmatrix}
  \]
  where row \(k\) corresponds to leg \(k\).
- The singular values \(\sigma_1 \ge \cdots \ge \sigma_6\) of \(J\) yield the conditioning metrics used later (\(\sigma_{\min} = \sigma_6\), \(\sigma_{\max} = \sigma_1\)).

### 3.3 Dynamic load and torque estimation

When the platform carries a payload of mass \(m\), oscillating with stroke \(s\) at frequency \(f\), the peak vertical acceleration is approximated by \(a = (2\pi f)^2 (s/1000)\). Each leg shares one sixth of the total load, so the force per servo is \(F = m (g + a)/6\). The required servo torque at the horn is then:
\[
\tau = F \cdot h
\]
which is the computation implemented in `computeTorque` (gravity constant \(g = 9.81\,\text{m/s}^2\)).【F:optimizer.js†L101-L107】

### 3.4 Constraint model

- **Servo limits:** enforce \(\alpha_{\min} \le \alpha_k \le \alpha_{\max}\) when reintroducing the workspace loops.
- **Ball-joint limit:** clamp the relative angle between rod vector \(\mathbf{l}_k\) and its nominal direction to `ballJointLimitDeg` when `ballJointClamp` is enabled; otherwise treat limit violations as failures.【F:optimizer.js†L10-L130】【F:index.html†L205-L338】
- **Collision/geometry checks:** verify rod length, mechanical interference, and base/platform collisions within the workspace sweep.

## 4. Workspace Evaluation Algorithm

Restore the commented scan in `workspace.js` to perform the exhaustive workspace coverage computation:

1. Build discrete ranges for each DOF using the `ranges` parameter provided by the UI (min, max, step for `x`, `y`, `z`, `rx`, `ry`, `rz`).【F:index.html†L184-L324】
2. Iterate over the Cartesian product of all six ranges. For each pose:
   - Compute platform quaternion from the Euler angles using `Quaternion` or the override provided in options.【F:workspace.js†L4-L17】
   - Evaluate the IK equations above to obtain \(\alpha_k\) and rod vectors.
   - Accumulate constraints: servo angle bounds, rod length deviation, ball-joint angle, payload torque, and optional servo torque limit.
   - Record the pose as reachable when all constraints pass; otherwise capture the failing condition in the `violations` array.
3. Workspace coverage is the ratio of reachable poses to total poses scanned, reported as a percentage for downstream optimization.【F:workspace.js†L20-L53】
4. Persist the raw pose classifications in `reachable`/`unreachable` arrays for visualization or debugging.

## 5. Optimization Workflow

### 5.1 Population management

- Initialize the optimizer with the Stewart instance and the user-specified settings (`populationSize`, `generations`, range data, payload, stroke, frequency, mutation rate, ball-joint policy). These parameters are read directly from the HTML inputs.【F:index.html†L310-L332】【F:optimizer.js†L9-L36】
- Seed the population by cloning the supplied layout and (once re-enabled) generating `populationSize - 1` perturbed variants using `randomizeLayout`, which jitters horn length, rod length, and anchor coordinates with bounded noise.【F:optimizer.js†L43-L66】

### 5.2 Evaluation metrics

For each layout candidate:

1. Run `computeWorkspace` to obtain the workspace coverage and violation ledger.
2. Compute the mechanical objective vector:
   - `coverage`: fraction of reachable poses.
   - `dexterity`: reciprocal of the Jacobian condition number (\(1/\kappa(J) = \sigma_{\min}/\sigma_{\max}\)) once the Jacobian is implemented.【F:optimizer.js†L91-L98】
   - `stiffness`: directly use the smallest singular value \(\sigma_{\min}\) of \(J\).【F:optimizer.js†L96-L99】
   - `torque`: negated in the objective vector so smaller torque is treated as better.【F:optimizer.js†L101-L131】
3. Store the tuple \((\text{layout}, \text{coverage}, \text{dexterity}, \text{stiffness}, \text{torque})\) for Pareto analysis.【F:optimizer.js†L119-L131】

### 5.3 Genetic operators (to restore)

Implement a multi-objective evolutionary algorithm inspired by NSGA-II:

- **Selection:** sort individuals by non-dominated fronts (Pareto rank) and crowding distance, keeping the top `populationSize` layouts.
- **Crossover:** mix base and platform anchors, horn lengths, and rod lengths between randomly chosen parents. Enforce physical symmetry if required.
- **Mutation:** reuse `randomizeLayout` to inject Gaussian noise scaled by `mutationRate`.
- **Elitism:** always carry the current Pareto set forward to preserve best solutions.

The scaffolding for these loops is present in `initialize` and `step`; reinstate the commented loops to evaluate the entire population and iterate for the configured number of generations.【F:optimizer.js†L58-L150】

### 5.4 Execution lifecycle

- `start(callback)` guards against concurrent runs, initializes the population, executes the optimization asynchronously, and invokes the supplied callback once finished so the UI can update status text and display the best layout.【F:optimizer.js†L152-L167】【F:index.html†L333-L339】
- `exportBest()` serializes the best-known layout to JSON and triggers a browser download through a dynamically created anchor element.【F:optimizer.js†L173-L197】【F:index.html†L346-L356】

## 6. Step-by-step Rebuild Plan for Another Assistant

1. **Recreate UI**: Implement the HTML/CSS structure shown in `index.html`, including layout import buttons, optimization parameter inputs, safety toggles, and execution controls. Ensure the script block dynamically imports `optimizer.js` and responds to user actions exactly as in the reference file.【F:index.html†L160-L361】
2. **Implement Stewart class hooks**: Port the legacy `Stewart` class (from `stewart.min.js`) or rewrite equivalent logic providing `initCustom` and the IK methods to compute servo angles given anchors, horn/rod lengths, and platform pose.
3. **Restore workspace sweeps**: In `workspace.js`, reintroduce the nested loops, pose evaluation, constraint checks, and coverage calculation described in Section 4 so `computeWorkspace` returns meaningful metrics.【F:workspace.js†L27-L53】
4. **Complete Jacobian math**: Fill in `computeJacobian` and `jacobianMetrics` in `optimizer.js` using the formulas in Section 3.2 to provide non-zero dexterity and stiffness metrics.【F:optimizer.js†L73-L99】
5. **Finalize evolutionary loop**: Reinstate the population evaluation loop, Pareto sorting, crossover, and mutation inside `initialize`/`step` to execute the configured number of generations.【F:optimizer.js†L58-L150】
6. **Hook results into UI**: Update the callback triggered by `start` so the textarea displays the best layout JSON and the status area reports success or failures, mirroring the behaviour presently stubbed in the HTML logic.【F:index.html†L333-L339】
7. **Validate**: Run the optimizer with the provided reference layouts (`Raw Information/Circular_Layout.json`, etc.), confirm coverage, torque, and dexterity outputs are sensible, and export the resulting JSON for regression comparisons.

## 7. Mathematical Appendix

### 7.1 Quaternion to rotation matrix

Given a unit quaternion \(q = (w, x, y, z)\), construct the rotation matrix used in the IK and Jacobian computations:
\[
R(q) = \begin{bmatrix}
1-2(y^2+z^2) & 2(xy-wz) & 2(xz+wy) \\
2(xy+wz) & 1-2(x^2+z^2) & 2(yz-wx) \\
2(xz-wy) & 2(yz+wx) & 1-2(x^2+y^2)
\end{bmatrix}
\]

### 7.2 Ball-joint angular deviation

For each leg, the ball-joint angle relative to its nominal axis can be computed by:
\[
\theta_k = \arccos\!\left( \frac{\mathbf{l}_k \cdot \hat{z}_k}{\lVert \mathbf{l}_k \rVert} \right)
\]
where \(\hat{z}_k\) is the joint axis direction in the servo frame. Clamp \(\theta_k\) to the limit configured by the user; flag the pose when \(\theta_k\) exceeds the limit and `ballJointClamp` is `false`.

### 7.3 Workspace coverage metric

Let \(N_{\text{total}}\) be the total number of poses scanned and \(N_{\text{reachable}}\) the count of poses that satisfy all constraints. Then the coverage percentage is:
\[
\text{coverage} = 100 \times \frac{N_{\text{reachable}}}{N_{\text{total}}}
\]
This scalar feeds the first element of the optimizer objective vector.【F:optimizer.js†L119-L131】

---

With the above specifications, another Codex instance can reproduce the optimizer: rebuild the UI, restore the math kernels, and wire the optimization loop back together. The combination of code references and explicit formulas removes ambiguity about the expected mechanics, constraints, and objectives.
