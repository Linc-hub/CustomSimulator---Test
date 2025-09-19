# Stewart Platform Optimizer — Quick Start Guide

This guide summarizes how to get started with the Stewart Platform Optimizer using the provided specification, schema, and examples.

---

## 1. Input Requirements

All optimizer runs start with a **requirements JSON** file that follows the schema (v2).

### Example Input
```json
{
  "payload": {
    "mass_kg": 15,
    "cycle_mm": 254,
    "frequency_hz": 4,
    "cycle_axis": "x"
  },
  "workspace": {
    "x_range_mm": [-127, 127],
    "y_range_mm": [-44.5, 44.5],
    "z_range_mm": [-44.5, 44.5]
  },
  "rotations": {
    "rx_range_deg": [-10, 10],
    "ry_range_deg": [-10, 10],
    "rz_range_deg": [-10, 10]
  },
  "constraints": {
    "ball_joint_max_deg": 45,
    "servo_max_deg": 90
  }
}
```

- **Payload**: cycle amplitude, axis, mass, frequency.  
- **Workspace**: translation ranges.  
- **Rotations**: angular ranges.  
- **Constraints**: optional (ball joints, servo angles, rod/horn bounds).

If rod length, horn length, or servo travel bounds are omitted, the optimizer will explore wide defaults.

---

## 2. What the Optimizer Does

- Builds candidate **geometries** (servo anchors, horn lengths, platform anchors, rod lengths).  
- Runs **inverse kinematics** to check feasibility.  
- Evaluates designs against:  
  1. Workspace coverage  
  2. Torque/speed demand for the cycle  
  3. Dexterity (Jacobian condition)  
  4. Stiffness/compliance  
  5. Load sharing  
  6. Joint and servo limits  

- Uses **NSGA-II (multi-objective GA)** to produce a **Pareto front** of designs.

---

## 3. Outputs

- JSON/CSV files with optimized geometry.  
- Metrics for torque, workspace, stiffness, dexterity.  
- Pareto plots for trade-offs.  
- Example layouts (AYVA, Circular, Hex) included for testing.

---

## 4. Typical Workflow

1. Write a requirements file (`requirements.json`).  
2. Run the optimizer with population size and generations (e.g., 200 × 100).  
3. Wait for optimization to complete.  
4. Inspect Pareto front solutions.  
5. Export chosen layouts to JSON or CSV for simulation or CAD.

---

## 5. References

See `Stewart_Optimizer_Final_Spec_v3.md` for full math, references, and pseudocode.

---
