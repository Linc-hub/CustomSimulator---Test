const DEFAULTS = {
  ball_joint_max_deg: 45,
  servo_travel_bounds_deg: [-120, 120],
  rod_length_bounds_mm: [160, 420],
  horn_length_bounds_mm: [30, 110],
};

function normalizeRangeInput(range) {
  if (Array.isArray(range)) {
    return range;
  }
  if (range && typeof range === 'object') {
    if ('min' in range && 'max' in range) {
      return [range.min, range.max];
    }
    if ('from' in range && 'to' in range) {
      return [range.from, range.to];
    }
  }
  return null;
}

function validateRange(range, name) {
  const candidate = normalizeRangeInput(range);
  if (!candidate || candidate.length !== 2) {
    throw new Error(`${name} must define min/max values.`);
  }
  const [min, max] = candidate;
  if (!Number.isFinite(min) || !Number.isFinite(max) || max < min) {
    throw new Error(`${name} must contain numeric min/max values with max >= min.`);
  }
  return { min, max };
}

function deriveStep(min, max, fallback) {
  const span = Math.abs(max - min);
  if (span === 0) return fallback;
  const step = span / 10;
  return step > 0 ? step : fallback;
}

function ensureCycleAxis(value) {
  if (typeof value !== 'string') {
    throw new Error('cycle_axis must be a string.');
  }
  const axis = value.toLowerCase();
  if (!['x', 'y', 'z'].includes(axis)) {
    throw new Error('cycle_axis must be one of "x", "y", or "z".');
  }
  return axis;
}

function buildWorkspaceRanges(source, translationStep, rotationStep) {
  const xRange = validateRange(source.x_range_mm, 'x_range_mm');
  const yRange = validateRange(source.y_range_mm, 'y_range_mm');
  const zRange = validateRange(source.z_range_mm, 'z_range_mm');
  const rxRange = validateRange(source.rx_range_deg, 'rx_range_deg');
  const ryRange = validateRange(source.ry_range_deg, 'ry_range_deg');
  const rzRange = validateRange(source.rz_range_deg, 'rz_range_deg');

  return {
    x: { min: xRange.min, max: xRange.max, step: deriveStep(xRange.min, xRange.max, translationStep) },
    y: { min: yRange.min, max: yRange.max, step: deriveStep(yRange.min, yRange.max, translationStep) },
    z: { min: zRange.min, max: zRange.max, step: deriveStep(zRange.min, zRange.max, translationStep) },
    rx: { min: rxRange.min, max: rxRange.max, step: deriveStep(rxRange.min, rxRange.max, rotationStep) },
    ry: { min: ryRange.min, max: ryRange.max, step: deriveStep(ryRange.min, ryRange.max, rotationStep) },
    rz: { min: rzRange.min, max: rzRange.max, step: deriveStep(rzRange.min, rzRange.max, rotationStep) },
  };
}

function parseNestedRequirements(data) {
  const payload = data.payload || {};
  const workspace = data.workspace || {};
  const rotations = data.rotations || {};
  const constraints = data.constraints || {};

  const requiredPayload = ['mass_kg', 'cycle_mm', 'frequency_hz', 'cycle_axis'];
  const requiredWorkspace = ['x_range_mm', 'y_range_mm', 'z_range_mm'];
  const requiredRotations = ['rx_range_deg', 'ry_range_deg', 'rz_range_deg'];

  for (const field of requiredPayload) {
    if (!(field in payload)) {
      throw new Error(`Requirements missing payload field: ${field}`);
    }
  }

  for (const field of requiredWorkspace) {
    if (!(field in workspace)) {
      throw new Error(`Requirements missing workspace field: ${field}`);
    }
  }

  for (const field of requiredRotations) {
    if (!(field in rotations)) {
      throw new Error(`Requirements missing rotations field: ${field}`);
    }
  }

  const normalized = {
    mass_kg: payload.mass_kg,
    cycle_mm: payload.cycle_mm,
    frequency_hz: payload.frequency_hz,
    cycle_axis: ensureCycleAxis(payload.cycle_axis),
    x_range_mm: normalizeRangeInput(workspace.x_range_mm) || workspace.x_range_mm,
    y_range_mm: normalizeRangeInput(workspace.y_range_mm) || workspace.y_range_mm,
    z_range_mm: normalizeRangeInput(workspace.z_range_mm) || workspace.z_range_mm,
    rx_range_deg: normalizeRangeInput(rotations.rx_range_deg) || rotations.rx_range_deg,
    ry_range_deg: normalizeRangeInput(rotations.ry_range_deg) || rotations.ry_range_deg,
    rz_range_deg: normalizeRangeInput(rotations.rz_range_deg) || rotations.rz_range_deg,
  };

  normalized.ball_joint_max_deg = constraints.ball_joint_max_deg ?? DEFAULTS.ball_joint_max_deg;
  const rodBounds = normalizeRangeInput(constraints.rod_length_bounds_mm);
  normalized.rod_length_bounds_mm = rodBounds ? [rodBounds[0], rodBounds[1]] : DEFAULTS.rod_length_bounds_mm.slice();
  const hornBounds = normalizeRangeInput(constraints.horn_length_bounds_mm);
  normalized.horn_length_bounds_mm = hornBounds ? [hornBounds[0], hornBounds[1]] : DEFAULTS.horn_length_bounds_mm.slice();

  if (Array.isArray(constraints.servo_travel_bounds_deg)) {
    normalized.servo_travel_bounds_deg = constraints.servo_travel_bounds_deg.slice();
  } else if (Number.isFinite(constraints.servo_max_deg)) {
    const max = Math.abs(constraints.servo_max_deg);
    normalized.servo_travel_bounds_deg = [-max, max];
  } else {
    normalized.servo_travel_bounds_deg = DEFAULTS.servo_travel_bounds_deg.slice();
  }

  return { normalized };
}

function parseFlatRequirements(data) {
  const requiredFields = [
    'mass_kg',
    'cycle_mm',
    'frequency_hz',
    'cycle_axis',
    'x_range_mm',
    'y_range_mm',
    'z_range_mm',
    'rx_range_deg',
    'ry_range_deg',
    'rz_range_deg',
  ];

  for (const field of requiredFields) {
    if (!(field in data)) {
      throw new Error(`Requirements missing field: ${field}`);
    }
  }

  const normalized = { ...data };
  normalized.cycle_axis = ensureCycleAxis(normalized.cycle_axis);

  const rangeFields = [
    'x_range_mm',
    'y_range_mm',
    'z_range_mm',
    'rx_range_deg',
    'ry_range_deg',
    'rz_range_deg',
  ];
  for (const field of rangeFields) {
    const candidate = normalizeRangeInput(normalized[field]);
    if (!candidate) {
      throw new Error(`${field} must define min/max values.`);
    }
    normalized[field] = [candidate[0], candidate[1]];
  }

  if ('rod_length_bounds_mm' in normalized) {
    const candidate = normalizeRangeInput(normalized.rod_length_bounds_mm);
    if (candidate) {
      normalized.rod_length_bounds_mm = [candidate[0], candidate[1]];
    }
  }

  if ('horn_length_bounds_mm' in normalized) {
    const candidate = normalizeRangeInput(normalized.horn_length_bounds_mm);
    if (candidate) {
      normalized.horn_length_bounds_mm = [candidate[0], candidate[1]];
    }
  }

  if (!('servo_travel_bounds_deg' in normalized) && Number.isFinite(normalized.servo_max_deg)) {
    const max = Math.abs(normalized.servo_max_deg);
    normalized.servo_travel_bounds_deg = [-max, max];
  }

  for (const [key, value] of Object.entries(DEFAULTS)) {
    if (!(key in normalized) || normalized[key] === undefined) {
      normalized[key] = Array.isArray(value) ? value.slice() : value;
    } else if (Array.isArray(normalized[key])) {
      normalized[key] = normalized[key].slice();
    }
  }

  return { normalized };
}

export function parseRequirements(text) {
  let data;
  try {
    data = JSON.parse(text);
  } catch (error) {
    throw new Error('Requirements JSON is invalid.');
  }

  if (!data || typeof data !== 'object') {
    throw new Error('Requirements must be a JSON object.');
  }

  const usesNested = 'payload' in data || 'workspace' in data || 'rotations' in data || 'constraints' in data;
  const { normalized } = usesNested ? parseNestedRequirements(data) : parseFlatRequirements(data);

  const workspaceSource = {
    x_range_mm: normalized.x_range_mm,
    y_range_mm: normalized.y_range_mm,
    z_range_mm: normalized.z_range_mm,
    rx_range_deg: normalized.rx_range_deg,
    ry_range_deg: normalized.ry_range_deg,
    rz_range_deg: normalized.rz_range_deg,
  };

  const workspace = buildWorkspaceRanges(workspaceSource, 5, 5);

  normalized.x_range_mm = [workspace.x.min, workspace.x.max];
  normalized.y_range_mm = [workspace.y.min, workspace.y.max];
  normalized.z_range_mm = [workspace.z.min, workspace.z.max];
  normalized.rx_range_deg = [workspace.rx.min, workspace.rx.max];
  normalized.ry_range_deg = [workspace.ry.min, workspace.ry.max];
  normalized.rz_range_deg = [workspace.rz.min, workspace.rz.max];

  return { normalized, workspace };
}

export async function loadDefaultRequirements() {
  const response = await fetch('Additional_Repo_Stuff/examples/Sample_Requirements.json');
  if (!response.ok) {
    throw new Error(`Failed to load sample requirements: HTTP ${response.status}`);
  }
  const json = await response.json();
  return JSON.stringify(json, null, 2);
}
