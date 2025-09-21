const REQUIRED_FIELDS = [
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

const DEFAULTS = {
  ball_joint_max_deg: 45,
  servo_travel_bounds_deg: [-120, 120],
  rod_length_bounds_mm: [160, 420],
  horn_length_bounds_mm: [30, 110],
};

function validateRange(range, name) {
  if (!Array.isArray(range) || range.length !== 2) {
    throw new Error(`${name} must be an array with [min, max] values.`);
  }
  const [min, max] = range;
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

export function parseRequirements(text) {
  let data;
  try {
    data = JSON.parse(text);
  } catch (error) {
    throw new Error('Requirements JSON is invalid.');
  }

  for (const field of REQUIRED_FIELDS) {
    if (!(field in data)) {
      throw new Error(`Requirements missing field: ${field}`);
    }
  }

  const normalized = { ...data };
  for (const [key, value] of Object.entries(DEFAULTS)) {
    if (!(key in normalized)) {
      normalized[key] = Array.isArray(value) ? value.slice() : value;
    }
  }

  const xRange = validateRange(normalized.x_range_mm, 'x_range_mm');
  const yRange = validateRange(normalized.y_range_mm, 'y_range_mm');
  const zRange = validateRange(normalized.z_range_mm, 'z_range_mm');
  const rxRange = validateRange(normalized.rx_range_deg, 'rx_range_deg');
  const ryRange = validateRange(normalized.ry_range_deg, 'ry_range_deg');
  const rzRange = validateRange(normalized.rz_range_deg, 'rz_range_deg');

  const translationStep = 5;
  const rotationStep = 5;

  const workspace = {
    x: { min: xRange.min, max: xRange.max, step: deriveStep(xRange.min, xRange.max, translationStep) },
    y: { min: yRange.min, max: yRange.max, step: deriveStep(yRange.min, yRange.max, translationStep) },
    z: { min: zRange.min, max: zRange.max, step: deriveStep(zRange.min, zRange.max, translationStep) },
    rx: { min: rxRange.min, max: rxRange.max, step: deriveStep(rxRange.min, rxRange.max, rotationStep) },
    ry: { min: ryRange.min, max: ryRange.max, step: deriveStep(ryRange.min, ryRange.max, rotationStep) },
    rz: { min: rzRange.min, max: rzRange.max, step: deriveStep(rzRange.min, rzRange.max, rotationStep) },
  };

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
