// Module to sweep workspace of a Stewart platform
export async function computeWorkspace(platform, ranges, options = {}) {
  const {
    payload = 0, // kg
    stroke = 0, // mm
    frequency = 0, // Hz
    servoTorqueLimit = Infinity,
    ballJointLimitDeg = 90,
    ballJointClamp = true,
    onProgress = null,
    useWasm = true
  } = options;

  const QuaternionObj = (typeof Quaternion !== 'undefined') ? Quaternion : options.Quaternion;
  if (!platform || !QuaternionObj) {
    throw new Error('platform and Quaternion are required');
  }

  if (useWasm) {
    const wasmResult = await tryComputeWorkspaceWasm(
      platform,
      ranges,
      { payload, stroke, frequency, servoTorqueLimit, ballJointLimitDeg, ballJointClamp },
      onProgress
    );
    if (wasmResult) {
      return wasmResult;
    }
  }

  const dist3 = (a, b) => {
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    const dz = a[2] - b[2];
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  };

  const mapLoadToForce = () => {
    // Simple dynamic model: payload shared by 6 legs plus inertial component
    const mass = payload; // assume already kg
    const accel = Math.pow(2 * Math.PI * frequency, 2) * (stroke / 1000); // m/s^2
    return mass * (9.81 + accel) / 6; // force per leg (N)
  };

  const legForce = mapLoadToForce();
  const torquePerForce = platform.hornLength || 1; // Nm per Newton

  const reachable = [];
  const failures = [];
  const violationCounts = {};

  const list = (axis) => {
    const { min = 0, max = 0, step } = ranges[axis] || {};
    const defaultStep = axis.startsWith('r') ? 5 : 5; // 5° or 5 mm
    const s = step > 0 ? step : defaultStep;
    const arr = [];
    for (let v = min; v <= max + 1e-9; v += s) arr.push(v);
    return arr;
  };

  const xs = list('x'), ys = list('y'), zs = list('z');
  const rxs = list('rx'), rys = list('ry'), rzs = list('rz');

  const total = xs.length * ys.length * zs.length * rxs.length * rys.length * rzs.length;
  let processed = 0;

  for (const x of xs) {
    for (const y of ys) {
      for (const z of zs) {
        for (const rx of rxs) {
          for (const ry of rys) {
            for (const rz of rzs) {
              processed++;
              const pos = [x, y, z];
              const q = QuaternionObj.fromEuler(
                rx * Math.PI / 180,
                ry * Math.PI / 180,
                rz * Math.PI / 180,
                'XYZ'
              );
              let ok = true;
              let reason = '';
              try {
                const prevPos = platform.translation ? platform.translation.slice() : [0, 0, 0];
                const prevQ = platform.orientation || QuaternionObj.ONE;
                platform.update(pos, q);
                const angles = platform.computeAngles && platform.computeAngles();
                if (!angles || angles.some(a => a === null)) {
                  ok = false; reason = 'IK';
                }
                if (ok && platform.servoRange) {
                  for (const a of angles) {
                    if (a < platform.servoRange[0] || a > platform.servoRange[1]) {
                      ok = false; reason = 'servo range'; break;
                    }
                  }
                }
                if (ok && platform.B && platform.H && platform.hornLength) {
                  const tol = Math.max(1e-3 * platform.hornLength, 0.5);
                  for (let i = 0; i < platform.H.length; i++) {
                    if (dist3(platform.H[i], platform.B[i]) > platform.hornLength + tol) {
                      ok = false; reason = 'horn stretch'; break;
                    }
                  }
                }
                if (ok && ballJointClamp && platform.B && platform.H && platform.P && platform.cosBeta && platform.sinBeta) {
                  const toDeg = (rad) => rad * 180 / Math.PI;
                  const platNormal = platform.orientation.rotateVector([0, 0, 1]);
                  for (let i = 0; i < platform.P.length; i++) {
                    const rodVec = [
                      platform.P[i][0] - platform.H[i][0],
                      platform.P[i][1] - platform.H[i][1],
                      platform.P[i][2] - platform.H[i][2]
                    ];
                    const magR = Math.sqrt(
                      rodVec[0] * rodVec[0] +
                      rodVec[1] * rodVec[1] +
                      rodVec[2] * rodVec[2]
                    );
                    const baseAxis = [platform.cosBeta[i], platform.sinBeta[i], 0];
                    const cosBase = (rodVec[0] * baseAxis[0] + rodVec[1] * baseAxis[1] + rodVec[2] * baseAxis[2]) / magR;
                    const cosPlat = (rodVec[0] * platNormal[0] + rodVec[1] * platNormal[1] + rodVec[2] * platNormal[2]) / magR;
                    const angBase = toDeg(Math.acos(Math.min(Math.max(Math.abs(cosBase), -1), 1)));
                    const angPlat = toDeg(Math.acos(Math.min(Math.max(Math.abs(cosPlat), -1), 1)));
                    if (angBase > ballJointLimitDeg || angPlat > ballJointLimitDeg) { ok = false; reason = 'ball joint'; break; }
                  }
                }
                if (ok) {
                  const torque = legForce * torquePerForce;
                  if (torque > servoTorqueLimit) {
                    ok = false; reason = 'torque';
                  }
                }
                platform.update(prevPos, prevQ);
              } catch (e) {
                ok = false; reason = 'error';
              }

              const pose = { x, y, z, rx, ry, rz };
              if (ok) {
                reachable.push(pose);
              } else {
                failures.push({ pose, reason });
                violationCounts[reason] = (violationCounts[reason] || 0) + 1;
              }

              if (onProgress && processed % 50 === 0) {
                try { onProgress(processed / total); } catch (_) { }
              }
              if (processed % 200 === 0) {
                await new Promise(r => setTimeout(r, 0));
              }
            }
          }
        }
      }
    }
  }

  if (onProgress) { try { onProgress(1); } catch (_) { } }

  const coverage = total ? reachable.length / total : 0;
  const violations = Object.entries(violationCounts).map(([reason, count]) => ({ reason, count }));
  return { coverage, violations, reachable, unreachable: failures };
}

export function exportResults(result, format = 'json') {
  if (!result) return;
  let data = '';
  let mime = '';
  if (format === 'json') {
    data = JSON.stringify(result, null, 2);
    mime = 'application/json';
  } else if (format === 'csv') {
    const rows = [['x','y','z','rx','ry','rz','ok','reason']];
    const add = (p, ok, reason='') => rows.push([p.x,p.y,p.z,p.rx,p.ry,p.rz,ok,reason]);
    result.reachable.forEach(p => add(p, true));
    result.unreachable.forEach(f => add(f.pose, false, f.reason));
    data = rows.map(r => r.join(',')).join('\n');
    mime = 'text/csv';
  }
  const blob = new Blob([data], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'workspace.' + (format === 'json' ? 'json' : 'csv');
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

let workspaceWasmModulePromise = null;
let workspaceWasmModuleLoadError = null;

async function tryComputeWorkspaceWasm(platform, ranges, wasmOptions, onProgress) {
  const wasmModule = await loadWorkspaceWasmModule();
  if (!wasmModule || typeof wasmModule.compute_workspace !== 'function') {
    return null;
  }

  const wasmPlatform = preparePlatformForWasm(platform);
  if (!wasmPlatform) {
    const detail = formatWasmPlatformIssues(preparePlatformForWasm.lastErrors);
    console.warn(`Workspace wasm path skipped: invalid platform data${detail}`);
    return null;
  }

  const preparedRanges = prepareRangesForWasm(ranges);
  const preparedOptions = prepareWorkspaceOptionsForWasm(wasmOptions);

  try {
    const result = wasmModule.compute_workspace(wasmPlatform, preparedRanges, preparedOptions);
    const resolved = result && typeof result.then === 'function' ? await result : result;
    if (resolved && onProgress) {
      try { onProgress(1); } catch (_) { }
    }
    return resolved || null;
  } catch (err) {
    console.warn('WASM workspace computation failed, falling back to JS', err);
    return null;
  }
}

async function loadWorkspaceWasmModule() {
  if (workspaceWasmModuleLoadError) {
    return null;
  }
  if (!workspaceWasmModulePromise) {
    workspaceWasmModulePromise = import('./stewart_sim/pkg/stewart_sim.js').catch(err => {
      workspaceWasmModuleLoadError = err;
      return null;
    });
  }
  const module = await workspaceWasmModulePromise;
  if (!module && !workspaceWasmModuleLoadError) {
    workspaceWasmModuleLoadError = new Error('Unknown workspace wasm load failure');
  }
  return module || null;
}

function prepareWorkspaceOptionsForWasm(options) {
  const opts = options || {};
  const servoLimit = typeof opts.servoTorqueLimit === 'number'
    ? (Number.isFinite(opts.servoTorqueLimit) ? opts.servoTorqueLimit : Number.MAX_VALUE)
    : Number.MAX_VALUE;
  return {
    payload: isFiniteNumber(opts.payload) ? opts.payload : 0,
    stroke: isFiniteNumber(opts.stroke) ? opts.stroke : 0,
    frequency: isFiniteNumber(opts.frequency) ? opts.frequency : 0,
    servo_torque_limit: servoLimit,
    ball_joint_limit_deg: isFiniteNumber(opts.ballJointLimitDeg) ? opts.ballJointLimitDeg : 90,
    ball_joint_clamp: Boolean(opts.ballJointClamp)
  };
}

function prepareRangesForWasm(ranges) {
  if (!ranges || typeof ranges !== 'object') {
    return {};
  }
  const prepared = {};
  for (const [axis, spec] of Object.entries(ranges)) {
    if (!spec || typeof spec !== 'object') {
      continue;
    }
    const { min = 0, max = 0, step = 0 } = spec;
    prepared[axis] = {
      min: toFiniteNumber(min, 0),
      max: toFiniteNumber(max, 0),
      step: toFiniteNumber(step, 0)
    };
  }
  return prepared;
}

function preparePlatformForWasm(platform) {
  const issues = { missing: [], invalid: [] };
  if (!platform || typeof platform !== 'object') {
    issues.missing.push('platform object');
    preparePlatformForWasm.lastErrors = issues;
    return null;
  }

  const requiredMethods = [
    { key: 'update', label: 'update() method' },
    { key: 'computeAngles', label: 'computeAngles() method' }
  ];
  for (const { key, label } of requiredMethods) {
    if (typeof platform[key] !== 'function') {
      issues.missing.push(label);
    }
  }

  const sanitized = {};
  let legCount = null;

  const sanitizeVectorField = (key, label) => {
    const value = platform[key];
    if (!Array.isArray(value)) {
      issues.missing.push(label);
      return null;
    }
    if (!value.length) {
      issues.invalid.push(`${label} is empty`);
      return null;
    }
    if (legCount === null) {
      legCount = value.length;
    } else if (value.length !== legCount) {
      issues.invalid.push(`${label} length ${value.length} (expected ${legCount})`);
    }
    const sanitizedVectors = [];
    value.forEach((vec, index) => {
      if (!Array.isArray(vec) || vec.length < 3) {
        issues.invalid.push(`${label}[${index}] not a 3-vector`);
        return;
      }
      const coords = vec.slice(0, 3).map(num => Number(num));
      if (coords.some(n => !Number.isFinite(n))) {
        issues.invalid.push(`${label}[${index}] contains non-numeric values`);
        return;
      }
      sanitizedVectors.push(coords);
    });
    return sanitizedVectors.length === value.length ? sanitizedVectors : null;
  };

  const bPoints = sanitizeVectorField('B', 'B (base joint positions)');
  const hPoints = sanitizeVectorField('H', 'H (servo horn positions)');
  const pPoints = sanitizeVectorField('P', 'P (platform joint positions)');
  if (bPoints) sanitized.B = bPoints;
  if (hPoints) sanitized.H = hPoints;
  if (pPoints) sanitized.P = pPoints;

  const sanitizeNumericArray = (key, label) => {
    const value = platform[key];
    if (!Array.isArray(value)) {
      issues.missing.push(label);
      return null;
    }
    if (legCount !== null && value.length !== legCount) {
      issues.invalid.push(`${label} length ${value.length} (expected ${legCount})`);
    }
    const sanitizedArray = value.map(num => Number(num));
    if (sanitizedArray.some(n => !Number.isFinite(n))) {
      issues.invalid.push(`${label} contains non-numeric values`);
      return null;
    }
    return sanitizedArray;
  };

  const cosBeta = sanitizeNumericArray('cosBeta', 'cosBeta');
  const sinBeta = sanitizeNumericArray('sinBeta', 'sinBeta');
  if (cosBeta) sanitized.cosBeta = cosBeta;
  if (sinBeta) sanitized.sinBeta = sinBeta;

  if (platform.hornLength !== undefined) {
    if (!isFiniteNumber(platform.hornLength)) {
      issues.invalid.push('hornLength (expected finite number)');
    } else {
      sanitized.hornLength = Number(platform.hornLength);
    }
  }

  if (platform.servoRange !== undefined && platform.servoRange !== null) {
    if (!Array.isArray(platform.servoRange) || platform.servoRange.length < 2) {
      issues.invalid.push('servoRange (expected [min, max])');
    } else {
      const range = platform.servoRange.slice(0, 2).map(num => Number(num));
      if (range.some(n => !Number.isFinite(n))) {
        issues.invalid.push('servoRange contains non-numeric values');
      } else if (range[0] > range[1]) {
        issues.invalid.push('servoRange min greater than max');
      } else {
        sanitized.servoRange = range;
      }
    }
  }

  sanitized.translation = Array.isArray(platform.translation) && platform.translation.length >= 3
    ? platform.translation.slice(0, 3).map(value => toFiniteNumber(value, 0))
    : [0, 0, 0];

  sanitized.orientation = extractQuaternionComponents(platform.orientation);

  const hasIssues = issues.missing.length > 0 || issues.invalid.length > 0;
  preparePlatformForWasm.lastErrors = hasIssues ? issues : null;
  return hasIssues ? null : sanitized;
}

preparePlatformForWasm.lastErrors = null;

function formatWasmPlatformIssues(detail) {
  if (!detail) {
    return '';
  }
  const parts = [];
  if (detail.missing && detail.missing.length) {
    parts.push(`missing fields: ${detail.missing.join(', ')}`);
  }
  if (detail.invalid && detail.invalid.length) {
    parts.push(`invalid entries: ${detail.invalid.join(', ')}`);
  }
  return parts.length ? ` (${parts.join('; ')})` : '';
}

function extractQuaternionComponents(q) {
  if (!q) {
    return [0, 0, 0, 1];
  }
  if (typeof q.toArray === 'function') {
    const arr = q.toArray();
    if (Array.isArray(arr) && arr.length === 4) {
      const nums = arr.map(num => Number(num));
      if (nums.every(val => Number.isFinite(val))) {
        return nums;
      }
    }
  }
  if (Array.isArray(q) && q.length === 4) {
    const arr = q.map(num => Number(num));
    if (arr.every(val => Number.isFinite(val))) {
      return arr;
    }
  }
  if (typeof q === 'object') {
    const keys = ['x', 'y', 'z', 'w'];
    if (keys.every(key => isFiniteNumber(q[key]))) {
      return [q.x, q.y, q.z, q.w];
    }
    const alt = ['w', 'x', 'y', 'z'];
    if (alt.every(key => isFiniteNumber(q[key]))) {
      return [q.x, q.y, q.z, q.w];
    }
  }
  return [0, 0, 0, 1];
}

function isFiniteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

function toFiniteNumber(value, fallback) {
  const num = Number(value);
  return Number.isFinite(num) ? num : fallback;
}
