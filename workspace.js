// Module to sweep workspace of a Stewart platform
export function computeWorkspace(platform, ranges, options = {}) {
  const {
    payload = 0, // kg
    stroke = 0, // mm
    frequency = 0, // Hz
    servoTorqueLimit = Infinity,
    ballJointLimitDeg = 90
  } = options;

  const QuaternionObj = (typeof Quaternion !== 'undefined') ? Quaternion : options.Quaternion;
  if (!platform || !QuaternionObj) {
    throw new Error('platform and Quaternion are required');
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
    const { min = 0, max = 0, step = 1 } = ranges[axis] || {};
    const arr = [];
    for (let v = min; v <= max + 1e-9; v += step) arr.push(v);
    return arr;
  };

  const xs = list('x'), ys = list('y'), zs = list('z');
  const rxs = list('rx'), rys = list('ry'), rzs = list('rz');

  let total = 0;
  for (const x of xs) {
    for (const y of ys) {
      for (const z of zs) {
        for (const rx of rxs) {
          for (const ry of rys) {
            for (const rz of rzs) {
              total++;
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
                if (ok && platform.B && platform.H && platform.P) {
                  const toDeg = (rad) => rad * 180 / Math.PI;
                  for (let i = 0; i < platform.P.length; i++) {
                    const hornVec = [
                      platform.H[i][0] - platform.B[i][0],
                      platform.H[i][1] - platform.B[i][1],
                      platform.H[i][2] - platform.B[i][2]
                    ];
                    const rodVec = [
                      platform.P[i][0] - platform.H[i][0],
                      platform.P[i][1] - platform.H[i][1],
                      platform.P[i][2] - platform.H[i][2]
                    ];
                    const dot = hornVec[0] * rodVec[0] + hornVec[1] * rodVec[1] + hornVec[2] * rodVec[2];
                    const magH = Math.sqrt(hornVec[0] ** 2 + hornVec[1] ** 2 + hornVec[2] ** 2);
                    const magR = Math.sqrt(rodVec[0] ** 2 + rodVec[1] ** 2 + rodVec[2] ** 2);
                    const angle = toDeg(Math.acos(Math.min(Math.max(dot / (magH * magR), -1), 1)));
                    if (angle > ballJointLimitDeg) { ok = false; reason = 'ball joint'; break; }
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
                console.log('reachable', pose);
              } else {
                failures.push({ pose, reason });
                violationCounts[reason] = (violationCounts[reason] || 0) + 1;
              }
            }
          }
        }
      }
    }
  }

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
