import {
  degToRad,
  buildRange,
  rotationMatrixFromEuler,
  rotateVector,
  vectorAdd,
  vectorSub,
  vectorMagnitude,
  vectorNormalize,
  vectorCross,
  vectorDot,
  clamp,
} from './math.js';

const EPS = 1e-8;

function ensureLayout(layout) {
  if (!layout) {
    throw new Error('Layout is required for workspace evaluation.');
  }
  const { baseAnchors, platformAnchors, betaAngles, hornLength, rodLength } = layout;
  if (!Array.isArray(baseAnchors) || baseAnchors.length !== 6) {
    throw new Error('Layout must provide six base anchors.');
  }
  if (!Array.isArray(platformAnchors) || platformAnchors.length !== 6) {
    throw new Error('Layout must provide six platform anchors.');
  }
  if (!Array.isArray(betaAngles) || betaAngles.length !== 6) {
    throw new Error('Layout must provide six servo orientation angles.');
  }
  if (!Number.isFinite(hornLength) || hornLength <= 0) {
    throw new Error('Layout horn length must be a positive number.');
  }
  if (!Number.isFinite(rodLength) || rodLength <= 0) {
    throw new Error('Layout rod length must be a positive number.');
  }
  return layout;
}

function toRadiansRange(range) {
  if (!range) return range;
  return {
    min: degToRad(range.min || 0),
    max: degToRad(range.max || 0),
    step: degToRad(range.step || 1),
  };
}

function computeHornTip(baseAnchor, hornLength, beta, alpha) {
  const cosAlpha = Math.cos(alpha);
  const sinAlpha = Math.sin(alpha);
  const cosBeta = Math.cos(beta);
  const sinBeta = Math.sin(beta);
  return [
    baseAnchor[0] + hornLength * cosAlpha * cosBeta,
    baseAnchor[1] + hornLength * cosAlpha * sinBeta,
    baseAnchor[2] + hornLength * sinAlpha,
  ];
}

export function evaluatePose(layout, pose, options = {}) {
  ensureLayout(layout);
  const {
    ballJointLimitDeg = 45,
    ballJointClamp = true,
    servoRangeRad = layout.servoRangeRad || [-Math.PI / 2, Math.PI / 2],
    rodLengthTolerance = 0.5,
    recordLegData = false,
  } = options;

  const ballJointLimitRad = degToRad(ballJointLimitDeg);
  const translation = [pose.x || 0, pose.y || 0, pose.z || 0];
  const rotation = [pose.rx || 0, pose.ry || 0, pose.rz || 0];

  const homeOffset = layout.homeHeight || 0;
  const translated = [translation[0], translation[1], translation[2] + homeOffset];
  const rotationMatrix = rotationMatrixFromEuler(rotation[0], rotation[1], rotation[2]);

  const servoAngles = [];
  const rodLengths = [];
  const legDirections = [];
  const jacobianRows = [];
  const hornTips = recordLegData ? [] : null;
  const rodVectors = recordLegData ? [] : null;
  const platformPoints = recordLegData ? [] : null;
  const violations = [];

  let reachable = true;

  for (let i = 0; i < 6; i++) {
    const base = layout.baseAnchors[i];
    const platformAnchor = layout.platformAnchors[i];
    const beta = layout.betaAngles[i];

    const rotatedPlatform = rotateVector(rotationMatrix, platformAnchor);
    const q = vectorAdd(translated, rotatedPlatform);
    if (recordLegData) {
      platformPoints.push(q);
    }

    const legVector = vectorSub(q, base);
    const e = 2 * layout.hornLength * legVector[2];
    const f = 2 * layout.hornLength * (Math.cos(beta) * legVector[0] + Math.sin(beta) * legVector[1]);
    const g = legVector[0] * legVector[0]
      + legVector[1] * legVector[1]
      + legVector[2] * legVector[2]
      - (layout.rodLength * layout.rodLength - layout.hornLength * layout.hornLength);

    const denom = Math.sqrt(e * e + f * f);
    if (!Number.isFinite(denom) || denom < EPS) {
      violations.push({ type: 'degenerateFourBar', leg: i, value: denom });
      reachable = false;
      break;
    }

    const ratio = g / denom;
    if (!Number.isFinite(ratio) || Math.abs(ratio) > 1 + 1e-6) {
      violations.push({ type: 'invalidGeometry', leg: i, value: ratio });
      reachable = false;
      break;
    }

    const clampedRatio = clamp(ratio, -1, 1);
    const alpha = Math.asin(clampedRatio) - Math.atan2(f, e);
    servoAngles.push(alpha);

    if (alpha < servoRangeRad[0] - 1e-6 || alpha > servoRangeRad[1] + 1e-6) {
      violations.push({ type: 'servoLimit', leg: i, value: alpha });
      reachable = false;
      break;
    }

    const hornTip = computeHornTip(base, layout.hornLength, beta, alpha);
    const rodVector = vectorSub(q, hornTip);
    const rodLength = vectorMagnitude(rodVector);
    rodLengths.push(rodLength);

    if (Math.abs(rodLength - layout.rodLength) > rodLengthTolerance) {
      violations.push({ type: 'rodLength', leg: i, value: rodLength, target: layout.rodLength });
      reachable = false;
      break;
    }

    let servoAxis = vectorNormalize(vectorSub(hornTip, base));
    if (servoAxis[0] === 0 && servoAxis[1] === 0 && servoAxis[2] === 0) {
      servoAxis = [0, 0, 1];
    }
    const rodDirection = vectorNormalize(rodVector);
    const jointAngle = Math.acos(clamp(vectorDot(servoAxis, rodDirection), -1, 1));

    if (jointAngle > ballJointLimitRad + 1e-6) {
      violations.push({ type: 'ballJoint', leg: i, value: jointAngle, limit: ballJointLimitRad });
      if (!ballJointClamp) {
        reachable = false;
        break;
      }
    }

    const legDirection = vectorNormalize(legVector);
    legDirections.push(legDirection);
    const moment = vectorCross(q, legDirection);
    jacobianRows.push([...legDirection, ...moment]);

    if (recordLegData) {
      hornTips.push(hornTip);
      rodVectors.push(rodVector);
    }
  }

  return {
    reachable,
    violations,
    servoAngles,
    rodLengths,
    legDirections,
    jacobianRows,
    hornTips,
    rodVectors,
    platformPoints,
    translation: translated,
    rotationMatrix,
  };
}

export async function computeWorkspace(layout, ranges = {}, options = {}) {
  ensureLayout(layout);
  const {
    ballJointLimitDeg = 45,
    ballJointClamp = true,
    payload = 0,
    stroke = 0,
    frequency = 0,
  } = options;

  const xs = buildRange(ranges.x, 0);
  const ys = buildRange(ranges.y, 0);
  const zs = buildRange(ranges.z, 0);
  const rxs = buildRange(toRadiansRange(ranges.rx), 0);
  const rys = buildRange(toRadiansRange(ranges.ry), 0);
  const rzs = buildRange(toRadiansRange(ranges.rz), 0);

  const totalPoses = xs.length * ys.length * zs.length * rxs.length * rys.length * rzs.length;
  const reachable = [];
  const unreachable = [];
  const violations = [];

  if (totalPoses === 0) {
    return {
      coverage: 0,
      total: 0,
      reachable: [],
      unreachable: [],
      violations: [],
      payload,
      stroke,
      frequency,
    };
  }

  let reachableCount = 0;

  for (const x of xs) {
    for (const y of ys) {
      for (const z of zs) {
        for (const rx of rxs) {
          for (const ry of rys) {
            for (const rz of rzs) {
              const pose = { x, y, z, rx, ry, rz };
              const result = evaluatePose(layout, pose, {
                ballJointLimitDeg,
                ballJointClamp,
                servoRangeRad: layout.servoRangeRad,
              });
              const poseRecord = { pose, result };
              if (result.reachable) {
                reachable.push(poseRecord);
                reachableCount += 1;
              } else {
                unreachable.push(poseRecord);
                if (result.violations.length) {
                  violations.push({ pose, violations: result.violations });
                }
              }
            }
          }
        }
      }
    }
  }

  const coverage = (reachableCount / totalPoses) * 100;

  return {
    coverage,
    total: totalPoses,
    reachable,
    unreachable,
    violations,
    payload,
    stroke,
    frequency,
  };
}
