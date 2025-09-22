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
  singularValues,
  average,
  standardDeviation,
} from './math.js';
import { getWorkspaceAccelerator } from './workspace-accel.js';

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
  const ballJointAngles = [];
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
    ballJointAngles.push(jointAngle);

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
    ballJointAngles,
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
    sampleLimit = 200,
    violationSampleLimit = sampleLimit,
    forceFullSweep = false,
  } = options;

  const xs = buildRange(ranges.x, 0);
  const ys = buildRange(ranges.y, 0);
  const zs = buildRange(ranges.z, 0);
  const rxs = buildRange(toRadiansRange(ranges.rx), 0);
  const rys = buildRange(toRadiansRange(ranges.ry), 0);
  const rzs = buildRange(toRadiansRange(ranges.rz), 0);

  const axisArrays = [xs, ys, zs, rxs, rys, rzs];
  const axisLengths = axisArrays.map((axis) => axis.length);
  const totalPoses = axisLengths.reduce((acc, len) => acc * len, 1);

  const normalizedSampleLimit = Math.max(0, Math.floor(sampleLimit));
  const normalizedViolationSampleLimit = Math.max(0, Math.floor(violationSampleLimit));
  const evaluationCapRaw = Math.max(normalizedSampleLimit, normalizedViolationSampleLimit);
  const evaluationCap = evaluationCapRaw > 0 ? evaluationCapRaw : totalPoses;
  const shouldFullSweep = forceFullSweep || evaluationCap >= totalPoses;

  const emptyResult = (samplingMode = 'empty', target = 0) => ({
    coverage: 0,
    total: totalPoses,
    reachable: [],
    unreachable: [],
    violations: [],
    payload,
    stroke,
    frequency,
    stats: {
      reachableCount: 0,
      evaluatedCount: 0,
      totalPoses,
      sampleRatio: 0,
      averageIsotropy: 0,
      averageStiffness: 0,
      loadBalanceScore: 0,
      servoUsage: new Array(6).fill(0),
      servoUsageAvg: 0,
      servoUsagePeak: 0,
      ballJointMax: new Array(6).fill(0),
      ballJointOverallMax: 0,
      ballJointAverage: 0,
      violationCounts: {},
      violationRate: 0,
      acceleratedEvaluations: 0,
    },
    counts: {
      reachable: 0,
      unreachable: 0,
      violationPoses: 0,
      sampled: 0,
      total: totalPoses,
    },
    sampling: {
      mode: samplingMode,
      evaluated: 0,
      target,
      total: totalPoses,
      acceleratorUsed: 0,
    },
    samples: {
      reachable: [],
      unreachable: [],
      violations: [],
      limits: {
        reachable: normalizedSampleLimit,
        unreachable: normalizedSampleLimit,
        violations: normalizedViolationSampleLimit,
        evaluations: target,
      },
    },
  });

  if (totalPoses === 0) {
    return emptyResult('empty', 0);
  }

  const targetEvaluations = shouldFullSweep ? totalPoses : Math.min(totalPoses, evaluationCap);

  if (targetEvaluations <= 0) {
    return emptyResult(shouldFullSweep ? 'full' : 'sampled', targetEvaluations);
  }

  const reachableSamples = [];
  const unreachableSamples = [];
  const violationSamples = [];
  const violationCounts = {};
  const isotropySamples = [];
  const stiffnessSamples = [];
  const loadShareSamples = [];
  const ballJointSamples = [];
  const servoRanges = Array.from({ length: 6 }, () => ({ min: Infinity, max: -Infinity }));
  const ballJointMax = new Array(6).fill(0);

  const recordSample = (collection, limit, seenCount, sample) => {
    if (limit <= 0) return;
    if (collection.length < limit) {
      collection.push(sample);
    } else {
      const replaceIndex = Math.floor(Math.random() * seenCount);
      if (replaceIndex < limit) {
        collection[replaceIndex] = sample;
      }
    }
  };

  let reachableCount = 0;
  let unreachableCount = 0;
  let violationPoseCount = 0;
  let reachableSeen = 0;
  let unreachableSeen = 0;
  let violationSeen = 0;
  let evaluatedCount = 0;
  let acceleratedEvaluations = 0;

  const accelerator = await getWorkspaceAccelerator();
  const jacobianBuffer = accelerator ? new Float64Array(36) : null;

  const lenX = axisLengths[0];
  const lenY = axisLengths[1];
  const lenZ = axisLengths[2];
  const lenRx = axisLengths[3];
  const lenRy = axisLengths[4];
  const lenRz = axisLengths[5];

  const pose = { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 };

  const assignPoseFromIndex = (index) => {
    let remainder = index;
    const idxX = remainder % lenX;
    remainder = Math.floor(remainder / lenX);
    const idxY = remainder % lenY;
    remainder = Math.floor(remainder / lenY);
    const idxZ = remainder % lenZ;
    remainder = Math.floor(remainder / lenZ);
    const idxRx = remainder % lenRx;
    remainder = Math.floor(remainder / lenRx);
    const idxRy = remainder % lenRy;
    remainder = Math.floor(remainder / lenRy);
    const idxRz = remainder % lenRz;

    pose.x = xs[idxX];
    pose.y = ys[idxY];
    pose.z = zs[idxZ];
    pose.rx = rxs[idxRx];
    pose.ry = rys[idxRy];
    pose.rz = rzs[idxRz];
    return pose;
  };

  const clonePose = () => ({
    x: pose.x,
    y: pose.y,
    z: pose.z,
    rx: pose.rx,
    ry: pose.ry,
    rz: pose.rz,
  });

  const computeSigmaExtents = (rows) => {
    if (!rows || rows.length !== 6) return null;

    if (accelerator && jacobianBuffer) {
      let offset = 0;
      for (let r = 0; r < 6; r++) {
        const row = rows[r];
        if (!row || row.length !== 6) {
          offset = -1;
          break;
        }
        for (let c = 0; c < 6; c++) {
          jacobianBuffer[offset++] = row[c];
        }
      }
      if (offset === 36) {
        const estimate = accelerator.estimate(jacobianBuffer);
        if (estimate && Number.isFinite(estimate.sigmaMax) && Number.isFinite(estimate.sigmaMin)) {
          acceleratedEvaluations += 1;
          return estimate;
        }
      }
    }

    const sv = singularValues(rows);
    if (!sv || !sv.length) return null;
    let sigmaMax = -Infinity;
    let sigmaMin = Infinity;
    for (const value of sv) {
      if (!Number.isFinite(value)) continue;
      if (value > sigmaMax) sigmaMax = value;
      if (value < sigmaMin) sigmaMin = value;
    }
    if (!Number.isFinite(sigmaMax) || !Number.isFinite(sigmaMin) || sigmaMax <= EPS || sigmaMin <= 0) {
      return null;
    }
    return { sigmaMin, sigmaMax };
  };

  const processIndex = (index) => {
    const currentPose = assignPoseFromIndex(index);
    const result = evaluatePose(layout, currentPose, {
      ballJointLimitDeg,
      ballJointClamp,
      servoRangeRad: layout.servoRangeRad,
      recordLegData: false,
    });

    evaluatedCount += 1;

    const hasViolations = Array.isArray(result.violations) && result.violations.length > 0;

    if (result.reachable) {
      reachableCount += 1;
      reachableSeen += 1;
      recordSample(reachableSamples, normalizedSampleLimit, reachableSeen, { pose: clonePose() });

      const sigma = computeSigmaExtents(result.jacobianRows);
      if (sigma) {
        const { sigmaMin, sigmaMax } = sigma;
        if (sigmaMax > EPS && sigmaMin > EPS) {
          isotropySamples.push(sigmaMin / sigmaMax);
          stiffnessSamples.push(sigmaMin);
        }
      }

      if (Array.isArray(result.legDirections) && result.legDirections.length === 6) {
        const shares = result.legDirections.map((dir) => Math.abs(dir[2]));
        const sumShares = shares.reduce((acc, value) => acc + value, 0) || 1;
        const normalized = shares.map((value) => value / sumShares);
        const loadStd = standardDeviation(normalized);
        const loadScore = 1 / (1 + loadStd);
        loadShareSamples.push(loadScore);
      }

      if (Array.isArray(result.servoAngles)) {
        for (let leg = 0; leg < Math.min(6, result.servoAngles.length); leg++) {
          const angle = result.servoAngles[leg];
          const servo = servoRanges[leg];
          if (angle < servo.min) servo.min = angle;
          if (angle > servo.max) servo.max = angle;
        }
      }

      if (Array.isArray(result.ballJointAngles)) {
        const maxAngle = Math.max(
          ...result.ballJointAngles.map((value) => (Number.isFinite(value) ? value : 0)),
          0,
        );
        ballJointSamples.push(maxAngle);
        for (let leg = 0; leg < Math.min(6, result.ballJointAngles.length); leg++) {
          const angle = result.ballJointAngles[leg];
          if (Number.isFinite(angle) && angle > ballJointMax[leg]) {
            ballJointMax[leg] = angle;
          }
        }
      }
    } else {
      unreachableCount += 1;
      unreachableSeen += 1;
      recordSample(unreachableSamples, normalizedSampleLimit, unreachableSeen, { pose: clonePose() });
    }

    if (hasViolations) {
      violationPoseCount += 1;
      violationSeen += 1;
      const violationCopy = result.violations.map((violation) => ({ ...violation }));
      recordSample(violationSamples, normalizedViolationSampleLimit, violationSeen, {
        pose: clonePose(),
        violations: violationCopy,
      });
      for (const violation of result.violations) {
        violationCounts[violation.type] = (violationCounts[violation.type] || 0) + 1;
      }
    }
  };

  if (shouldFullSweep) {
    for (let index = 0; index < totalPoses; index += 1) {
      processIndex(index);
    }
  } else {
    const chosen = new Set();
    const stride = totalPoses / targetEvaluations;
    for (let i = 0; i < targetEvaluations; i += 1) {
      let candidate = Math.floor(i * stride + Math.random() * stride);
      if (candidate >= totalPoses) {
        candidate = totalPoses - 1;
      }
      while (chosen.has(candidate)) {
        candidate = (candidate + 1) % totalPoses;
      }
      chosen.add(candidate);
    }
    for (const index of chosen) {
      processIndex(index);
    }
  }

  const coverage = evaluatedCount > 0 ? (reachableCount / evaluatedCount) * 100 : 0;

  const servoUsage = servoRanges.map((range) => {
    if (range.min === Infinity || range.max === -Infinity) return 0;
    return range.max - range.min;
  });
  const servoUsageAvg = average(servoUsage);
  const servoUsagePeak = Math.max(0, ...servoUsage);

  const violationTotal = Object.values(violationCounts).reduce((acc, value) => acc + value, 0);
  const violationRate = evaluatedCount > 0 ? Math.min(violationTotal / evaluatedCount, 1) : 0;

  const stats = {
    reachableCount,
    evaluatedCount,
    totalPoses,
    sampleRatio: totalPoses > 0 ? evaluatedCount / totalPoses : 0,
    averageIsotropy: average(isotropySamples),
    averageStiffness: average(stiffnessSamples),
    loadBalanceScore: average(loadShareSamples),
    servoUsage,
    servoUsageAvg,
    servoUsagePeak,
    ballJointMax,
    ballJointOverallMax: Math.max(0, ...ballJointMax),
    ballJointAverage: average(ballJointSamples),
    violationCounts,
    violationRate,
    acceleratedEvaluations,
  };

  return {
    coverage,
    total: totalPoses,
    reachable: reachableSamples,
    unreachable: unreachableSamples,
    violations: violationSamples,
    payload,
    stroke,
    frequency,
    stats,
    counts: {
      reachable: reachableCount,
      unreachable: unreachableCount,
      violationPoses: violationPoseCount,
      sampled: evaluatedCount,
      total: totalPoses,
    },
    sampling: {
      mode: shouldFullSweep ? 'full' : 'sampled',
      evaluated: evaluatedCount,
      target: targetEvaluations,
      total: totalPoses,
      acceleratorUsed: acceleratedEvaluations,
    },
    samples: {
      reachable: reachableSamples,
      unreachable: unreachableSamples,
      violations: violationSamples,
      limits: {
        reachable: normalizedSampleLimit,
        unreachable: normalizedSampleLimit,
        violations: normalizedViolationSampleLimit,
        evaluations: targetEvaluations,
      },
    },
  };
}

