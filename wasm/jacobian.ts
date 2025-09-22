const N: i32 = 6;
const SIZE: i32 = N * N;
const EPS: f64 = 1e-12;
const ITERATIONS: i32 = 18;

const jacobian = new StaticArray<f64>(SIZE);
const jtj = new StaticArray<f64>(SIZE);
const vectorA = new StaticArray<f64>(N);
const vectorB = new StaticArray<f64>(N);
const vectorC = new StaticArray<f64>(N);
const output = new StaticArray<f64>(2);

export const jacobianPtr: usize = changetype<usize>(jacobian);
export const resultPtr: usize = changetype<usize>(output);

function zeroArray(arr: StaticArray<f64>): void {
  for (let i = 0; i < arr.length; i++) {
    arr[i] = 0.0;
  }
}

function copyArray(source: StaticArray<f64>, target: StaticArray<f64>): void {
  for (let i = 0; i < source.length; i++) {
    target[i] = source[i];
  }
}

function initVectorOnes(target: StaticArray<f64>): void {
  for (let i = 0; i < target.length; i++) {
    target[i] = 1.0;
  }
}

function initVectorRamp(target: StaticArray<f64>): void {
  for (let i = 0; i < target.length; i++) {
    target[i] = 1.0 - f64(i) * 0.15;
  }
}

function normalizeVector(vec: StaticArray<f64>): f64 {
  let norm = 0.0;
  for (let i = 0; i < vec.length; i++) {
    const value = vec[i];
    norm += value * value;
  }
  norm = Math.sqrt(norm);
  if (norm < EPS) {
    return 0.0;
  }
  const inv = 1.0 / norm;
  for (let i = 0; i < vec.length; i++) {
    vec[i] = vec[i] * inv;
  }
  return norm;
}

function multiply(matrix: StaticArray<f64>, vector: StaticArray<f64>, out: StaticArray<f64>): void {
  for (let row = 0; row < N; row++) {
    let sum = 0.0;
    for (let col = 0; col < N; col++) {
      sum += matrix[row * N + col] * vector[col];
    }
    out[row] = sum;
  }
}

function multiplyShifted(matrix: StaticArray<f64>, shift: f64, vector: StaticArray<f64>, out: StaticArray<f64>): void {
  for (let row = 0; row < N; row++) {
    let sum = 0.0;
    for (let col = 0; col < N; col++) {
      sum += matrix[row * N + col] * vector[col];
    }
    out[row] = shift * vector[row] - sum;
  }
}

function rayleigh(matrix: StaticArray<f64>, vector: StaticArray<f64>, scratch: StaticArray<f64>): f64 {
  multiply(matrix, vector, scratch);
  let result = 0.0;
  for (let i = 0; i < vector.length; i++) {
    result += vector[i] * scratch[i];
  }
  return result;
}

function rayleighShifted(matrix: StaticArray<f64>, shift: f64, vector: StaticArray<f64>, scratch: StaticArray<f64>): f64 {
  multiplyShifted(matrix, shift, vector, scratch);
  let result = 0.0;
  for (let i = 0; i < vector.length; i++) {
    result += vector[i] * scratch[i];
  }
  return result;
}

function powerIteration(matrix: StaticArray<f64>, vector: StaticArray<f64>, nextVector: StaticArray<f64>, scratch: StaticArray<f64>): f64 {
  if (normalizeVector(vector) == 0.0) {
    vector[0] = 1.0;
    normalizeVector(vector);
  }
  for (let iter = 0; iter < ITERATIONS; iter++) {
    multiply(matrix, vector, nextVector);
    if (normalizeVector(nextVector) == 0.0) {
      break;
    }
    copyArray(nextVector, vector);
  }
  return rayleigh(matrix, vector, scratch);
}

function powerIterationShifted(matrix: StaticArray<f64>, shift: f64, vector: StaticArray<f64>, nextVector: StaticArray<f64>, scratch: StaticArray<f64>): f64 {
  if (normalizeVector(vector) == 0.0) {
    vector[0] = 1.0;
    normalizeVector(vector);
  }
  for (let iter = 0; iter < ITERATIONS; iter++) {
    multiplyShifted(matrix, shift, vector, nextVector);
    if (normalizeVector(nextVector) == 0.0) {
      break;
    }
    copyArray(nextVector, vector);
  }
  return rayleighShifted(matrix, shift, vector, scratch);
}

function computeJTJ(): void {
  for (let row = 0; row < N; row++) {
    for (let col = row; col < N; col++) {
      let sum = 0.0;
      for (let k = 0; k < N; k++) {
        const a = load<f64>(jacobianPtr + ((k * N + row) << 3));
        const b = load<f64>(jacobianPtr + ((k * N + col) << 3));
        sum += a * b;
      }
      jtj[row * N + col] = sum;
      jtj[col * N + row] = sum;
    }
  }
}

export function evaluate(): void {
  computeJTJ();
  initVectorOnes(vectorA);
  zeroArray(vectorB);
  zeroArray(vectorC);
  const lambdaMax = powerIteration(jtj, vectorA, vectorB, vectorC);
  if (lambdaMax <= EPS) {
    output[0] = 0.0;
    output[1] = 0.0;
    return;
  }

  const safety = Math.max(lambdaMax * 0.05, 1e-6);
  const shift = lambdaMax + safety;

  initVectorRamp(vectorA);
  zeroArray(vectorB);
  zeroArray(vectorC);
  const shiftedEigen = powerIterationShifted(jtj, shift, vectorA, vectorB, vectorC);
  let lambdaMin = shift - shiftedEigen;
  if (lambdaMin < 0.0) {
    lambdaMin = 0.0;
  }

  output[0] = Math.sqrt(lambdaMin);
  output[1] = Math.sqrt(lambdaMax);
}
