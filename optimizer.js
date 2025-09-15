import { computeWorkspace } from './workspace.js';

/**
 * Simple genetic optimizer for Stewart platform geometry.
 * Optimizes horn/rod lengths and anchor positions.
 * Fitness incorporates workspace coverage, torque, dexterity, and stiffness.
 */
export class Optimizer {
  constructor(platform, {
      populationSize = 20,
      generations = 50,
      ranges = {},
      payload = 0,
      stroke = 0,
      frequency = 0,
      mutationRate = 0.2,
      algorithm = 'genetic',
      ballJointLimitDeg = 90,
      ballJointClamp = true
    } = {}) {
    this.platform = platform;
    this.populationSize = populationSize;
    this.generations = generations;
    this.ranges = ranges;
    this.payload = payload;
    this.stroke = stroke;
    this.frequency = frequency;
    this.mutationRate = mutationRate;
      this.algorithm = algorithm;
      this.ballJointLimitDeg = ballJointLimitDeg;
      this.ballJointClamp = ballJointClamp;
    this.population = [];
    this.fitness = [];
    this.generation = 0;
    this.running = false;
    this.pareto = [];
  }

  /** Deep clone of layout */
  cloneLayout(layout) {
    return JSON.parse(JSON.stringify(layout));
  }

  /** Randomize layout by tweaking horn/rod lengths and anchors */
  randomizeLayout(base) {
    const l = this.cloneLayout(base);
    const rand = (v, scale = 1) => v + (Math.random() * 2 - 1) * scale;
    l.hornLength = rand(l.hornLength || 0, 5);
    l.rodLength = rand(l.rodLength || 0, 5);
    if (l.B) {
      l.B = l.B.map(p => p.map(c => rand(c, 5)));
    }
    if (l.P) {
      l.P = l.P.map(p => p.map(c => rand(c, 5)));
    }
    return l;
  }

  /** Initialize random population around current platform layout */
  initialize() {
    this.population = [];
    for (let i = 0; i < this.populationSize; i++) {
      this.population.push(this.randomizeLayout(this.platform.layout || {}));
    }
    this.generation = 0;
  }

  /** Compute geometric Jacobian for a given layout */
  computeJacobian(layout) {
    const B = layout.B || this.platform.B;
    const P = layout.P || this.platform.P;
    if (!B || !P || B.length !== 6 || P.length !== 6) return null;
    const orient = layout.orientation || this.platform.orientation;
    const trans = layout.translation || this.platform.translation || [0, 0, 0];
    const rotate = orient && typeof orient.rotateVector === 'function'
      ? v => orient.rotateVector(v)
      : v => v;
    const J = [];
    for (let i = 0; i < 6; i++) {
      const pi = rotate(P[i]);
      const qi = [pi[0] + trans[0], pi[1] + trans[1], pi[2] + trans[2]];
      const li = [qi[0] - B[i][0], qi[1] - B[i][1], qi[2] - B[i][2]];
      const len = Math.sqrt(li[0]*li[0] + li[1]*li[1] + li[2]*li[2]);
      if (!len) return null;
      const u = [li[0]/len, li[1]/len, li[2]/len];
      const cross = [
        pi[1]*u[2] - pi[2]*u[1],
        pi[2]*u[0] - pi[0]*u[2],
        pi[0]*u[1] - pi[1]*u[0]
      ];
      J.push([...u, ...cross]);
    }
    return J;
  }

  /** Compute singular value metrics for a matrix */
  jacobianMetrics(M) {
    if (!M) return { cond: 0, sigmaMin: 0, sigmaMax: 0 };
    const transpose = (A) => A[0].map((_, i) => A.map(r => r[i]));
    const multiplyMat = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
    const multiplyVec = (A, v) => A.map(r => r.reduce((s, val, i) => s + val * v[i], 0));
    const invert = (A) => {
      const n = A.length;
      const M = A.map((r, i) => r.concat(Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))));
      for (let i = 0; i < n; i++) {
        let pivot = i;
        for (let j = i + 1; j < n; j++) if (Math.abs(M[j][i]) > Math.abs(M[pivot][i])) pivot = j;
        if (!M[pivot][i]) return null;
        if (pivot !== i) [M[i], M[pivot]] = [M[pivot], M[i]];
        const div = M[i][i];
        for (let j = 0; j < 2 * n; j++) M[i][j] /= div;
        for (let j = 0; j < n; j++) if (j !== i) {
          const factor = M[j][i];
          for (let k = 0; k < 2 * n; k++) M[j][k] -= factor * M[i][k];
        }
      }
      return M.map(r => r.slice(n));
    };
    const powerEigen = (A) => {
      const n = A.length;
      let v = Array(n).fill(1);
      const norm = v => Math.sqrt(v.reduce((s, x) => s + x * x, 0));
      for (let iter = 0; iter < 20; iter++) {
        const w = multiplyVec(A, v);
        const nrm = norm(w);
        if (!nrm) break;
        v = w.map(x => x / nrm);
      }
      const Av = multiplyVec(A, v);
      return v.reduce((s, x, i) => s + x * Av[i], 0);
    };
    const JT = transpose(M);
    const JTJ = multiplyMat(JT, M);
    const lambdaMax = powerEigen(JTJ);
    let sigmaMax = Math.sqrt(Math.max(lambdaMax, 0));
    let sigmaMin = 0;
    const JTJInv = invert(JTJ);
    if (JTJInv) {
      const lambdaInvMax = powerEigen(JTJInv);
      if (lambdaInvMax) sigmaMin = 1 / Math.sqrt(lambdaInvMax);
    }
    const cond = sigmaMin ? sigmaMax / sigmaMin : 0;
    return { cond, sigmaMin, sigmaMax };
  }

  /** Compute dexterity index as inverse Jacobian condition number */
  computeDexterity(layout) {
    const J = this.computeJacobian(layout);
    const { cond } = this.jacobianMetrics(J);
    return cond ? 1 / cond : 0;
  }

  /** Approximate stiffness as smallest singular value of Jacobian */
  computeStiffness(layout) {
    const J = this.computeJacobian(layout);
    const { sigmaMin } = this.jacobianMetrics(J);
    return sigmaMin;
  }

  /** Evaluate objectives of a layout */
  async evaluateLayout(layout) {
    const platformClone = Object.assign(
      Object.create(Object.getPrototypeOf(this.platform)),
      this.platform,
      layout
    );
    const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());
    const evalStart = now();
    const ws = await computeWorkspace(platformClone, this.ranges, {
      payload: this.payload,
      stroke: this.stroke,
      frequency: this.frequency,
      ballJointLimitDeg: this.ballJointLimitDeg,
      ballJointClamp: this.ballJointClamp
    });
    const evalTime = now() - evalStart;
    if (isNaN(ws.coverage)) {
      console.warn('Workspace evaluation returned invalid coverage', ws);
    }
    if (this.debug) {
      console.log(`Layout evaluated in ${evalTime.toFixed(2)}ms, coverage ${ws.coverage}`);
    }
    const torque = this.computeTorque(layout);
    const dex = this.computeDexterity(layout);
    const stiff = this.computeStiffness(layout);
    return {
      layout,
      coverage: ws.coverage,
      torque,
      dexterity: dex,
      stiffness: stiff,
      objectives: [ws.coverage, dex, stiff, -torque]
    };
  }

  computeTorque(layout) {
    const mass = this.payload;
    const accel = Math.pow(2 * Math.PI * this.frequency, 2) * (this.stroke / 1000);
    const force = mass * (9.81 + accel) / 6; // per leg
    const horn = layout.hornLength || this.platform.hornLength || 1;
    return force * horn;
  }

  /** Check Pareto dominance (maximize all objectives) */
  dominates(a, b) {
    const A = a.objectives, B = b.objectives;
    let better = false;
    for (let i = 0; i < A.length; i++) {
      if (A[i] < B[i]) return false;
      if (A[i] > B[i]) better = true;
    }
    return better;
  }

  /** Non-dominated sorting */
  sortPopulation(evals) {
    const fronts = [];
    const individuals = evals.map(e => ({ ...e, dominationCount: 0, dominates: [], rank: 0, crowding: 0 }));
    for (let i = 0; i < individuals.length; i++) {
      for (let j = 0; j < individuals.length; j++) {
        if (i === j) continue;
        if (this.dominates(individuals[i], individuals[j])) {
          individuals[i].dominates.push(individuals[j]);
        } else if (this.dominates(individuals[j], individuals[i])) {
          individuals[i].dominationCount++;
        }
      }
      if (individuals[i].dominationCount === 0) {
        individuals[i].rank = 0;
        (fronts[0] = fronts[0] || []).push(individuals[i]);
      }
    }
    let r = 0;
    while (fronts[r] && fronts[r].length) {
      const next = [];
      for (const p of fronts[r]) {
        for (const q of p.dominates) {
          q.dominationCount--;
          if (q.dominationCount === 0) {
            q.rank = r + 1;
            next.push(q);
          }
        }
      }
      r++;
      if (next.length) fronts[r] = next;
    }
    return fronts;
  }

  /** Crowding distance for a front */
  crowdingDistance(front) {
    const m = front[0].objectives.length;
    for (const p of front) p.crowding = 0;
    for (let i = 0; i < m; i++) {
      front.sort((a, b) => a.objectives[i] - b.objectives[i]);
      front[0].crowding = front[front.length - 1].crowding = Infinity;
      const min = front[0].objectives[i];
      const max = front[front.length - 1].objectives[i];
      if (max === min) continue;
      for (let j = 1; j < front.length - 1; j++) {
        front[j].crowding += (front[j + 1].objectives[i] - front[j - 1].objectives[i]) / (max - min);
      }
    }
  }

  /** Run one generation */
  async step() {
    const gen = this.generation;
    console.log(`Starting generation ${gen}`);
    console.time(`generation-${gen}`);
    const evaluated = [];
    for (const p of this.population) {
      evaluated.push(await this.evaluateLayout(p));
    }
    const fronts = this.sortPopulation(evaluated);
    fronts.forEach(f => this.crowdingDistance(f));
    const sorted = fronts.flat().sort((a, b) => a.rank - b.rank || b.crowding - a.crowding);
    this.fitness = sorted;
    this.pareto = fronts[0] || [];
    const survivorLayouts = sorted.slice(0, Math.floor(this.populationSize / 2)).map(s => s.layout);
    const offspring = [];
    while (survivorLayouts.length + offspring.length < this.populationSize) {
      const parent = this.cloneLayout(survivorLayouts[Math.floor(Math.random() * survivorLayouts.length)]);
      offspring.push(this.mutate(parent));
    }
    this.population = survivorLayouts.concat(offspring);
    console.timeEnd(`generation-${gen}`);
    console.log(`Completed generation ${gen} with best coverage ${(this.fitness[0]?.coverage * 100).toFixed(1)}%`);
    this.generation++;
  }

  mutate(layout) {
    const l = this.cloneLayout(layout);
    const chance = this.mutationRate;
    const rand = (v, scale = 1) => v + (Math.random() * 2 - 1) * scale;
    if (Math.random() < chance) l.hornLength = rand(l.hornLength, 5);
    if (Math.random() < chance) l.rodLength = rand(l.rodLength, 5);
    if (l.B) {
      l.B = l.B.map(p => p.map(c => (Math.random() < chance ? rand(c, 5) : c)));
    }
    if (l.P) {
      l.P = l.P.map(p => p.map(c => (Math.random() < chance ? rand(c, 5) : c)));
    }
    return l;
  }

  /** Start async optimization */
  start(callback) {
    if (this.running) return;
    this.initialize();
    this.running = true;
    console.log(`Optimizer started: ${this.generations} generations, population ${this.populationSize}`);
    const run = async () => {
      if (!this.running || this.generation >= this.generations) {
        this.running = false;
        console.log('Optimizer finished');
        if (callback) callback(null, this);
        return;
      }
      try {
        await this.step();
        setTimeout(run, 0);
      } catch (err) {
        this.running = false;
        console.error('Optimizer error', err);
        this.lastError = err;
        if (callback) callback(err, this); else throw err;
      }
    };
    run();
  }

  /** Stop optimization */
  stop() { this.running = false; }

  /** Export best layout */
  exportBest(format = 'json') {
    if (!this.fitness.length) return;
    const best = this.fitness[0].layout;
    let data = '';
    if (format === 'json') {
      data = JSON.stringify(best, null, 2);
    } else {
      return data;
    }
    this.download(data, 'layout.json', 'application/json');
  }

  /** Export current Pareto front */
  exportPareto(format = 'json') {
    if (!this.pareto.length) return;
    let data = '';
    let mime = 'application/json';
    if (format === 'json') {
      data = JSON.stringify(this.pareto, null, 2);
    } else if (format === 'csv') {
      const rows = [['hornLength','rodLength','coverage','torque','dexterity','stiffness']];
      this.pareto.forEach(p => {
        rows.push([
          p.layout.hornLength,
          p.layout.rodLength,
          p.coverage,
          p.torque,
          p.dexterity,
          p.stiffness
        ]);
      });
      data = rows.map(r => r.join(',')).join('\n');
      mime = 'text/csv';
    } else {
      return;
    }
    this.download(data, 'pareto.' + (format === 'csv' ? 'csv' : 'json'), mime);
  }

  /** Utility to trigger file download in browser */
  download(data, filename, mime) {
    const blob = new Blob([data], { type: mime });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }
}

/** Attach UI hooks */
export function attachUI(optimizer, { startBtn, stopBtn, paretoCanvas, exportBtn, paretoExportBtn }) {
  if (startBtn) startBtn.addEventListener('click', () => optimizer.start());
  if (stopBtn) stopBtn.addEventListener('click', () => optimizer.stop());
  if (exportBtn) exportBtn.addEventListener('click', () => optimizer.exportBest());
  if (paretoExportBtn) paretoExportBtn.addEventListener('click', () => optimizer.exportPareto());
  if (paretoCanvas) {
    const ctx = paretoCanvas.getContext('2d');
    const draw = () => {
      ctx.clearRect(0, 0, paretoCanvas.width, paretoCanvas.height);
      const front = optimizer.pareto || [];
      front.forEach(f => {
        const x = f.torque;
        const y = f.coverage;
        ctx.fillRect(x, paretoCanvas.height - y * paretoCanvas.height, 4, 4);
      });
      if (optimizer.running) requestAnimationFrame(draw);
    };
    optimizer.start = ((orig) => (...args) => { orig.apply(optimizer, args); requestAnimationFrame(draw); })(optimizer.start);
  }
}
