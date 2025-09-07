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
    algorithm = 'genetic'
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

  /** Compute dexterity index (placeholder) */
  computeDexterity(layout) {
    if (this.platform.computeJacobian) {
      const J = this.platform.computeJacobian(layout);
      const cond = this.conditionNumber(J);
      return cond ? 1 / cond : 0;
    }
    return 0;
  }

  /** Compute stiffness index (placeholder) */
  computeStiffness(layout) {
    if (this.platform.computeCompliance) {
      const C = this.platform.computeCompliance(layout);
      const trace = C.reduce((sum, row) => sum + row.reduce((s, v) => s + v, 0), 0);
      return trace ? 1 / trace : 0;
    }
    return 0;
  }

  /** Simple condition number using power iteration */
  conditionNumber(M) {
    if (!M || !M.length) return 0;
    const m = M.length, n = M[0].length;
    const multiply = (v) => {
      const r = Array(m).fill(0);
      for (let i = 0; i < m; i++) {
        for (let j = 0; j < n; j++) r[i] += M[i][j] * v[j];
      }
      return r;
    };
    const norm = v => Math.sqrt(v.reduce((s, x) => s + x * x, 0));
    const power = (transpose = false) => {
      let v = Array(transpose ? m : n).fill(1);
      for (let k = 0; k < 10; k++) {
        const w = transpose ? multiplyT(v) : multiply(v);
        const nrm = norm(w);
        if (!nrm) break;
        v = w.map(x => x / nrm);
      }
      return v;
    };
    const multiplyT = (v) => {
      const r = Array(n).fill(0);
      for (let i = 0; i < m; i++) {
        for (let j = 0; j < n; j++) r[j] += M[i][j] * v[i];
      }
      return r;
    };
    const v1 = power(false); const sigmaMax = norm(multiply(v1));
    const v2 = power(true); const sigmaMin = norm(multiplyT(v2));
    return sigmaMin ? sigmaMax / sigmaMin : 0;
  }

  /** Evaluate fitness of layout */
  evaluateLayout(layout) {
    const ws = computeWorkspace({ ...this.platform, ...layout }, this.ranges, {
      payload: this.payload,
      stroke: this.stroke,
      frequency: this.frequency
    });
    const torque = this.computeTorque(layout);
    const dex = this.computeDexterity(layout);
    const stiff = this.computeStiffness(layout);
    const score = ws.coverage - 0.1 * torque + 0.05 * dex + 0.05 * stiff;
    return { layout, coverage: ws.coverage, torque, dexterity: dex, stiffness: stiff, score };
  }

  computeTorque(layout) {
    const mass = this.payload;
    const accel = Math.pow(2 * Math.PI * this.frequency, 2) * (this.stroke / 1000);
    const force = mass * (9.81 + accel) / 6; // per leg
    const horn = layout.hornLength || this.platform.hornLength || 1;
    return force * horn;
  }

  /** Compute Pareto front (non dominated set) */
  computePareto(evals) {
    const front = [];
    for (const a of evals) {
      let dominated = false;
      for (const b of evals) {
        if (b === a) continue;
        if (b.coverage >= a.coverage && b.torque <= a.torque &&
            b.dexterity >= a.dexterity && b.stiffness >= a.stiffness &&
            (b.coverage > a.coverage || b.torque < a.torque ||
             b.dexterity > a.dexterity || b.stiffness > a.stiffness)) {
          dominated = true; break;
        }
      }
      if (!dominated) front.push(a);
    }
    return front;
  }

  /** Run one generation */
  step() {
    const evaluated = this.population.map(p => this.evaluateLayout(p));
    evaluated.sort((a, b) => b.score - a.score);
    this.fitness = evaluated;
    this.pareto = this.computePareto(evaluated);
    const survivors = evaluated.slice(0, Math.floor(this.populationSize / 2)).map(e => e.layout);
    const offspring = [];
    while (survivors.length + offspring.length < this.populationSize) {
      const parent = this.cloneLayout(survivors[Math.floor(Math.random() * survivors.length)]);
      offspring.push(this.mutate(parent));
    }
    this.population = survivors.concat(offspring);
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
    const run = () => {
      if (!this.running || this.generation >= this.generations) {
        this.running = false; if (callback) callback(this); return; }
      this.step();
      setTimeout(run, 0);
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
    const blob = new Blob([data], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'layout.json';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }
}

/** Attach UI hooks */
export function attachUI(optimizer, { startBtn, stopBtn, paretoCanvas, exportBtn }) {
  if (startBtn) startBtn.addEventListener('click', () => optimizer.start());
  if (stopBtn) stopBtn.addEventListener('click', () => optimizer.stop());
  if (exportBtn) exportBtn.addEventListener('click', () => optimizer.exportBest());
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
