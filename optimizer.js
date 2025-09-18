import { computeWorkspace } from './workspace.js';

/**
 * Minimal placeholder optimizer for the Stewart platform geometry.
 * The heavy calculation loops from the previous implementation have been
 * commented out so they can be reintroduced and verified incrementally.
 */
export class Optimizer {
  constructor(platform, {
    populationSize = 20,
    generations = 10,
    ranges = {},
    payload = 0,
    stroke = 0,
    frequency = 0,
    mutationRate = 0.2,
    ballJointLimitDeg = 90,
    ballJointClamp = true,
  } = {}) {
    this.platform = platform;
    this.populationSize = populationSize;
    this.generations = generations;
    this.ranges = ranges;
    this.payload = payload;
    this.stroke = stroke;
    this.frequency = frequency;
    this.mutationRate = mutationRate;
    this.ballJointLimitDeg = ballJointLimitDeg;
    this.ballJointClamp = ballJointClamp;

    this.population = [];
    this.fitness = [];
    this.pareto = [];
    this.generation = 0;
    this.running = false;
  }

  /** Deep clone of a layout object. */
  cloneLayout(layout = {}) {
    return JSON.parse(JSON.stringify(layout));
  }

  /** Randomize a layout (kept for future use). */
  randomizeLayout(base = {}) {
    const l = this.cloneLayout(base);
    const rand = (v, scale = 1) => v + (Math.random() * 2 - 1) * scale;
    if (typeof l.hornLength === 'number') l.hornLength = rand(l.hornLength, 5);
    if (typeof l.rodLength === 'number') l.rodLength = rand(l.rodLength, 5);
    if (Array.isArray(l.B)) {
      l.B = l.B.map(p => p.map(c => rand(c, 5)));
    }
    if (Array.isArray(l.P)) {
      l.P = l.P.map(p => p.map(c => rand(c, 5)));
    }
    return l;
  }

  /** Prepare population (calculation loop intentionally disabled). */
  initialize() {
    this.population = [];
    if (this.platform && this.platform.layout) {
      this.population.push(this.cloneLayout(this.platform.layout));
    }
    // Calculation loop intentionally disabled for staged debugging of the
    // original implementation. The simplified optimizer still fills the
    // population so that subsequent generations have layouts to mutate.
    for (let i = this.population.length; i < this.populationSize; i++) {
      this.population.push(this.randomizeLayout(this.platform?.layout || {}));
    }
    this.generation = 0;
    this.fitness = [];
    this.pareto = [];
  }

  /** Mutation helper used while the full evolutionary loop is offline. */
  mutateLayout(layout) {
    const mutated = this.cloneLayout(layout);
    const jitter = (scale = 1) => (Math.random() * 2 - 1) * scale;
    if (typeof mutated.hornLength === 'number') {
      mutated.hornLength += jitter(1);
    }
    if (typeof mutated.rodLength === 'number') {
      mutated.rodLength += jitter(1);
    }
    if (Array.isArray(mutated.B)) {
      mutated.B = mutated.B.map((point) => point.map((value) => value + jitter(2)));
    }
    if (Array.isArray(mutated.P)) {
      mutated.P = mutated.P.map((point) => point.map((value) => value + jitter(2)));
    }
    return mutated;
  }

  scoreEvaluation(evaluation) {
    if (!evaluation) return -Infinity;
    const [coverage = 0, dexterity = 0, stiffness = 0, negativeTorque = 0] = evaluation.objectives || [];
    return (
      coverage * 1.0 +
      dexterity * 0.1 +
      stiffness * 0.1 +
      negativeTorque * 0.01
    );
  }

  selectParent(evaluated) {
    if (!evaluated.length) return null;
    const tournamentSize = Math.min(3, evaluated.length);
    let best = null;
    for (let i = 0; i < tournamentSize; i++) {
      const candidate = evaluated[Math.floor(Math.random() * evaluated.length)];
      if (!best || this.scoreEvaluation(candidate) > this.scoreEvaluation(best)) {
        best = candidate;
      }
    }
    return best;
  }

  /** Placeholder Jacobian calculation (loop disabled). */
  computeJacobian(layout) {
    const B = layout.B || this.platform?.B;
    const P = layout.P || this.platform?.P;
    if (!B || !P || B.length !== 6 || P.length !== 6) return null;
    // Calculation loop intentionally disabled for incremental verification.
    // const J = [];
    // for (let i = 0; i < 6; i++) {
    //   ... heavy Jacobian math ...
    // }
    return null;
  }

  /** Jacobian metrics placeholder while loops are disabled. */
  jacobianMetrics() {
    return { cond: 0, sigmaMin: 0, sigmaMax: 0 };
  }

  computeDexterity(layout) {
    const { cond } = this.jacobianMetrics(this.computeJacobian(layout));
    return cond ? 1 / cond : 0;
  }

  computeStiffness(layout) {
    const { sigmaMin } = this.jacobianMetrics(this.computeJacobian(layout));
    return sigmaMin;
  }

  computeTorque(layout) {
    const mass = this.payload;
    const accel = Math.pow(2 * Math.PI * this.frequency, 2) * (this.stroke / 1000);
    const force = mass * (9.81 + accel) / 6;
    const horn = layout.hornLength || this.platform?.hornLength || 1;
    return force * horn;
  }

  /** Evaluate a layout using the workspace module. */
  async evaluateLayout(layout) {
    const workspaceResult = await computeWorkspace({ ...this.platform, ...layout }, this.ranges, {
      payload: this.payload,
      stroke: this.stroke,
      frequency: this.frequency,
      ballJointLimitDeg: this.ballJointLimitDeg,
      ballJointClamp: this.ballJointClamp,
    });

    const coverage = typeof workspaceResult.coverage === 'number' ? workspaceResult.coverage : 0;
    const torque = this.computeTorque(layout);
    const dexterity = this.computeDexterity(layout);
    const stiffness = this.computeStiffness(layout);

    return {
      layout,
      coverage,
      torque,
      dexterity,
      stiffness,
      objectives: [coverage, dexterity, stiffness, -torque],
    };
  }

  /** Optimization step with calculation loops disabled. */
  async step() {
    if (!this.population.length && this.platform?.layout) {
      this.population.push(this.cloneLayout(this.platform.layout));
    }

    const evaluated = [];
    // Evaluation loop disabled in the original implementation; the simplified
    // placeholder still iterates through the current population so progress is
    // visible in the UI while the full scoring math is reinstated.
    for (const candidate of this.population) {
      evaluated.push(await this.evaluateLayout(candidate));
    }

    evaluated.sort((a, b) => this.scoreEvaluation(b) - this.scoreEvaluation(a));
    this.fitness = evaluated;
    this.pareto = evaluated.slice(0, Math.max(1, Math.floor(evaluated.length * 0.2)));
    this.generation += 1;

    const survivors = evaluated.slice(0, Math.max(1, Math.floor(this.populationSize * 0.2)));
    const nextPopulation = survivors.map((item) => this.cloneLayout(item.layout));
    while (nextPopulation.length < this.populationSize && evaluated.length) {
      const parent = this.selectParent(evaluated) || evaluated[0];
      nextPopulation.push(this.mutateLayout(parent.layout));
    }
    this.population = nextPopulation;

    const best = evaluated[0];
    if (best) {
      console.log('[Optimizer] Generation', this.generation, 'best coverage:', best.coverage);
    }
    return best;
  }

  start(callback) {
    if (this.running) return;
    this.initialize();
    this.running = true;

    const run = async () => {
      try {
        while (this.running && this.generation < this.generations) {
          const best = await this.step();
          if (typeof callback === 'function') {
            callback(this, { generation: this.generation, best });
          }
        }
      } finally {
        this.running = false;
      }
    };

    run().catch((err) => {
      console.error(err);
      if (typeof callback === 'function') {
        callback(this, { error: err });
      }
    });
  }

  stop() {
    this.running = false;
  }

  exportBest(format = 'json') {
    const bestLayout = this.fitness[0]?.layout || this.platform?.layout;
    if (!bestLayout) {
      console.warn('No layout available to export.');
      return false;
    }
    if (format !== 'json') {
      console.warn('Only JSON export is supported in the simplified optimizer.');
      return false;
    }
    const data = JSON.stringify(bestLayout, null, 2);
    this.download(data, 'layout.json', 'application/json');
    return true;
  }

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
