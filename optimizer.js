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
    // Calculation loop intentionally disabled for staged debugging.
    // for (let i = 0; i < this.populationSize; i++) {
    //   this.population.push(this.randomizeLayout(this.platform.layout || {}));
    // }
    if (this.platform && this.platform.layout) {
      this.population.push(this.cloneLayout(this.platform.layout));
    }
    this.generation = 0;
    this.fitness = [];
    this.pareto = [];
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
    const baseLayout = this.population[0]
      ? this.cloneLayout(this.population[0])
      : this.cloneLayout(this.platform?.layout || {});

    // Evaluation loop disabled for incremental bring-up.
    // const evaluated = [];
    // for (const candidate of this.population) {
    //   evaluated.push(await this.evaluateLayout(candidate));
    // }

    const evaluation = await this.evaluateLayout(baseLayout);
    this.fitness = [evaluation];
    this.pareto = [evaluation];
    this.generation = this.generations;
  }

  start(callback) {
    if (this.running) return;
    this.initialize();
    this.running = true;

    this.step()
      .catch((err) => {
        console.error(err);
      })
      .finally(() => {
        this.running = false;
        if (typeof callback === 'function') {
          callback(this);
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
      return;
    }
    if (format !== 'json') {
      console.warn('Only JSON export is supported in the simplified optimizer.');
      return;
    }
    const data = JSON.stringify(bestLayout, null, 2);
    this.download(data, 'layout.json', 'application/json');
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
