import init, { optimize_demo } from './stewart_sim/pkg';

export async function optimize() {
  await init();
  return optimize_demo();
}
