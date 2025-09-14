import { computeWorkspace } from '../../workspace.js';
const Quaternion = { fromEuler: () => ({ rotateVector: v => v }) };
const platform = { update(){}, computeAngles(){ return [0,0,0,0,0,0]; } };
const ranges = {
  x:{min:0,max:0,step:1},
  y:{min:0,max:0,step:1},
  z:{min:0,max:0,step:1},
  rx:{min:0,max:0,step:1},
  ry:{min:0,max:0,step:1},
  rz:{min:0,max:0,step:1}
};
const result = await computeWorkspace(platform, ranges, { Quaternion, ballJointClamp:false });
console.log(JSON.stringify(result));
