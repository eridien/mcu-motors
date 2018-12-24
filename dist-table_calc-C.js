
/*  
  node /root/dev/p3/mcu-motors/dist-table_calc-C.js
*/

fs = require('fs');

// js utility to calculate deceleration distance table
// and prepare data for the C file for inclusion in dist-table.c

// table is 8 (accel) by 256 (speed), 2048 words
// entry is dist of decel in 1/8 steps

// for 1/40 mm steps: none, 100, 200, 500, 1000, 2000, 5000, 10000 mm/sec/sec
const accelTab = [0, 4000, 8000, 20000, 40000, 80000, 200000, 400000];

// speed resolution of 3.2 mm/sec (128/40)  (assuming 40 steps/mm)
// 256 speed values, 128 delta, (6.4 to 819.2 mm/sec)
// up to 32,767 steps/sec (4 kHz pps)

let file = fs.openSync('/root/dev/p3/mcu-motors/dist-table-data.c', 'w');

fs.writeSync(file, "disttable[2048] = {\n", 'w');

const uStepPhaseMask = [0x07, 0x03, 0x01, 0x00];
const uStepDist = [8, 4, 2, 1];

let curUstep = 0;
let phase = 0;

const ustep = speed => {
  let tgtUstep;
  if (speed > (8192 + 4096) / 2) tgtUstep = 0;
  else if (speed > (4096 + 2048) / 2) tgtUstep = 1;
  else if (speed > (2048 + 1024) / 2) tgtUstep = 2;
  else tgtUstep = 3;
  if (tgtUstep != curUstep) {
    // you can only change ustep when the drv8825 phase is correct
    if ((phase & uStepPhaseMask[tgtUstep]) == 0) {
      curUstep = tgtUstep;
    }
  }
  return uStepDist[curUstep];
}

let tooBigCount = 0;
for (let accelIdx = 0; accelIdx < 8; accelIdx++) {
  let accel = accelTab[accelIdx];
  for (let speedIdx = 0; speedIdx < 256; speedIdx++) {
    let speed = speedIdx * 128;
    curUstep = 0;
    phase = 0;
    dist = 0;
    if (speedIdx > 0) {
      while (speed > 0) {
        speed -= Math.max(Math.floor(accel / speed), 1);
        let stepSize = ustep(speed);
        dist += stepSize;
        phase = (phase + stepSize) % 8;
      };
    }
    if (dist > 0xffff) {
      dist = 0xffff;
      tooBigCount++;
    }
    fs.writeSync(file, dist + ',\n');
  }
}
fs.writeSync(file, "};\n", 'w');
fs.closeSync(file);
console.log('tooBigCount:', tooBigCount);
