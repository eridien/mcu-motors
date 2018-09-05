
// js utility to calculate decelerations distance table
// and prepare the .asm file for inclusion in the mcu-motors mcu code

// table is 4 (ustep) by 8 (accel) by 64 (speed)
// entry is dist of decel

const CLK_TICKS_PER_SEC = 25000;

// stepsPerPulse = 1,2,4,8, (ustep 0..3)
// accel: 1500, 1250, 1000, 800, 600, 400, 200, 100
const accelTab = [4000, 8000, 16000, 24000, 32000, 40000, 50000, 60000];
// speed resolution of 5 mm/sec
// 64 speed values, 200 delta, (1..64)*200 (5 to 320 mm/sec)

let line = 0;
for (let stepsPP = 8; stepsPP > 0; stepsPP >>= 1) {
  for (let accelIdx = 0; accelIdx < 8; accelIdx++) {
    let accel = accelTab[accelIdx];
    for (let speedIdx = 1; speedIdx <= 64; speedIdx++) {
      let dist = 0;
      for (let speed = speedIdx * 200; speed > 0; 
           speed -= Math.max(Math.floor(accel / speed),1), 
           dist += stepsPP);
      console.log(line++, stepsPP, accel, speedIdx * 200, dist);
    }
  }
}
