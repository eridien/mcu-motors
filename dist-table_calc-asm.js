
/*  
  node /root/dev/p3/mcu-motors/dist-table_calc-asm.js
*/

fs = require('fs');

// js utility to calculate deceleration distance table
// and prepare the .asm file for inclusion in the mcu-motors mcu code

// table is 8 (accel) by 256 (speed)
// entry is dist of decel in 1/8 steps

// accel steps/sec/sec (assuming 40 steps/mm): 
//     1500, 1250, 1000, 800, 600, 400, 200, 0 (off)
const accelTab = [0, 8000, 16000, 24000, 32000, 40000, 50000, 60000];

// speed resolution of 3.2 mm/sec (128/40)  (assuming 40 steps/mm)
// 256 speed values, 128 delta, (6.4 to 819.2 mm/sec)
// up to 32,767 steps/sec (4 kHz pps)

let file = fs.openSync('/root/dev/p3/mcu-motors/disttable.asm', 'w');

fs.writeSync(file, `
PSECT disttblsect,class=CODE,local,delta=2
GLOBAL _disttable
_disttable:
`, 'w');

const ustep = speed => {
  if (speed >= (8192 + 4096) / 2) return 1;
  if (speed >= (4096 + 2048) / 2) return 2;
  if (speed >= (2048 + 1024) / 2) return 4;
  else return 8;
}

let tooBigCount = 0;
for (let accelIdx = 0; accelIdx < 8; accelIdx++) {
  let accel = accelTab[accelIdx];
  for (let speedIdx = 0; speedIdx < 256; speedIdx++) {
    let speed = speedIdx * 256;
    let dist = 0;
    for (; speed > 0; 
          speed -= Math.max( Math.floor(accel / speed), 1), 
          dist += ustep(speed));
    if(dist >= 0x4000) {
      dist = 0x3fff;
      tooBigCount++;
    }
    let val = dist.toString(16);
    while (val.length < 4) val = '0' + val;
    fs.writeSync(file, `DW 0X${val}  ; ${dist}, ${accel}, ${speedIdx * 256}\n`);
  }
}
fs.closeSync(file);
console.log('tooBigCount:', tooBigCount);
