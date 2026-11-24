// So limits.mjs với đáp án Python. Dùng: node 05_cmp.mjs cases.json joints.json  (thoát 1 nếu lệch > 1e-6 rad)
import fs from 'fs'; import { elbowMax, wristMax, clampArm } from '../limits.mjs';
const C = JSON.parse(fs.readFileSync(process.argv[2])), J = JSON.parse(fs.readFileSync(process.argv[3]));
let m = [0, 0, 0, 0], bad = 0;
for (const c of C) {
  const a = Math.abs(elbowMax(J, c.s) - c.es), b = Math.abs(wristMax(J, c.s, c.e) - c.wm);
  const o = clampArm(J, { pan: 0, shoulder: c.s, elbow: c.e, wrist_flex: c.w, wrist_roll: 0 });
  const q = c.clamp, d1 = Math.abs(o.shoulder - q.left_shoulder), d2 = Math.abs(o.elbow - q.left_elbow), d3 = Math.abs(o.wrist_flex - q.left_wrist_flex);
  m = [Math.max(m[0], a), Math.max(m[1], b), Math.max(m[2], d1, d2), Math.max(m[3], d3)];
  if (a > 1e-6 || b > 1e-6 || d1 > 1e-6 || d2 > 1e-6 || d3 > 1e-6) bad++;
}
console.log(JSON.stringify({ diem: C.length, lech_max_rad: { elbowMax: m[0], wristMax: m[1], clamp_vai_khuyu: m[2], clamp_cotay: m[3] }, so_diem_lech: bad }));
process.exit(bad ? 1 : 0);
