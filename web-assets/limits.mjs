// Giới hạn ghép M1 cho sim three.js: bản JS của spec.elbow_max / spec.wrist_max / limits.clamp. Đọc bảng từ joints.json (couplings).
// Dùng: import { elbowMax, wristMax, clampArm } from './limits.mjs'; const J = await (await fetch('joints.json')).json();
// Mọi góc ở radian (như SDK); bảng trong joints.json ở độ.
const rad = (d) => (d * Math.PI) / 180, deg = (r) => (r * 180) / Math.PI;
const EPS = 1e-6; // độ: dung sai nút bảng (an toàn nếu giới hạn bị làm tròn)
function bracket(nodes, x) {
  x = nodes.reduce((a, n) => (Math.abs(a - n) < EPS ? n : a), x);
  if (x <= nodes[0]) return [0, 0];
  const n = nodes.length;
  if (x >= nodes[n - 1]) return [n - 1, n - 1];
  for (let i = 0; i < n - 1; i++) if (nodes[i] <= x && x <= nodes[i + 1]) return [i, i + 1];
  return [n - 1, n - 1];
}
export function elbowMax(J, qs) {
  const { shoulder_deg: xs, max_deg: ys } = J.couplings.elbow_max, d = deg(qs);
  let v;
  if (d <= xs[0]) v = ys[0];
  else if (d >= xs[xs.length - 1]) v = ys[ys.length - 1];
  else { let i = 0; for (let k = 0; k < xs.length - 1; k++) if (xs[k] <= d) i = k; v = ys[i] + ((ys[i + 1] - ys[i]) * (d - xs[i])) / (xs[i + 1] - xs[i]); }
  return Math.min(rad(v), J.joints.left_elbow.limit_soft[1]);
}
export function wristMax(J, qs, qe) {
  const { shoulder_deg: S, elbow_deg: E, max_deg: M } = J.couplings.wrist_max;
  const [i0, i1] = bracket(S, deg(qs)), [j0, j1] = bracket(E, deg(qe));
  const m = Math.min(M[i0][j0], M[i0][j1], M[i1][j0], M[i1][j1]);
  return Math.min(rad(m), J.joints.left_wrist_flex.limit_soft[1]);
}
// Kẹp một tay: q = {pan, shoulder, elbow, wrist_flex, wrist_roll} (rad). Kẹp khoảng khớp, rồi khuỷu theo vai, rồi cổ tay theo (vai, khuỷu).
export function clampArm(J, q) {
  const o = {}, c = (k) => J.joints['left_' + k].limit_soft;
  for (const k of ['pan', 'shoulder', 'elbow', 'wrist_flex', 'wrist_roll']) o[k] = Math.min(Math.max(q[k], c(k)[0]), c(k)[1]);
  o.elbow = Math.min(o.elbow, elbowMax(J, o.shoulder));
  o.wrist_flex = Math.min(o.wrist_flex, wristMax(J, o.shoulder, o.elbow));
  return o;
}
