// run_tests.mjs — the numeric tests of tests.html, run headless with deno (there is no node on this Mac):
//   deno run --allow-read IRE_Web/assets/run_tests.mjs
await import(new URL("./robot.js", import.meta.url).href);
await import(new URL("./ch1.js", import.meta.url).href);
await import(new URL("./ch2.js", import.meta.url).href);
await import(new URL("./ch3.js", import.meta.url).href);
await import(new URL("./ch4.js", import.meta.url).href);
/* the scene files too: the tip readout of the simulations is tested against the kinematics */
for (const m of ["scene3d.js", "ch1_3d.js", "ch2_3d.js", "ch3_3d.js"]) await import(new URL("./" + m, import.meta.url).href);
await import(new URL("./tests.js", import.meta.url).href);
let n = 0, bad = 0;
globalThis.RBTESTS((name, got, exp, tol = 1e-6) => { n++; const ok = typeof exp === "string" ? String(got) === exp : Math.abs(+got - exp) <= tol;
  if (!ok) { bad++; console.log(`FAIL ${n} ${name}: got ${got}, expected ${exp}`); } });
console.log(bad ? `${bad} of ${n} FAILED` : `ALL PASS (${n} tests)`);
if (bad) Deno.exit(1);
