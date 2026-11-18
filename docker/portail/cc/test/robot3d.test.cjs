// Logique pure de robot3d.js (sans navigateur) :  node docker/portail/cc/test/robot3d.test.cjs
const assert = require("assert");
const R = require("../robot3d.js")._t;
const near = (a, b, e = 1e-9) => assert.ok(Math.abs(a - b) < e, a + " != " + b);
const fx = (n) => require("fs").readFileSync(require("path").join(__dirname, "fixtures", n), "utf8");
const parts = JSON.parse(fx("parts.json"));
let n = 0; const t = (name, f) => { f(); n++; console.log("ok -", name); };

t("rotation URDF : lacet de 90 deg envoie x sur y, roulis de 90 deg envoie y sur z", () => {
  const a = R.apply({ R: R.rotRpy(0, 0, Math.PI / 2), t: [0, 0, 0] }, [1, 0, 0]); near(a[0], 0); near(a[1], 1);
  const b = R.apply({ R: R.rotRpy(Math.PI / 2, 0, 0), t: [0, 0, 0] }, [0, 1, 0]); near(b[1], 0); near(b[2], 1);
});
t("composition : A o B applique B puis A", () => {
  const A = { R: R.rotRpy(0, 0, Math.PI / 2), t: [1, 0, 0] }, B = { R: R.rotRpy(0, 0, 0), t: [1, 0, 0] };
  const p = R.apply(R.compose(A, B), [0, 0, 0]); near(p[0], 1); near(p[1], 1);
});
t("boite : 8 coins, 12 aretes ; grand chassis maille", () => {
  const e = R.edgesOf({ kind: "box", size: [0.02, 0.02, 0.02], R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] }); assert.strictEqual(e.lines.length, 12); assert.strictEqual(e.pts.length, 8);
  const g = R.edgesOf({ kind: "box", size: [0.245, 0.23, 0.08], R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] }); assert.ok(g.lines.length > 12);
});
t("disque : les rayons ne sortent jamais du tableau (regression i + n/2)", () => {
  const e = R.edgesOf({ kind: "cyl", r: 0.04, h: 0.0425, R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] });
  e.lines.forEach((l) => l.forEach((p) => assert.ok(Array.isArray(p) && p.length === 3 && p.every(isFinite))));
});
t("enveloppe convexe d'un carre avec un point interieur", () => {
  assert.strictEqual(R.hull2([[0, 0], [2, 0], [2, 2], [0, 2], [1, 1]]).length, 4);
});
t("un visuel dessine a la main devient une forme, estimee par defaut ; un lien urdf n'en est pas une", () => {
  assert.strictEqual(R.shapeFromVisuel({ forme: "boite", pos: [0, 0, 0], taille: [1, 2, 3] }).estime, true);
  assert.strictEqual(R.shapeFromVisuel({ forme: "boite", pos: [0, 0, 0], taille: [1, 2, 3], estime: false }).estime, false);
  assert.strictEqual(R.shapeFromVisuel({ urdf: "laser" }), null);
});
t("modele reel sans URDF : seuls les blocs dessines existent, les liens URDF sont signales manquants", () => {
  const m = R.buildModel(parts, ""); assert.strictEqual(m.root, "robot"); assert.strictEqual(m.hasUrdf, false);
  assert.strictEqual(Object.keys(m.shapes).length, 17); assert.strictEqual(m.missing.length, 8);
});
const m = R.buildModel(parts, "");
const lv = (x) => (x.niveau === "ERROR" ? 2 : 1);
const inc = (l) => ({ incidents: l });
const mk = (racine, niveau, cons, impact) => ({ racine, niveau, consequences: cons || [], impact: impact || [] });
t("racine > consequence > impact ; le pire gagne", () => {
  const d = R.directStates(inc([mk("rosbridge", "ERROR", ["bamboo_control"], ["bamboo_control", "navigation"]), mk("manette", "WARN", [], ["bamboo_control"])]), lv);
  assert.strictEqual(d.rosbridge.role, "root"); assert.strictEqual(d.bamboo_control.role, "cons"); assert.strictEqual(d.navigation.role, "impact");
});
t("l'incident d'un sous-composant colore le bloc du niveau ; l'impact ne remonte pas", () => {
  const d = R.directStates(inc([mk("tb6612fng", "WARN", [], ["motor_fl"])]), lv);
  const s = R.levelState("esp32_board", m.kids, d); assert.strictEqual(s.role, "root"); assert.strictEqual(s.via, "tb6612fng");
  assert.strictEqual(R.levelState("rpi", m.kids, R.directStates(inc([mk("laser", "ERROR", [], ["navigation"])]), lv)), null);
});
t("un logiciel en defaut remonte au RPi, pas a la carte ESP32", () => {
  const d = R.directStates(inc([mk("rosbridge", "ERROR", [], [])]), lv);
  assert.strictEqual(R.levelState("rpi", m.kids, d).via, "rosbridge"); assert.strictEqual(R.levelState("esp32_board", m.kids, d), null);
});
t("ancetre de niveau : le bloc du niveau qui contient la part", () => {
  assert.strictEqual(R.levelAncestor(m, "tb6612fng", "robot"), "esp32_board"); assert.strictEqual(R.levelAncestor(m, "tb6612fng", "esp32_board"), "tb6612fng");
  assert.strictEqual(R.levelAncestor(m, "rpi", "esp32_board"), null);
});
console.log(n + " tests passes");
