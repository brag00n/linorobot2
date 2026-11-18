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
  const reels = Object.keys(m.shapes).filter((id) => !m.virtual[id]);
  assert.strictEqual(reels.length, 17); assert.strictEqual(m.missing.length, 8);
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
t("les services logiciels n'ont pas de cote : modules virtuels ranges AU-DESSUS du RPi, marques estimes", () => {
  const sw = Object.keys(m.virtual); assert.ok(sw.includes("driver_esp32") && sw.includes("rosbridge") && sw.includes("navigation"));
  const rpi = m.shapes.rpi;
  sw.forEach((id) => { const s = m.shapes[id]; assert.strictEqual(s.virtuel, true); assert.strictEqual(s.estime, true); assert.ok(s.t[2] > rpi.t[2] + rpi.size[2] / 2, id); assert.strictEqual(m.byId[id].parent, "rpi"); });
  assert.ok(!m.virtual.manette && !m.shapes.manette);                       // la manette n'est pas logicielle : toujours « hors modele 3D »
});
t("eclatement : nul au repos, proportionnel a k, ecarte du centre moyen, souleve en niveau profond", () => {
  const c = [[0, 0, 0], [2, 0, 0], [0, 2, 0]];
  assert.deepStrictEqual(R.explodeOffsets(c, 2, 0, 0.1).map((o) => o.map((v) => Math.abs(v))), [[0, 0, 0], [0, 0, 0], [0, 0, 0]]);
  const full = R.explodeOffsets(c, 2, 1, 0.1), half = R.explodeOffsets(c, 2, 0.5, 0.1);
  full.forEach((o, i) => o.forEach((v, j) => near(half[i][j], v / 2)));
  assert.ok(full[1][0] > 0 && full[0][0] < 0);                                // le bloc de droite part a droite, celui de gauche a gauche
  assert.ok(full.every((o) => o[2] >= 0.1 - 1e-9 - Math.abs((c[0][2] - 0) )));
});
t("point dans un polygone (selection a la souris)", () => {
  const sq = [[0, 0], [4, 0], [4, 4], [0, 4]];
  assert.ok(R.pointInPoly([2, 2], sq)); assert.ok(!R.pointInPoly([5, 2], sq)); assert.ok(!R.pointInPoly([2, -1], sq));
});

t("selection a la souris : la plus petite piece sous le curseur gagne (le chassis recouvre ce qu'il contient)", () => {
  const big = { id: "base_link", poly: [[0, 0], [100, 0], [100, 100], [0, 100]], cx: 50, cy: 50, area: 10000 };
  const small = { id: "rpi", poly: [[40, 40], [60, 40], [60, 60], [40, 60]], cx: 50, cy: 50, area: 400 };
  assert.strictEqual(R.pickHit([big, small], [50, 50]).id, "rpi");
  assert.strictEqual(R.pickHit([small, big], [50, 50]).id, "rpi");
  assert.strictEqual(R.pickHit([big, small], [10, 10]).id, "base_link");           // hors de la petite piece : le chassis
  assert.strictEqual(R.pickHit([big, small], [200, 200]), null);                    // dans le vide
  assert.strictEqual(R.polyArea([[0, 0], [4, 0], [4, 3], [0, 3]]), 12);
});
t("coque : seul le chassis (grande boite) est translucide, pas une carte ni une roue", () => {
  assert.strictEqual(R.isShell({ kind: "box", size: [0.245, 0.23, 0.08] }), true);
  assert.strictEqual(R.isShell({ kind: "box", size: [0.085, 0.06, 0.012] }), false);
  assert.strictEqual(R.isShell({ kind: "cyl", r: 0.04, h: 0.04 }), false);
});
t("webcam : posee sur le haut du chassis (pas flottante), devant le lidar et dans l'emprise du chassis", () => {
  const sh = R.parseUrdf ? null : null;                                     // parseUrdf exige DOMParser : la pose est verifiee dans le test navigateur (r3d_int)
  const x = fx("robot_description.xml"), m = /<link name="camera_link">[\s\S]*?<origin rpy="0 0 0" xyz="([-\d. ]+)"\/>\s*<\/joint>/.exec(x.replace(/<link name="camera_link">/, '<link name="camera_link">'));
  const j = /<joint name="camera_to_base_link"[\s\S]*?<origin rpy="[^"]*" xyz="([-\d. ]+)"/.exec(x);
  assert.ok(j, "joint camera");
  const [cx, cy, cz] = j[1].trim().split(/\s+/).map(Number);
  assert.ok(cz <= 0.04 + 0.015 + 1e-9 && cz >= 0.04, "la webcam doit reposer sur le haut du chassis (z = 0,04 + demi-cube), pas flotter : z=" + cz);
  assert.ok(cx > 0.0775 && cx + 0.015 <= 0.1225 + 1e-9, "devant le lidar et dans l'emprise du chassis : x=" + cx);
});
t("via : la manette (sans forme) allume l'adaptateur Bluetooth, donc le RPi ; pas la carte ESP32", () => {
  const mv = R.buildModel(parts, "");
  assert.strictEqual(mv.via.manette, "adaptateur_bluetooth");
  const d = R.directStates(inc([mk("manette", "WARN", [], ["bamboo_control"])]), lv, mv.via);
  assert.strictEqual(d.manette.role, "root"); assert.strictEqual(d.adaptateur_bluetooth.role, "root");
  const s = R.levelState("rpi", mv.kids, d); assert.strictEqual(s.role, "root"); assert.strictEqual(s.via, "adaptateur_bluetooth");
  assert.strictEqual(R.levelState("esp32_board", mv.kids, d), null);
  assert.strictEqual(R.directStates(inc([mk("manette", "WARN")]), lv).adaptateur_bluetooth, undefined);       // sans via : inchange
});
console.log(n + " tests passes");
