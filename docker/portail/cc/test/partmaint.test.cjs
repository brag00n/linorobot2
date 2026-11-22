// Logique pure du slide Maintenance (partmaint.js, partmaint.catalog.js) + coherence des fiches docs/parts/*.md :
//   node docker/portail/cc/test/partmaint.test.cjs
const assert = require("assert"), fs = require("fs"), path = require("path");
const PM = require("../partmaint.js")._t, CAT = require("../partmaint.catalog.js"), MD = require("../partmd.js");
const parts = JSON.parse(fs.readFileSync(path.join(__dirname, "fixtures", "parts.json"), "utf8")).parts;
const DOCS = path.join(__dirname, "..", "..", "..", "..", "docs", "parts");
const ids = new Set(parts.map((p) => p.id));
let n = 0; const t = (name, f) => { f(); n++; console.log("ok -", name); };

t("fil d'Ariane : de la racine a la part", () => {
  assert.deepStrictEqual(PM.parentChain(parts, "tb6612fng").map((p) => p.id), ["robot", "esp32_board", "tb6612fng"]);
  assert.deepStrictEqual(PM.parentChain(parts, "inconnue"), []);
});
t("valeurs numeriques reconnues, texte ignore (pour les courbes)", () => {
  assert.strictEqual(PM.numeric("12.14"), 12.14); assert.strictEqual(PM.numeric(0.52), 0.52); assert.strictEqual(PM.numeric("-4.4"), -4.4);
  ["branche", "?", "v 0.000 w 0.000", "caoutchouc_parquet", "/dev/esp32", "0.5 / 0"].forEach((v) => assert.ok(!isFinite(PM.numeric(v)), v));
});
t("historique : borne a 180 points, les plus anciens partent", () => {
  const h = {}; for (let i = 0; i < 400; i++) PM.push(h, "k", i, i); assert.strictEqual(h.k.length, 180); assert.strictEqual(h.k[0].v, 220);
});
t("une tache de diagnostic n'est listee qu'une fois par part", () => {
  assert.deepStrictEqual(PM.dedupeDiag({ diag: ["a", "b", "a"] }), ["a", "b"]); assert.deepStrictEqual(PM.dedupeDiag(null), []);
});
t("conditions : liaison saine et traction desarmee ; etat inconnu = refuse", () => {
  const a = { requiert: ["arret", "liaison"] };
  assert.ok(PM.allowed(a, { link: true, armed: false }).ok);
  assert.ok(!PM.allowed(a, { link: false, armed: false }).ok);
  assert.ok(!PM.allowed(a, { link: true, armed: true }).ok);
  assert.ok(!PM.allowed(a, { link: true, armed: null }).ok, "etat de la traction inconnu : on refuse");
  assert.ok(PM.allowed({ requiert: [] }, { link: false, armed: true }).ok);
  assert.match(PM.allowed(a, { link: true, armed: true }).why, /armée/);
});
t("popup : requise pour les risques moyen et eleve, jamais pour faible ni lecture", () => {
  assert.ok(PM.needsConfirm({ risque: "moyen" })); assert.ok(PM.needsConfirm({ risque: "eleve" }));
  assert.ok(!PM.needsConfirm({ risque: "faible" })); assert.ok(!PM.needsConfirm({ risque: "eleve", lecture: true }));
});
t("appui maintenu : 3 s exactes, jamais avant", () => {
  assert.strictEqual(PM.HOLD_MS, 3000);
  assert.strictEqual(PM.holdProgress(0, 0), 0); assert.strictEqual(PM.holdProgress(0, 1500), 0.5); assert.ok(PM.holdProgress(0, 2999) < 1); assert.strictEqual(PM.holdProgress(0, 3000), 1); assert.strictEqual(PM.holdProgress(0, 9000), 1);
  assert.strictEqual(PM.holdProgress(100, 50), 0);
});

// ---------------------------------------------------------------- catalogue
t("catalogue : chaque part citee existe dans la nomenclature", () => {
  Object.keys(CAT.actions).concat(Object.keys(CAT.metrics)).forEach((id) => assert.ok(ids.has(id), "part inconnue du catalogue : " + id));
});
t("catalogue : champs obligatoires, risque et conditions valides, appels aux services de la liste blanche", () => {
  const wl = fs.readFileSync(path.join(__dirname, "..", "..", "..", "rosbridge", "start.sh"), "utf8");
  const svc = (wl.match(/SERVICES="([\s\S]*?)"/) || [, ""])[1].replace(/\\\n/g, "").split(",").map((x) => x.trim());
  const ok = (s) => svc.some((w) => w === s || (w.endsWith("/*") && s.startsWith(w.slice(0, -1))));
  Object.keys(CAT.actions).forEach((pid) => CAT.actions[pid].forEach((a) => {
    ["id", "label", "service", "type", "args", "risque", "decrit", "effet", "requiert"].forEach((k) => assert.ok(a[k] != null && a[k] !== "", pid + "/" + a.id + " : " + k));
    assert.ok(CAT.RISQUES.includes(a.risque), a.id); a.requiert.forEach((r) => assert.ok(CAT.REQUIERT.includes(r), a.id + " : " + r));
    assert.ok(ok(a.service), pid + "/" + a.id + " : " + a.service + " absent de la liste blanche rosbridge (docker/rosbridge/start.sh)");
  }));
});
t("catalogue : une action qui peut faire rouler le robot est de risque ELEVE, donc avec popup", () => {
  const roule = (a) => /^\/cc\/guard\/arm/.test(a.service) && a.args.data === true;
  Object.values(CAT.actions).flat().filter(roule).forEach((a) => assert.strictEqual(a.risque, "eleve", a.id));
  const pile = Object.values(CAT.actions).flat().filter((a) => a.service === "/bamboo/nav_stack");
  assert.ok(pile.length && pile.every((a) => PM.needsConfirm(a)), "demarrer/arreter la pile exige la popup");
});
t("catalogue : la recalibration de l'IMU exige la traction desarmee", () => {
  const a = CAT.actions.imu_link.find((x) => x.id === "recalibrer_gyro"); assert.ok(a.requiert.includes("arret")); assert.ok(PM.needsConfirm(a));
});
t("catalogue : aucune action ne peut ecrire un parametre arbitraire ni commander les moteurs", () => {
  Object.values(CAT.actions).flat().forEach((a) => { assert.ok(!/set_parameters/.test(a.service), a.id); assert.ok(!/cmd_vel/.test(a.service), a.id); });
});

// ---------------------------------------------------------------- fiches Markdown docs/parts/*.md
const files = fs.existsSync(DOCS) ? fs.readdirSync(DOCS).filter((f) => f.endsWith(".md")) : [];
t("fiches : le dossier docs/parts contient des fiches", () => assert.ok(files.length >= 5, files.length + " fiche(s)"));
t("fiches : chaque fichier porte l'identifiant d'une part existante et un titre de niveau 1", () => {
  files.forEach((f) => { assert.ok(ids.has(f.replace(/\.md$/, "")), "fiche sans part : " + f); assert.match(fs.readFileSync(path.join(DOCS, f), "utf8"), /^# \S/, f + " : titre manquant"); });
});
t("fiches : tous les liens part:<id> visent une part existante, aucun lien refuse", () => {
  files.forEach((f) => {
    const links = MD.collectLinks(MD.parse(fs.readFileSync(path.join(DOCS, f), "utf8")).nodes);
    links.forEach((l) => { assert.ok(l.target, f + " : lien refuse " + l.raw); if (l.target.type === "part") assert.ok(ids.has(l.target.id), f + " : part inconnue " + l.target.id); });
  });
});
t("fiche generee : toute part sans fiche a un repli lisible avec ses liens", () => {
  parts.filter((p) => !files.includes(p.id + ".md")).forEach((p) => {
    const md = PM.genericDoc(p, parts); assert.match(md, new RegExp("^# ")); MD.collectLinks(MD.parse(md).nodes).forEach((l) => { assert.ok(l.target && (l.target.type !== "part" || ids.has(l.target.id)), p.id + " : " + l.raw); });
  });
});
t("resolutions d'une part : seulement celles de l'incident dont elle est la racine", () => {
  const inc = { incidents: [{ racine: "laser", resolution: ["a", "b"] }, { racine: "manette", resolution: ["c"] }] };
  assert.deepStrictEqual(PM.resolutionsFor(inc, "laser"), ["a", "b"]); assert.deepStrictEqual(PM.resolutionsFor(inc, "rpi"), []); assert.deepStrictEqual(PM.resolutionsFor(null, "laser"), []);
});
console.log(n + " tests passes");
