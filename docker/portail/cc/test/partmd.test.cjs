// Mini-rendu Markdown du slide Maintenance (partmd.js), sans navigateur :  node docker/portail/cc/test/partmd.test.cjs
const assert = require("assert");
const M = require("../partmd.js");
let n = 0; const t = (name, f) => { f(); n++; console.log("ok -", name); };

// Faux DOM minimal : assez pour verifier ce que render() construit, et surtout ce qu'il n'interprete PAS.
function fakeDoc() {
  const mk = (tag) => ({ tag, className: "", children: [], dataset: {}, attrs: {}, _text: null, _l: {},
    appendChild(c) { this.children.push(c); return c; }, setAttribute(k, v) { this.attrs[k] = v; },
    addEventListener(ev, f) { (this._l[ev] = this._l[ev] || []).push(f); },
    querySelector(sel) { const m = /data-anchor="([^"]+)"/.exec(sel); const find = (e) => { for (const c of e.children) { if (c.dataset && c.dataset.anchor === m[1]) return c; const r = find(c); if (r) return r; } return null; }; return find(this); },
    set textContent(v) { this._text = String(v); }, get textContent() { return this._text == null ? this.children.map((c) => c.textContent).join("") : this._text; },
    set innerHTML(v) { throw new Error("innerHTML interdit : " + v); } });
  return { createElement: mk, createTextNode: (v) => ({ tag: "#text", _text: String(v), get textContent() { return this._text; } }) };
}
const all = (e, acc = []) => { acc.push(e); (e.children || []).forEach((c) => all(c, acc)); return acc; };

t("titres, paragraphes, listes, code, regle", () => {
  const { nodes } = M.parse("# Titre\n\nUn **gras**, un *italique* et du `code`.\n\n- a\n- b\n\n1. un\n2. deux\n\n---\n\n```sh\ndocker restart x\n```\n");
  assert.deepStrictEqual(nodes.map((x) => x.t), ["h", "p", "ul", "ol", "hr", "pre"]);
  assert.strictEqual(nodes[2].items.length, 2); assert.strictEqual(nodes[5].v, "docker restart x"); assert.strictEqual(nodes[5].lang, "sh");
});
t("tableau simple", () => {
  const { nodes } = M.parse("| Part | Role |\n|---|---|\n| esp32 | carte |\n| imu | gyro |\n");
  assert.strictEqual(nodes[0].t, "table"); assert.strictEqual(nodes[0].head.length, 2); assert.strictEqual(nodes[0].rows.length, 2);
});
t("ancres : les titres ont un id stable et sans accent, les doublons sont numerotes", () => {
  const a = M.parse("# Résolutions\n## Résolutions\n").nodes.map((x) => x.id); assert.deepStrictEqual(a, ["resolutions", "resolutions-2"]);
});
t("liens : part:, ancre, https acceptes", () => {
  assert.deepStrictEqual(M.linkTarget("part:esp32_board"), { type: "part", id: "esp32_board" });
  assert.deepStrictEqual(M.linkTarget("#pannes-connues"), { type: "anchor", id: "pannes-connues" });
  assert.strictEqual(M.linkTarget("https://exemple.org/x?y=1").type, "url");
});
t("liens : tout le reste est REFUSE (javascript:, data:, //hote, file:, part: mal forme, caracteres de controle)", () => {
  ["javascript:alert(1)", "JaVaScRiPt:alert(1)", " javascript:alert(1)", "data:text/html,<script>1</script>", "//evil.example/x", "file:///etc/passwd", "vbscript:x",
    "part:../../etc", "part:Esp32", "part:", "#a b", "http://", "https://a.b/\u0000x", "ftp://x.y", ""].forEach((x) => assert.strictEqual(M.linkTarget(x), null, JSON.stringify(x)));
});
t("injection : du HTML dans le texte reste du TEXTE, rien n'est interprete", () => {
  const doc = fakeDoc(), root = M.render(doc, "# <img src=x onerror=alert(1)>\n\n<script>alert(1)</script> et [clic](javascript:alert(1)) et `<b>x</b>`\n");
  const txt = root.textContent;
  assert.ok(txt.includes("<img src=x onerror=alert(1)>") && txt.includes("<script>alert(1)</script>"), txt);
  assert.ok(!all(root).some((e) => e.tag === "img" || e.tag === "script"));
  assert.ok(!all(root).some((e) => e.tag === "a"), "un lien javascript: ne doit pas devenir un <a>");
  assert.ok(all(root).some((e) => e.className === "pm-badlink"));
});
t("un lien https s'ouvre dans un nouvel onglet avec noopener ; un lien part: appelle le rappel", () => {
  let got = null; const doc = fakeDoc(), root = M.render(doc, "[web](https://exemple.org) et [carte](part:esp32_board)", { onPart: (id) => { got = id; } });
  const links = all(root).filter((e) => e.tag === "a");
  assert.strictEqual(links.length, 2);
  assert.strictEqual(links[0].target, "_blank"); assert.strictEqual(links[0].rel, "noopener noreferrer"); assert.strictEqual(links[0].href, "https://exemple.org");
  links[1]._l.click[0]({ preventDefault() {} }); assert.strictEqual(got, "esp32_board");
});
t("echappements et blocs non fermes : pas de plantage, texte brut", () => {
  M.parse("**non ferme\n\n`code non ferme\n\n[lien](sans fin\n\n```\nbloc jamais ferme");
  assert.strictEqual(M.inline("\\*pas italique\\*")[0].v, "*pas italique*");
});
t("collectLinks : tous les liens du document, y compris tableaux, listes et citations", () => {
  const { nodes } = M.parse("[a](part:x)\n\n- [b](part:y)\n\n> [c](part:z)\n\n| t |\n|---|\n| [d](part:w) |\n");
  assert.deepStrictEqual(M.collectLinks(nodes).map((l) => l.raw).sort(), ["part:w", "part:x", "part:y", "part:z"]);
});
console.log(n + " tests passes");
