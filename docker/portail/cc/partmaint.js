/* partmaint.js : slide GAUCHE « Maintenance d'une Part » du CC (2026-11-22). MODULE OPTIONNEL : retirer ce fichier, partmd.js, partmaint.catalog.js et les lignes
   `window.PartMaint` d'index.html suffit a le supprimer ; le reste du CC ne le connait pas.

   Une part est au centre. Deux zones, de haut en bas :
     - DIAGNOSTIC ET PILOTAGE (defilement HORIZONTAL) : une carte par tache de diagnostic de la part (valeurs en direct + mini-courbes), les metriques propres
       (batterie, IMU, RPi), puis les ACTIONS du catalogue ;
     - DOCUMENTATION puis RESOLUTIONS (defilement VERTICAL) : la fiche Markdown docs/parts/<id>.md (mini-rendu partmd.js), puis les pistes de `resolution`
       de parts.yaml telles que publiees dans /bamboo/incidents.

   SECURITE. Toute action RISQUEE (moyen, eleve) passe par une popup qui decrit le risque ; on valide par un APPUI MAINTENU de 3 s, ou on annule. Echap ne ferme
   pas la popup (Echap = DESARMER dans le CC). La page n'est jamais la seule barriere : le robot refuse aussi. Aucun texte venu du robot ou d'un .md n'est
   interprete comme du HTML (textContent partout). */
(function (root) {
  "use strict";

  var HOLD_MS = 3000, HIST_N = 180, HIST_EVERY_MS = 1000;
  var ctx = null, dlg = null, state = { id: null, section: "doc", open: false }, hist = {}, histT = 0, cache = {}, lastRes = {}, hold = { raf: 0, t0: 0 }, pop = null, gateT = 0;

  /* ---------------------------------------------------------------- utilitaires purs (testes sous Node) */
  function parentChain(parts, id) {
    var by = {}, out = [], g = 0; (parts || []).forEach(function (p) { by[p.id] = p; });
    for (var c = by[id]; c && g++ < 32; c = by[c.parent]) out.unshift(c);
    return out;
  }
  function numeric(v) { var n = typeof v === "number" ? v : parseFloat(String(v).replace(",", ".")); return isFinite(n) && /^-?[\d.,]+\s*[%A-Za-z°/]*$/.test(String(v).trim()) ? n : NaN; }
  function push(histo, key, v, now) {
    var a = histo[key] || (histo[key] = []);
    a.push({ t: now, v: v }); if (a.length > HIST_N) a.shift();
    return a;
  }
  function dedupeDiag(part) { var seen = {}; return (part && part.diag || []).filter(function (d) { return seen[d] ? false : (seen[d] = true); }); }
  /* Conditions d'une action -> {ok, why}. `armed` : traction armee ; `link` : liaison CC saine. */
  function allowed(action, env) {
    var req = action.requiert || [], why = [];
    if (req.indexOf("liaison") >= 0 && !env.link) why.push(env.linkWhy || "liaison CC ou vidéo non saine");
    if (req.indexOf("arret") >= 0) { if (env.armed == null) why.push("état de la traction inconnu"); else if (env.armed) why.push("traction armée : désarmer d'abord"); }
    return { ok: why.length === 0, why: why.join(" · ") };
  }
  function needsConfirm(action) { return !action.lecture && action.risque !== "faible"; }
  function holdProgress(t0, now, ms) { return Math.max(0, Math.min(1, (now - t0) / (ms || HOLD_MS))); }
  function resolutionsFor(inc, id) {
    var out = [];
    ((inc && inc.incidents) || []).forEach(function (x) { if (x.racine === id) out = out.concat(x.resolution || []); });
    return out;
  }
  function genericDoc(part, parts) {
    if (!part) return "# Part inconnue\n";
    var by = {}; (parts || []).forEach(function (p) { by[p.id] = p; });
    var L = ["# " + part.label, "", "*Fiche générée : aucune fiche rédigée dans `docs/parts/" + part.id + ".md`.*", "", "| | |", "|---|---|", "| Identifiant | `" + part.id + "` |", "| Type | " + part.type + " |",
      "| Criticité | " + part.criticite + " |", "| Certitude | " + (part.certitude || "?") + " |"];
    if (part.parent && by[part.parent]) L.push("| Appartient à | [" + by[part.parent].label + "](part:" + part.parent + ") |");
    if (part.depends_on && part.depends_on.length) L.push("| Dépend de | " + part.depends_on.map(function (d) { return "[" + ((by[d] || {}).label || d) + "](part:" + d + ")"; }).join(", ") + " |");
    var kids = (parts || []).filter(function (p) { return p.parent === part.id; });
    if (kids.length) L.push("| Contient | " + kids.map(function (k) { return "[" + k.label + "](part:" + k.id + ")"; }).join(", ") + " |");
    return L.join("\n") + "\n";
  }

  /* ---------------------------------------------------------------- DOM */
  var h = function (tag, cls, txt) { var e = document.createElement(tag); if (cls) e.className = cls; if (txt != null) e.textContent = txt; return e; };
  var $ = function (s, r) { return (r || document).querySelector(s); };

  function ensureDialog() {
    if (dlg) return dlg;
    dlg = document.createElement("dialog"); dlg.id = "dlgM"; dlg.className = "slide slide-left"; dlg.setAttribute("aria-label", "Maintenance d'une part");
    document.body.appendChild(dlg);
    dlg.addEventListener("close", function () { state.open = false; stopHold(); closePopup(); document.documentElement.classList.remove("pm-split"); if (ctx && ctx.onClose) ctx.onClose(); });
    window.addEventListener("resize", function () { if (state.open) bounds(); });
    return dlg;
  }
  function bounds() {
    var z = parseFloat(getComputedStyle(document.documentElement).getPropertyValue("--z")) || 1, hd = document.querySelector(".hdr"), ft = document.querySelector(".foot"), gap = 8 / z;
    dlg.style.setProperty("--jt", (hd ? hd.getBoundingClientRect().bottom / z + gap : 0) + "px");
    dlg.style.setProperty("--jb", (ft ? (innerHeight - ft.getBoundingClientRect().top) / z + gap : 0) + "px");
  }

  function levelOf(part) { var i = ctx.inc && ctx.inc(); var lv = 0; ((i && i.incidents) || []).forEach(function (x) { if (x.racine === part.id || (x.consequences || []).indexOf(part.id) >= 0) lv = Math.max(lv, ctx.incLv(x)); }); return lv; }

  function render() {
    var parts = (ctx.parts() || {}).parts || [], part = parts.filter(function (p) { return p.id === state.id; })[0];
    dlg.replaceChildren();
    var head = h("div", "pm-head"), body = h("div", "pm-body");
    if (!part) { head.appendChild(h("div", "pm-title", "Maintenance")); body.appendChild(h("p", "pm-empty", state.id ? "Part inconnue : " + state.id : "En attente de /bamboo/parts (nœud incident_node absent ?).")); dlg.append(head, body, foot()); return; }
    var lv = levelOf(part), top = h("div", "pm-tag"), chain = h("div", "pm-crumb");
    top.appendChild(h("span", "micro", "Maintenance // part")); top.appendChild(h("i", "pm-dot")); top.firstChild.nextSibling.dataset.lv = lv;
    parentChain(parts, part.id).forEach(function (p, i, a) {
      var last = i === a.length - 1, b = h("button", "pm-c" + (last ? " cur" : ""), shortLabel(p.label)); b.type = "button"; b.title = p.id;
      if (!last) b.onclick = function () { api.open(p.id); }; else b.disabled = true; chain.appendChild(b); if (!last) chain.appendChild(h("span", "pm-sep", "›"));
    });
    var title = h("h3", "pm-title", part.label), meta = h("div", "pm-meta", part.type + " · criticité " + part.criticite + " · certitude " + (part.certitude || "?") + (lv ? " · " + (lv === 2 ? "ERREUR" : "AVERTISSEMENT") : ""));
    title.dataset.lv = lv; head.append(top, chain, title, meta);
    var seg = h("div", "pm-seg"); [["diag", "Diagnostic"], ["doc", "Documentation"], ["res", "Résolutions"]].forEach(function (s) {
      var b = h("button", "pm-s", s[1]); b.type = "button"; b.setAttribute("aria-pressed", String(s[0] === "diag" ? false : false)); b.onclick = function () { scrollTo(s[0]); }; seg.appendChild(b); seg.lastChild.dataset.k = s[0];
    });
    head.appendChild(seg);

    var upper = h("section", "pm-up"), rail = h("div", "pm-rail"); rail.id = "pmRail"; rail.setAttribute("aria-label", "Diagnostic et pilotage");
    buildCards(part, rail); upper.appendChild(h("div", "pm-sectitle", "Diagnostic et pilotage")); upper.appendChild(rail);
    var lower = h("section", "pm-low"), scroll = h("div", "pm-scroll"); scroll.id = "pmScroll"; lower.appendChild(h("div", "pm-sectitle", "Documentation et résolutions")); lower.appendChild(scroll);
    var docBox = h("div", "pm-doc"); docBox.id = "pmDoc"; docBox.dataset.k = "doc"; scroll.appendChild(docBox);
    var resBox = h("div", "pm-res"); resBox.id = "pmRes"; resBox.dataset.k = "res"; scroll.appendChild(resBox);
    body.append(upper, lower); dlg.append(head, body, foot());
    loadDoc(part, parts, docBox); fillRes(part, resBox);
    if (state.section === "res") setTimeout(function () { scrollTo("res"); }, 0);
  }
  function foot() { var f = h("div", "pm-foot"), b = h("button", "btn pri", "Fermer"); b.type = "button"; b.onclick = function () { api.close(); }; f.appendChild(b); return f; }
  var shortLabel = function (s) { s = String(s || "").replace(/\s*[(«].*$/, ""); return s.length > 24 ? s.slice(0, 23) + "…" : s; };
  function scrollTo(k) {
    var sc = $("#pmScroll"), t = sc && sc.querySelector('[data-k="' + k + '"]'); state.section = k === "res" ? "res" : "doc";
    if (k === "diag") { var r = $("#pmRail"); if (r && r.scrollIntoView) r.scrollIntoView({ block: "nearest" }); return; }
    if (sc && t) sc.scrollTo ? sc.scrollTo({ top: t.offsetTop - sc.offsetTop, behavior: "smooth" }) : (sc.scrollTop = t.offsetTop - sc.offsetTop);
  }

  /* ---------------------------------------------------------------- zone haute : cartes de metriques puis d'actions */
  function buildCards(part, rail) {
    var DT = ctx.dt() || {}, any = false;
    dedupeDiag(part).forEach(function (name) {
      var t = DT[name], card = h("article", "pm-card"); card.dataset.task = name; any = true;
      card.dataset.lv = t ? t.level : 0; card.appendChild(h("div", "pm-ct", name.split(": ").slice(1).join(": ") || name));
      card.appendChild(h("div", "pm-cm", t ? t.msg : "tâche absente de l'arbre de diagnostic"));
      var vals = h("div", "pm-vals"); card.appendChild(vals); card._fill = function (tt) { fillVals(vals, name, tt); }; card._fill(t); rail.appendChild(card);
    });
    ((ctx.catalog.metrics || {})[part.id] || []).forEach(function (m) {
      var card = h("article", "pm-card"); card.dataset.metric = m.id; any = true; card.appendChild(h("div", "pm-ct", m.label));
      var vals = h("div", "pm-vals"); card.appendChild(vals); card._fillMetric = function () { fillMetric(vals, m); }; card._fillMetric(); rail.appendChild(card);
    });
    var acts = (ctx.catalog.actions || {})[part.id] || [];
    acts.forEach(function (a) { rail.appendChild(actionCard(part, a)); });
    if (!any && !acts.length) rail.appendChild(h("article", "pm-card pm-none", "Aucune métrique ni action pour cette part. La documentation est en dessous."));
  }
  function fillVals(box, task, t) {
    box.replaceChildren(); if (!t) return;
    Object.keys(t.vals || {}).forEach(function (k) {
      var row = h("div", "pm-v"), v = t.vals[k]; row.appendChild(h("span", "pm-k", k.replace(/_/g, " "))); row.appendChild(h("span", "pm-x", String(v)));
      var key = task + "|" + k, series = hist[key];
      if (series && series.length > 2 && isFinite(numeric(v))) row.appendChild(spark(series));
      box.appendChild(row);
    });
  }
  function fillMetric(box, m) {
    box.replaceChildren(); var src = ctx.vars(), v = src && src[m.cle], row = h("div", "pm-v");
    row.appendChild(h("span", "pm-k", m.label)); row.appendChild(h("span", "pm-x", isFinite(v) ? (Math.round(v * 100) / 100) + " " + (m.unite || "") : "—"));
    var s = hist["m|" + m.id]; if (m.courbe && s && s.length > 2) row.appendChild(spark(s)); box.appendChild(row);
  }
  function spark(series) {
    var c = document.createElement("canvas"); c.className = "pm-spark"; c.width = 120; c.height = 22; var g = c.getContext("2d"); if (!g) return c;
    var lo = Infinity, hi = -Infinity; series.forEach(function (p) { lo = Math.min(lo, p.v); hi = Math.max(hi, p.v); }); if (hi - lo < 1e-9) { hi = lo + 1; }
    g.strokeStyle = "rgba(86,200,222,.9)"; g.lineWidth = 1.2; g.beginPath();
    series.forEach(function (p, i) { var x = i / (series.length - 1) * 118 + 1, y = 20 - (p.v - lo) / (hi - lo) * 18; i ? g.lineTo(x, y) : g.moveTo(x, y); }); g.stroke();
    return c;
  }
  /* echantillonnage periodique (1 Hz) : alimente les courbes sans abonnement supplementaire ; ne tourne que slide ouvert */
  function refreshGates(now) {                                                // conditions des actions et de la popup : 4 fois par seconde, jamais plus lentement (la traction peut s'armer a tout moment)
    if (now - gateT < 250) return; gateT = now;
    var rail = $("#pmRail"); if (rail) [].forEach.call(rail.querySelectorAll(".pm-act"), function (card) { if (card._refresh) card._refresh(); });
    if (pop && pop._refresh) pop._refresh();
  }
  function sample(now) {
    if (!state.open) return; refreshGates(now);
    if (now - histT < HIST_EVERY_MS) return; histT = now;
    var DT = ctx.dt() || {}, part = ((ctx.parts() || {}).parts || []).filter(function (p) { return p.id === state.id; })[0]; if (!part) return;
    dedupeDiag(part).forEach(function (name) { var t = DT[name]; if (t) Object.keys(t.vals || {}).forEach(function (k) { var n = numeric(t.vals[k]); if (isFinite(n)) push(hist, name + "|" + k, n, now); }); });
    ((ctx.catalog.metrics || {})[part.id] || []).forEach(function (m) { var v = (ctx.vars() || {})[m.cle]; if (m.courbe && isFinite(v)) push(hist, "m|" + m.id, v, now); });
    var rail = $("#pmRail"); if (!rail) return;
    [].forEach.call(rail.querySelectorAll(".pm-card"), function (card) {
      if (card.dataset.task) { var t = DT[card.dataset.task]; card.dataset.lv = t ? t.level : 0; var m = card.querySelector(".pm-cm"); if (m) m.textContent = t ? t.msg : "tâche absente de l'arbre de diagnostic"; if (card._fill) card._fill(t); }
      else if (card._fillMetric) card._fillMetric();
      if (card._refresh) card._refresh();
    });
  }

  /* ---------------------------------------------------------------- actions */
  function actionCard(part, a) {
    var card = h("article", "pm-card pm-act"), risk = a.lecture ? "lecture" : a.risque; card.dataset.risk = risk; card.dataset.action = a.id;
    card.appendChild(h("div", "pm-ct", a.label)); card.appendChild(h("div", "pm-cm", a.decrit || ""));
    card.appendChild(h("div", "pm-risk", "Risque : " + risk));
    var btn = h("button", "btn pm-go", needsConfirm(a) ? "Préparer…" : "Exécuter"); btn.type = "button"; var why = h("div", "pm-why"), res = h("div", "pm-res1");
    btn.onclick = function () { if (btn.disabled) return; if (needsConfirm(a)) openPopup(part, a); else run(part, a, res); };
    card._refresh = function () { var v = allowed(a, ctx.env()); btn.disabled = !v.ok; why.textContent = v.ok ? "" : v.why; };
    card._refresh(); card.append(btn, why, res); if (lastRes[part.id + "/" + a.id]) res.textContent = lastRes[part.id + "/" + a.id];
    return card;
  }
  function run(part, a, out) {
    var v = allowed(a, ctx.env());
    if (!v.ok) { if (out) out.textContent = "Refusé : " + v.why; ctx.log(1, "Maintenance " + part.id + " : " + a.label + " refusé (" + v.why + ")"); return; }
    ctx.log(0, "Maintenance " + part.id + " : " + a.label + " demandé");
    ctx.rbCall(a.service, a.args || {}, function (m) {
      var ok = !!(m && m.result && (!m.values || m.values.success !== false)), msg = (m && m.values && m.values.message) || (ok ? "ok" : "erreur");
      var txt = (ok ? "✓ " : "✗ ") + msg; lastRes[part.id + "/" + a.id] = txt; if (out) out.textContent = txt;
      ctx.log(ok ? 0 : 1, "Maintenance " + part.id + " : " + a.label + " → " + msg);
      var live = $('.pm-act[data-action="' + a.id + '"] .pm-res1'); if (live) live.textContent = txt;
    });
  }

  /* ---------------------------------------------------------------- popup : decrit le risque, valide par appui MAINTENU de 3 s, Echap ne ferme pas */
  function openPopup(part, a) {
    closePopup();
    pop = h("div", "pm-pop"); pop.setAttribute("role", "alertdialog"); pop.setAttribute("aria-modal", "true");
    var box = h("div", "pm-popbox"); box.dataset.risk = a.risque;
    box.appendChild(h("div", "pm-poptag", "Validation requise · risque " + a.risque));
    box.appendChild(h("h4", "pm-poph", a.label + " — " + part.label));
    box.appendChild(h("p", "", a.decrit || "")); var eff = h("p", "pm-popeff", a.effet || ""); box.appendChild(eff);
    var cond = h("p", "pm-popcond"); box.appendChild(cond);
    var go = h("button", "hold pm-hold" + (a.risque === "eleve" ? " danger" : "")), sp = h("span", "", "Maintenir 3 s pour valider"); go.type = "button"; go.appendChild(sp);
    var cancel = h("button", "btn", "Annuler"); cancel.type = "button"; cancel.onclick = closePopup;
    var row = h("div", "pm-poprow"); row.append(cancel, go); box.appendChild(row); pop.appendChild(box); dlg.appendChild(pop);
    var refresh = function () { var v = allowed(a, ctx.env()); go.disabled = !v.ok; cond.textContent = v.ok ? "Conditions remplies." : "Conditions non remplies : " + v.why; cond.dataset.ok = v.ok ? "1" : "0"; if (!v.ok) stopHold(); };
    pop._refresh = refresh; refresh();
    var start = function (e) { if (e) e.preventDefault(); if (go.disabled || hold.raf) return; hold.t0 = performance.now(); hold.btn = go;
      var step = function () { var p = holdProgress(hold.t0, performance.now()); go.style.setProperty("--p", p); if (p >= 1) { stopHold(); closePopup(); run(part, a, null); return; } hold.raf = requestAnimationFrame(step); }; hold.raf = requestAnimationFrame(step); };
    go.addEventListener("pointerdown", start);
    ["pointerup", "pointerleave", "pointercancel", "blur"].forEach(function (n) { go.addEventListener(n, stopHold); });
    go.addEventListener("keydown", function (e) { if ((e.key === " " || e.key === "Enter") && !e.repeat) start(e); }); go.addEventListener("keyup", stopHold);
    cancel.focus();
  }
  function stopHold() { if (hold.raf) cancelAnimationFrame(hold.raf); hold.raf = 0; if (hold.btn) hold.btn.style.setProperty("--p", 0); }
  function closePopup() { stopHold(); if (pop && pop.parentNode) pop.parentNode.removeChild(pop); pop = null; }

  /* ---------------------------------------------------------------- zone basse */
  function loadDoc(part, parts, box) {
    var show = function (md, generated) {
      if (state.id !== part.id) return; box.replaceChildren();
      if (generated) box.appendChild(h("div", "pm-note", "Aucune fiche rédigée : fiche générée depuis la nomenclature."));
      box.appendChild(root.PartMd.render(document, md, { onPart: function (id) { api.open(id); } }));
    };
    var url = "docs/parts/" + encodeURIComponent(part.id) + ".md";
    var hit = cache[part.id];                                                     // {md: texte} si la fiche existe, {absent: true} sinon
    if (hit) { show(hit.absent ? genericDoc(part, parts) : hit.md, !!hit.absent); return; }
    box.appendChild(h("p", "pm-empty", "Chargement de la fiche…"));
    if (typeof fetch !== "function" || ctx.noFetch) { cache[part.id] = { absent: true }; return show(genericDoc(part, parts), true); }
    fetch(url, { cache: "no-cache" }).then(function (r) { if (!r.ok) throw new Error("HTTP " + r.status); return r.text(); })
      .then(function (txt) { if (/^\s*<(!doctype|html)/i.test(txt)) throw new Error("pas du Markdown"); cache[part.id] = { md: txt }; show(txt, false); })
      .catch(function () { cache[part.id] = { absent: true }; show(genericDoc(part, parts), true); });
  }
  function fillRes(part, box) {
    box.replaceChildren(); box.appendChild(h("h5", "pm-reshead", "Résolutions de l'incident"));
    var inc = ctx.inc && ctx.inc(), mine = ((inc && inc.incidents) || []).filter(function (x) { return x.racine === part.id; }), steps = resolutionsFor(inc, part.id);
    if (!mine.length) { box.appendChild(h("p", "pm-empty", "Aucun incident actif sur cette part.")); }
    mine.forEach(function (x) {
      var blk = h("div", "pm-incblk"); blk.dataset.lv = ctx.incLv(x); blk.appendChild(h("div", "pm-inch", x.label + " · depuis " + Math.round(x.depuis_s || 0) + " s"));
      var ol = h("ol", "pm-steps"); (x.resolution || []).forEach(function (st) {
        var li = h("li", "", st), cmd = ctx.incCmd ? ctx.incCmd(st) : ""; if (cmd) { var b = h("button", "btn pm-cp", "Copier la commande"); b.type = "button"; b.title = cmd; b.onclick = function () { ctx.incCopy(cmd, b); }; li.appendChild(b); }
        ol.appendChild(li); });
      blk.appendChild(ol); box.appendChild(blk);
    });
    if (mine.length && !steps.length) box.appendChild(h("p", "pm-empty", "Aucune piste de résolution écrite pour cette part."));
  }

  /* ---------------------------------------------------------------- API publique */
  var api = {
    init: function (c) { ctx = c; ctx.catalog = ctx.catalog || root.PartMaintCatalog || { actions: {}, metrics: {} }; ensureDialog(); mountButton(); },
    open: function (id, opts) {
      if (!ctx) return false; ensureDialog(); state.id = id; state.section = (opts && opts.section) || "doc";
      closeRight(); bounds(); closePopup(); render(); if (!dlg.open) dlg.show(); state.open = true; setBtn(true); return true;
    },
    close: function () { if (!dlg || !dlg.open) return; closePopup(); stopHold(); if (matchMedia("(prefers-reduced-motion: reduce)").matches) dlg.close(); else { dlg.classList.add("leaving"); var done = false, fin = function () { if (done) return; done = true; dlg.classList.remove("leaving"); if (dlg.open) dlg.close(); }; dlg.addEventListener("transitionend", fin); setTimeout(fin, 450); } setBtn(false); },
    toggle: function () { if (state.open) api.close(); else api.open(state.id || (((ctx.parts() || {}).parts || [])[0] || {}).id || "robot"); },
    isOpen: function () { return !!state.open; },
    tick: sample,
    state: function () { return { id: state.id, section: state.section, open: state.open, popup: !!pop, hist: Object.keys(hist).length }; },
    _t: { parentChain: parentChain, numeric: numeric, push: push, allowed: allowed, needsConfirm: needsConfirm, holdProgress: holdProgress, resolutionsFor: resolutionsFor, genericDoc: genericDoc, dedupeDiag: dedupeDiag, HOLD_MS: HOLD_MS }
  };
  /* Les deux panneaux couvrent chacun la moitie de l'ecran : ouvrir la maintenance FERME le panneau de droite (Incidents, Journal, Touches), sauf si
     c'est l'ouverture depuis la vue 3D ou la liste des incidents : la, on veut garder la vue 3D, donc le panneau de droite RESTE et on reduit les deux. */
  function closeRight() {
    var r = document.getElementById("dlg"); if (!r || !r.open || !r.classList.contains("slide")) return;
    var inc = r.querySelector(".r3d");                                           // vue 3D ouverte : on la garde, les deux panneaux se partagent l'ecran
    document.documentElement.classList.toggle("pm-split", !!inc);
    if (!inc) { r.classList.add("leaving"); setTimeout(function () { r.classList.remove("leaving"); if (r.open) r.close(); }, 330); }
  }
  var btn = null;
  function mountButton() {
    var ft = document.querySelector(".foot"); if (!ft || document.getElementById("mbtn")) return;
    btn = h("button", "btn jbtn", "Maintenance"); btn.id = "mbtn"; btn.type = "button"; btn.setAttribute("aria-haspopup", "dialog"); btn.title = "Maintenance d'une part [P]"; btn.onclick = api.toggle;
    var first = ft.querySelector("#ibtn"); ft.insertBefore(btn, first || ft.querySelector("#kbtn") || null);
  }
  function setBtn(on) { if (btn) btn.setAttribute("aria-pressed", String(!!on)); }

  root.PartMaint = api;
  if (typeof module !== "undefined" && module.exports) module.exports = api;
})(typeof window !== "undefined" ? window : this);
