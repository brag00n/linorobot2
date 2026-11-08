/* widgets.js -- widgets du theme « console », PARTAGES par le Command Center (cc/) et le catalogue (theme/).
   Aucun acces reseau ici : une valeur arrive par widget.set(v). Etat : voir StatusDot (vert/orange/rouge). */
(function (G) {
"use strict";
var $ = function (s, r) { return (r || document).querySelector(s); };
var NS = "http://www.w3.org/2000/svg";
var clamp = function (v, a, b) { return Math.min(b, Math.max(a, v)); };
var lerp = function (a, b, t) { return a + (b - a) * t; };
var reduce = matchMedia("(prefers-reduced-motion: reduce)").matches;
var seed = 1337;                                   // bruit reproductible : mêmes captures à chaque essai
function rng() { seed |= 0; seed = seed + 0x6D2B79F5 | 0; var t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }
function rnd(a, b) { return a + rng() * (b - a); }
function h(tag, cls, html) { var e = document.createElement(tag); if (cls) e.className = cls; if (html != null) e.innerHTML = html; return e; }
function sv(tag, attrs) { var e = document.createElementNS(NS, tag); for (var k in attrs) e.setAttribute(k, attrs[k]); return e; }
function pad(n) { return (n < 10 ? "0" : "") + n; }
function hms(s) { s = Math.max(0, Math.floor(s)); return pad(Math.floor(s / 3600)) + ":" + pad(Math.floor(s / 60) % 60) + ":" + pad(s % 60); }
function ms2(s) { s = Math.max(0, Math.floor(s)); return pad(Math.floor(s / 60)) + ":" + pad(s % 60); }
var HASH = {}; location.hash.replace(/^#/, "").split("&").forEach(function (p) { if (p) { var a = p.split("="); HASH[a[0]] = a[1] === undefined ? true : a[1]; } });

function lvOf(v, w, c, dir) { if (!isFinite(v) || (w == null && c == null)) return 0; if (dir === "down") return v <= c ? 2 : v <= w ? 1 : 0; return v >= c ? 2 : v >= w ? 1 : 0; }
function num(x) { return typeof x === "number" && isFinite(x) ? x : NaN; }
function f(v, d, u) { return isFinite(v) ? v.toFixed(d) + (u || "") : "—"; }

/* ======================= registre de widgets ======================= */
var REG = [], RAFP = [], DREG = [], DRAF = [];
function flashEl(e) { e.classList.remove("flash"); void e.offsetWidth; e.classList.add("flash"); }
function fmt(v, d) { return d ? v.toFixed(d) : String(Math.round(v)); }
/* pastille d'état : s = "ok" (reçu) | "wait" (pas d'info) | "todo" (non implémenté). Le « todo » vient de la source déclarée (✗ …). */
function StatusDot(parent, src) {
  var d = h("i", "sdot"); d.setAttribute("aria-hidden", "true"); d.dataset.s = src && src.charAt(0) === "\u2717" ? "todo" : "wait"; parent.classList.add("wd"); parent.appendChild(d);
  return { el: d, todo: d.dataset.s === "todo", set: function (ok) { if (this.todo) return; var n = ok ? "ok" : "wait"; if (d.dataset.s !== n) { d.dataset.s = n; if (ok) { d.classList.remove("tick"); void d.offsetWidth; d.classList.add("tick"); } } } };
}
function srcTag(parent, src) { if (!src) return; var e = h("em", "src" + (src.charAt(0) === "✗" ? " todo" : "")); e.textContent = src; parent.appendChild(e); }
function scramble(e, txt, ms) {
  if (reduce) { e.textContent = txt; return; }
  e._scr = true; var n = txt.length, t0 = performance.now();
  var iv = setInterval(function () {
    var k = (performance.now() - t0) / (ms || 650);
    // A la fin on pose la valeur la PLUS RECENTE (e._want, posee par set() pendant l'animation) et non
    // celle capturee au lancement : une valeur arrivee entre-temps (ou stable) restait sinon masquee.
    if (k >= 1) { e.textContent = e._want != null ? e._want : txt; e._want = null; e._scr = false; clearInterval(iv); return; }
    var s = ""; for (var i = 0; i < n; i++) { var c = txt.charAt(i); s += (i < k * n || !/[0-9]/.test(c)) ? c : String(Math.floor(rng() * 10)); }
    e.textContent = s;
  }, 45);
}
/* attache un widget au registre : intro animée (entrée échelonnée), puis mise à jour à chaque tick */
function attach(w, getter, i, reg) {
  w.ready = false; w.root.classList.add("init");
  (reg || REG).push(function () { if (w.ready) w.set(getter()); });
  setTimeout(function () {
    w.ready = true; w.set(getter()); if (w.intro) w.intro();
    setTimeout(function () { w.root.classList.remove("init"); }, 1700);
  }, reduce ? 0 : 160 + (i || 0) * 80);
  return w;
}

function Gauge(o) {
  var min = o.min == null ? 0 : o.min, max = o.max == null ? 100 : o.max, N = 40;
  var root = h("div", "gauge" + (o.sm ? " sm" : "")); root.dataset.lv = "0";
  var svg = sv("svg", { viewBox: "0 0 200 200", "aria-hidden": "true" });
  var P = function (a, r) { return [100 + r * Math.sin(a * Math.PI / 180), 100 - r * Math.cos(a * Math.PI / 180)]; };
  var p0 = P(-120, 78), p1 = P(120, 78), d = "M" + p0[0].toFixed(2) + " " + p0[1].toFixed(2) + " A78 78 0 1 1 " + p1[0].toFixed(2) + " " + p1[1].toFixed(2);
  svg.appendChild(sv("path", { "class": "trk", d: d }));
  var val = sv("path", { "class": "val", d: d, pathLength: 100, "stroke-dasharray": 100, "stroke-dashoffset": 100 }); svg.appendChild(val);
  var tk = [];
  for (var i = 0; i <= N; i++) { var a = -120 + 240 * i / N, mj = i % 5 === 0, pa = P(a, mj ? 85 : 88.5), pb = P(a, 95);
    var l = sv("line", { "class": "tk" + (mj ? " mj" : ""), x1: pa[0], y1: pa[1], x2: pb[0], y2: pb[1] }); l.style.setProperty("--k", i); svg.appendChild(l); tk.push(l); }
  svg.appendChild(sv("circle", { "class": "rip", cx: 100, cy: 100, r: 70 }));
  root.appendChild(svg);
  var ctr = h("div", "ctr"), gv = h("div", "gv", "0"), gu = h("div", "gu", o.unit || ""); ctr.append(gv, gu); root.appendChild(ctr);
  root.appendChild(h("div", "gl", o.label)); srcTag(root, o.src); var sd = StatusDot(root, o.src);
  var last = "", lastLv = 0;
  var w = { root: root, cur: "", setUnit: function (t) { if (gu.textContent !== t) gu.textContent = t; }, set: function (v) {
    var ok = isFinite(v); sd.set(ok); if (!ok) v = min; var p = clamp((v - min) / (max - min), 0, 1); val.style.strokeDashoffset = 100 - p * 100;
    var on = Math.round(p * N); for (var j = 0; j <= N; j++) tk[j].classList.toggle("on", p > 0 && j <= on);
    var s = ok ? fmt(v, o.dec || 0) : "—";
    if (s !== last) { if (gv._scr) gv._want = s; else gv.textContent = s; if (last !== "") flashEl(gv); last = s; }
    var L = lvOf(v, o.warn, o.crit, o.dir); if (L !== lastLv) { root.dataset.lv = L; root.classList.remove("rp"); void root.offsetWidth; root.classList.add("rp"); lastLv = L; }
  }, intro: function () { scramble(gv, last, 700); } };
  return w;
}
function Bar(o) {
  var min = o.min == null ? 0 : o.min, max = o.max == null ? 100 : o.max, N = o.seg || 26;
  var root = h("div", "bar"); root.dataset.lv = "0";
  var bh = h("div", "bh"), bn = h("span", "micro", o.label), bw = h("span", "bv"), bv = h("b", ""), bu = h("small", "", o.unit || ""); bw.append(bv, bu); bh.append(bn, bw);
  var segs = h("div", "segs"), it = []; for (var i = 0; i < N; i++) { var s = h("i"); s.style.setProperty("--k", i); segs.appendChild(s); it.push(s); }
  root.append(bh, segs); srcTag(root, o.src); var sd = StatusDot(root, o.src);
  var last = "", lastLv = 0;
  return { root: root, set: function (v) {
    var ok = isFinite(v); sd.set(ok); var on = ok ? Math.round(clamp((v - min) / (max - min), 0, 1) * N) : 0; for (var j = 0; j < N; j++) it[j].classList.toggle("on", j < on);
    var s2 = !ok ? "—" : o.fn ? o.fn(v) : fmt(v, o.dec || 0); if (s2 !== last) { if (bv._scr) bv._want = s2; else bv.textContent = s2; if (last !== "") flashEl(bv); last = s2; }
    var L = lvOf(v, o.warn, o.crit, o.dir); if (L !== lastLv) { root.dataset.lv = L; lastLv = L; }
  }, intro: function () { scramble(bv, last, 600); } };
}
function Stat(o) {
  var root = h("div", "stat fr"); root.dataset.lv = "0";
  var vr = h("div", "vrow"), v = h("span", "v", "0"), u = h("span", "u", o.unit || ""); vr.append(v, u);
  var spw = h("div", "spw"), svg = sv("svg", { "class": "sp", viewBox: "0 0 100 30", preserveAspectRatio: "none", "aria-hidden": "true" });
  var area = sv("path", {}), line = sv("polyline", {}); svg.append(area, line); var dot = h("i", "spd"); spw.append(svg, dot);
  root.append(h("div", "micro", o.label), vr, spw); srcTag(root, o.src); var sd = StatusDot(root, o.src);
  var hist = [], last = "", lastLv = 0;
  return { root: root, set: function (x) {
    var ok = isFinite(x); sd.set(ok); if (!ok) { var sx = "—"; if (sx !== last) { if (v._scr) v._want = sx; else v.textContent = sx; last = sx; } root.dataset.lv = "0"; lastLv = 0; return; }
    hist.push(x); if (hist.length > 44) hist.shift();
    var lo = o.min != null ? o.min : Math.min.apply(null, hist), hi = o.max != null ? o.max : Math.max.apply(null, hist); if (hi - lo < 1e-6) { hi = lo + 1; }
    var pts = hist.map(function (y, i) { return (i / 43 * 100).toFixed(1) + "," + (28 - clamp((y - lo) / (hi - lo), 0, 1) * 26).toFixed(1); });
    var off = 44 - hist.length; pts = pts.map(function (p) { var q = p.split(","); return (parseFloat(q[0]) + off / 43 * 100).toFixed(1) + "," + q[1]; });
    line.setAttribute("points", pts.join(" ")); area.setAttribute("d", "M" + pts.join("L") + "L100,30L" + pts[0].split(",")[0] + ",30Z");
    dot.style.top = (parseFloat(pts[pts.length - 1].split(",")[1]) / 30 * 100) + "%";
    var s = o.fn ? o.fn(x) : fmt(x, o.dec || 0); if (s !== last) { if (v._scr) v._want = s; else v.textContent = s; if (last !== "") flashEl(v); last = s; }
    var L = o.lvFn ? o.lvFn(x) : lvOf(x, o.warn, o.crit, o.dir); if (L !== lastLv) { root.dataset.lv = L; lastLv = L; }
  }, intro: function () { scramble(v, last, 700); } };
}
function Horizon() {
  var root = h("div", "hz"), svg = sv("svg", { viewBox: "-100 -100 200 200", "aria-hidden": "true" });
  var defs = sv("defs", {}), cp = sv("clipPath", { id: "hzc" }); cp.appendChild(sv("circle", { r: 84 })); defs.appendChild(cp); svg.appendChild(defs);
  var clip = sv("g", { "clip-path": "url(#hzc)" }), pl = sv("g", {});
  pl.appendChild(sv("rect", { x: -200, y: -200, width: 400, height: 200, fill: "#0b2a33" })); pl.appendChild(sv("rect", { x: -200, y: 0, width: 400, height: 200, fill: "#06161b" }));
  pl.appendChild(sv("line", { x1: -200, x2: 200, y1: 0, y2: 0, stroke: "#9fc6d2", "stroke-width": 1.4 }));
  [-30, -20, -10, 10, 20, 30].forEach(function (a) { var y = -a * 2.2, w = Math.abs(a) % 20 === 0 ? 28 : 18;
    pl.appendChild(sv("line", { x1: -w, x2: w, y1: y, y2: y, stroke: "#5f98a6", "stroke-width": 1 })); });
  clip.appendChild(pl); svg.appendChild(clip);
  svg.appendChild(sv("circle", { r: 84, fill: "none", stroke: "#3c8ea3", "stroke-width": 1.5 }));
  for (var a = -60; a <= 60; a += 15) { var r = a * Math.PI / 180; svg.appendChild(sv("line", { x1: 84 * Math.sin(r), y1: -84 * Math.cos(r), x2: 92 * Math.sin(r), y2: -92 * Math.cos(r), stroke: "#8fc7d4", "stroke-width": a % 30 === 0 ? 2 : 1 })); }
  svg.appendChild(sv("path", { d: "M-34 0h22l8 8h8l8-8h22", fill: "none", stroke: "#f1e8b4", "stroke-width": 2.4 }));
  svg.appendChild(sv("path", { d: "M0 -84l-5 -10h10z", fill: "#f1e8b4" })); root.appendChild(svg); srcTag(root, "/imu/data · orientation"); var sd = StatusDot(root, "");
  return { root: root, set: function (r, p, ok) { sd.set(ok !== false); pl.setAttribute("transform", "rotate(" + (-r) + ") translate(0 " + (p * 2.2) + ")"); } };
}



function Scope() {
  var root = h("div", "scope"), cv = h("canvas"), ctx = cv.getContext("2d"), W = 0, H = 0; root.appendChild(cv);
  var fit = function () { var r = root.getBoundingClientRect(), d = Math.min(2, window.devicePixelRatio || 1); if (r.width < 2) return; cv.width = Math.round(r.width * d); cv.height = Math.round(r.height * d); W = cv.width; H = cv.height; };
  new ResizeObserver(fit).observe(root); srcTag(root, "MotionCompensationState · wz_in, gz, w_wheels (10 Hz, 30 s)"); var sd = StatusDot(root, "");
  return { root: root, setOk: function (ok) { sd.set(ok); }, draw: function () {
    if (!W) fit(); if (!W) return; var u = H / 170; ctx.clearRect(0, 0, W, H); ctx.strokeStyle = "rgba(29,74,87,.9)"; ctx.lineWidth = 1;
    for (var i = 1; i < 8; i++) { ctx.beginPath(); ctx.moveTo(W * i / 8, 0); ctx.lineTo(W * i / 8, H); ctx.stroke(); } for (var j = 1; j < 4; j++) { ctx.beginPath(); ctx.moveTo(0, H * j / 4); ctx.lineTo(W, H * j / 4); ctx.stroke(); }
    var BUF = (window.CCSTATE && window.CCSTATE.SC) || []; var n = BUF.length, mx = .3; BUF.forEach(function (s) { for (var k = 0; k < 3; k++) if (isFinite(s[k])) mx = Math.max(mx, Math.abs(s[k]) * 1.15); });
    var Y = function (v) { return H / 2 - v / mx * H * .42; };
    [[0, "#f1e8b4", 2], [1, "#9fc6d2", 2], [2, "rgba(95,152,166,.9)", 1.2]].forEach(function (tr) { ctx.strokeStyle = tr[1]; ctx.lineWidth = tr[2] * u; ctx.beginPath(); var st = false;
      BUF.forEach(function (s, k) { var v = s[tr[0]]; if (!isFinite(v)) { st = false; return; } var x = W - (n - k) * (W / 300), y = Y(v); if (st) ctx.lineTo(x, y); else { ctx.moveTo(x, y); st = true; } }); ctx.stroke(); });
    ctx.fillStyle = "#5f98a6"; ctx.font = "500 " + Math.round(10 * u * 1.6) + "px Oxanium, sans-serif"; ctx.fillText("±" + mx.toFixed(2) + " rad/s", 8 * u, 14 * u * 1.4);
  } };
}

function block(title, idx, extra) { var b = h("section", "blk rv"); b.style.setProperty("--i", idx); b.appendChild(h("div", "bt", "<i>" + pad(idx + 1) + "</i>" + title + (extra ? "<em>" + extra + "</em>" : ""))); return b; }
function KV(rows, src) {
  var t = h("table", "kv"), cells = rows.map(function (r) { var tr = h("tr"); tr.append(h("th", "", r[0])); var td = h("td"); tr.appendChild(td); t.appendChild(tr); return td; });
  var kvOk = function () { return null; }; REG.push(function () { var got = false; rows.forEach(function (r, i) { var v = r[1](), lv = 0; if (Array.isArray(v)) { lv = v[1]; v = v[0]; } if (v !== "—" && v != null) got = true; if (cells[i].textContent !== v) cells[i].textContent = v; cells[i].style.color = lv ? ["", "var(--amber)", "var(--alert)"][lv] : ""; }); if (t._w && t._w.setOk) t._w.setOk(got); });
  if (src) { var w = h("div", "card fr kvw"); w.style.padding = "4px 12px"; w.appendChild(t); srcTag(w, src); var sd = StatusDot(w, src); w.setOk = function (ok) { sd.set(ok); }; t._w = w; return w; } return t;
}
function Matrix() {
  var mx = h("div", "mx"), cells = {}, keys = ""; srcTag(mx, "/diagnostics_agg · une case par tâche"); var sd = StatusDot(mx, "");
  function rebuild(ks) { mx.querySelectorAll("i").forEach(function (c) { c.remove(); }); cells = {}; ks.forEach(function (k) { var c = h("i"); c.dataset.lv = "0"; mx.appendChild(c); cells[k] = c; }); keys = ks.join("|"); }
  return { root: mx, update: function () {
    var DT = (window.CCSTATE && window.CCSTATE.DT) || {}; var ks = Object.keys(DT).sort(); sd.set(ks.length > 0); if (ks.join("|") !== keys) rebuild(ks);
    ks.forEach(function (k) { var c = cells[k], lv = String(DT[k].level); c.title = k + " — " + DT[k].msg; if (c.dataset.lv !== lv) { c.dataset.lv = lv; c.classList.remove("chg"); void c.offsetWidth; c.classList.add("chg"); } }); } };
}


/* ---------- sections dépliables : multi=true -> plusieurs ouvertes en meme temps ; multi=false -> une seule ---------- */
function Accordion(host, list, multi, reg) {
  return function (title, sumFn, body, open) {
    var root = h("div", "acc"), hd = h("button", "acch"), sum = h("span", "asum"), wrap = h("div", "accb"), inner = h("div", "acci"), pane = h("div", "accp");
    hd.type = "button"; hd.setAttribute("aria-expanded", "false"); hd.appendChild(h("span", "at", title)); hd.append(sum, h("i", "chev")); pane.appendChild(body); inner.appendChild(pane); wrap.appendChild(inner); root.append(hd, wrap);
    var a = { root: root, hd: hd, onToggle: null, isOpen: function () { return root.classList.contains("open"); },
      setOpen: function (o, quiet) { if (!multi && o) list.forEach(function (x) { if (x !== a) { x.root.classList.remove("open"); x.hd.setAttribute("aria-expanded", "false"); } });
        root.classList.toggle("open", o); hd.setAttribute("aria-expanded", o ? "true" : "false"); if (!quiet && a.onToggle) a.onToggle(); } };
    hd.onclick = function () { a.setOpen(!a.isOpen()); };
    var lastS = "", lastL = -9;
    (reg || []).push(function () {
      var r = sumFn(), t = Array.isArray(r) ? r[0] : r, lv = Array.isArray(r) ? r[1] : 0;
      if (t !== lastS || lv !== lastL) { lastS = t; lastL = lv; sum.replaceChildren(); var d = h("i", "sd " + (lv < 0 ? "o" : ["", "w", "e"][lv])); sum.appendChild(d); sum.appendChild(document.createTextNode(t)); sum.className = "asum" + (lv > 0 ? " lv" + lv : ""); sum.title = t; }
    });
    list.push(a); host.appendChild(root); if (open) a.setOpen(true, true); return a;
  };
}

G.W = { Accordion: Accordion, $: $, h: h, sv: sv, pad: pad, hms: hms, ms2: ms2, clamp: clamp, lerp: lerp, rng: rng, rnd: rnd, reduce: reduce, lvOf: lvOf, num: num, f: f, fmt: fmt, flashEl: flashEl, srcTag: srcTag,
  StatusDot: StatusDot, scramble: scramble, attach: attach, Gauge: Gauge, Bar: Bar, Stat: Stat, Horizon: Horizon, Scope: Scope, KV: KV, Matrix: Matrix, block: block,
  setRegistry: function (r) { REG = r; }, REGget: function () { return REG; } };
})(window);
