/* robot3d.js : vue 3D filaire du robot pour le panneau Incidents du CC (2026-11-18). Canvas 2D, AUCUNE bibliotheque, aucune ressource externe.

   Entrees (deux topics latches, lus par la page) :
     /robot_description   l'URDF REEL (XML) : formes des liens (boite, cylindre) et leur pose dans base_link ;
     /bamboo/parts        l'arbre de la nomenclature (parts.yaml) : hierarchie parent/enfant et cle `visuel` de chaque part.
   Une part se dessine de deux facons : `visuel: {urdf: <lien>}` (forme lue dans l'URDF) ou un bloc dessine a la main (`estime`, trait en POINTILLES :
   cotes jamais mesurees). Une part SANS visuel n'a pas de forme : elle n'est pas dessinee ; son incident remonte au parent dessine, sinon il est liste
   a part (« hors modele 3D »).

   Repere : celui de base_link, en metres, x avant / y gauche / z haut. Vue par defaut : 3/4 avant gauche.
   Couleurs d'incident : racine = couleur pleine ; consequence = plus pale ; impact = trait fin. L'incident d'un SOUS-COMPOSANT remonte a son bloc visible
   (la carte ESP32 s'allume si le TB6612 est en defaut) ; l'impact (ce qui tombe avec la panne) ne remonte pas : un logiciel ne colore pas le RPi.

   Etape 2 (ce fichier) : scene statique. Les fonctions pures (algebre, formes, etats) sont exportees dans Robot3D._t pour un test hors navigateur. */
(function (root) {
  "use strict";

  /* ---------------------------------------------------------------- algebre : matrices 3x3 a plat (ligne par ligne) et vecteurs */
  var I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  function rotRpy(r, p, y) {                                       // convention URDF : R = Rz(y) · Ry(p) · Rx(r)
    var cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp, cp * sr, cp * cr];
  }
  function mmul(A, B) {
    var o = new Array(9), i, j;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) o[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
    return o;
  }
  function mvec(R, v) { return [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]]; }
  function apply(T, v) { var r = mvec(T.R, v); return [r[0] + T.t[0], r[1] + T.t[1], r[2] + T.t[2]]; }
  function compose(A, B) { var t = mvec(A.R, B.t); return { R: mmul(A.R, B.R), t: [t[0] + A.t[0], t[1] + A.t[1], t[2] + A.t[2]] }; }   // A ∘ B
  var IDT = { R: I3, t: [0, 0, 0] };

  /* ---------------------------------------------------------------- URDF -> formes {kind, size|r+h, R, t} dans base_link */
  function parseUrdf(xml) {
    if (typeof DOMParser === "undefined") return null;
    var doc = new DOMParser().parseFromString(String(xml || ""), "application/xml");
    if (!doc.documentElement || doc.getElementsByTagName("parsererror").length) return null;
    var nums = function (s, n, d) { var a = String(s || "").trim().split(/\s+/).map(Number); return a.length === n && a.every(isFinite) ? a : d; };
    var origin = function (el) { var o = el && el.getElementsByTagName("origin")[0], rpy = nums(o && o.getAttribute("rpy"), 3, [0, 0, 0]); return { R: rotRpy(rpy[0], rpy[1], rpy[2]), t: nums(o && o.getAttribute("xyz"), 3, [0, 0, 0]) }; };
    var joints = {}, shapes = {};
    [].forEach.call(doc.getElementsByTagName("joint"), function (j) {
      var par = j.getElementsByTagName("parent")[0], ch = j.getElementsByTagName("child")[0];
      if (par && ch) { var o = origin(j); joints[ch.getAttribute("link")] = { parent: par.getAttribute("link"), R: o.R, t: o.t }; }
    });
    var pose = function (link, depth) {
      if (link === "base_link") return IDT;                       // base_link est la RACINE de la scene (base_footprint est au sol, hors scene)
      var j = joints[link]; if (!j || depth > 24) return null;
      var pp = pose(j.parent, depth + 1); return pp ? compose(pp, { R: j.R, t: j.t }) : null;
    };
    [].forEach.call(doc.getElementsByTagName("link"), function (l) {
      var name = l.getAttribute("name"), pl = pose(name, 0); if (!pl) return;
      var vis = l.getElementsByTagName("visual")[0], geo = vis && vis.getElementsByTagName("geometry")[0], T;
      if (!geo) { shapes[name] = { kind: "point", R: pl.R, t: pl.t, estime: false }; return; }   // repere sans forme (imu_link)
      T = compose(pl, origin(vis));
      var b = geo.getElementsByTagName("box")[0], c = geo.getElementsByTagName("cylinder")[0];
      if (b) { var s = nums(b.getAttribute("size"), 3, null); if (s) shapes[name] = { kind: "box", size: s, R: T.R, t: T.t, estime: false }; }
      else if (c) { var r = +c.getAttribute("radius"), h = +c.getAttribute("length"); if (r > 0 && h > 0) shapes[name] = { kind: "cyl", r: r, h: h, R: T.R, t: T.t, estime: false }; }
    });
    return shapes;
  }

  /* un `visuel` dessine a la main -> forme (l'axe d'un cylindre est z local ; axe y/x : tourne) */
  function shapeFromVisuel(v) {
    if (!v || v.urdf || !v.pos) return null;
    var est = v.estime !== false;
    if (v.forme === "boite" && v.taille && v.taille.length === 3) return { kind: "box", size: v.taille.slice(), R: I3, t: v.pos.slice(), estime: est };
    if (v.forme === "cylindre" && v.taille && v.taille.length === 2) {
      var R = v.axe === "y" ? rotRpy(Math.PI / 2, 0, 0) : v.axe === "x" ? rotRpy(0, Math.PI / 2, 0) : I3;
      return { kind: "cyl", r: v.taille[0], h: v.taille[1], R: R, t: v.pos.slice(), estime: est };
    }
    return null;
  }

  /* ---------------------------------------------------------------- aretes d'une forme (en base_link) et points pour l'enveloppe */
  function edgesOf(sh) {
    var lines = [], pts = [], T = { R: sh.R, t: sh.t }, P = function (x, y, z) { var p = apply(T, [x, y, z]); pts.push(p); return p; }, i, j, k;
    if (sh.kind === "point") {
      var a = 0.012; lines.push([P(-a, 0, 0), P(a, 0, 0)], [P(0, -a, 0), P(0, a, 0)], [P(0, 0, -a), P(0, 0, a)]);
    } else if (sh.kind === "box") {
      var hx = sh.size[0] / 2, hy = sh.size[1] / 2, hz = sh.size[2] / 2, c = [];
      for (i = 0; i < 8; i++) c.push(P(i & 1 ? hx : -hx, i & 2 ? hy : -hy, i & 4 ? hz : -hz));
      [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]].forEach(function (e) { lines.push([c[e[0]], c[e[1]]]); });
      if (Math.max(sh.size[0], sh.size[1], sh.size[2]) > 0.15) {         // grande piece (chassis) : maillage de ses six faces, une maille tous les ~5 cm
        var half = [hx, hy, hz], div = sh.size.map(function (d) { return Math.max(1, Math.round(d / 0.05)); });
        for (var ax = 0; ax < 3; ax++) {
          var u = (ax + 1) % 3, w = (ax + 2) % 3;
          for (var sgn = -1; sgn <= 1; sgn += 2) for (k = 1; k < div[u]; k++) {
            var q1 = [0, 0, 0], q2 = [0, 0, 0]; q1[ax] = q2[ax] = sgn * half[ax]; q1[u] = q2[u] = -half[u] + 2 * half[u] * k / div[u]; q1[w] = -half[w]; q2[w] = half[w];
            lines.push([apply(T, q1), apply(T, q2)]);
          }
          for (sgn = -1; sgn <= 1; sgn += 2) for (k = 1; k < div[w]; k++) {
            var r1 = [0, 0, 0], r2 = [0, 0, 0]; r1[ax] = r2[ax] = sgn * half[ax]; r1[w] = r2[w] = -half[w] + 2 * half[w] * k / div[w]; r1[u] = -half[u]; r2[u] = half[u];
            lines.push([apply(T, r1), apply(T, r2)]);
          }
        }
      }
    } else if (sh.kind === "cyl") {
      var n = 24, top = [], bot = [], hh = sh.h / 2;
      for (i = 0; i < n; i++) { var an = i / n * Math.PI * 2; top.push(P(sh.r * Math.cos(an), sh.r * Math.sin(an), hh)); bot.push(P(sh.r * Math.cos(an), sh.r * Math.sin(an), -hh)); }
      top.push(top[0]); bot.push(bot[0]); lines.push(top, bot);
      for (i = 0; i < n; i += 4) lines.push([top[i], bot[i]]);
      if (sh.h < 2 * sh.r) {                                            // disque (roue, lidar) : rayons sur les deux faces, pour que la rotation se voie
        for (i = 0; i < n / 2; i += 3) { lines.push([top[i], top[i + n / 2]]); lines.push([bot[i], bot[i + n / 2]]); }       // diametres : i + n/2 reste dans [0, n)
      }
    }
    return { lines: lines, pts: pts };
  }

  function hull2(p) {                                                  // enveloppe convexe 2D (chaine monotone) : silhouette exacte d'un solide convexe
    var a = p.map(function (q) { return [q[0], q[1]]; }).sort(function (u, v) { return u[0] - v[0] || u[1] - v[1]; });
    if (a.length < 3) return a;
    var cr = function (o, u, v) { return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0]); }, lo = [], up = [], i;
    for (i = 0; i < a.length; i++) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], a[i]) <= 0) lo.pop(); lo.push(a[i]); }
    for (i = a.length - 1; i >= 0; i--) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], a[i]) <= 0) up.pop(); up.push(a[i]); }
    lo.pop(); up.pop(); return lo.concat(up);
  }

  /* ---------------------------------------------------------------- etat des incidents par part */
  var RANK = { root: 6, cons: 4, impact: 2 };
  function rankOf(role, lv) { return RANK[role] - (lv === 2 ? 0 : 1); }
  /* direct : {id: {role, lv}} ; une part touchee par plusieurs incidents garde le pire (racine > consequence > impact). */
  function directStates(inc, lvFn) {
    var d = {}, put = function (id, role, lv) { if (!d[id] || rankOf(role, lv) > rankOf(d[id].role, d[id].lv)) d[id] = { role: role, lv: lv }; };
    ((inc && inc.incidents) || []).forEach(function (x) {
      var lv = lvFn(x);
      (x.impact || []).forEach(function (i) { put(i, "impact", lv); });
      (x.consequences || []).forEach(function (c) { put(c, "cons", lv); });
      put(x.racine, "root", lv);
    });
    return d;
  }
  /* Etat d'un bloc affiche = le pire entre lui-meme et ses DESCENDANTS en defaut (racine / consequence). L'impact ne remonte pas. */
  function levelState(id, kids, direct) {
    var best = direct[id] ? { role: direct[id].role, lv: direct[id].lv, via: id } : null, stack = (kids[id] || []).slice();
    while (stack.length) {
      var c = stack.pop(), s = direct[c];
      if (s && s.role !== "impact" && (!best || rankOf(s.role, s.lv) > rankOf(best.role, best.lv))) best = { role: s.role, lv: s.lv, via: c };
      (kids[c] || []).forEach(function (k) { stack.push(k); });
    }
    return best;
  }

  /* ---------------------------------------------------------------- modele : arbre + formes */
  function buildModel(partsDoc, urdfXml) {
    if (!partsDoc || !partsDoc.parts) return null;
    var urdf = urdfXml ? parseUrdf(urdfXml) : null, byId = {}, kids = {}, shapes = {}, missing = [], rootId = null;
    partsDoc.parts.forEach(function (p) { byId[p.id] = p; (kids[p.parent || ""] = kids[p.parent || ""] || []).push(p.id); if (!p.parent) rootId = p.id; });
    partsDoc.parts.forEach(function (p) {
      var v = p.visuel; if (!v) return;
      var sh = v.urdf ? (urdf && urdf[v.urdf]) : shapeFromVisuel(v);
      if (sh) shapes[p.id] = sh; else missing.push(p.id);
    });
    return { byId: byId, kids: kids, shapes: shapes, missing: missing, root: rootId, hasUrdf: !!urdf };
  }
  function parentOf(model, id) { var p = model.byId[id]; return p && p.parent; }
  /* l'ancetre (ou la part elle-meme) qui est un ENFANT DIRECT de `focus` : le bloc de ce niveau qui contient `id`. null si `id` n'est pas sous `focus`. */
  function levelAncestor(model, id, focus) {
    var cur = id, g = 0;
    while (cur && g++ < 64) { if (parentOf(model, cur) === focus) return cur; cur = parentOf(model, cur); }
    return null;
  }

  /* ---------------------------------------------------------------- rendu */
  var CYAN = [86, 200, 222], AMBER = [231, 162, 74], ALERT = [229, 96, 74], CREAM = [241, 232, 180];
  var rgba = function (c, a) { return "rgba(" + c[0] + "," + c[1] + "," + c[2] + "," + Math.max(0, Math.min(1, a)) + ")"; };
  function cssRgb(name, dflt) {
    try { var v = getComputedStyle(document.documentElement).getPropertyValue(name).trim(), m = /^#([0-9a-f]{6})$/i.exec(v); if (m) return [parseInt(m[1].slice(0, 2), 16), parseInt(m[1].slice(2, 4), 16), parseInt(m[1].slice(4, 6), 16)]; } catch (e) { }
    return dflt;
  }
  var shortLabel = function (s) { s = String(s || "").replace(/\s*[(«].*$/, ""); return s.length > 30 ? s.slice(0, 29) + "…" : s; };

  function create(canvas, opts) {
    opts = opts || {};
    var self = {
      focus: null, az: opts.az != null ? opts.az : 38, el: opts.el != null ? opts.el : 24,        // degres : camera en avant-gauche, un peu en hauteur (3/4 gauche)
      model: null, inc: null, lvFn: opts.lvFn || function (x) { return x.niveau === "ERROR" || x.niveau === "STALE" || x.niveau >= 2 ? 2 : 1; },
      info: { drawn: [], unplaced: [], labels: [] }, w: 0, h: 0
    };
    var ctx = canvas.getContext("2d"), amber = cssRgb("--amber", AMBER), alert = cssRgb("--alert", ALERT), ro = null, dead = false;

    self.setModel = function (partsDoc, urdfXml) { self.model = buildModel(partsDoc, urdfXml); if (self.model && !self.focus) self.focus = self.model.root; self.draw(); };
    self.setIncidents = function (inc) { self.inc = inc; self.draw(); };
    self.resize = function () {
      var z = 1; try { z = parseFloat(getComputedStyle(document.documentElement).getPropertyValue("--z")) || 1; } catch (e) { }
      var w = canvas.clientWidth, h = canvas.clientHeight, d = (window.devicePixelRatio || 1) * z;
      if (!w || !h) return;
      self.w = w; self.h = h; canvas.width = Math.round(w * d); canvas.height = Math.round(h * d); ctx.setTransform(d, 0, 0, d, 0, 0); self.draw();
    };
    self.destroy = function () { dead = true; if (ro) ro.disconnect(); };

    function message(txt) {
      ctx.clearRect(0, 0, self.w, self.h); ctx.fillStyle = rgba(CYAN, 0.7); ctx.font = "12px ui-monospace,Consolas,monospace"; ctx.textAlign = "center"; ctx.fillText(txt, self.w / 2, self.h / 2); ctx.textAlign = "left";
    }

    self.draw = function () {
      if (dead || !self.w || !self.h) return;
      var m = self.model, W = self.w, H = self.h, i;
      if (!m) { message("Modèle 3D indisponible : en attente de /bamboo/parts et /robot_description"); return; }
      var levelIds = (m.kids[self.focus] || []).filter(function (id) { return m.shapes[id]; });
      if (!levelIds.length) { message("Aucune forme à dessiner à ce niveau"); return; }

      /* ---- etats d'incident (par bloc du niveau) et parts sans forme ---- */
      var direct = directStates(self.inc, self.lvFn), state = {}, unplaced = [];
      levelIds.forEach(function (id) { var s = levelState(id, m.kids, direct); if (s) state[id] = s; });
      ((self.inc && self.inc.incidents) || []).forEach(function (x) {
        [x.racine].concat(x.consequences || []).forEach(function (pid) {
          var la = levelAncestor(m, pid, self.focus);
          if (la && !m.shapes[la] && !unplaced.some(function (u) { return u.id === pid; })) unplaced.push({ id: pid, label: (m.byId[pid] || {}).label || pid, lv: self.lvFn(x) });
        });
      });

      /* ---- geometrie de chaque bloc ---- */
      var blocks = levelIds.map(function (id) { var e = edgesOf(m.shapes[id]); return { id: id, sh: m.shapes[id], lines: e.lines, pts: e.pts }; });
      var all = []; blocks.forEach(function (b) { b.pts.forEach(function (p) { all.push(p); }); });
      var lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9]; all.forEach(function (p) { for (i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], p[i]); hi[i] = Math.max(hi[i], p[i]); } });
      var tgt = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2], rad = Math.sqrt(Math.pow(hi[0] - lo[0], 2) + Math.pow(hi[1] - lo[1], 2) + Math.pow(hi[2] - lo[2], 2)) / 2;

      /* ---- camera : position sur une sphere autour de la cible (az depuis +x vers +y, el au-dessus de l'horizon), projection en perspective douce ---- */
      var az = self.az * Math.PI / 180, el = self.el * Math.PI / 180, D = rad * 4.2;
      var C = [tgt[0] + D * Math.cos(el) * Math.cos(az), tgt[1] + D * Math.cos(el) * Math.sin(az), tgt[2] + D * Math.sin(el)];
      var f = [tgt[0] - C[0], tgt[1] - C[1], tgt[2] - C[2]], fl = Math.hypot(f[0], f[1], f[2]); f = f.map(function (v) { return v / fl; });
      var rr = [f[1], -f[0], 0], rl = Math.hypot(rr[0], rr[1]); rr = rr.map(function (v) { return v / rl; });              // droite = f × z
      var uu = [rr[1] * f[2] - rr[2] * f[1], rr[2] * f[0] - rr[0] * f[2], rr[0] * f[1] - rr[1] * f[0]];
      uu = [-uu[0], -uu[1], -uu[2]]; if (uu[2] < 0) uu = [-uu[0], -uu[1], -uu[2]];                                           // haut de l'ecran : z positif
      var proj0 = function (p) { var d = [p[0] - C[0], p[1] - C[1], p[2] - C[2]], dep = d[0] * f[0] + d[1] * f[1] + d[2] * f[2]; return [(d[0] * rr[0] + d[1] * rr[1] + d[2] * rr[2]) / dep, (d[0] * uu[0] + d[1] * uu[1] + d[2] * uu[2]) / dep, dep]; };
      var xs = all.map(proj0), x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9, dmin = 1e9, dmax = -1e9;
      xs.forEach(function (q) { x0 = Math.min(x0, q[0]); x1 = Math.max(x1, q[0]); y0 = Math.min(y0, q[1]); y1 = Math.max(y1, q[1]); dmin = Math.min(dmin, q[2]); dmax = Math.max(dmax, q[2]); });
      var mx = Math.max(110, W * 0.17), my = 34, sc = Math.min((W - 2 * mx) / (x1 - x0), (H - 2 * my) / (y1 - y0)), ox = W / 2 - (x0 + x1) / 2 * sc, oy = H / 2 + (y0 + y1) / 2 * sc;
      var P = function (p) { var q = proj0(p); return [ox + q[0] * sc, oy - q[1] * sc, q[2]]; };

      /* ---- fond : points + sol quadrille ---- */
      ctx.clearRect(0, 0, W, H); ctx.shadowBlur = 0; ctx.setLineDash([]);
      ctx.fillStyle = rgba(CYAN, 0.16); for (var gx = 8; gx < W; gx += 18) for (var gy = 8; gy < H; gy += 18) ctx.fillRect(gx, gy, 1, 1);
      var zf = lo[2], g = 0.05, gxm = 0.30, gym = 0.25;
      ctx.strokeStyle = rgba(CYAN, 0.16); ctx.lineWidth = 1; ctx.beginPath();
      for (var gi = -gxm; gi <= gxm + 1e-9; gi += g) { var a1 = P([gi, -gym, zf]), a2 = P([gi, gym, zf]); ctx.moveTo(a1[0], a1[1]); ctx.lineTo(a2[0], a2[1]); }
      for (var gj = -gym; gj <= gym + 1e-9; gj += g) { var b1 = P([-gxm, gj, zf]), b2 = P([gxm, gj, zf]); ctx.moveTo(b1[0], b1[1]); ctx.lineTo(b2[0], b2[1]); }
      ctx.stroke();

      /* ---- blocs, du plus loin au plus proche ---- */
      blocks.forEach(function (b) { var c = P(b.sh.t); b.dep = c[2]; b.cx = c[0]; b.cy = c[1]; });
      blocks.sort(function (u, v) { return v.dep - u.dep; });
      var drawn = [];
      blocks.forEach(function (b) {
        var st = state[b.id], col = st ? (st.lv === 2 ? alert : amber) : CYAN, role = st && st.role;
        var near = dmax > dmin ? 1 - (b.dep - dmin) / (dmax - dmin) : 1, al = 0.55 + 0.45 * near;
        var scr = b.lines.map(function (l) { return l.map(P); }), hp = hull2(b.pts.map(P));
        if (hp.length > 2) {
          ctx.beginPath(); hp.forEach(function (q, k) { k ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }); ctx.closePath();
          ctx.fillStyle = rgba(col, role === "root" ? 0.26 : role === "cons" ? 0.15 : role === "impact" ? 0.08 : 0.05); ctx.fill();
        }
        ctx.shadowColor = rgba(col, 0.9); ctx.shadowBlur = role === "root" ? 12 : role ? 8 : 5;
        ctx.strokeStyle = rgba(col, al * (role === "impact" ? 0.6 : 1)); ctx.lineWidth = role === "root" ? 2 : role === "cons" ? 1.5 : 1.1;
        ctx.setLineDash(b.sh.estime ? [3, 3] : []);
        ctx.beginPath(); scr.forEach(function (l) { l.forEach(function (q, k) { k ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }); }); ctx.stroke();
        ctx.shadowBlur = 0; ctx.setLineDash([]);
        drawn.push({ id: b.id, x: b.cx, y: b.cy, role: role || null, lv: st ? st.lv : 0, via: st ? st.via : null, estime: !!b.sh.estime });
      });

      /* ---- etiquettes des racines en incident : trait fin du bloc vers la marge, comme sur un ecran de film ---- */
      ctx.font = "13px ui-monospace,Consolas,monospace"; ctx.textBaseline = "middle";
      var roots = drawn.filter(function (d) { return d.role === "root"; }).sort(function (u, v) { return u.y - v.y; }), labels = [], slotL = 0, slotR = 0;
      roots.forEach(function (d) {
        var left = d.x < W / 2, ly = 28 + (left ? slotL++ : slotR++) * 38, col = d.lv === 2 ? alert : amber, via = m.byId[d.via] || {}, txt = shortLabel(via.label || d.via), tag = d.lv === 2 ? "ERREUR" : "AVERT.";
        var ex = left ? 12 + ctx.measureText(txt).width : W - 12 - ctx.measureText(txt).width;
        ctx.strokeStyle = rgba(col, 0.8); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(d.x, d.y); ctx.lineTo(left ? mx - 10 : W - mx + 10, ly + 6); ctx.lineTo(ex + (left ? 0 : 0), ly + 6); ctx.stroke();
        ctx.fillStyle = rgba(col, 1); ctx.beginPath(); ctx.arc(d.x, d.y, 2.5, 0, Math.PI * 2); ctx.fill();
        ctx.textAlign = left ? "left" : "right"; var tx = left ? 12 : W - 12;
        ctx.fillStyle = rgba(col, 1); ctx.fillText(txt, tx, ly - 4); ctx.fillStyle = rgba(col, 0.75); ctx.font = "11px ui-monospace,Consolas,monospace"; ctx.fillText(tag + " · " + d.via, tx, ly + 9); ctx.font = "11px ui-monospace,Consolas,monospace";
        labels.push({ id: d.via, text: txt, side: left ? "L" : "R" });
      });

      /* ---- bandeau : titre, parts sans forme, legende ---- */
      ctx.textAlign = "left"; ctx.font = "11px ui-monospace,Consolas,monospace"; ctx.fillStyle = rgba(CYAN, 0.75);
      ctx.fillText("BAMBOO // VUE 3/4 // NIVEAU " + String((m.byId[self.focus] || {}).label || self.focus).toUpperCase().slice(0, 28), 10, H - 10);
      if (unplaced.length) {
        ctx.textAlign = "right"; unplaced.forEach(function (u, k) { var col = u.lv === 2 ? alert : amber; ctx.fillStyle = rgba(col, 1); ctx.fillText("▲ " + shortLabel(u.label) + " · hors modèle 3D", W - 10, H - 10 - k * 13); });
        ctx.textAlign = "left";
      }
      var est = drawn.some(function (d) { return d.estime; });
      if (est) { ctx.setLineDash([3, 3]); ctx.strokeStyle = rgba(CYAN, 0.8); ctx.strokeRect(10, H - 32, 14, 8); ctx.setLineDash([]); ctx.fillStyle = rgba(CYAN, 0.75); ctx.fillText("cotes estimées (non mesurées)", 30, H - 27); }
      self.info = { drawn: drawn, unplaced: unplaced, labels: labels, states: state, focus: self.focus, missing: m.missing };
    };

    if (typeof ResizeObserver !== "undefined") { ro = new ResizeObserver(function () { self.resize(); }); ro.observe(canvas); }
    self.resize();
    return self;
  }

  root.Robot3D = { create: create, _t: { rotRpy: rotRpy, mmul: mmul, apply: apply, compose: compose, edgesOf: edgesOf, hull2: hull2, shapeFromVisuel: shapeFromVisuel, directStates: directStates, levelState: levelState, levelAncestor: levelAncestor, buildModel: buildModel, parseUrdf: parseUrdf } };
  if (typeof module !== "undefined" && module.exports) module.exports = root.Robot3D;
})(typeof window !== "undefined" ? window : this);
