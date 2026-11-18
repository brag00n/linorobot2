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

   Etape 3 : animation (oscillation lente, scan lumineux, pulsation des incidents), saisie a la souris, survol, clic = zoom + eclatement en sous-composants,
   clic dans le vide = retour. Les fonctions pures (algebre, formes, etats) sont exportees dans Robot3D._t pour un test hors navigateur. */
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
  function directStates(inc, lvFn, via) {
    var d = {}, put = function (id, role, lv) {
      if (!d[id] || rankOf(role, lv) > rankOf(d[id].role, d[id].lv)) d[id] = { role: role, lv: lv };
      if (via && via[id] && via[id] !== id && (!d[via[id]] || rankOf(role, lv) > rankOf(d[via[id]].role, d[via[id]].lv))) d[via[id]] = { role: role, lv: lv };     // pas de forme propre : l'etat s'affiche sur la part `via`
    };
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
    var urdf = urdfXml ? parseUrdf(urdfXml) : null, byId = {}, kids = {}, shapes = {}, missing = [], rootId = null, via = {};
    partsDoc.parts.forEach(function (p) { byId[p.id] = p; (kids[p.parent || ""] = kids[p.parent || ""] || []).push(p.id); if (!p.parent) rootId = p.id; });
    partsDoc.parts.forEach(function (p) {
      var v = p.visuel; if (!v) return;
      if (v.via) { if (partsDoc.parts.some(function (q) { return q.id === v.via; })) via[p.id] = v.via; return; }          // pas de forme : affichee sur la part `via`
      var sh = v.urdf ? (urdf && urdf[v.urdf]) : shapeFromVisuel(v);
      if (sh) shapes[p.id] = sh; else missing.push(p.id);
    });
    /* Parts LOGICIELLES et d'infrastructure sans forme : modules schematiques poses en rangee au-dessus de leur parent s'il a une boite (le RPi), pour que
       l'eclatement du RPi montre les services. Position purement conventionnelle : marquee `virtuel`, jamais confondue avec une cote. */
    var virtual = {};
    Object.keys(kids).forEach(function (pid) {
      var par = shapes[pid]; if (!par || par.kind !== "box") return;
      var sw = (kids[pid] || []).filter(function (id) { var t = byId[id].type; return !shapes[id] && (t === "logiciel" || t === "infrastructure"); });
      var cols = Math.min(5, Math.max(1, sw.length)), step = 0.016;
      sw.forEach(function (id, n) {
        var cx = n % cols, cy = Math.floor(n / cols), nr = Math.ceil(sw.length / cols);
        shapes[id] = { kind: "box", size: [0.012, 0.012, 0.005], R: I3,
          t: [par.t[0] + (cx - (cols - 1) / 2) * step, par.t[1] + (cy - (nr - 1) / 2) * step, par.t[2] + par.size[2] / 2 + 0.012], estime: true, virtuel: true };
        virtual[id] = true;
      });
    });
    return { byId: byId, kids: kids, shapes: shapes, missing: missing, root: rootId, hasUrdf: !!urdf, virtual: virtual, edges: {}, via: via };
  }
  function parentOf(model, id) { var p = model.byId[id]; return p && p.parent; }
  /* l'ancetre (ou la part elle-meme) qui est un ENFANT DIRECT de `focus` : le bloc de ce niveau qui contient `id`. null si `id` n'est pas sous `focus`. */
  function levelAncestor(model, id, focus) {
    var cur = id, g = 0;
    while (cur && g++ < 64) { if (parentOf(model, cur) === focus) return cur; cur = parentOf(model, cur); }
    return null;
  }

  /* ---------------------------------------------------------------- rendu */
  var CYAN = [86, 200, 222], AMBER = [231, 162, 74], ALERT = [229, 96, 74], CREAM = [241, 232, 180], WHITE = [236, 250, 255];
  var rgba = function (c, a) { return "rgba(" + Math.round(c[0]) + "," + Math.round(c[1]) + "," + Math.round(c[2]) + "," + Math.max(0, Math.min(1, a)) + ")"; };
  var lighten = function (c, a) { return [c[0] + (WHITE[0] - c[0]) * a, c[1] + (WHITE[1] - c[1]) * a, c[2] + (WHITE[2] - c[2]) * a]; };
  function cssRgb(name, dflt) {
    try { var v = getComputedStyle(document.documentElement).getPropertyValue(name).trim(), m = /^#([0-9a-f]{6})$/i.exec(v); if (m) return [parseInt(m[1].slice(0, 2), 16), parseInt(m[1].slice(2, 4), 16), parseInt(m[1].slice(4, 6), 16)]; } catch (e) { }
    return dflt;
  }
  var shortLabel = function (s) { s = String(s || "").replace(/\s*[(«].*$/, ""); return s.length > 30 ? s.slice(0, 29) + "…" : s; };
  var ease = function (u) { u = Math.max(0, Math.min(1, u)); return u < 0.5 ? 4 * u * u * u : 1 - Math.pow(-2 * u + 2, 3) / 2; };
  var lerp3 = function (a, b, e) { return [a[0] + (b[0] - a[0]) * e, a[1] + (b[1] - a[1]) * e, a[2] + (b[2] - a[2]) * e]; };

  function pointInPoly(pt, poly) {                                     // lancer de rayon : vrai si le point est dans le polygone
    var inside = false, i, j;
    for (i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      var xi = poly[i][0], yi = poly[i][1], xj = poly[j][0], yj = poly[j][1];
      if ((yi > pt[1]) !== (yj > pt[1]) && pt[0] < (xj - xi) * (pt[1] - yi) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
  }
  /* « Coque » : grande boite qui en contient d'autres (le chassis). Dessinee translucide et en premier. */
  function isShell(sh) { return sh.kind === "box" && Math.max(sh.size[0], sh.size[1], sh.size[2]) > 0.15; }
  function polyArea(poly) { var a = 0, i, j; for (i = 0, j = poly.length - 1; i < poly.length; j = i++) a += poly[j][0] * poly[i][1] - poly[i][0] * poly[j][1]; return Math.abs(a) / 2; }
  /* Pièce visée : parmi celles dont la silhouette contient le point (ou dont le centre en est proche), la PLUS PETITE. Le chassis recouvre en projection tout
     ce qu'il contient : sans cette regle, la carte ou le RPi ne seraient jamais atteignables a la souris. */
  function pickHit(hits, p) {
    var best = null;
    hits.forEach(function (h) {
      var d = Math.hypot(p[0] - h.cx, p[1] - h.cy), inside = pointInPoly(p, h.poly);
      if (!inside && d >= 9) return;
      var score = inside ? h.area : 40 + d;                                 // un centre tout proche mais hors silhouette passe apres une vraie inclusion
      if (!best || score < best.score) best = { h: h, score: score };
    });
    return best && best.h;
  }
  /* Eclatement : chaque centre s'ecarte du centre moyen (facteur E*k), plus une legere elevation (lift*k) pour decoller les enfants du fantome du parent. */
  function explodeOffsets(centers, E, k, lift) {
    var n = centers.length, c0 = [0, 0, 0], i;
    centers.forEach(function (c) { for (i = 0; i < 3; i++) c0[i] += c[i] / n; });
    return centers.map(function (c) { return [(c[0] - c0[0]) * E * k, (c[1] - c0[1]) * E * k, (c[2] - c0[2]) * E * k + lift * k]; });
  }

  function create(canvas, opts) {
    opts = opts || {};
    var reduce = false; try { reduce = !!(window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches); } catch (e) { }
    var self = {
      az: opts.az != null ? opts.az : 38, el: opts.el != null ? opts.el : 24, zoom: 1,                       // degres : camera en avant-gauche, un peu en hauteur (3/4 gauche)
      model: null, inc: null, lvFn: opts.lvFn || function (x) { return x.niveau === "ERROR" || x.niveau === "STALE" || x.niveau >= 2 ? 2 : 1; },
      info: { drawn: [], unplaced: [], labels: [], path: [], hover: null }, w: 0, h: 0
    };
    var ctx = canvas.getContext("2d"), amber = cssRgb("--amber", AMBER), alert = cssRgb("--alert", ALERT), ro = null, dead = false, raf = 0, lastT = 0;
    var labelHits = [], path = [], cam = { c: [0, 0, 0], rad: 0.2 }, k = 0, anim = null, sel = null, hov = null, hits = [], pdown = null, ptr = null, idleUntil = 0, oscAmp = 0, groundZ = -0.085;
    var now0 = function () { return (typeof performance !== "undefined" ? performance.now() : Date.now()); };

    var focus = function () { return path[path.length - 1]; };
    var restK = function (f) { return self.model && f === self.model.root ? 0 : 1; };
    var shapedKids = function (f) { var m = self.model; return ((m && m.kids[f]) || []).filter(function (id) { return m.shapes[id]; }); };
    var edges = function (id) { var m = self.model; return m.edges[id] || (m.edges[id] = edgesOf(m.shapes[id])); };
    var offsetsFor = function (f, ids, kk) { var root = f === self.model.root; return explodeOffsets(ids.map(function (id) { return self.model.shapes[id].t; }), root ? 1.5 : 2.4, kk, root ? 0 : 0.025); };
    function frameOf(f, kk) {                                         // centre et rayon de la boite englobante du niveau `f` (blocs eclates a `kk`, fantome du parent compris)
      var ids = shapedKids(f), offs = offsetsFor(f, ids, kk), lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9], add = function (p, o) { for (var i = 0; i < 3; i++) { var v = p[i] + (o ? o[i] : 0); lo[i] = Math.min(lo[i], v); hi[i] = Math.max(hi[i], v); } };
      ids.forEach(function (id, i) { edges(id).pts.forEach(function (p) { add(p, offs[i]); }); });
      if (f !== self.model.root && self.model.shapes[f]) edges(f).pts.forEach(function (p) { add(p, null); });
      if (lo[0] > hi[0]) return { c: [0, 0, 0], rad: 0.2 };
      return { c: [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2], rad: Math.max(0.03, 0.9 * Math.sqrt(Math.pow(hi[0] - lo[0], 2) + Math.pow(hi[1] - lo[1], 2) + Math.pow(hi[2] - lo[2], 2)) / 2) };
    }
    var changed = function () { if (opts.onChange) opts.onChange(path.slice(), self.model); };
    function startAnim(k0, k1, instant) {
      var to = frameOf(focus(), restK(focus())), now = now0();
      anim = { t0: now, dur: (reduce || instant) ? 0 : 650, from: { c: cam.c.slice(), rad: cam.rad }, to: to, k0: k0, k1: k1 };
      changed(); self.draw(now);
    }

    self.path = function () { return path.slice(); };
    self.setModel = function (partsDoc, urdfXml) {
      self.model = buildModel(partsDoc, urdfXml); var m = self.model;
      if (!m) { self.draw(); return; }
      if (!path.length || path.some(function (id) { return !m.byId[id]; })) path = [m.root];
      var lo = 1e9; Object.keys(m.shapes).forEach(function (id) { if (!m.virtual[id]) edges(id).pts.forEach(function (p) { lo = Math.min(lo, p[2]); }); }); if (lo < 1e8) groundZ = lo;
      var f = frameOf(focus(), restK(focus())); cam = { c: f.c, rad: f.rad }; k = restK(focus()); anim = null; changed(); self.draw();
    };
    self.setIncidents = function (inc) { self.inc = inc; self.draw(); };
    self.zoomable = function (id) { return !!self.model && shapedKids(id).length > 0; };
    self.zoomTo = function (id) {
      if (!self.zoomable(id) || id === focus()) return false;
      path.push(id); sel = null; startAnim(0, 1); return true;
    };
    self.back = function () { if (path.length < 2) return false; path.pop(); sel = null; startAnim(k, restK(focus())); return true; };
    self.goPath = function (i) { if (i < 0 || i >= path.length - 1) return false; path = path.slice(0, i + 1); sel = null; startAnim(k, restK(focus())); return true; };
    self.reveal = function (id) {                                     // amene la vue au niveau ou `id` est un bloc, et le met en evidence
      var m = self.model; if (m && m.via && m.via[id]) id = m.via[id];
      if (!m || !m.byId[id]) return false;
      var chain = [], cur = m.byId[id].parent, g = 0; while (cur && g++ < 64) { chain.unshift(cur); cur = m.byId[cur].parent; }
      path = chain.length ? chain : [m.root];
      while (path.length > 1 && !shapedKids(path[path.length - 1]).length) path.pop();
      sel = id; startAnim(0, restK(focus())); return true;
    };
    self.resize = function () {
      var z = 1; try { z = parseFloat(getComputedStyle(document.documentElement).getPropertyValue("--z")) || 1; } catch (e) { }
      var w = canvas.clientWidth, h = canvas.clientHeight, d = (window.devicePixelRatio || 1) * z;
      if (!w || !h) return;
      self.w = w; self.h = h; canvas.width = Math.round(w * d); canvas.height = Math.round(h * d); ctx.setTransform(d, 0, 0, d, 0, 0); self.draw();
    };
    self.destroy = function () { dead = true; if (raf) cancelAnimationFrame(raf); if (ro) ro.disconnect(); };

    function message(txt) {
      ctx.clearRect(0, 0, self.w, self.h); ctx.fillStyle = rgba(CYAN, 0.7); ctx.font = "13px ui-monospace,Consolas,monospace"; ctx.textAlign = "center"; ctx.fillText(txt, self.w / 2, self.h / 2); ctx.textAlign = "left";
    }

    self.draw = function (ts) {
      if (dead || !self.w || !self.h) return;
      var m = self.model, W = self.w, H = self.h, i, now = ts || now0();
      if (!m) { message("Modèle 3D indisponible : en attente de /bamboo/parts et /robot_description"); return; }
      var f0 = focus(), ids = shapedKids(f0);
      if (!ids.length) { message("Aucune forme à dessiner à ce niveau"); return; }

      /* ---- animations : transition de niveau (camera + eclatement), oscillation lente, pause pendant la saisie ---- */
      if (anim) {
        var u = anim.dur > 0 ? (now - anim.t0) / anim.dur : 1, e = ease(u);
        cam = { c: lerp3(anim.from.c, anim.to.c, e), rad: anim.from.rad + (anim.to.rad - anim.from.rad) * e }; k = anim.k0 + (anim.k1 - anim.k0) * e;
        if (u >= 1) { cam = { c: anim.to.c.slice(), rad: anim.to.rad }; k = anim.k1; anim = null; }
      }
      var dragging = !!(pdown && pdown.moved);
      oscAmp += (((reduce || dragging || now < idleUntil) ? 0 : 1) - oscAmp) * 0.06;
      var osc = oscAmp * 13 * Math.sin(now / 1000 * Math.PI * 2 / 20), pu = reduce ? 1 : 0.5 + 0.5 * Math.sin(now * 0.0075), pulse = reduce ? 1 : 0.6 + 0.4 * pu;

      /* ---- etats d'incident (par bloc du niveau) et parts sans forme ---- */
      var direct = directStates(self.inc, self.lvFn, m.via), state = {}, unplaced = [];
      ids.forEach(function (id) { var s = levelState(id, m.kids, direct); if (s) state[id] = s; });
      var fstate = f0 !== m.root ? levelState(f0, m.kids, direct) : null;
      ((self.inc && self.inc.incidents) || []).forEach(function (x) {
        [x.racine].concat(x.consequences || []).forEach(function (pid) {
          var la = levelAncestor(m, pid, f0);
          if (la && !m.shapes[la] && !m.via[pid] && !unplaced.some(function (q) { return q.id === pid; })) unplaced.push({ id: pid, label: (m.byId[pid] || {}).label || pid, lv: self.lvFn(x) });
        });
      });

      /* ---- camera : sphere autour du centre anime ; l'echelle est FIXE (le zoom vient du rayon, pas d'un recadrage) ---- */
      var offs = offsetsFor(f0, ids, k), tgt = cam.c, mx = Math.max(110, W * 0.17), my = 34;
      var basis = function (azDeg) {
        var az = azDeg * Math.PI / 180, el = self.el * Math.PI / 180, D = cam.rad * 3.4 * self.zoom;
        var C = [tgt[0] + D * Math.cos(el) * Math.cos(az), tgt[1] + D * Math.cos(el) * Math.sin(az), tgt[2] + D * Math.sin(el)];
        var fv = [tgt[0] - C[0], tgt[1] - C[1], tgt[2] - C[2]], fl = Math.hypot(fv[0], fv[1], fv[2]); fv = fv.map(function (v) { return v / fl; });
        var rr = [fv[1], -fv[0], 0], rl = Math.hypot(rr[0], rr[1]) || 1; rr = rr.map(function (v) { return v / rl; });
        var uu = [rr[1] * fv[2] - rr[2] * fv[1], rr[2] * fv[0] - rr[0] * fv[2], rr[0] * fv[1] - rr[1] * fv[0]]; if (uu[2] < 0) uu = [-uu[0], -uu[1], -uu[2]];
        return { C: C, f: fv, r: rr, u: uu, D: D };
      };
      var B = basis(self.az + osc), B0 = basis(self.az), sc = Math.min(W / 2 - mx, H / 2 - my) * 0.95 / 0.238;
      var proj = function (b, p) { var d = [p[0] - b.C[0], p[1] - b.C[1], p[2] - b.C[2]], dep = d[0] * b.f[0] + d[1] * b.f[1] + d[2] * b.f[2]; return [(d[0] * b.r[0] + d[1] * b.r[1] + d[2] * b.r[2]) / dep, (d[0] * b.u[0] + d[1] * b.u[1] + d[2] * b.u[2]) / dep, dep]; };
      var P = function (p) { var q = proj(B, p); return [W / 2 + q[0] * sc, H / 2 - q[1] * sc, q[2]]; };
      var near = B.D * 0.05;

      /* ---- fond : points, sol quadrille ---- */
      ctx.clearRect(0, 0, W, H); ctx.shadowBlur = 0; ctx.setLineDash([]);
      ctx.fillStyle = rgba(CYAN, 0.16); for (var gx = 8; gx < W; gx += 18) for (var gy = 8; gy < H; gy += 18) ctx.fillRect(gx, gy, 1, 1);
      ctx.strokeStyle = rgba(CYAN, 0.15); ctx.lineWidth = 1; ctx.beginPath();
      var gline = function (a, b) { var p1 = P(a), p2 = P(b); if (p1[2] > near && p2[2] > near) { ctx.moveTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]); } };
      for (var gi = -0.30; gi <= 0.30 + 1e-9; gi += 0.05) gline([gi, -0.25, groundZ], [gi, 0.25, groundZ]);
      for (var gj = -0.25; gj <= 0.25 + 1e-9; gj += 0.05) gline([-0.30, gj, groundZ], [0.30, gj, groundZ]);
      ctx.stroke();

      /* ---- fantome du parent (niveau eclate) : contour pointille, couleur de son etat ---- */
      if (f0 !== m.root && m.shapes[f0]) {
        var gc = fstate ? (fstate.lv === 2 ? alert : amber) : CYAN;
        ctx.strokeStyle = rgba(gc, fstate ? 0.5 : 0.28); ctx.lineWidth = 1; ctx.setLineDash([2, 4]); ctx.beginPath();
        edges(f0).lines.forEach(function (l) { l.forEach(function (p, q) { var s = P(p); q ? ctx.lineTo(s[0], s[1]) : ctx.moveTo(s[0], s[1]); }); }); ctx.stroke(); ctx.setLineDash([]);
      }

      /* ---- blocs : geometrie decalee par l'eclatement, du plus loin au plus proche ---- */
      var blocks = ids.map(function (id, n) {
        var e = edges(id), o = offs[n], sh = m.shapes[id], c = P([sh.t[0] + o[0], sh.t[1] + o[1], sh.t[2] + o[2]]);
        return { id: id, sh: sh, off: o, e: e, cx: c[0], cy: c[1], dep: c[2], home: P(sh.t), shell: isShell(sh) };
      });
      blocks.sort(function (a, b) { return (b.shell - a.shell) || (b.dep - a.dep); });                 // la coque d'abord (au fond), puis du plus loin au plus proche
      var dmin = 1e9, dmax = -1e9; blocks.forEach(function (b) { dmin = Math.min(dmin, b.dep); dmax = Math.max(dmax, b.dep); });
      var zlo = 1e9, zhi = -1e9; blocks.forEach(function (b) { b.e.pts.forEach(function (p) { zlo = Math.min(zlo, p[2] + b.off[2]); zhi = Math.max(zhi, p[2] + b.off[2]); }); });
      var scanOn = !reduce, zs = zhi - (zhi - zlo) * ((now / 4500) % 1), band = Math.max(0.008, (zhi - zlo) * 0.07), glow = [], drawn = [];
      hits = [];
      blocks.forEach(function (b) {
        var st = state[b.id], role = st && st.role, isHov = hov === b.id, isSel = sel === b.id;
        var col = st ? (st.lv === 2 ? alert : amber) : CYAN; if (isHov || isSel) col = lighten(col, isSel ? 0.7 : 0.55);
        var nearK = dmax > dmin ? 1 - (b.dep - dmin) / (dmax - dmin) : 1, al = (0.55 + 0.45 * nearK) * ((role === "root" || role === "cons") ? pulse : 1);
        var ghost = b.shell && !role && !isHov && !isSel;                                  // coque saine : translucide, pour laisser voir ce qu'elle contient
        if (ghost) al *= 0.38;
        var o = b.off, tp = function (p) { return P([p[0] + o[0], p[1] + o[1], p[2] + o[2]]); }, scr = b.e.lines.map(function (l) { return l.map(tp); }), hp = hull2(b.e.pts.map(tp));
        if (hp.length > 2) {
          ctx.beginPath(); hp.forEach(function (q, n) { n ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }); ctx.closePath();
          ctx.fillStyle = rgba(col, ((role === "root" ? 0.30 : role === "cons" ? 0.17 : role === "impact" ? 0.08 : ghost ? 0.012 : 0.05) * ((role === "root" || role === "cons") ? 0.45 + 1.1 * pu : 1)) + (isHov ? 0.14 : 0)); ctx.fill();
        }
        if (k > 0.02 && (o[0] || o[1] || o[2])) {                                       // trait de rappel : de la piece eclatee vers sa place d'origine
          ctx.strokeStyle = rgba(CYAN, 0.28 * Math.min(1, k)); ctx.lineWidth = 1; ctx.setLineDash([1, 3]); ctx.beginPath(); ctx.moveTo(b.home[0], b.home[1]); ctx.lineTo(b.cx, b.cy); ctx.stroke(); ctx.setLineDash([]);
        }
        ctx.shadowColor = rgba(col, 0.9); ctx.shadowBlur = isHov ? 14 : role === "root" ? 12 : role ? 8 : ghost ? 2 : 5;
        ctx.strokeStyle = rgba(col, Math.min(1, (isHov || isSel ? 1 : al) * (role === "impact" ? 0.6 : 1))); ctx.lineWidth = (role === "root" ? 2 : role === "cons" ? 1.5 : 1.1) + (isHov ? 0.6 : 0);
        ctx.setLineDash(b.sh.estime ? [3, 3] : []);
        ctx.beginPath(); scr.forEach(function (l) { l.forEach(function (q, n) { n ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }); }); ctx.stroke();
        ctx.shadowBlur = 0; ctx.setLineDash([]);
        if (scanOn && !b.shell) {                                                          // scan (sauf la coque) : la PIECE ENTIERE s'allume d'un coup, arêtes et surface, pas segment par segment
          var pz0 = 1e9, pz1 = -1e9; b.e.pts.forEach(function (q) { pz0 = Math.min(pz0, q[2] + o[2]); pz1 = Math.max(pz1, q[2] + o[2]); });
          var dz = zs < pz0 ? pz0 - zs : zs > pz1 ? zs - pz1 : 0, bo = Math.exp(-Math.pow(dz / band, 2));         // 1 tant que le plan traverse la piece, puis decroit vite
          if (bo > 0.1) glow.push({ lines: scr, hull: hp, bo: bo });
        }
        if (self.zoomable(b.id)) {                                                       // marque « + » : ce bloc s'eclate
          ctx.strokeStyle = rgba(isHov ? WHITE : col, isHov ? 1 : 0.7); ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(b.cx - 4, b.cy); ctx.lineTo(b.cx + 4, b.cy); ctx.moveTo(b.cx, b.cy - 4); ctx.lineTo(b.cx, b.cy + 4); ctx.stroke();
        }
        hits.push({ id: b.id, poly: hp, cx: b.cx, cy: b.cy, dep: b.dep, area: polyArea(hp), zoom: self.zoomable(b.id) });
        drawn.push({ id: b.id, x: b.cx, y: b.cy, role: role || null, lv: st ? st.lv : 0, via: st ? st.via : null, estime: !!b.sh.estime, zoom: self.zoomable(b.id) });
      });
      if (scanOn) {
        ctx.shadowColor = rgba(CREAM, 0.9); ctx.shadowBlur = 10; ctx.lineWidth = 1.7;
        glow.forEach(function (g) {
          if (g.hull.length > 2) { ctx.beginPath(); g.hull.forEach(function (q, n) { n ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }); ctx.closePath(); ctx.fillStyle = rgba(CREAM, 0.22 * g.bo); ctx.fill(); }
          ctx.strokeStyle = rgba(CREAM, 0.9 * g.bo); ctx.beginPath(); g.lines.forEach(function (l) { l.forEach(function (q, n) { n ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }); }); ctx.stroke();
        });
        ctx.shadowBlur = 0;
        // le plan du scan n'est volontairement PAS dessine : seul son effet sur les pieces reste visible
      }

      /* ---- etiquettes : marges gauche/droite, traits fins vers le bloc. Niveau racine : seulement les incidents ; niveau eclate : toutes les pieces. ---- */
      ctx.font = "13px ui-monospace,Consolas,monospace"; ctx.textBaseline = "middle";
      var rootLevel = f0 === m.root, cand = drawn.filter(function (d) { return rootLevel ? (d.role === "root" || d.role === "cons") : true; }).map(function (d) {
        var b0 = proj(B0, blocks.filter(function (b) { return b.id === d.id; })[0].sh.t.map(function (v, n) { return v + blocks.filter(function (b) { return b.id === d.id; })[0].off[n]; }));
        return { d: d, side: b0[0] < 0 ? "L" : "R", key: b0[1] };         // cote et rang calcules sur la vue de BASE (sans oscillation) : pas de saut d'etiquette
      }).sort(function (a, b) { return b.key - a.key; });
      var labels = [], slot = { L: 44, R: 44 }; labelHits = [];
      cand.forEach(function (c) {
        var d = c.d, left = c.side === "L", inc = !!d.role, col = d.lv === 2 ? alert : d.lv === 1 ? amber : CYAN, id = d.id, part = m.byId[id] || {}, txt = shortLabel(part.label || id), cause = inc && d.via && d.via !== d.id ? shortLabel((m.byId[d.via] || {}).label || d.via) : "";
        var ly = slot[c.side]; slot[c.side] += inc ? 38 : 22; if (ly > H - 30) return;
        var tx = left ? 12 : W - 12, hot = d.id === hov || d.id === sel;
        ctx.strokeStyle = rgba(hot ? WHITE : col, inc ? 0.8 : 0.4); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(d.x, d.y); ctx.lineTo(left ? mx - 10 : W - mx + 10, ly + 6); ctx.lineTo(left ? 12 + ctx.measureText(txt).width : W - 12 - ctx.measureText(txt).width, ly + 6); ctx.stroke();
        ctx.fillStyle = rgba(col, inc ? 1 : 0.8); ctx.beginPath(); ctx.arc(d.x, d.y, inc ? 2.5 : 1.8, 0, Math.PI * 2); ctx.fill();
        ctx.textAlign = left ? "left" : "right"; ctx.fillStyle = rgba(hot ? WHITE : col, inc ? 1 : 0.85); ctx.fillText(txt, tx, ly - 4);
        if (inc) { ctx.font = "11px ui-monospace,Consolas,monospace"; ctx.fillStyle = rgba(col, 0.75); ctx.fillText(cause || id, tx, ly + 9); ctx.font = "13px ui-monospace,Consolas,monospace"; }
        var lw = Math.max(ctx.measureText(txt).width, inc ? 118 : 0) + 6, lx0 = left ? tx - 3 : tx - lw + 3, rect = { x0: lx0, y0: ly - 13, x1: lx0 + lw, y1: ly + (inc ? 16 : 5) };
        labelHits.push({ id: id, rect: rect });                                            // l'etiquette est cliquable comme le bloc (zoom) et l'eclaire au survol
        labels.push({ id: id, text: txt, side: c.side, rect: rect });
      });

      /* ---- infobulle de survol ---- */
      if (hov && ptr && !dragging) {
        var hp2 = m.byId[hov] || {}, kn = shapedKids(hov).length, lines = [shortLabel(hp2.label || hov), kn ? kn + " sous-composant" + (kn > 1 ? "s" : "") + " · clic pour zoomer" : "pièce terminale", m.shapes[hov] && m.shapes[hov].virtuel ? "module logiciel (position schématique)" : (m.shapes[hov] && m.shapes[hov].estime ? "cotes estimées" : "cotes de l'URDF")];
        ctx.font = "12px ui-monospace,Consolas,monospace"; ctx.textAlign = "left";
        var tw = Math.max.apply(null, lines.map(function (s) { return ctx.measureText(s).width; })) + 14, th = lines.length * 16 + 8, bx = Math.min(W - tw - 4, ptr[0] + 14), by = Math.min(H - th - 4, ptr[1] + 14);
        ctx.fillStyle = "rgba(4,8,11,0.88)"; ctx.fillRect(bx, by, tw, th); ctx.strokeStyle = rgba(CYAN, 0.7); ctx.lineWidth = 1; ctx.strokeRect(bx + 0.5, by + 0.5, tw, th);
        lines.forEach(function (s, n) { ctx.fillStyle = rgba(n ? CYAN : WHITE, n ? 0.85 : 1); ctx.fillText(s, bx + 7, by + 12 + n * 16); });
      }

      /* ---- bandeau bas ---- */
      ctx.textAlign = "left"; ctx.font = "11px ui-monospace,Consolas,monospace"; ctx.fillStyle = rgba(CYAN, 0.75);
      ctx.fillText("BAMBOO // VUE 3/4 // " + String((m.byId[f0] || {}).label || f0).toUpperCase().slice(0, 30), 10, H - 10);
      if (unplaced.length) {
        ctx.textAlign = "right"; unplaced.forEach(function (q, n) { ctx.fillStyle = rgba(q.lv === 2 ? alert : amber, 1); ctx.fillText("▲ " + shortLabel(q.label) + " · hors modèle 3D", W - 10, H - 10 - n * 13); });
        ctx.textAlign = "left";
      }
      if (drawn.some(function (q) { return q.estime; })) { ctx.setLineDash([3, 3]); ctx.strokeStyle = rgba(CYAN, 0.8); ctx.strokeRect(10, H - 32, 14, 8); ctx.setLineDash([]); ctx.fillStyle = rgba(CYAN, 0.75); ctx.fillText("cotes estimées (non mesurées)", 30, H - 27); }
      self.info = { drawn: drawn, unplaced: unplaced, labels: labels, states: state, focus: f0, path: path.slice(), hover: hov, selected: sel, k: k, animating: !!anim, osc: osc, missing: m.missing, w: W, h: H };
    };

    /* ---------------------------------------------------------------- souris / tactile */
    var local = function (e) { var r = canvas.getBoundingClientRect(); return [(e.clientX - r.left) * (self.w / (r.width || 1)), (e.clientY - r.top) * (self.h / (r.height || 1))]; };
    var hitAt = function (p) {
      var lab = null; labelHits.forEach(function (l) { if (p[0] >= l.rect.x0 && p[0] <= l.rect.x1 && p[1] >= l.rect.y0 && p[1] <= l.rect.y1) lab = l; });
      if (lab) return { id: lab.id, zoom: self.zoomable(lab.id), label: true };
      return pickHit(hits, p);
    };
    canvas.addEventListener("pointerdown", function (e) {
      var p = local(e); pdown = { x: p[0], y: p[1], az: self.az, el: self.el, moved: false };
      try { canvas.setPointerCapture(e.pointerId); } catch (er) { }
    });
    canvas.addEventListener("pointermove", function (e) {
      var p = local(e); ptr = p;
      if (pdown) {
        if (Math.hypot(p[0] - pdown.x, p[1] - pdown.y) > 4) pdown.moved = true;
        if (pdown.moved) { self.az = pdown.az - (p[0] - pdown.x) * 0.4; self.el = Math.max(-5, Math.min(80, pdown.el + (p[1] - pdown.y) * 0.3)); canvas.style.cursor = "grabbing"; }
      } else { var h = hitAt(p); hov = h ? h.id : null; canvas.style.cursor = h ? (h.zoom ? "zoom-in" : "pointer") : "grab"; }
      if (reduce) self.draw();
    });
    var release = function (e) {
      if (!pdown) return; var wasMoved = pdown.moved; pdown = null; idleUntil = now0() + 2500;
      try { canvas.releasePointerCapture(e.pointerId); } catch (er) { }
      if (wasMoved) { canvas.style.cursor = "grab"; self.draw(); return; }
      var h = hitAt(local(e));
      if (h) { if (h.zoom) self.zoomTo(h.id); else { sel = h.id; if (opts.onPick) opts.onPick(h.id); self.draw(); } }
      else self.back();                                                                // clic dans le vide : un niveau en arriere
    };
    canvas.addEventListener("pointerup", release);
    canvas.addEventListener("pointercancel", function () { pdown = null; });
    canvas.addEventListener("pointerleave", function () { hov = null; ptr = null; if (reduce) self.draw(); });
    canvas.addEventListener("wheel", function (e) { e.preventDefault(); self.zoom = Math.max(0.55, Math.min(1.8, self.zoom * (e.deltaY > 0 ? 1.08 : 0.92))); idleUntil = now0() + 2500; if (reduce) self.draw(); }, { passive: false });
    canvas.style.touchAction = "none"; canvas.style.cursor = "grab";

    var tick = function (ts) {
      if (dead) return; raf = requestAnimationFrame(tick);
      if (document.hidden || ts - lastT < 33) return;                                  // ~30 images/s, et rien quand l'onglet est masque
      lastT = ts; self.draw(ts);
    };
    if (typeof ResizeObserver !== "undefined") { ro = new ResizeObserver(function () { self.resize(); }); ro.observe(canvas); }
    self.resize();
    if (!reduce && typeof requestAnimationFrame !== "undefined") raf = requestAnimationFrame(tick);
    return self;
  }

  root.Robot3D = { create: create, _t: { rotRpy: rotRpy, mmul: mmul, apply: apply, compose: compose, edgesOf: edgesOf, hull2: hull2, shapeFromVisuel: shapeFromVisuel, directStates: directStates, levelState: levelState, levelAncestor: levelAncestor, buildModel: buildModel, explodeOffsets: explodeOffsets, pointInPoly: pointInPoly, pickHit: pickHit, polyArea: polyArea, isShell: isShell, parseUrdf: parseUrdf } };
  if (typeof module !== "undefined" && module.exports) module.exports = root.Robot3D;
})(typeof window !== "undefined" ? window : this);
