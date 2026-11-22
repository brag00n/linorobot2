/* partmd.js : mini-rendu Markdown du slide « Maintenance d'une Part » (2026-11-22). Aucune dependance, aucun HTML interprete.

   Le texte est LU comme du Markdown puis traduit en un ARBRE de noeuds (parse), puis seulement construit dans le DOM avec createElement/textContent (render).
   Rien ne passe par innerHTML : un fichier .md (ou un message du robot) qui contient « <img onerror=...> » s'affiche tel quel, en texte.

   Sous-ensemble volontairement reduit : titres (# a ####), paragraphes, listes a puces et numerotees, citations (>), blocs de code (```), regle (---),
   tableaux simples (| a | b |), et en ligne : **gras**, *italique*, `code`, [texte](cible). Ce qui sort de ce sous-ensemble reste du texte brut plutot que faux.

   Liens : `part:<id>` (fiche d'une autre part, ouvre dans le slide), `#ancre` (defile vers un titre du document), `http(s)://` (nouvel onglet, noopener).
   Tout autre schema (javascript:, data:, file:, //hote...) est REFUSE : le lien devient du texte simple, marque `refuse`. */
(function (root) {
  "use strict";

  var SAFE_URL = /^https?:\/\/[^\s<>"']+$/i, PART_ID = /^[a-z0-9_]+$/;

  /* ---------------------------------------------------------------- cible d'un lien -> {type, ...} ou null (refuse) */
  function linkTarget(raw) {
    var t = String(raw == null ? "" : raw).trim();
    if (!t || /[\u0000-\u001f\u007f]/.test(t)) return null;
    if (t.indexOf("part:") === 0) { var id = t.slice(5); return PART_ID.test(id) ? { type: "part", id: id } : null; }
    if (t.charAt(0) === "#") { var a = t.slice(1); return /^[\w\-]+$/.test(a) ? { type: "anchor", id: a } : null; }
    if (SAFE_URL.test(t)) return { type: "url", href: t };
    return null;
  }

  /* ---------------------------------------------------------------- en ligne : [{t:"text",v}, {t:"b",c:[..]}, {t:"i",c}, {t:"code",v}, {t:"a",c,target,raw}] */
  function inline(s) {
    var out = [], i = 0, buf = "";
    var flush = function () { if (buf) { out.push({ t: "text", v: buf }); buf = ""; } };
    while (i < s.length) {
      var c = s.charAt(i);
      if (c === "\\" && i + 1 < s.length && "\\`*_[]()#".indexOf(s.charAt(i + 1)) >= 0) { buf += s.charAt(i + 1); i += 2; continue; }
      if (c === "`") {                                                        // code : jamais interprete a l'interieur
        var e = s.indexOf("`", i + 1);
        if (e > i) { flush(); out.push({ t: "code", v: s.slice(i + 1, e) }); i = e + 1; continue; }
      }
      if (c === "*" && s.charAt(i + 1) === "*") {
        var e2 = s.indexOf("**", i + 2);
        if (e2 > i + 2) { flush(); out.push({ t: "b", c: inline(s.slice(i + 2, e2)) }); i = e2 + 2; continue; }
      }
      if (c === "*" && s.charAt(i + 1) !== "*" && s.charAt(i + 1) !== " ") {
        var e3 = s.indexOf("*", i + 1);
        if (e3 > i + 1 && s.charAt(e3 - 1) !== " ") { flush(); out.push({ t: "i", c: inline(s.slice(i + 1, e3)) }); i = e3 + 1; continue; }
      }
      if (c === "[") {
        var close = s.indexOf("](", i + 1), end = close > 0 ? s.indexOf(")", close + 2) : -1;
        if (close > i && end > close) {
          flush();
          var label = s.slice(i + 1, close), raw = s.slice(close + 2, end);
          out.push({ t: "a", c: inline(label), raw: raw, target: linkTarget(raw) });
          i = end + 1; continue;
        }
      }
      buf += c; i++;
    }
    flush();
    return out;
  }

  /* ---------------------------------------------------------------- blocs */
  function isTableSep(l) { return /^\s*\|?\s*:?-{2,}:?\s*(\|\s*:?-{2,}:?\s*)*\|?\s*$/.test(l); }
  function cells(l) { var t = l.trim(); if (t.charAt(0) === "|") t = t.slice(1); if (t.charAt(t.length - 1) === "|") t = t.slice(0, -1); return t.split("|").map(function (x) { return x.trim(); }); }
  function slug(s) {
    return String(s).toLowerCase().normalize("NFD").replace(/[̀-ͯ]/g, "").replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "") || "section";
  }

  function parse(md) {
    var lines = String(md == null ? "" : md).replace(/\r\n?/g, "\n").split("\n"), out = [], i = 0, used = {};
    var anchor = function (text) { var b = slug(text), n = b, k = 2; while (used[n]) n = b + "-" + (k++); used[n] = true; return n; };
    while (i < lines.length) {
      var l = lines[i], m;
      if (!l.trim()) { i++; continue; }
      if ((m = /^```\s*([\w\-]*)\s*$/.exec(l))) {                              // bloc de code
        var code = []; i++;
        while (i < lines.length && !/^```\s*$/.test(lines[i])) code.push(lines[i++]);
        i++; out.push({ t: "pre", lang: m[1] || "", v: code.join("\n") }); continue;
      }
      if ((m = /^(#{1,4})\s+(.*?)\s*#*\s*$/.exec(l))) { out.push({ t: "h", n: m[1].length, c: inline(m[2]), id: anchor(m[2]) }); i++; continue; }
      if (/^\s*(-{3,}|\*{3,}|_{3,})\s*$/.test(l)) { out.push({ t: "hr" }); i++; continue; }
      if (/^>\s?/.test(l)) {                                                  // citation
        var q = []; while (i < lines.length && /^>\s?/.test(lines[i])) q.push(lines[i++].replace(/^>\s?/, ""));
        out.push({ t: "quote", c: parse(q.join("\n")).nodes }); continue;
      }
      if (l.indexOf("|") >= 0 && i + 1 < lines.length && isTableSep(lines[i + 1])) {   // tableau
        var head = cells(l).map(inline), rows = []; i += 2;
        while (i < lines.length && lines[i].indexOf("|") >= 0 && lines[i].trim()) rows.push(cells(lines[i++]).map(inline));
        out.push({ t: "table", head: head, rows: rows }); continue;
      }
      if ((m = /^(\s*)([-*+]|\d+[.)])\s+(.*)$/.exec(l))) {                    // liste (un seul niveau, la suite indentee prolonge l'item)
        var ordered = /\d/.test(m[2]), items = [];
        while (i < lines.length && (m = /^(\s*)([-*+]|\d+[.)])\s+(.*)$/.exec(lines[i])) && /\d/.test(m[2]) === ordered) {
          var txt = m[3]; i++;
          while (i < lines.length && /^\s{2,}\S/.test(lines[i]) && !/^\s*([-*+]|\d+[.)])\s+/.test(lines[i])) txt += " " + lines[i++].trim();
          items.push(inline(txt));
        }
        out.push({ t: ordered ? "ol" : "ul", items: items }); continue;
      }
      var p = [l.trim()]; i++;                                                // paragraphe : lignes consecutives
      while (i < lines.length && lines[i].trim() && !/^(#{1,4}\s|```|>|\s*([-*+]|\d+[.)])\s+|\s*(-{3,}|\*{3,})\s*$)/.test(lines[i]) && !(lines[i].indexOf("|") >= 0 && i + 1 < lines.length && isTableSep(lines[i + 1]))) p.push(lines[i++].trim());
      out.push({ t: "p", c: inline(p.join(" ")) });
    }
    return { nodes: out, anchors: Object.keys(used) };
  }

  /* ---------------------------------------------------------------- liens extraits (pour les tests de coherence et le chargement des fiches) */
  function collectLinks(nodes, acc) {
    acc = acc || [];
    var walk = function (list) { (list || []).forEach(function (n) { if (n.t === "a") acc.push(n); if (n.c) walk(n.c); }); };
    (nodes || []).forEach(function (n) {
      if (n.c && n.t !== "quote") walk(n.c);
      if (n.t === "quote") collectLinks(n.c, acc);
      (n.items || []).forEach(walk); (n.head || []).forEach(walk); (n.rows || []).forEach(function (r) { r.forEach(walk); });
    });
    return acc;
  }

  /* ---------------------------------------------------------------- DOM : uniquement createElement + textContent */
  function render(doc, md, opts) {
    opts = opts || {};
    var d = doc, tree = parse(md), root = d.createElement("div"); root.className = "pm-md";
    var el = function (tag, cls, txt) { var e = d.createElement(tag); if (cls) e.className = cls; if (txt != null) e.textContent = txt; return e; };
    var fill = function (parent, nodes) {
      nodes.forEach(function (n) {
        if (n.t === "text") parent.appendChild(d.createTextNode(n.v));
        else if (n.t === "code") parent.appendChild(el("code", "", n.v));
        else if (n.t === "b") { var b = el("strong"); fill(b, n.c); parent.appendChild(b); }
        else if (n.t === "i") { var it = el("em"); fill(it, n.c); parent.appendChild(it); }
        else if (n.t === "a") {
          var tg = n.target;
          if (!tg) { var bad = el("span", "pm-badlink"); bad.title = "lien refusé : " + n.raw; fill(bad, n.c); parent.appendChild(bad); return; }
          var a = el("a", "pm-link pm-" + tg.type);
          if (tg.type === "url") { a.href = tg.href; a.target = "_blank"; a.rel = "noopener noreferrer"; }
          else {
            a.href = "#"; a.setAttribute("role", "link");
            a.addEventListener("click", function (ev) { ev.preventDefault(); if (tg.type === "part" && opts.onPart) opts.onPart(tg.id); else if (tg.type === "anchor") { var t = root.querySelector('[data-anchor="' + tg.id + '"]'); if (t && t.scrollIntoView) t.scrollIntoView({ block: "start" }); } });
          }
          fill(a, n.c); parent.appendChild(a);
        }
      });
    };
    tree.nodes.forEach(function (n) {
      var e;
      if (n.t === "h") { e = el("h" + Math.min(6, n.n + 1), "pm-h"); e.dataset.anchor = n.id; fill(e, n.c); }
      else if (n.t === "p") { e = el("p"); fill(e, n.c); }
      else if (n.t === "ul" || n.t === "ol") { e = el(n.t); n.items.forEach(function (it) { var li = el("li"); fill(li, it); e.appendChild(li); }); }
      else if (n.t === "pre") { e = el("pre"); e.appendChild(el("code", "", n.v)); }
      else if (n.t === "hr") e = el("hr");
      else if (n.t === "quote") { e = el("blockquote"); fillNodes(e, n.c); }
      else if (n.t === "table") {
        e = el("table"); var th = el("thead"), tr = el("tr"); n.head.forEach(function (c) { var x = el("th"); fill(x, c); tr.appendChild(x); }); th.appendChild(tr); e.appendChild(th);
        var tb = el("tbody"); n.rows.forEach(function (r) { var rr = el("tr"); r.forEach(function (c) { var x = el("td"); fill(x, c); rr.appendChild(x); }); tb.appendChild(rr); }); e.appendChild(tb);
      }
      if (e) root.appendChild(e);
    });
    function fillNodes(parent, nodes) {                                       // contenu d'une citation : memes regles, sous-arbre deja analyse
      nodes.forEach(function (n) {
        var e2;
        if (n.t === "p") { e2 = el("p"); fill(e2, n.c); } else if (n.t === "ul" || n.t === "ol") { e2 = el(n.t); n.items.forEach(function (it) { var li = el("li"); fill(li, it); e2.appendChild(li); }); }
        else if (n.t === "pre") { e2 = el("pre"); e2.appendChild(el("code", "", n.v)); } else if (n.t === "h") { e2 = el("h" + Math.min(6, n.n + 1), "pm-h"); fill(e2, n.c); }
        if (e2) parent.appendChild(e2);
      });
    }
    return root;
  }

  var api = { parse: parse, render: render, inline: inline, linkTarget: linkTarget, collectLinks: collectLinks, slug: slug };
  root.PartMd = api;
  if (typeof module !== "undefined" && module.exports) module.exports = api;
})(typeof window !== "undefined" ? window : this);
