// Rejoue le VRAI aria-client.js (ARIA_CLIENT_JS=chemin, telecharge depuis le service : curl -k https://localhost:7861/aria-client.js) avec un faux fetch et un faux Audio.
// Verrouille la regle du CC : le client est TOUJOURS cree avec speak:true et SEUL mute() (bouton haut-parleur) decide. Cree en speak:false, mute(false) ne rend plus la voix (bogue du 2026-11-15).
// Lancement : ARIA_CLIENT_JS=./aria-client.js node docker/portail/cc/test/voice_flow.test.cjs
// Execute le VRAI aria-client.js avec un faux fetch (flux SSE contenant un bloc audio) et un faux Audio.
// Objectif : savoir QUAND le client demande l'audio au serveur et QUAND il le joue, selon les appels que fait le CC.
const AriaClient = require(process.env.ARIA_CLIENT_JS || "./aria-client.js");

function sse(events) { return events.map(([n, d]) => `event: ${n}\ndata: ${JSON.stringify(d)}\n\n`).join(""); }
function mk(opts) {
  const log = { bodies: [], plays: [], created: [] };
  const fetchFn = async (url, init) => {
    const body = JSON.parse(init.body); log.bodies.push({ url: url.replace(/^.*\/v1/, "/v1"), speak: body.speak });
    const evs = [["state", { state: "thinking" }], ["text", { text: "Bonjour" }]];
    if (body.speak) evs.push(["audio", { id: "a1", format: "wav", data: "AAAA", text: "Bonjour" }]);
    evs.push(["done", { ms: 5 }]);
    const enc = new TextEncoder().encode(sse(evs)); let sent = false;
    return { ok: true, status: 200, body: { getReader: () => ({ read: async () => sent ? { done: true } : (sent = true, { done: false, value: enc }) }) } };
  };
  class FakeAudio { constructor(u) { log.created.push(u.slice(0, 22)); this.volume = 1; }
    play() { log.plays.push("play"); setTimeout(() => this.onended && this.onended(), 5); return Promise.resolve(); } pause() { } }
  const c = AriaClient.create({ baseUrl: "https://x", apiKey: "ak_t", session: "s", fetch: fetchFn, Audio: FakeAudio, ...opts });
  return { c, log };
}
const wait = (ms) => new Promise((r) => setTimeout(r, ms));
(async () => {
  const R = {};
  // A. Reglage ACTUEL du CC : client cree avec speak:false + mute(true) ; l'utilisateur active ensuite le haut-parleur : mute(false)
  { const { c, log } = mk({ voice: true, speak: false }); c.mute(true); c.mute(false); await c.say("salut"); await wait(80);
    R["A_cree_speak_false_puis_haut_parleur_active"] = { demande_audio_au_serveur: log.bodies.map(b => b.speak), lectures: log.plays.length }; }
  // B. Reglage CORRIGE : client TOUJOURS cree avec speak:true ; le haut-parleur decide par mute()
  { const { c, log } = mk({ voice: true, speak: true }); c.mute(false); await c.say("salut"); await wait(80);
    R["B_cree_speak_true_haut_parleur_actif"] = { demande_audio_au_serveur: log.bodies.map(b => b.speak), lectures: log.plays.length }; }
  { const { c, log } = mk({ voice: true, speak: true }); c.mute(true); await c.say("salut"); await wait(80);
    R["C_cree_speak_true_haut_parleur_COUPE"] = { demande_audio_au_serveur: log.bodies.map(b => b.speak), lectures: log.plays.length }; }
  // D. Bascule en cours de session : coupe puis active, sans recreer le client
  { const { c, log } = mk({ voice: true, speak: true }); c.mute(true); await c.say("un"); await wait(60); c.mute(false); await c.say("deux"); await wait(80);
    R["D_coupe_puis_active_sans_recreer"] = { demande_audio_au_serveur: log.bodies.map(b => b.speak), lectures: log.plays.length }; }
  // E. Voix coupee AU MILIEU d'une phrase : interrupt() vide la file
  { const { c, log } = mk({ voice: true, speak: true }); c.mute(false); await c.say("salut"); c.mute(true); c.interrupt(); await wait(60);
    R["E_coupure_pendant_lecture"] = { lectures: log.plays.length, etat: c.state }; }
  for (const k in R) console.log(k.padEnd(48), JSON.stringify(R[k]));
})();
