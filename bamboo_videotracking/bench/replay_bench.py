#!/usr/bin/env python3
"""Banc REPRODUCTIBLE du groupe bamboo_videotracking : fps et latence par etage.

POURQUOI un banc et pas une observation a l'oeil : la cadence de detection depend de la
scene (nombre de visages, contraste, flou de bouge) et de l'eclairage. Comparer C++ contre
Python, ou detecteur A contre B, sur deux scenes differentes ne mesure rien. Ici la
SEQUENCE EST FIXE : ce sont les memes octets a chaque passe, injectes a la meme cadence.

Trois modes, et un seul des trois mesure :
  --record DIR    CAPTURE la sequence, ne mesure rien. A faire UNE FOIS, avant toute
                  comparaison : sans lui il n'y a pas de sequence figee et le banc n'a rien a
                  rejouer. La sequence vient de la VRAIE camera (memes optique, meme
                  quantification JPEG, meme eclairage) parce qu'un lot de JPEG venus
                  d'ailleurs ferait mesurer le decodage d'images etrangeres au robot.
  --images DIR    rejoue ce dossier en boucle sur le topic d'entree du groupe. C'est le mode
                  de COMPARAISON : memes octets a chaque passe.
  --observe-only  n'injecte rien et ne fait que mesurer ce qui passe -- pour une vraie
                  camera ou un `ros2 bag play` lance a cote. Honnete mais NON reproductible :
                  a n'utiliser que pour constater, jamais pour comparer deux variantes.

Ce que le banc mesure lui-meme (il ne fait pas confiance a /videotracking/stats, qui est
justement l'objet du controle) :
  - la cadence d'INJECTION reellement tenue (si elle s'effondre, la mesure est a jeter) ;
  - la cadence de chaque etage, comptee sur ses publications ;
  - la latence bout en bout, `now - header.stamp` de la trame ANNOTEE. C'est valide parce
    que le groupe REPORTE l'horodatage de la trame d'entree sans jamais le reecrire ;
  - le taux de perte de verrou, sur les TrackState recus.
`/videotracking/stats` est lu en PARALLELE et affiche a cote : un ecart entre les deux
colonnes est un bug de comptage du groupe, et c'est une information utile.

VERSIONNE DANS LE PAQUET, deliberement : les scripts de banc du chantier 1 vivaient dans
/tmp du conteneur et ont ete perdus avec lui.

Exemples (dans le conteneur) :
  # une seule fois, groupe VIDEO seul, un visage devant la camera :
  ros2 run bamboo_videotracking replay_bench.py --record /root/data/bench/seq01 --count 120
  # puis, a chaque variante a comparer, groupe TRACKING lance :
  ros2 run bamboo_videotracking replay_bench.py --images /root/data/bench/seq01 \
      --rate 30 --duration 30 --label cpp-mil-320 --json /root/data/bench/cpp.json

La sequence n'est PAS versionnee (des JPEG dans git, et le depot grossit a chaque essai) :
elle vit dans le volume nomme, comme la galerie de visages. Ce qui est versionne, c'est le
MOYEN de la refaire -- ce fichier.
"""
import argparse
import glob
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from bamboo_interfaces.msg import RecognitionResult, TrackState, TrackingStats


class _Counter:
    """Compteur de cadence : nombre d'evenements et fenetre de temps reellement couverte."""

    def __init__(self):
        self.n = 0
        self.first = None
        self.last = None

    def hit(self, t=None):
        t = time.monotonic() if t is None else t
        if self.first is None:
            self.first = t
        self.last = t
        self.n += 1

    @property
    def fps(self):
        # On divise par la fenetre OBSERVEE, pas par la duree demandee : un etage qui
        # demarre en retard ne doit pas voir sa cadence diluee par son propre demarrage.
        if self.n < 2 or self.first is None or self.last <= self.first:
            return 0.0
        return (self.n - 1) / (self.last - self.first)


class SequenceRecorder(Node):
    """Capture une sequence figee depuis le topic d'entree. NE MESURE RIEN.

    Ecrit les JPEG TELS QU'ILS ARRIVENT, sans re-encoder : re-encoder changerait la taille des
    donnees, donc le cout de decodage, c'est-a-dire precisement l'etage qu'on veut mesurer
    ensuite. Corollaire : le groupe video doit publier en `jpeg` -- un autre format est REFUSE
    plutot que renomme en .jpg, sinon la sequence serait illisible au rejeu et on chercherait
    la panne dans le banc.

    A lancer avec le groupe VIDEO seul, sans le tracking : capturer pendant que le tracking
    tourne fait payer la capture a la chaine qu'on cherche a mesurer, et la sequence porterait
    alors la trace de sa propre mesure.
    """

    def __init__(self, args):
        super().__init__("bench_recorder")
        self.args = args
        self.n = 0
        os.makedirs(args.record, exist_ok=True)
        existing = glob.glob(os.path.join(args.record, "*.jpg"))
        if existing:
            # On REFUSE d'ecraser : une sequence deja capturee est la reference de toutes les
            # passes deja mesurees. La reecrire en silence invaliderait les comparaisons
            # anciennes sans que rien ne le signale.
            raise SystemExit(
                "%s contient deja %d .jpg : choisir un autre dossier, ou le vider "
                "explicitement si cette sequence ne sert plus de reference."
                % (args.record, len(existing)))
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(
            CompressedImage, args.input_topic, self._onFrame, qos)
        self.get_logger().info(
            "capture : %d trames depuis %s vers %s"
            % (args.count, args.input_topic, args.record))

    def _onFrame(self, msg):
        if self.n >= self.args.count:
            return
        fmt = (msg.format or "").lower()
        if "jpeg" not in fmt and "jpg" not in fmt:
            raise SystemExit(
                "format `%s` sur %s : ce banc rejoue du JPEG. Le groupe video tourne-t-il en "
                "`stream_mode:=mjpg` ? (en h264 le flux ne passe pas par ROS du tout)"
                % (msg.format, self.args.input_topic))
        with open(os.path.join(self.args.record, "%04d.jpg" % self.n), "wb") as fh:
            fh.write(bytes(msg.data))
        self.n += 1

    @property
    def done(self):
        return self.n >= self.args.count


class ReplayBench(Node):
    """Injecte la sequence et mesure. Ne depend d'AUCUN controleur : robot eteint suffit."""

    def __init__(self, args):
        super().__init__("replay_bench")
        self.args = args

        # reliable partout : `sensor_data` est best-effort, donc une trame perdue serait
        # comptee comme un decrochage du groupe alors que c'est le transport qui l'a jetee.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.frames = []
        if not args.observe_only:
            self.frames = self._loadImages(args.images)
            self.pub = self.create_publisher(CompressedImage, args.input_topic, qos)

        self.c_inject = _Counter()
        self.c_track = _Counter()
        self.c_recog = _Counter()
        self.c_overlay = _Counter()

        self.lat = []          # latences bout en bout, en ms
        self.locked = 0        # TrackState verrouilles
        self.last_stats = None

        self.create_subscription(TrackState, args.track_topic, self._onTrack, qos)
        self.create_subscription(
            RecognitionResult, args.recog_topic, lambda _m: self.c_recog.hit(), qos)
        self.create_subscription(
            CompressedImage, args.enriched_topic, self._onEnriched, qos)
        self.create_subscription(
            TrackingStats, args.stats_topic, self._onStats, qos)

        self.idx = 0
        self.t0 = time.monotonic()
        if not args.observe_only:
            self.create_timer(1.0 / max(args.rate, 0.1), self._inject)

    # ---------------------------------------------------------------- sequence

    def _loadImages(self, directory):
        """Charge la sequence EN MEMOIRE avant de commencer a mesurer.

        Deliberement : lire le disque pendant la mesure ajouterait la latence de la carte SD
        du RPi au resultat, et elle n'a rien a voir avec le groupe qu'on evalue.
        """
        paths = sorted(glob.glob(os.path.join(directory, "*.jpg"))
                       + glob.glob(os.path.join(directory, "*.jpeg")))
        if not paths:
            raise SystemExit("aucun .jpg dans %s : sequence introuvable" % directory)
        out = []
        for p in paths:
            with open(p, "rb") as fh:
                out.append(fh.read())
        self.get_logger().info("sequence : %d trames depuis %s" % (len(out), directory))
        return out

    def _inject(self):
        m = CompressedImage()
        # L'horodatage est la REFERENCE de latence : le groupe le reporte tel quel jusqu'a la
        # trame annotee. Le poser ici, et pas a la lecture du fichier, est ce qui rend la
        # mesure juste.
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "bench"
        m.format = "jpeg"
        m.data = self.frames[self.idx % len(self.frames)]
        self.idx += 1
        self.pub.publish(m)
        self.c_inject.hit()

    # ---------------------------------------------------------------- mesures

    def _onTrack(self, msg):
        self.c_track.hit()
        if msg.locked:
            self.locked += 1

    def _onEnriched(self, msg):
        self.c_overlay.hit()
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return          # etage qui n'a pas reporte l'horodatage : on ne l'invente pas
        now = self.get_clock().now()
        dt_ms = (now.nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)) / 1e6
        if 0.0 <= dt_ms < 10000.0:
            self.lat.append(dt_ms)

    def _onStats(self, msg):
        self.last_stats = msg

    # ---------------------------------------------------------------- synthese

    def report(self):
        lat = sorted(self.lat)
        n = len(lat)

        def pct(q):
            return lat[min(n - 1, int(q * n))] if n else 0.0

        r = {
            "label": self.args.label,
            "mode": "observe" if self.args.observe_only else "replay",
            "sequence": None if self.args.observe_only else os.path.abspath(self.args.images),
            "frames_in_sequence": len(self.frames),
            "duration_s": round(time.monotonic() - self.t0, 2),
            "rate_requested_fps": None if self.args.observe_only else self.args.rate,
            "inject_fps": round(self.c_inject.fps, 2),
            "detect_fps": round(self.c_track.fps, 2),
            "recog_fps": round(self.c_recog.fps, 2),
            "overlay_fps": round(self.c_overlay.fps, 2),
            "latency_ms_p50": round(pct(0.50), 1),
            "latency_ms_p95": round(pct(0.95), 1),
            "latency_ms_max": round(lat[-1], 1) if n else 0.0,
            "latency_samples": n,
            # Taux de perte de verrou mesure par le banc, sur les TrackState recus.
            "loss_rate": round(1.0 - (self.locked / self.c_track.n), 3) if self.c_track.n else None,
        }
        if self.last_stats is not None:
            # Colonne de CONTROLE : ce que le groupe dit de lui-meme. Un ecart avec les
            # colonnes ci-dessus est un bug de comptage du groupe, pas du banc.
            s = self.last_stats
            r["group_reported"] = {
                "capture_fps": round(s.capture_fps, 2),
                "detect_fps": round(s.detect_fps, 2),
                "recog_fps": round(s.recog_fps, 2),
                "overlay_fps": round(s.overlay_fps, 2),
                "latency_ms": round(s.latency_ms, 1),
                "latency_ms_max": round(s.latency_ms_max, 1),
                "loss_rate": round(s.loss_rate, 3),
                "dropped_frames": int(s.dropped_frames),
            }
        return r


def _printTable(r):
    print("")
    print("=== banc bamboo_videotracking : %s (%s, %.1f s) ==="
          % (r["label"], r["mode"], r["duration_s"]))
    if r["sequence"]:
        print("sequence : %s  (%d trames)" % (r["sequence"], r["frames_in_sequence"]))
    g = r.get("group_reported")
    print("")
    print("%-14s %10s %10s" % ("etage", "banc", "groupe"))
    print("-" * 36)
    rows = (("injection", "inject_fps", "capture_fps"),
            ("detection", "detect_fps", "detect_fps"),
            ("reconnaiss.", "recog_fps", "recog_fps"),
            ("incrustation", "overlay_fps", "overlay_fps"))
    for label, mine, theirs in rows:
        print("%-14s %10.2f %10s"
              % (label, r[mine], ("%.2f" % g[theirs]) if g else "-"))
    print("")
    print("latence bout en bout  p50 %.1f ms   p95 %.1f ms   max %.1f ms   (%d ech.)"
          % (r["latency_ms_p50"], r["latency_ms_p95"], r["latency_ms_max"],
             r["latency_samples"]))
    if r["loss_rate"] is not None:
        print("perte de verrou       %.1f %%" % (100.0 * r["loss_rate"]))
    if r["latency_samples"] == 0:
        print("!! aucune trame annotee recue : le groupe tourne-t-il, "
              "et overlay_node publie-t-il bien sur %s ?" % "l'enriched_topic attendu")
    if not r["mode"] == "observe" and r["rate_requested_fps"]:
        # Une injection qui ne tient pas la cadence demandee invalide la comparaison : le
        # groupe n'a pas ete sollicite pareil d'une passe a l'autre.
        ratio = r["inject_fps"] / r["rate_requested_fps"]
        if ratio < 0.9:
            print("!! injection a %.0f %% de la cadence demandee : resultat NON comparable"
                  % (100.0 * ratio))
    print("")

def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--images", help="dossier de JPEG rejoue en boucle (mode comparable)")
    src.add_argument("--observe-only", action="store_true",
                     help="ne rien injecter, mesurer ce qui passe (NON reproductible)")
    src.add_argument("--record", metavar="DIR",
                     help="CAPTURER la sequence dans ce dossier et sortir : ne mesure rien")
    p.add_argument("--count", type=int, default=120,
                   help="nombre de trames a capturer avec --record (defaut 120, soit 4 s a "
                        "30 fps -- assez pour un mouvement de tete complet, assez court pour "
                        "que la sequence tienne en memoire au rejeu)")
    p.add_argument("--rate", type=float, default=30.0,
                   help="cadence d'injection en fps (defaut 30, celle du passthrough MJPG)")
    p.add_argument("--duration", type=float, default=30.0,
                   help="duree de mesure en s (defaut 30)")
    p.add_argument("--warmup", type=float, default=3.0,
                   help="secondes ignorees au debut : chargement des .onnx, premiere "
                        "detection, montee en cache (defaut 3)")
    p.add_argument("--label", default="run",
                   help="etiquette de la passe, reportee dans la table et le JSON")
    p.add_argument("--json", dest="json_out",
                   help="ecrit la synthese en JSON (pour comparer deux passes)")
    p.add_argument("--input-topic", default="/video/raw/compressed")
    p.add_argument("--track-topic", default="/videotracking/track_state")
    p.add_argument("--recog-topic", default="/videotracking/recognition")
    p.add_argument("--enriched-topic", default="/videotracking/enriched/compressed")
    p.add_argument("--stats-topic", default="/videotracking/stats")
    args = p.parse_args(argv)

    rclpy.init()

    if args.record:
        # Chemin COURT et separe : l'enregistrement ne partage ni compteur ni rodage avec la
        # mesure. Les melanger dans une seule classe donnerait un objet dont la moitie des
        # champs est morte selon le mode.
        rec = SequenceRecorder(args)
        try:
            # Borne de temps EN PLUS du compte : si le topic ne publie pas, on sort en le
            # disant au lieu d'attendre indefiniment un flux qui n'existe pas.
            deadline = time.monotonic() + max(10.0, 3.0 * args.count / 30.0)
            while rclpy.ok() and not rec.done and time.monotonic() < deadline:
                rclpy.spin_once(rec, timeout_sec=0.1)
            if not rec.done:
                print("!! %d trames capturees sur %d : %s publie-t-il ? (verifier "
                      "`ros2 topic hz %s`)"
                      % (rec.n, args.count, args.input_topic, args.input_topic))
                return 1
            print("sequence capturee : %d trames dans %s"
                  % (rec.n, os.path.abspath(args.record)))
        finally:
            rec.destroy_node()
            rclpy.shutdown()
        return 0

    node = ReplayBench(args)
    try:
        # Rodage : on fait tourner la chaine, PUIS on remet les compteurs a zero. Sans cela
        # le chargement des .onnx (plusieurs centaines de ms) se retrouve dans la latence et
        # ecrase la mesure -- et la premiere passe paraitrait toujours la moins bonne.
        end = time.monotonic() + args.warmup
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.c_inject = _Counter()
        node.c_track = _Counter()
        node.c_recog = _Counter()
        node.c_overlay = _Counter()
        node.lat = []
        node.locked = 0
        node.t0 = time.monotonic()

        end = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        r = node.report()
        _printTable(r)
        if args.json_out:
            with open(args.json_out, "w", encoding="utf-8") as fh:
                json.dump(r, fh, indent=2, sort_keys=True)
            print("synthese ecrite dans %s" % args.json_out)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
