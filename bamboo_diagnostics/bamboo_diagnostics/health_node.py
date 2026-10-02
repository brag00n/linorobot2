"""bamboo_health - observateur des composants qu'on ne peut pas instrumenter.

Les noeuds du depot publient eux-memes leurs /diagnostics (diagnostic_updater). Ce noeud
couvre TOUT LE RESTE, de l'exterieur, et publie sur le meme topic sous le nom du composant
qu'il decrit -- jamais sous le sien : l'agregateur range les statuts par prefixe de nom, et
un statut "bamboo_health: gscam_node" atterrirait dans le mauvais module.

  - noeuds tiers (gscam_node, web_video_server, joy_linux_node, rosbridge, foxglove,
    robot_state_publisher) : PRESENCE dans le graphe ;
  - cadences deja mesurees ailleurs : /camera/capture_fps et /camera/stream_fps (publies par
    camera_fps, qui recoit deja le flux) et /gamepad/connected (bamboo_teleop). REGLE DU LOT :
    un signal se mesure LA OU IL EST DEJA CONSOMME -- ce noeud ne s'abonne a AUCUN flux image,
    la memoire partagee DDS etant coupee entre conteneurs (une trame qui traverse coute plein
    tarif) ;
  - la couche MATERIELLE de la manette : adaptateur Bluetooth et peripherique /dev/input/js0,
    CONSTATES par /sys et jamais ouverts (ce serait un second lecteur du peripherique de
    joy_linux_node) ;
  - MediaMTX, par ses metriques Prometheus (lecture seule, cf. docker/mediamtx.yml) ;
  - le systeme du RPi : temperature, alimentation (get_throttled), charge, memoire, disque.

NIVEAUX : ERROR = le composant ne peut pas faire son travail ; WARN = degrade ou a surveiller ;
OK = nominal, y compris un etat INFORMATIF. Une manette absente est un WARN : elle est
optionnelle, et la signaler en ERROR rendrait le robot "en panne" chaque fois qu'on la range.
"""

import os
import shutil
import time
import urllib.request

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

# Bits de get_throttled (firmware Raspberry Pi). Les quatre bas disent ce qui se passe
# MAINTENANT, les quatre hauts ce qui s'est passe DEPUIS LE BOOT : d'ou deux niveaux distincts.
# La sous-tension est le signal qui compte ici -- le RPi est alimente par un BEC sur le pack 3S,
# et une chute de tension pendant un pic des moteurs se lit ICI, nulle part ailleurs.
THROTTLE_NOW = {
    0: "sous-tension",
    1: "frequence ARM plafonnee",
    2: "bridage en cours",
    3: "limite thermique douce",
}
THROTTLE_PAST = {
    16: "sous-tension",
    17: "frequence ARM plafonnee",
    18: "bridage",
    19: "limite thermique douce",
}


def kv(key, value):
    return KeyValue(key=str(key), value=str(value))


def status(name, hwid, level, message, values=()):
    s = DiagnosticStatus()
    s.name = name
    s.hardware_id = hwid
    s.level = level
    s.message = message
    s.values = [kv(k, v) for k, v in values]
    return s


def read_text(path):
    """Lecture d'un pseudo-fichier /sys ou /proc ; None s'il n'existe pas ou est illisible."""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read().strip()
    except OSError:
        return None


class RateCounter:
    """Compte les messages d'un topic entre deux periodes et retient l'heure du dernier."""

    def __init__(self):
        self.n = 0
        self.last = None
        self.value = None

    def tick(self, value=None):
        self.n += 1
        self.last = time.monotonic()
        if value is not None:
            self.value = value

    def take(self, period_s):
        hz = self.n / period_s if period_s > 0 else 0.0
        self.n = 0
        return hz

    def age(self):
        return None if self.last is None else time.monotonic() - self.last


class HealthNode(Node):

    def __init__(self):
        super().__init__("bamboo_health")
        p = self.declare_parameter
        self._period = max(0.2, float(p("period_s", 1.0).value))
        # Au demarrage, la decouverte DDS met quelques secondes a peupler le graphe : sans ce
        # delai, tous les composants tiers seraient "absents" a chaque boot, ce qui apprendrait
        # a ignorer le diagnostic des le premier jour.
        self._grace = float(p("startup_grace_s", 15.0).value)
        # Noeud a surveiller -> service compose (module) qui le porte.
        names = list(p("watched_nodes", [
            "gscam_node", "web_video_server", "camera_fps", "joy_linux_node",
            "rosbridge_websocket", "foxglove_bridge", "robot_state_publisher"]).value)
        modules = list(p("watched_modules", [
            "bamboo_video", "bamboo_video", "bamboo_video", "bamboo_control",
            "rosbridge", "foxglovebridge", "robotdesc"]).value)
        if len(names) != len(modules):
            raise ValueError("watched_nodes et watched_modules doivent avoir la meme longueur")
        self._watched = list(zip(names, modules))

        # Video : cadences deja mesurees par camera_fps.
        self._capWarn = float(p("capture_warn_hz", 5.0).value)
        self._streamWarn = float(p("stream_warn_hz", 5.0).value)
        self._fpsStale = float(p("fps_stale_s", 3.0).value)

        # Manette : la couche materielle, du plus bas au plus haut.
        self._hciPath = p("bt_adapter_path", "/sys/class/bluetooth/hci0").value
        self._jsDev = p("gamepad_device", "/dev/input/js0").value
        self._jsSys = p("gamepad_sysfs", "/sys/class/input/js0/device").value
        # Le NOM sert a reconnaitre le piege Home+Y : la meme manette s'annonce alors
        # "Pro Controller", se connecte, cree js0... et n'emet RIEN (hid-nintendo absent du
        # noyau). Present mais muet : sans ce controle, rien ne le distinguerait d'une manette
        # au repos.
        self._jsName = p("expected_gamepad_name", "GamepadX").value
        self._gpRateWarn = float(p("gamepad_state_warn_hz", 1.0).value)

        # MediaMTX, par ses metriques (lecture seule).
        self._mtxUrl = p("mediamtx_metrics_url", "http://camera.h264:9998/metrics").value
        self._mtxTimeout = float(p("mediamtx_timeout_s", 1.0).value)

        # Systeme du RPi. Seuils a recaler a T26 sur les mesures reelles.
        self._tempWarn = float(p("cpu_temp_warn_c", 70.0).value)
        self._tempErr = float(p("cpu_temp_error_c", 80.0).value)
        self._loadWarn = float(p("load_per_core_warn", 1.0).value)
        self._loadErr = float(p("load_per_core_error", 1.5).value)
        self._memWarn = float(p("mem_available_warn_pct", 15.0).value)
        self._memErr = float(p("mem_available_error_pct", 5.0).value)
        self._diskPath = p("disk_path", "/").value
        self._diskWarn = float(p("disk_used_warn_pct", 85.0).value)
        self._diskErr = float(p("disk_used_error_pct", 95.0).value)

        self._t0 = time.monotonic()
        self._cap = RateCounter()
        self._stream = RateCounter()
        self._gp = RateCounter()
        self.create_subscription(
            Float32, p("capture_fps_topic", "/camera/capture_fps").value,
            lambda m: self._cap.tick(m.data), 10)
        self.create_subscription(
            Float32, p("stream_fps_topic", "/camera/stream_fps").value,
            lambda m: self._stream.tick(m.data), 10)
        self.create_subscription(
            Bool, p("gamepad_state_topic", "/gamepad/connected").value,
            lambda m: self._gp.tick(m.data), 10)

        self._pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(self._period, self._onTick)
        self.get_logger().info(
            "bamboo_health pret : %d noeuds surveilles, metriques MediaMTX %s, periode %.1f s"
            % (len(self._watched), self._mtxUrl, self._period))

    # ------------------------------------------------------------------ cycle
    def _onTick(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = arr.status
        starting = (time.monotonic() - self._t0) < self._grace
        st.extend(self._presence(starting))
        st.extend(self._video())
        st.extend(self._gamepad())
        st.append(self._mediamtx())
        st.extend(self._system())
        st.append(status("bamboo_health: observateur", "bamboo_diagnostics", OK,
                         "en marche", [("periode_s", self._period)]))
        self._pub.publish(arr)

    # --------------------------------------------------------------- presence
    def _presence(self, starting):
        alive = set(self.get_node_names())
        out = []
        for name, module in self._watched:
            if name in alive:
                out.append(status(name + ": presence", module, OK, "present dans le graphe"))
            elif starting:
                out.append(status(name + ": presence", module, WARN,
                                  "graphe en cours de decouverte (demarrage)"))
            else:
                out.append(status(name + ": presence", module, ERROR,
                                  "absent du graphe : service %s arrete ?" % module))
        return out

    # ------------------------------------------------------------------ video
    def _rateStatus(self, name, module, counter, warn_hz, what):
        """Cadence PUBLIEE par camera_fps (un Float32 par seconde), et non comptee ici."""
        age = counter.age()
        if age is None or age > self._fpsStale:
            # Silence de la MESURE, pas forcement du flux : c'est camera_fps qui manque.
            return status(name, module, WARN, "mesure absente (camera_fps arrete ?)",
                          [("age_s", "jamais" if age is None else "%.1f" % age)])
        fps = float(counter.value or 0.0)
        vals = [("fps", "%.1f" % fps), ("seuil_warn_hz", warn_hz)]
        if fps <= 0.0:
            return status(name, module, ERROR, "%s a l'arret (0 fps)" % what, vals)
        if fps < warn_hz:
            return status(name, module, WARN, "%s lent : %.1f fps" % (what, fps), vals)
        return status(name, module, OK, "%s : %.1f fps" % (what, fps), vals)

    def _video(self):
        return [
            self._rateStatus("gscam_node: cadence capture", "bamboo_video", self._cap,
                             self._capWarn, "capture"),
            self._rateStatus("web_video_server: cadence flux", "bamboo_video", self._stream,
                             self._streamWarn, "flux web"),
        ]

    # ---------------------------------------------------------------- manette
    def _gamepad(self):
        out = []
        # 1. Adaptateur Bluetooth, la couche la plus basse.
        if not os.path.isdir(self._hciPath):
            out.append(status("bluetooth: adaptateur", "bamboo_control", ERROR,
                              "aucun adaptateur (%s) : radio coupee au firmware, cf. etape 1 "
                              "de install_bambooWS.sh" % self._hciPath))
        else:
            blocked = self._btBlocked()
            if blocked:
                out.append(status("bluetooth: adaptateur", "bamboo_control", ERROR,
                                  "adaptateur bloque par rfkill (%s)" % blocked))
            else:
                out.append(status("bluetooth: adaptateur", "bamboo_control", OK,
                                  "adaptateur present", [("chemin", self._hciPath)]))
        # 2. Peripherique js0 et identite de la manette.
        if not os.path.exists(self._jsDev):
            out.append(status("js0: peripherique", "bamboo_control", WARN,
                              "manette absente (eteinte, hors de portee ou non appairee)"))
        else:
            name = read_text(os.path.join(self._jsSys, "name")) or "?"
            mac = read_text(os.path.join(self._jsSys, "uniq")) or "?"
            vals = [("nom", name), ("mac", mac), ("nom_attendu", self._jsName)]
            if self._jsName and name != self._jsName:
                out.append(status("js0: peripherique", "bamboo_control", WARN,
                                  "manette inattendue : %s (mode Home+Y ? il est connecte "
                                  "mais muet)" % name, vals))
            else:
                out.append(status("js0: peripherique", "bamboo_control", OK,
                                  "manette %s presente" % name, vals))
        # 5. Sortie /gamepad/connected de bamboo_teleop.
        hz = self._gp.take(self._period)
        age = self._gp.age()
        vals = [("hz", "%.1f" % hz), ("derniere_valeur", self._gp.value)]
        if age is None or age > 3.0:
            out.append(status("gamepad_state: sortie", "bamboo_control", ERROR,
                              "silence : bamboo_teleop ne publie plus (pastille MANETTE grise)",
                              vals))
        elif hz < self._gpRateWarn:
            out.append(status("gamepad_state: sortie", "bamboo_control", WARN,
                              "cadence basse : %.1f Hz" % hz, vals))
        else:
            out.append(status("gamepad_state: sortie", "bamboo_control", OK,
                              "publie (%.1f Hz)" % hz, vals))
        return out

    @staticmethod
    def _btBlocked():
        """Raison du blocage rfkill d'un emetteur Bluetooth, ou '' s'il ne l'est pas."""
        base = "/sys/class/rfkill"
        try:
            entries = os.listdir(base)
        except OSError:
            return ""
        for e in entries:
            if read_text(os.path.join(base, e, "type")) != "bluetooth":
                continue
            if read_text(os.path.join(base, e, "hard")) == "1":
                return "blocage materiel"
            if read_text(os.path.join(base, e, "soft")) == "1":
                return "blocage logiciel"
        return ""

    # --------------------------------------------------------------- MediaMTX
    def _mediamtx(self):
        name, module = "mediamtx: flux H.264", "camera.h264"
        try:
            with urllib.request.urlopen(self._mtxUrl, timeout=self._mtxTimeout) as r:
                text = r.read().decode("utf-8", "replace")
        except Exception as e:  # noqa: BLE001 -- toute panne de sonde dit la meme chose
            return status(name, module, ERROR, "metriques injoignables : %s" % e,
                          [("url", self._mtxUrl)])
        ready, readers = [], 0
        for line in text.splitlines():
            if line.startswith("paths{") and 'state="ready"' in line and line.endswith(" 1"):
                ready.append(line.split('name="', 1)[1].split('"', 1)[0])
            elif line.startswith("paths_readers{"):
                try:
                    readers += int(float(line.rsplit(" ", 1)[1]))
                except ValueError:
                    pass
        vals = [("chemins_publies", ",".join(ready) or "aucun"), ("lecteurs", readers)]
        # SOURCE A LA DEMANDE (mediamtx.yml) : sans lecteur, ffmpeg ne tourne pas et AUCUN
        # chemin n'est publie -- c'est l'etat NOMINAL, qui laisse /dev/video0 libre pour gscam.
        if not ready:
            return status(name, module, OK, "en veille (source a la demande, aucun lecteur)", vals)
        return status(name, module, OK, "diffuse %s" % ", ".join(ready), vals)

    # ---------------------------------------------------------------- systeme
    def _system(self):
        out = []
        hw = "systeme"
        # Temperature du SoC.
        t = read_text("/sys/class/thermal/thermal_zone0/temp")
        if t is None:
            out.append(status("systeme: temperature CPU", hw, WARN, "illisible"))
        else:
            c = int(t) / 1000.0
            lvl = ERROR if c >= self._tempErr else WARN if c >= self._tempWarn else OK
            out.append(status("systeme: temperature CPU", hw, lvl, "%.1f degC" % c,
                              [("celsius", "%.1f" % c), ("warn", self._tempWarn),
                               ("error", self._tempErr)]))
        # Alimentation et bridage.
        raw = read_text("/sys/devices/platform/soc/soc:firmware/get_throttled")
        if raw is None:
            out.append(status("systeme: alimentation", hw, WARN,
                              "get_throttled illisible (pas un RPi, ou noyau trop ancien)"))
        else:
            v = int(raw, 16)
            now = [txt for b, txt in THROTTLE_NOW.items() if v & (1 << b)]
            past = [txt for b, txt in THROTTLE_PAST.items() if v & (1 << b)]
            vals = [("get_throttled", "0x%x" % v)]
            if now:
                out.append(status("systeme: alimentation", hw, ERROR,
                                  "EN COURS : " + ", ".join(now), vals))
            elif past:
                out.append(status("systeme: alimentation", hw, WARN,
                                  "depuis le boot : " + ", ".join(past), vals))
            else:
                out.append(status("systeme: alimentation", hw, OK,
                                  "aucune sous-tension ni bridage", vals))
        # Charge CPU, ramenee au coeur.
        try:
            l1, l5, l15 = os.getloadavg()
            ncpu = os.cpu_count() or 1
            per = l1 / ncpu
            lvl = ERROR if per >= self._loadErr else WARN if per >= self._loadWarn else OK
            out.append(status("systeme: charge CPU", hw, lvl,
                              "%.2f par coeur (%d coeurs)" % (per, ncpu),
                              [("load1", "%.2f" % l1), ("load5", "%.2f" % l5),
                               ("load15", "%.2f" % l15), ("coeurs", ncpu)]))
        except OSError:
            out.append(status("systeme: charge CPU", hw, WARN, "loadavg illisible"))
        # Memoire disponible.
        mem = {}
        for line in (read_text("/proc/meminfo") or "").splitlines():
            k, _, rest = line.partition(":")
            try:
                mem[k] = int(rest.split()[0])
            except (ValueError, IndexError):
                pass
        if "MemTotal" in mem and "MemAvailable" in mem:
            pct = 100.0 * mem["MemAvailable"] / mem["MemTotal"]
            lvl = ERROR if pct <= self._memErr else WARN if pct <= self._memWarn else OK
            out.append(status("systeme: memoire", hw, lvl, "%.0f %% disponible" % pct,
                              [("disponible_mo", mem["MemAvailable"] // 1024),
                               ("total_mo", mem["MemTotal"] // 1024)]))
        else:
            out.append(status("systeme: memoire", hw, WARN, "meminfo illisible"))
        # Disque : la racine du CONTENEUR est l'overlay de Docker, donc le disque de l'hote.
        try:
            du = shutil.disk_usage(self._diskPath)
            pct = 100.0 * du.used / du.total
            lvl = ERROR if pct >= self._diskErr else WARN if pct >= self._diskWarn else OK
            out.append(status("systeme: disque", hw, lvl, "%.0f %% occupe" % pct,
                              [("libre_go", "%.1f" % (du.free / 1e9)),
                               ("total_go", "%.1f" % (du.total / 1e9))]))
        except OSError as e:
            out.append(status("systeme: disque", hw, WARN, "illisible : %s" % e))
        return out


def main(args=None):
    rclpy.init(args=args)
    node = HealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
