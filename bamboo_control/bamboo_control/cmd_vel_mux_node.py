#!/usr/bin/env python3
"""cmd_vel_mux -- arbitrage des consignes de vitesse : la MANETTE garde toujours la main.

Trois producteurs de consignes et UN driver :
  manette (bamboo_teleop)          priorite 100
  calibration (bamboo_calibration)  priorite  90
  navigation (Nav2)                 priorite  10
Une consigne n'est TRANSMISE que si aucune source de priorite plus haute n'a parle depuis son
`timeout` : pousser le stick pendant une navigation donne la main a la manette, et la
navigation reprend `timeout` apres que la manette s'est tue (elle se tait des qu'on relache
le stick : trois zeros puis silence, cf. teleop_node). C'est le pilotage SEMI-AUTOMATIQUE.

MEME SEMANTIQUE ET MEME CONFIGURATION QUE twist_mux (topics.<nom>.{topic, timeout, priority}).
On ne l'utilise pas tel quel parce qu'il n'est pas dans l'image `hardware`, et la reconstruire
coute des heures sur le RPi (disque a 90 %). Passer a twist_mux plus tard = changer
l'executable, pas la configuration.

Les noms REELS des sujets viennent du wiring `control` (layout.yaml du robot), pousses par
control.launch.py. Le coeur (Mux) est en Python pur, teste sans ROS.
"""
import time


class Mux:
    """Sources {nom: (priorite, timeout_s)} ; dernier message et instant par source."""

    def __init__(self, sources):
        self.sources = dict(sources)
        self.last = {}

    def active(self, now):
        """Nom de la source qui a la main a `now` (None si toutes se taisent)."""
        live = [(p, n) for n, (p, to) in self.sources.items()
                if n in self.last and now - self.last[n] <= to]
        return max(live)[1] if live else None

    def offer(self, name, now):
        """Une consigne arrive de `name` : faut-il la transmettre ?"""
        self.last[name] = now
        return self.active(now) == name


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from diagnostic_msgs.msg import DiagnosticStatus
    from diagnostic_updater import Updater
    from geometry_msgs.msg import Twist

    class CmdVelMux(Node):
        def __init__(self):
            super().__init__("cmd_vel_mux")
            out = str(self.declare_parameter("output_topic", "/cmd_vel").value)
            overrides = getattr(self, "_parameter_overrides", {}) or {}
            names = sorted({k.split(".")[1] for k in overrides
                            if k.startswith("topics.") and len(k.split(".")) == 3})
            spec = {}
            for n in names:
                topic = str(self.declare_parameter("topics.%s.topic" % n, "").value)
                to = float(self.declare_parameter("topics.%s.timeout" % n, 0.5).value)
                prio = int(self.declare_parameter("topics.%s.priority" % n, 0).value)
                if topic:
                    spec[n] = (topic, prio, to)
            self.mux = Mux({n: (p, to) for n, (_t, p, to) in spec.items()})
            self.pub = self.create_publisher(Twist, out, 10)
            self.counts = {n: 0 for n in spec}
            for n, (topic, _p, _to) in spec.items():
                self.create_subscription(Twist, topic, lambda m, n=n: self._on(n, m), 10)
            self.updater = Updater(self)
            self.updater.setHardwareID("bamboo_control")
            self.updater.add("multiplexeur", self._diag)
            self.get_logger().info("multiplexeur -> %s : %s" % (out, ", ".join(
                "%s=%s (p%d, %.1fs)" % (n, t, p, to) for n, (t, p, to) in
                sorted(spec.items(), key=lambda kv: -kv[1][1]))))

        def _on(self, name, msg):
            if self.mux.offer(name, time.monotonic()):
                self.pub.publish(msg)
                self.counts[name] += 1

        def _diag(self, stat):
            a = self.mux.active(time.monotonic())
            stat.add("source_active", a or "aucune")
            for n, c in self.counts.items():
                stat.add("transmis_" + n, str(c))
            stat.summary(DiagnosticStatus.OK, "main a : %s" % (a or "personne (repos)"))
            return stat

    rclpy.init(args=argv)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
