#!/usr/bin/env python3
# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Multiplexeur de source video : UNE SEULE URL stable pour le consommateur externe.
#
# C'est ce noeud qui tient les deux exigences opposees du chantier :
#   - "bamboo_videotracking doit utiliser bamboo_video en s'enregistrant dessus pour
#     recuperer le flux video et le renvoyer enrichi a bamboo_video" -> le service
#     /video/register_overlay, par lequel un client DECLARE le topic sur lequel il
#     rendra l'enrichi ;
#   - "on doit pouvoir utiliser bamboo_video (streaming) sans que bamboo_videotracking
#     soit lance" -> registre vide = recopie du flux BRUT. Ce paquet n'a donc AUCUNE
#     dependance, ni de code ni de service, envers le groupe tracking : il ne connait
#     que le nom d'un topic qu'on lui donne a l'execution.
#
# Le benefice non evident est cote navigateur : l'URL ne change JAMAIS selon que le
# tracking tourne ou non, c'est la source qui commute derriere. Le cout est une recopie
# de CompressedImage (~30 Ko, ~0,9 Mo/s a 30 fps) -- a comparer aux 921 Ko que couterait
# UNE trame rgb8 640x480, la memoire partagee DDS etant coupee par l'entrypoint.
#
# DEGRADER, PAS COUPER : un groupe tracking qui meurt doit rendre le flux brut, pas un
# flux noir. Le retour au brut est donc automatique sur trois evenements -- appel
# `deregister`, disparition du publieur enrichi, ou absence de trame au-dela de
# `overlay_timeout_s`.
#
# QoS, et le detail qui fait rater un flux sans message d'erreur : on SOUSCRIT en
# best-effort (compatible avec un publieur best-effort comme avec un publieur reliable
# -- overlay_node publie en SensorDataQoS) et on PUBLIE en reliable (accepte par tout
# souscripteur, y compris rosbridge et web_video_server). L'inverse serait silencieux.

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from bamboo_interfaces.srv import RegisterOverlay

_SRC_RAW = "raw"


class StreamMux(Node):
    """Recopie vers `out_topic` soit le flux brut, soit le flux enrichi enregistre."""

    def __init__(self):
        super().__init__("stream_mux_node")

        self.declare_parameter("raw_topic", "/video/raw/compressed")
        self.declare_parameter("out_topic", "/video/stream/compressed")
        self.declare_parameter("source_topic", "/video/stream_source")
        # Delai sans trame enrichie au-dela duquel on revient au brut. 2 s = trois fois
        # la periode d'une chaine de tracking a 10 fps degradee : on ne commute pas sur
        # un simple hoquet de detection.
        self.declare_parameter("overlay_timeout_s", 2.0)
        self.declare_parameter("check_period_s", 0.5)

        self._raw_topic = str(self.get_parameter("raw_topic").value)
        self._out_topic = str(self.get_parameter("out_topic").value)
        self._timeout_s = float(self.get_parameter("overlay_timeout_s").value)

        # Publie en reliable et en profondeur 1 : une trame video perimee n'a aucune
        # valeur, la garder en file ne ferait que retarder la suivante.
        out_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._out = self.create_publisher(CompressedImage, self._out_topic, out_qos)

        # La source est un fait d'etat, pas un flux : transient_local pour qu'un client
        # qui arrive apres la commutation sache immediatement ce qu'il regarde. C'est la
        # reponse a "pourquoi je ne vois pas les cadres".
        self._src_pub = self.create_publisher(
            String,
            str(self.get_parameter("source_topic").value),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._raw_sub = self.create_subscription(
            CompressedImage, self._raw_topic, self._onRaw, self._img_qos)

        # Etat du registre : un seul enregistrement actif a la fois.
        self._holder = ""      # node_name du client, vide = aucun
        self._topic = ""       # topic enrichi souscrit
        self._enr_sub = None
        self._enr_timeout = self._timeout_s
        self._last_enr = 0.0   # horloge MONOTONE : une resynchro NTP ne doit pas
                               # provoquer un retour au brut (le RPi a deja derive de 9 h)
        self._n_raw = 0
        self._n_enr = 0

        self._srv = self.create_service(
            RegisterOverlay, "/video/register_overlay", self._onRegister)
        self.create_timer(
            float(self.get_parameter("check_period_s").value), self._onCheck)

        self._publishSource()
        self.get_logger().info(
            "mux pret : %s -> %s (aucun enrichi enregistre, on sert le brut)"
            % (self._raw_topic, self._out_topic))

    # ------------------------------------------------------------------ etat
    @property
    def _active(self):
        return bool(self._holder) and self._enr_sub is not None

    def _publishSource(self):
        msg = String()
        msg.data = self._holder if self._active else _SRC_RAW
        self._src_pub.publish(msg)

    def _dropOverlay(self, why):
        """Retour au brut. `why` est journalise : c'est la trace de diagnostic."""
        if not self._active:
            return
        holder, topic = self._holder, self._topic
        self.destroy_subscription(self._enr_sub)
        self._enr_sub = None
        self._holder = ""
        self._topic = ""
        self._publishSource()
        self.get_logger().warn(
            "retour au flux BRUT (%s) : client '%s', topic '%s'" % (why, holder, topic))

    # ------------------------------------------------------------------ flux
    def _onRaw(self, msg):
        self._n_raw += 1
        # Le brut n'est recopie que tant qu'aucun enrichi n'est actif : sinon on
        # publierait DEUX trames par cycle sur la meme sortie et le navigateur
        # alternerait entre l'image nue et l'image annotee.
        if not self._active:
            self._out.publish(msg)

    def _onEnriched(self, msg):
        self._n_enr += 1
        self._last_enr = time.monotonic()
        self._out.publish(msg)

    def _onCheck(self):
        if not self._active:
            return
        # 1. Le publieur a disparu (processus mort, conteneur arrete). On interroge le
        #    graphe plutot que de declarer une liveliness MANUAL_BY_TOPIC : celle-ci
        #    obligerait le client a la declarer aussi, donc couplerait les deux groupes.
        try:
            n_pub = self.count_publishers(self._topic)
        except Exception:
            n_pub = 1  # en cas de doute on ne coupe pas, le timeout tranchera
        if n_pub == 0:
            self._dropOverlay("plus aucun publieur sur le topic enrichi")
            return
        # 2. Le publieur est la mais ne produit plus (chaine de tracking bloquee).
        if (time.monotonic() - self._last_enr) > self._enr_timeout:
            self._dropOverlay(
                "aucune trame enrichie depuis %.1f s" % self._enr_timeout)

    # --------------------------------------------------------------- registre
    def _onRegister(self, req, resp):
        name = (req.node_name or "").strip()
        topic = (req.topic or "").strip()

        if req.deregister:
            if not self._active:
                resp.accepted = True
                resp.reason = "aucun enrichi enregistre, deja sur le brut"
                return resp
            # Seul le titulaire peut se desenregistrer : sinon n'importe quel noeud
            # pourrait couper l'incrustation d'un autre.
            if name and name != self._holder:
                resp.accepted = False
                resp.reason = ("desenregistrement refuse : le titulaire est '%s'"
                               % self._holder)
                return resp
            self._dropOverlay("desenregistrement demande par '%s'" % (name or "?"))
            resp.accepted = True
            resp.reason = _SRC_RAW
            return resp

        if not topic:
            resp.accepted = False
            resp.reason = "topic vide : rien a souscrire"
            return resp

        if self._active and name != self._holder:
            # Refus MOTIVE, pas ecrasement silencieux : deux incrustations sur la meme
            # sortie donneraient une image qui clignote entre deux annotations.
            resp.accepted = False
            resp.reason = ("deja enregistre par '%s' sur '%s'"
                           % (self._holder, self._topic))
            return resp

        if self._active and name == self._holder and topic == self._topic:
            resp.accepted = True
            resp.reason = "enregistrement deja actif (appel idempotent)"
            return resp

        # Re-enregistrement du meme client sur un autre topic : on lache l'ancien.
        if self._active:
            self._dropOverlay("re-enregistrement de '%s' sur un autre topic" % name)

        self._enr_timeout = (
            float(req.timeout_s) if req.timeout_s > 0.0 else self._timeout_s)
        self._holder = name or "anonyme"
        self._topic = topic
        # Compte a rebours arme MAINTENANT : un client qui s'enregistre puis ne publie
        # jamais doit rendre la main tout seul, sans intervention.
        self._last_enr = time.monotonic()
        self._enr_sub = self.create_subscription(
            CompressedImage, topic, self._onEnriched, self._img_qos)
        self._publishSource()
        self.get_logger().info(
            "enrichi enregistre : client '%s', topic '%s', timeout %.1f s"
            % (self._holder, self._topic, self._enr_timeout))
        resp.accepted = True
        resp.reason = self._holder
        return resp


def main(argv=None):
    rclpy.init(args=argv)
    node = StreamMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
