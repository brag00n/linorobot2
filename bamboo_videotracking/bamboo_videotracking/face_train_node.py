#!/usr/bin/env python3
"""face_train_node -- serveur d'action ROS2 de l'apprentissage (enrolement SFace).

Remplace le couple FaceTrainNode (thread worker) + topic `/recognition/train_state` du
prototype roslite. L'algorithme n'est PAS reecrit : il vit dans `face_trainer.py`,
recopie verbatim ; ce fichier n'est que l'enveloppe ROS.

POURQUOI UNE ACTION ET PAS UN SERVICE : un batch d'enrolement dure des dizaines de
secondes (embeddings de tous les lots), il a un avancement a montrer, et il doit etre
annulable. Un service bloquerait l'appelant sans rien lui dire.

UN SEUL GOAL A LA FOIS : un concurrent est REJETE, ce qui preserve exactement la
semantique du worker unique du prototype (et le verrou fichier `faces/.train.lock` du
trainer reste la deuxieme ligne de defense, y compris contre un autre processus).

CE NOEUD RESTE HORS DU CONTAINER DE COMPOSABLES : il n'est pas sur le chemin temps
reel, il n'a aucune image a partager, et il passerait son temps a bloquer un thread de
l'executeur multi-thread du groupe.

FIN DE BATCH -> RECHARGEMENT DE GALERIE. A la fin d'un batch reussi, le noeud publie
`ModeCmd{target: "training", value: "done", seq: ++}` sur le topic de modes. Sans cette
publication, `facerecog_node` (C++) continue de comparer a l'ANCIENNE galerie et la
personne tout juste enrolee reste "unknown" -- symptome classiquement pris pour un
echec d'enrolement alors que l'enrolement a reussi.
"""
import json
import os
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from bamboo_interfaces.action import TrainFaces
from bamboo_interfaces.msg import ModeCmd

# Imports ABSOLUS et non relatifs : ce fichier est installe comme PROGRAMME executable
# (install(PROGRAMS ...)) et lance directement par ros2 launch, donc sans paquet parent --
# un import relatif y leve ImportError. Le paquet reste importable grace a
# ament_python_install_package().
from bamboo_videotracking.face_recognizer import FaceRecognizer
from bamboo_videotracking.face_trainer import FaceTrainer


class _Cancelled(Exception):
    """Levee depuis le hook de journal du trainer pour l'interrompre proprement."""


class FaceTrainNode(Node):
    """Sert `bamboo_interfaces/action/TrainFaces` en appelant FaceTrainer."""

    def __init__(self):
        super().__init__("face_train_node")

        # --- parametres (valeurs du prototype, seule source de verite algorithmique)
        self.declare_parameter("faces_dir", "/root/data/faces")
        self.declare_parameter("sface_model", "")
        self.declare_parameter("cos_thr", 0.363)
        self.declare_parameter("min_imgs", 10)
        self.declare_parameter("recog_min_ok", 0.6)
        self.declare_parameter("garbage_frac", 0.8)
        self.declare_parameter("mode_topic", "/videotracking/mode_cmd")

        self._faces_dir = self.get_parameter("faces_dir").value
        self._model = self.get_parameter("sface_model").value

        # Le modele est facultatif A LA CONSTRUCTION : un noeud qui refuse de demarrer
        # parce que le .onnx n'est pas monte empecherait aussi de diagnostiquer POURQUOI
        # il ne l'est pas. L'echec est donc reporte au premier goal, avec son motif.
        if not self._model or not os.path.exists(self._model):
            self.get_logger().warn(
                "modele SFace absent (%s) : les goals TrainFaces echoueront avec ce "
                "motif tant que `sface_model` ne pointe pas un .onnx existant"
                % (self._model or "<non defini>",))

        # --- publication de fin de batch
        # transient_local + depth 1 : meme QoS que le producteur de modes, pour qu'un
        # facerecog_node demarre APRES la fin du batch rejoue quand meme le "done".
        mode_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mode_pub = self.create_publisher(
            ModeCmd, self.get_parameter("mode_topic").value, mode_qos)
        self._mode_seq = 0
        self._done_seq = 0

        # --- etat d'unicite du goal
        self._busy = threading.Lock()

        # Groupe REENTRANT : sans lui, la demande d'annulation ne serait servie qu'apres
        # la fin du batch -- autrement dit jamais utile.
        self._srv = ActionServer(
            self, TrainFaces, "videotracking/train_faces",
            execute_callback=self._execute,
            goal_callback=self._onGoal,
            cancel_callback=self._onCancel,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info("face_train_node pret (faces_dir=%s)" % self._faces_dir)

    # --- politique de goal ---------------------------------------------------
    def _onGoal(self, goal_request):
        """Un seul batch a la fois ; le concurrent est REJETE, pas mis en file."""
        if self._busy.locked():
            self.get_logger().warn("goal TrainFaces refuse : un batch est en cours")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _onCancel(self, goal_handle):
        """Annulation acceptee : le hook de journal du trainer l'honore a l'etape suivante.

        Le trainer n'a AUCUN point d'interruption propre : c'est son hook `log()`, appele
        a chaque etape, qui sert de point de sortie (exception `_Cancelled`). Consequence
        a assumer, pas a masquer : les images deja deplacees dans `learning/` y RESTENT.
        Une passe suivante rescanne et repart de cet etat.
        """
        self.get_logger().warn("annulation TrainFaces demandee")
        return CancelResponse.ACCEPT

    # --- execution -----------------------------------------------------------
    def _execute(self, goal_handle):
        result = TrainFaces.Result()
        if not self._busy.acquire(blocking=False):
            # Course theorique entre _onGoal et ici : on refuse plutot que d'empiler.
            goal_handle.abort()
            result.success = False
            result.summary = json.dumps({"ok": False, "error": "busy"})
            result.done_seq = self._done_seq
            return result
        try:
            return self._runBatch(goal_handle, result)
        finally:
            self._busy.release()

    def _runBatch(self, goal_handle, result):
        step = [0]
        fb = TrainFaces.Feedback()

        def log(event, **fields):
            """Hook de journal du trainer -> feedback ROS + /rosout + point d'annulation.

            `total` reste a 0 : le nombre d'etapes n'est pas connu d'avance (il depend du
            nombre de lots decouverts). Annoncer un total faux serait pire que 0, qui est
            documente comme "inconnu" dans le .action.
            """
            if goal_handle.is_cancel_requested:
                raise _Cancelled()
            step[0] += 1
            line = event
            if fields:
                line += " " + " ".join("%s=%s" % (k, v) for k, v in sorted(fields.items()))
            fb.line = line
            fb.step = step[0]
            fb.total = 0
            goal_handle.publish_feedback(fb)
            self.get_logger().info(line)

        # Le recognizer est reconstruit A CHAQUE batch : il porte l'etat de la galerie, et
        # la galerie a pu changer depuis le batch precedent (acquisition, suppression a la
        # main). Le cout est le chargement d'un .onnx, negligeable devant un enrolement.
        rec = FaceRecognizer(model_path=self._model,
                             faces_dir=self._faces_dir,
                             cos_thr=float(self.get_parameter("cos_thr").value))
        trainer = FaceTrainer(
            self._faces_dir, recognizer=rec,
            cos_thr=float(self.get_parameter("cos_thr").value),
            min_imgs=int(self.get_parameter("min_imgs").value),
            recog_min_ok=float(self.get_parameter("recog_min_ok").value),
            garbage_frac=float(self.get_parameter("garbage_frac").value),
            log=log)

        try:
            summary = trainer.run()
        except _Cancelled:
            goal_handle.canceled()
            result.success = False
            result.summary = json.dumps(
                {"ok": False, "error": "cancelled",
                 "warning": "lots partiellement deplaces dans learning/"})
            result.done_seq = self._done_seq
            self.get_logger().warn("batch TrainFaces annule")
            return result
        except Exception as e:                       # noqa: BLE001 -- on remonte le motif
            goal_handle.abort()
            result.success = False
            result.summary = json.dumps({"ok": False, "error": str(e)})
            result.done_seq = self._done_seq
            self.get_logger().error("batch TrainFaces en echec : %s" % e)
            return result

        ok = bool(summary.get("ok", False))
        if ok:
            self._done_seq += 1
            self._publishDone()
            goal_handle.succeed()
        else:
            # `run()` renvoie {"ok": False, "error": ...} sur modele manquant ou verrou
            # tenu : ce n'est pas une exception, mais ce n'est pas un succes non plus.
            goal_handle.abort()
            self.get_logger().warn("batch TrainFaces non abouti : %s"
                                   % summary.get("error", "?"))

        result.success = ok
        result.summary = json.dumps(summary, default=str)
        result.done_seq = self._done_seq
        return result

    def _publishDone(self):
        """Signale la fin de batch pour que facerecog_node recharge sa galerie."""
        self._mode_seq += 1
        m = ModeCmd()
        m.header.stamp = self.get_clock().now().to_msg()
        m.target = "training"        # cible canonique du .msg -- PAS "train"
        m.value = "done"
        m.seq = self._mode_seq
        self._mode_pub.publish(m)
        self.get_logger().info("galerie rechargeable : ModeCmd training/done seq=%d"
                               % self._mode_seq)


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrainNode()
    # MULTI-THREAD OBLIGATOIRE : l'execution d'un batch monopolise son thread pendant
    # des dizaines de secondes ; sans second thread, la demande d'annulation ne serait
    # jamais servie et l'action ne serait annulable que sur le papier.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
