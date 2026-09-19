#!/usr/bin/env python3
# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Noeud de metrique FPS video. Publie deux cadences en std_msgs/Float32 (types standard ->
# visibles via le MCP rosbridge, contrairement aux messages custom) :
#   - /camera/capture_fps : cadence de publication de /image_raw par v4l2_camera (cote ROS).
#   - /camera/stream_fps  : cadence REELLE des frames livrees par web_video_server, mesuree
#     par un client MJPEG interne (l'encodage VP9/H264 sur RPi4 peut ne pas suivre la capture
#     -> stream < capture ; web_video_server n'expose pas cette valeur, on la mesure a la
#     sortie HTTP en comptant les marqueurs de debut JPEG).
#
# ECHANTILLONNAGE PERIODIQUE : le client MJPEG ne reste PAS connecte en permanence. Un flux
# MJPEG ouvert force web_video_server a re-encoder rgb8->JPEG en continu (charge CPU constante
# sur un RPi4 deja sature par la capture). On ouvre donc le flux `measure_s` toutes les
# `sample_period_s`, on mesure, on ferme -> /dev/video0 et le CPU respirent entre deux mesures.
# La valeur mesuree est republiee telle quelle jusqu'a l'echantillon suivant.
#
# Noeud autonome (pas d'executable de paquet) : lance par chemin absolu depuis le bind-mount,
# meme schema que camera.launch.py. Les params ROS passent par --ros-args -p (rclpy lit argv).

import threading
import time
import urllib.request
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class CameraFps(Node):
    def __init__(self):
        super().__init__('camera_fps')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('stream_host', '127.0.0.1')
        self.declare_parameter('stream_port', 8080)
        self.declare_parameter('stream_topic', '/image_raw')
        self.declare_parameter('window_s', 3.0)
        self.declare_parameter('publish_rate_hz', 1.0)
        # Echantillonnage periodique du stream (voir en-tete) : duree d'une mesure et periode.
        self.declare_parameter('stream_measure_s', 3.0)
        self.declare_parameter('stream_sample_period_s', 30.0)

        image_topic = self.get_parameter('image_topic').value
        host = self.get_parameter('stream_host').value
        port = int(self.get_parameter('stream_port').value)
        stream_topic = self.get_parameter('stream_topic').value
        self._window = float(self.get_parameter('window_s').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._measure_s = float(self.get_parameter('stream_measure_s').value)
        self._sample_period_s = float(self.get_parameter('stream_sample_period_s').value)

        # Cote capture : horodatages glissants (monotone). Cote stream : une seule valeur de
        # fps (float) rafraichie a chaque echantillon periodique. Verrou partage : le thread
        # stream ecrit _stream_fps pendant que le timer ROS lit les deux.
        self._cap_ts = deque()
        self._stream_fps = 0.0
        self._lock = threading.Lock()

        # Cote capture : souscription best-effort (sensor_data) -> compatible que le
        # publieur soit reliable ou best-effort. Le callback ne fait que dater, pas de
        # traitement de l'image.
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)

        self._pub_cap = self.create_publisher(Float32, 'camera/capture_fps', 10)
        self._pub_stream = self.create_publisher(Float32, 'camera/stream_fps', 10)
        self.create_timer(1.0 / rate if rate > 0 else 1.0, self._on_timer)

        # Cote stream : echantillonnage periodique en thread daemon (voir en-tete).
        self._url = f'http://{host}:{port}/stream?topic={stream_topic}&type=mjpeg'
        self._running = True
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

        self.get_logger().info(
            f'camera_fps : capture={image_topic}, stream={self._url}, '
            f'mesure {self._measure_s}s / {self._sample_period_s}s '
            f'-> /camera/capture_fps, /camera/stream_fps')

    def _on_image(self, _msg):
        # Ne compte que l'arrivee (datation), pas de deserialisation utile de l'image.
        with self._lock:
            self._cap_ts.append(time.monotonic())

    def _measure_once(self):
        # Ouvre le flux MJPEG `_measure_s`, compte les marqueurs de debut d'image JPEG
        # (SOI 0xFFD8) et renvoie la cadence. Le calage d'octets JPEG garantit qu'aucun
        # 0xFFD8 n'apparait dans les donnees entropiques -> comptage fiable. On reporte 1
        # octet entre deux lectures pour rattraper un 0xFF|0xD8 coupe a la frontiere d'un chunk.
        resp = urllib.request.urlopen(self._url, timeout=5.0)
        try:
            count = 0
            carry = b''
            t0 = time.monotonic()
            deadline = t0 + self._measure_s
            while self._running and time.monotonic() < deadline:
                chunk = resp.read(8192)
                if not chunk:
                    break
                data = carry + chunk
                count += data.count(b'\xff\xd8')
                carry = data[-1:]  # rattrape un SOI coupe en deux
            elapsed = time.monotonic() - t0
            return count / elapsed if elapsed > 0 else 0.0
        finally:
            resp.close()  # ferme le flux -> web_video_server cesse de re-encoder

    def _stream_loop(self):
        # Echantillonnage periodique : mesure `_measure_s`, puis dort le reste de la periode.
        while self._running:
            try:
                fps = self._measure_once()
                with self._lock:
                    self._stream_fps = fps
            except Exception as exc:  # noqa: BLE001 (reessaie a la periode suivante)
                self.get_logger().warn(f'stream fps : mesure impossible ({exc})')
                with self._lock:
                    self._stream_fps = 0.0
            # Dort le reste de la periode (periode - duree de mesure), par pas de 0.5s pour
            # rester reactif a l'arret du noeud.
            slept = 0.0
            remaining = max(0.0, self._sample_period_s - self._measure_s)
            while self._running and slept < remaining:
                time.sleep(min(0.5, remaining - slept))
                slept += 0.5

    def _cap_fps(self, now):
        cutoff = now - self._window
        while self._cap_ts and self._cap_ts[0] < cutoff:
            self._cap_ts.popleft()
        return len(self._cap_ts) / self._window if self._window > 0 else 0.0

    def _on_timer(self):
        now = time.monotonic()
        with self._lock:
            cap = self._cap_fps(now)
            stream = self._stream_fps
        self._pub_cap.publish(Float32(data=float(cap)))
        self._pub_stream.publish(Float32(data=float(stream)))

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraFps()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
