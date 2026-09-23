#!/usr/bin/env python3
r"""esp32_mavlink_driver - pont MAVLink ESP32 <-> ROS2 (proprietaire UNIQUE du serie).

Reutilise TEL QUEL la couche hote `robot_control` : Esp32ComSerial (sous-classe de
RobotComSerial) ouvre le port CP210x en tache de fond, decode le protocole MAVLink
(dialecte bamboo, sysid=2) et expose snapshot()/encSpeed(). Aucun protocole reecrit ici.

Le noeud publie la telemetrie : odom integree (encodeurs), imu, batterie, rpm par roue,
magneto, et un etat brut de diagnostic (Esp32Status). L'ENVOI de /cmd_vel est GATE par le
parametre `enable_cmd_vel` :
  - false (defaut, lot L5) : LECTURE SEULE, aucune commande moteur emise ;
  - true  (lot L6) : actuation, apres calibration geometrie + roues surelevees.

Odometrie : la carte ne calcule PAS la pose (x, y, theta) ; on l'integre ici a partir des
2 voies encodeur (gauche/droite).

Geometrie et motorisation : ce noeud n'en declare AUCUN defaut. Elles viennent de la source
unique `config/robots/<robot>.yaml`, chargee par driver.launch.py. Un parametre physique
absent est une ERREUR FATALE et non un repli silencieux : les defauts litteraux dupliques
sont exactement ce qui a produit quatre diametres de roue contradictoires dans le depot.
Ces valeurs ne sont pas encore mesurees (lot 4) -> l'echelle de l'odom reste fausse.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState, MagneticField
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

from bamboo_interfaces.msg import WheelRpm, Esp32Status

from robot_control.communication.Esp32ComSerial import Esp32ComSerial


def _quat_from_euler(roll, pitch, yaw):
    """Quaternion (x, y, z, w) depuis roll/pitch/yaw en RADIANS (convention ZYX)."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Esp32MavlinkDriver(Node):
    # Parametres physiques attendus de la source unique config/robots/<robot>.yaml.
    # Aucun n'a de defaut : voir _req().
    _PHYS_DOUBLES = ("car_type", "counts_per_rev", "wheel_diameter_m", "wheel_separation_m",
                     "motor_max_rpm", "motor_operating_voltage", "motor_power_max_voltage",
                     "pid_kp", "pid_ki", "pid_kd")
    _PHYS_INTS = ("encoder_left_index", "encoder_right_index")
    _PHYS_BOOLS = ("invert_left", "invert_right")

    def _req(self, name):
        """Lit un parametre physique OBLIGATOIRE ; leve si la source unique ne l'a pas fourni.

        Selon la version de rclpy, un parametre declare sans valeur leve a la lecture ou
        renvoie une valeur None : les deux cas menent au meme echec explicite.
        """
        try:
            value = self.get_parameter(name).value
        except Exception:
            value = None
        if value is None:
            raise RuntimeError(
                f"parametre physique obligatoire absent : '{name}'. Il doit venir du fichier "
                "canonique bamboo_base/config/robots/<robot>.yaml (charge par "
                "driver.launch.py) ; ce noeud ne porte volontairement aucun defaut geometrique.")
        return value

    def __init__(self):
        super().__init__("esp32_mavlink_driver")

        # --- parametres de transport et de frames (faits d'hote : un defaut est licite) ---
        self.declare_parameter("port", "/dev/esp32")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("publish_rate_hz", 30.0)
        # --- parametres PHYSIQUES : declares SANS DEFAUT ---
        # Ils appartiennent a la source unique config/robots/<robot>.yaml. Declarer un type
        # sans valeur rend l'absence detectable : _req() leve alors une erreur nommant le
        # parametre, au lieu de laisser tourner l'odometrie sur un placeholder muet.
        for _name in self._PHYS_DOUBLES:
            self.declare_parameter(_name, Parameter.Type.DOUBLE)
        # encSpeed() renvoie [M1..M4] ; 2 voies physiques seulement (M3=M1, M4=M2 cote
        # encodeur). On mappe la voie gauche/droite par index.
        for _name in self._PHYS_INTS:
            self.declare_parameter(_name, Parameter.Type.INTEGER)
        for _name in self._PHYS_BOOLS:
            self.declare_parameter(_name, Parameter.Type.BOOL)
        self.declare_parameter("robot_base", Parameter.Type.STRING)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        # L'EKF possede le TF odom->base_footprint -> le driver ne le publie PAS par defaut.
        self.declare_parameter("publish_odom_tf", False)
        # Verrou d'actuation : false = lecture seule (L5), true = actuation (L6).
        self.declare_parameter("enable_cmd_vel", False)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)

        g = lambda n: self.get_parameter(n).value
        self.port = g("port")
        self.baud = int(g("baud"))
        rate = float(g("publish_rate_hz"))
        self.robot_base = str(self._req("robot_base"))
        self.car_type = float(self._req("car_type"))
        self.cpr = float(self._req("counts_per_rev"))
        self.wheel_diameter = float(self._req("wheel_diameter_m"))
        self.wheel_sep = float(self._req("wheel_separation_m"))
        # Motorisation et gains : lus ici pour echouer tot si la source est incomplete ;
        # leur diffusion vers la SRAM de la carte est le sujet du lot 2.
        self.motor_max_rpm = float(self._req("motor_max_rpm"))
        self.motor_voltage = float(self._req("motor_operating_voltage"))
        self.motor_max_voltage = float(self._req("motor_power_max_voltage"))
        self.pid = (float(self._req("pid_kp")), float(self._req("pid_ki")),
                    float(self._req("pid_kd")))
        self.left_idx = int(self._req("encoder_left_index"))
        self.right_idx = int(self._req("encoder_right_index"))
        self.inv_left = bool(self._req("invert_left"))
        self.inv_right = bool(self._req("invert_right"))
        self.odom_frame = g("odom_frame")
        self.base_frame = g("base_frame")
        self.imu_frame = g("imu_frame")
        self.publish_odom_tf = bool(g("publish_odom_tf"))
        self.enable_cmd_vel = bool(g("enable_cmd_vel"))
        self.cmd_vel_timeout = float(g("cmd_vel_timeout_s"))

        # --- lien serie (proprietaire unique) ---
        # vid_pid CP210x pour prioriser le bon port parmi le DOUBLE CP2102 (lidar + ESP32) ;
        # le sniff protocole + le sysid MAVLink departagent lidar vs ESP32 (cf. fiche materiel).
        self.link = Esp32ComSerial(self.port, self.baud, protocol="mavlink",
                                   cpr=self.cpr, vid_pid=["10C4:EA60"], telemetry=None)

        # --- publishers ---
        self.pub_odom = self.create_publisher(Odometry, "odom/unfiltered", 20)
        self.pub_imu = self.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self.pub_batt = self.create_publisher(BatteryState, "battery", 10)
        self.pub_rpm = self.create_publisher(WheelRpm, "wheel_rpm", 10)
        self.pub_mag = self.create_publisher(MagneticField, "mag", qos_profile_sensor_data)
        self.pub_status = self.create_publisher(Esp32Status, "esp32_status", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None

        # --- souscription cmd_vel (GATE par enable_cmd_vel) ---
        self._last_cmd_t = None
        if self.enable_cmd_vel:
            self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
            self.get_logger().warn(
                "enable_cmd_vel=TRUE : ACTUATION ACTIVE -> roues surelevees + geometrie calibree requises (L6).")
        else:
            self.get_logger().info(
                "enable_cmd_vel=false : LECTURE SEULE (L5), aucune commande moteur emise.")

        # --- etat odometrie integree ---
        self.x = self.y = self.th = 0.0
        self._last_t = self.get_clock().now()

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"driver ESP32 MAVLink demarre (port={self.port}, baud={self.baud}, cpr={self.cpr}).")

    # --- actuation (L6 uniquement) -----------------------------------------
    def _on_cmd_vel(self, msg):
        if not self.enable_cmd_vel:
            return
        self.link.sendCmdVel(msg.linear.x, msg.angular.z)
        self._last_cmd_t = self.get_clock().now()

    # --- boucle de publication ---------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        snap = self.link.snapshot()
        sp = self.link.encSpeed()
        cpr = self.link.cpr or self.cpr

        # watchdog cmd_vel : coupe si le flux de consignes se tait (actuation seulement).
        if self.enable_cmd_vel and self._last_cmd_t is not None:
            if (now - self._last_cmd_t).nanoseconds * 1e-9 > self.cmd_vel_timeout:
                self.link.sendCmdVel(0.0, 0.0)

        # rpm par moteur (tics/s / cpr * 60), cf. BoardNode.
        if sp is not None:
            rpm_msg = WheelRpm()
            rpm_msg.header.stamp = stamp
            rpm_msg.rpm = [s / cpr * 60.0 for s in sp]
            self.pub_rpm.publish(rpm_msg)

        # odom integree depuis les 2 voies encodeur.
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if sp is not None and dt > 0.0:
            dist_per_tic = math.pi * self.wheel_diameter / cpr
            v_l = sp[self.left_idx] * dist_per_tic * (-1.0 if self.inv_left else 1.0)
            v_r = sp[self.right_idx] * dist_per_tic * (-1.0 if self.inv_right else 1.0)
            v = 0.5 * (v_l + v_r)
            w = (v_r - v_l) / self.wheel_sep if self.wheel_sep > 1e-6 else 0.0
            self.th += w * dt
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
            qx, qy, qz, qw = _quat_from_euler(0.0, 0.0, self.th)

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w
            # Covariances diagonales indicatives (odom roues seule : theta peu fiable).
            odom.pose.covariance[0] = odom.pose.covariance[7] = 0.001
            odom.pose.covariance[35] = 0.01
            odom.twist.covariance[0] = 0.001
            odom.twist.covariance[35] = 0.01
            self.pub_odom.publish(odom)

            if self.tf_broadcaster is not None:
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = self.odom_frame
                tf.child_frame_id = self.base_frame
                tf.transform.translation.x = self.x
                tf.transform.translation.y = self.y
                tf.transform.rotation.x = qx
                tf.transform.rotation.y = qy
                tf.transform.rotation.z = qz
                tf.transform.rotation.w = qw
                self.tf_broadcaster.sendTransform(tf)

        # imu/data : roll/pitch/yaw sont en DEGRES dans le snapshot -> conversion en rad.
        if snap.get("yaw") is not None:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.imu_frame
            r = math.radians(snap.get("roll") or 0.0)
            p = math.radians(snap.get("pitch") or 0.0)
            yv = math.radians(snap.get("yaw") or 0.0)
            qx, qy, qz, qw = _quat_from_euler(r, p, yv)
            imu.orientation.x, imu.orientation.y = qx, qy
            imu.orientation.z, imu.orientation.w = qz, qw
            # La carte ne remonte pas gyro/accel via snapshot -> covariances a -1
            # (convention ROS : donnee absente, l'EKF ignore ces champs).
            imu.angular_velocity_covariance[0] = -1.0
            imu.linear_acceleration_covariance[0] = -1.0
            self.pub_imu.publish(imu)

        # batterie.
        if snap.get("battery") is not None:
            b = BatteryState()
            b.header.stamp = stamp
            b.voltage = float(snap["battery"])
            b.present = True
            self.pub_batt.publish(b)

        # magneto (optionnelle) : uT -> Tesla.
        mag = snap.get("mag")
        if mag is not None and len(mag) >= 3:
            m = MagneticField()
            m.header.stamp = stamp
            m.header.frame_id = self.imu_frame
            m.magnetic_field.x = float(mag[0]) * 1e-6
            m.magnetic_field.y = float(mag[1]) * 1e-6
            m.magnetic_field.z = float(mag[2]) * 1e-6
            self.pub_mag.publish(m)

        # etat brut agrege (diagnostic, observable directement via le MCP).
        st = Esp32Status()
        st.header.stamp = stamp
        st.battery_voltage = float(snap.get("battery") or 0.0)
        st.roll = float(snap.get("roll") or 0.0)
        st.pitch = float(snap.get("pitch") or 0.0)
        st.yaw = float(snap.get("yaw") or 0.0)
        st.vx = float(snap.get("vx") or 0.0)
        st.vy = float(snap.get("vy") or 0.0)
        st.vz = float(snap.get("vz") or 0.0)
        enc = snap.get("encoders")
        st.encoders = [int(e) for e in enc] if enc else []
        self.pub_status.publish(st)

    def destroy_node(self):
        try:
            if self.enable_cmd_vel:
                self.link.stop()
            self.link.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32MavlinkDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
