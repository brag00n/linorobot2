"""Ce que les DEUX drivers de carte font de la compensation de rotation et du gyro.

Meme principe que driver_diagnostics : stm32_mavlink_driver (bambooSTM32YB) et
esp32_mavlink_driver (BambooWS) parlent a leur carte par la meme couche hote
(RobotComSerial), donc le branchement est le meme pour les deux. Ce module le porte une fois :

  - MotionCompensator (motion_compensation.py) sur les parametres du driver ;
  - la consigne cmd_vel corrigee AVANT sendCmdVel ;
  - le lacet de /odom corrige ;
  - Imu.angular_velocity rempli depuis le gyro de la carte (absent jusqu'ici : covariance
    -1, donc l'EKF n'en tirait rien) ;
  - /motion_compensation/state, dont la calibration lit la vitesse roues BRUTE ;
  - le service ~/recalibrate_imu (biais gyro remesure par la carte, robot IMMOBILE).

Le driver garde la propriete du lien serie et de son callback de parametres ; il appelle
check()/apply() depuis ce callback.

Attributs du DRIVER lus ici : link, wheel_sep, enable_cmd_vel.
"""

import math

from std_srvs.srv import Trigger

from bamboo_interfaces.msg import MotionCompensationState
from bamboo_base.motion_compensation import MotionCompensator, body_rate_from_wheels

# Variance du gyro z annoncee dans /imu, (rad/s)^2. Ordre de grandeur du QMI8658 apres
# zone morte ; a affiner avec la mesure au repos de la calibration (etape IMU).
_GZ_VARIANCE = 0.0004
# Variance de l'accelerometre, (m/s2)^2 : ordre de grandeur, comme celle du gyro.
_ACC_VARIANCE = 0.01


class DriverMotion:

    def __init__(self, driver):
        self._d = driver
        self.comp = MotionCompensator(driver, driver.wheel_sep)
        self.pub_state = driver.create_publisher(
            MotionCompensationState, "motion_compensation/state", 10)
        driver.create_service(Trigger, "~/recalibrate_imu", self._on_recalibrate)
        self._v_wheels = 0.0
        self._w_wheels = 0.0
        self._w_odom = 0.0

    # ------------------------------------------------------------------ parametres
    def check(self, params):
        return self.comp.check(params)

    def apply(self, params):
        """True si l'ecartement en vigueur a change (geometrie a repousser a la carte)."""
        for prm in params:
            if prm.name == "wheel_separation_m":
                self.comp.set_base_wheel_sep(float(prm.value))
        return self.comp.apply(params)

    @property
    def wheel_sep(self):
        """Ecartement a utiliser PARTOUT (odometrie, APB de la carte)."""
        return self.comp.wheel_sep

    # ------------------------------------------------------------------ consigne
    def command(self, vx, wz, now_s):
        snap = self._d.link.snapshot()
        return self.comp.command(vx, wz, snap.get("gz"), snap.get("imu_age"), now_s)

    def reset(self):
        self.comp.reset()

    # ------------------------------------------------------------------ mesures
    def odometry(self, v_l, v_r):
        """(v, w) du corps depuis les vitesses de roue ; w corrige du glissement."""
        v = 0.5 * (v_l + v_r)
        w_wheels = body_rate_from_wheels(v_l, v_r, self.wheel_sep)
        w = self.comp.odometry(v, w_wheels)
        self._v_wheels, self._w_wheels, self._w_odom = v, w_wheels, w
        return v, w

    def fill_imu(self, imu, snap):
        """Gyro et accelerometre de la carte dans sensor_msgs/Imu, ou -1 (absents) sinon."""
        ax = snap.get("ax")
        age = snap.get("accel_age")
        if ax is not None and age is not None and age < 1.0:
            # SCALED_IMU (microcode ESP32 >= 0.5.0), deja en m/s2 et dans le repere carte.
            imu.linear_acceleration.x = float(ax)
            imu.linear_acceleration.y = float(snap.get("ay") or 0.0)
            imu.linear_acceleration.z = float(snap.get("az") or 0.0)
            for i in (0, 4, 8):
                imu.linear_acceleration_covariance[i] = _ACC_VARIANCE
        else:
            imu.linear_acceleration_covariance[0] = -1.0
        gz = snap.get("gz")
        if gz is None:
            imu.angular_velocity_covariance[0] = -1.0
            return
        s = self.comp.gz_sign
        # x/y : meme signe que z faute de mieux -- seul z est consomme (EKF vyaw,
        # compensation) et seul z est verifie a l'essai.
        imu.angular_velocity.x = float(snap.get("gx") or 0.0)
        imu.angular_velocity.y = float(snap.get("gy") or 0.0)
        imu.angular_velocity.z = s * float(gz)
        imu.angular_velocity_covariance[0] = _GZ_VARIANCE
        imu.angular_velocity_covariance[4] = _GZ_VARIANCE
        imu.angular_velocity_covariance[8] = _GZ_VARIANCE

    def publish_state(self, stamp, snap):
        c = self.comp.core
        m = MotionCompensationState()
        m.header.stamp = stamp
        m.variant = self.comp.variant_name
        m.identity = c.is_identity
        m.bypass = c.bypass
        m.armed = bool(self._d.enable_cmd_vel)
        st = c.state
        m.vx_in, m.wz_in = float(st["vx_in"]), float(st["wz_in"])
        m.vx_out, m.wz_out = float(st["vx_out"]), float(st["wz_out"])
        m.slip, m.pi_term, m.pi_active = float(st["slip"]), float(st["pi"]), st["pi_active"]
        m.v_wheels, m.w_wheels, m.w_odom = self._v_wheels, self._w_wheels, self._w_odom
        gz = snap.get("gz")
        m.gz = math.nan if gz is None else self.comp.gz_sign * float(gz)
        age = snap.get("imu_age")
        m.gz_age = -1.0 if (gz is None or age is None) else float(age)
        m.wheel_separation_m = float(self.wheel_sep)
        self.pub_state.publish(m)

    # ------------------------------------------------------------------ service
    def _on_recalibrate(self, _req, resp):
        builder = getattr(self._d.link, "recalibrateGyro", None)
        if builder is None:
            resp.success, resp.message = False, "couche hote trop ancienne (recalibrateGyro)."
            return resp
        ok, err = builder()
        resp.success = bool(ok)
        resp.message = ("recalibration demandee : la carte moyenne ~2 s, robot IMMOBILE"
                        if ok else (err or "port indisponible"))
        return resp
