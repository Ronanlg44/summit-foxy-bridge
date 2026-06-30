"""
Controleur I-P du Summit XL
pour suivre le Spot via AprilTag. Deux boucles : LINEAIRE + ANGULAIRE.

============================================================
STRUCTURE - DEUX BOUCLES I-P INDEPENDANTES
============================================================
LINEAIRE (axe X = devant le Summit) :
  err_lin       = spot_x - target_distance
  integral_lin += err_lin * dt   (anti-windup conditionnel)
  u_lin         = Kp_lin * err_lin + Ki_lin * integral_lin

ANGULAIRE (rotation autour de Z, cible = tag centre dans l'image) :
  bearing       = atan2(spot_y, spot_x)
  err_ang       = bearing                (target = 0)
  integral_ang += err_ang * dt   (anti-windup conditionnel)
  u_ang         = Kp_ang * err_ang + Ki_ang * integral_ang

COUPLAGE "aligner avant d'avancer" :
  align_factor = max(0, 1 - |bearing| / bearing_max)
  u_lin_final  = u_lin * align_factor
  - bearing = 0     -> align_factor = 1.0 (pleine vitesse linaire)
  - |bearing| >= bearing_max -> align_factor = 0 (lin coupe)
  - en zone intermediaire : decroissance lineaire

============================================================
GAINS - Validation Simulink 
============================================================
LINEAIRE  (Td_total = 0.2285 s) :
  Kp_lin = 1.804
  Ki_lin = 0.817
  PM ~ 67 deg, GM ~ 12 dB, settling ~1.5 s, overshoot ~0%

ANGULAIRE (Td_total = 0.292 s) :
  Kp_ang = 1.6056
  Ki_ang = 0.6215
  PM ~ 67 deg, GM ~ 12 dB, settling ~2.0 s, overshoot ~0%

============================================================
DEADBAND AVEC HYSTERESIS (anti-flickering)
============================================================
LINEAIRE  : entree a 3 cm, sortie a 5 cm
ANGULAIRE : entree a 2 deg, sortie a 3 deg
Quand on est dans la zone morte : cmd = 0, integ figee.

============================================================
SECURITE TEST INITIAL
============================================================
v_max         = 0.2 m/s   
w_max         = 0.5 rad/s 
bearing_max   = 30 deg    (couplage : lin coupe au-dela)
start_enabled = False     (activer via /ip_enable a la main)
publish_real_cmd = False  (a basculer manuellement)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float64


# --------- Topics ---------
POSE_TOPIC = '/spot_pose_in_summit'
CMD_DEBUG_TOPIC = '/cmd_vel_debug'
CMD_REAL_TOPIC = '/summit_xl/robotnik_base_control/cmd_vel'
ENABLE_TOPIC = '/ip_enable'

ERR_TOPIC = '/ip_debug/error_dist'
INTEG_TOPIC = '/ip_debug/integral'
CMD_RAW_TOPIC = '/ip_debug/cmd_raw'
CMD_SAT_TOPIC = '/ip_debug/cmd_saturated'
CMD_LIM_TOPIC = '/ip_debug/cmd_rate_limited'


class IpControllerDebug(Node):

    def __init__(self):
        super().__init__('ip_apriltag_debug')

        # ============ Parametres ROS 2 ============
        self.declare_parameter('target_distance', 1.0)

        # Gains Skogestad SIMC valides Simulink
        # LINEAIRE : Td_total = 0.2285 s
        self.declare_parameter('kp_lin', 1.804)
        self.declare_parameter('ki_lin', 0.817)

        # ANGULAIRE : Td_total = 0.292 s (Td_robot_ang + Td_vision)
        self.declare_parameter('kp_ang', 1.60555042185557)
        self.declare_parameter('ki_ang', 0.621528632958943)

        # Saturation REDUITE pour test
        self.declare_parameter('v_max', 0.2)              # m/s (test prudent)
        self.declare_parameter('w_max', 0.4)              # rad/s (~28 deg/s)

        # Rate limiter coherent avec v_max reduit
        self.declare_parameter('rising_accel', 1.5)       # m/s^2
        self.declare_parameter('falling_accel', 2.0)      # m/s^2
        self.declare_parameter('rising_alpha', 2.0)       # rad/s^2 (angulaire)
        self.declare_parameter('falling_alpha', 2.5)      # rad/s^2

        # Anti-windup
        self.declare_parameter('integ_max', 1.5)
        self.declare_parameter('integ_ang_max', 1.5)

        # ----------- Deadband avec hysteresis (LINEAIRE) -----------
        # Quand |err| < deadband_enter, on entre dans la zone morte (cmd=0)
        # Pour en sortir, il faut |err| > deadband_exit (hysteresis)
        # Evite le flickering aux bords et l'usure moteur sur le bruit AprilTag.
        self.declare_parameter('deadband_enter', 0.03)  # 3 cm
        self.declare_parameter('deadband_exit', 0.05)   # 5 cm

        # ----------- Deadband ANGULAIRE -----------
        # 2 deg / 3 deg en hysteresis
        self.declare_parameter('deadband_ang_enter', 0.035)  # ~2 deg
        self.declare_parameter('deadband_ang_exit', 0.052)   # ~3 deg

        # ----------- Couplage lineaire / angulaire -----------
        # Si |bearing| > bearing_max, le lineaire est completement coupe
        # (priorite : aligner avant d'avancer)
        # Facteur lineaire = max(0, 1 - |bearing| / bearing_max)
        self.declare_parameter('bearing_max', 0.79)  # ~45 deg

        # ----------- Securite -----------
        self.declare_parameter('pose_timeout', 0.5)
        self.declare_parameter('rate_hz', 20.0)

        # IMPORTANT : demarre DESACTIVE pour activer via /ip_enable a la main
        self.declare_parameter('start_enabled', False)

        # IMPORTANT : ne publie PAS la vraie commande par defaut
        # A basculer a True via ros2 param set quand on est pret
        self.declare_parameter('publish_real_cmd', False)

        # ============ Etat interne ============
        self.last_pose: Optional[PoseStamped] = None
        self.last_pose_time: Optional[rclpy.time.Time] = None

        self.integral_err = 0.0
        self.integral_ang = 0.0
        self.last_tick_time: Optional[rclpy.time.Time] = None

        self.last_cmd_lin = 0.0
        self.last_cmd_ang = 0.0

        # Etat de la deadband (hysteresis)
        # True quand on est dans la zone morte (a la cible)
        self.in_deadband = False
        self.in_deadband_ang = False

        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.publish_real = bool(self.get_parameter('publish_real_cmd').value)

        # ============ I/O ============
        self.create_subscription(PoseStamped, POSE_TOPIC, self.on_pose, 10)
        self.create_subscription(Bool, ENABLE_TOPIC, self.on_enable, 10)

        self.pub_cmd_debug = self.create_publisher(Twist, CMD_DEBUG_TOPIC, 10)
        self.pub_cmd_real = self.create_publisher(Twist, CMD_REAL_TOPIC, 10)

        self.pub_err = self.create_publisher(Float64, ERR_TOPIC, 10)
        self.pub_integ = self.create_publisher(Float64, INTEG_TOPIC, 10)
        self.pub_cmd_raw = self.create_publisher(Float64, CMD_RAW_TOPIC, 10)
        self.pub_cmd_sat = self.create_publisher(Float64, CMD_SAT_TOPIC, 10)
        self.pub_cmd_lim = self.create_publisher(Float64, CMD_LIM_TOPIC, 10)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(1.0, self._log_stats)

        # Banner
        kp_lin = self.get_parameter('kp_lin').value
        ki_lin = self.get_parameter('ki_lin').value
        kp_ang = self.get_parameter('kp_ang').value
        ki_ang = self.get_parameter('ki_ang').value
        target = self.get_parameter('target_distance').value
        v_max = self.get_parameter('v_max').value
        w_max = self.get_parameter('w_max').value
        db_enter = self.get_parameter('deadband_enter').value
        db_exit = self.get_parameter('deadband_exit').value
        bearing_max = self.get_parameter('bearing_max').value
        self.get_logger().info("=" * 70)
        self.get_logger().info("PI/IP Controller LIN + ANG - TEST BOUCLE FERMEE")
        self.get_logger().info(f"  Cible distance     : {target:.2f} m (regule sur spot_x)")
        self.get_logger().info(f"  Cible angulaire    : 0 rad (tag centre dans l'image)")
        self.get_logger().info(f"  Gains LIN          : Kp = {kp_lin:.4f}, Ki = {ki_lin:.4f}")
        self.get_logger().info(f"  Gains ANG          : Kp = {kp_ang:.4f}, Ki = {ki_ang:.4f}")
        self.get_logger().info(f"  Vitesse max        : +/- {v_max} m/s, +/- {w_max} rad/s")
        self.get_logger().info(f"  Deadband LIN       : {db_enter*100:.0f} cm / {db_exit*100:.0f} cm")
        self.get_logger().info(f"  Couplage           : lin coupe si |bearing| > {math.degrees(bearing_max):.0f} deg")
        self.get_logger().info(f"  Etat initial       : {'ENABLED' if self.enabled else 'DISABLED (active via /ip_enable)'}")
        if self.publish_real:
            self.get_logger().warn("=" * 70)
            self.get_logger().warn("  /!\\ publish_real_cmd = True /!\\")
            self.get_logger().warn("  LE ROBOT VA BOUGER QUAND TU ACTIVERAS /ip_enable !")
            self.get_logger().warn(f"  Topic reel : {CMD_REAL_TOPIC}")
            self.get_logger().warn("  Verifie l'espace libre et l'acces a l'E-stop")
            self.get_logger().warn("=" * 70)
        else:
            self.get_logger().info("  Mode DEBUG : pas de cmd envoyee au vrai robot")
            self.get_logger().info("  Pour activer : ros2 param set /ip_apriltag_debug publish_real_cmd true")
        self.get_logger().info("=" * 70)
        self.get_logger().info("Pour activer la regulation : ros2 topic pub /ip_enable std_msgs/Bool 'data: true' --once")
        self.get_logger().info("Pour la desactiver         : ros2 topic pub /ip_enable std_msgs/Bool 'data: false' --once")
        self.get_logger().info("=" * 70)

    # ============ Callbacks ============

    def on_pose(self, msg: PoseStamped):
        self.last_pose = msg
        self.last_pose_time = self.get_clock().now()

    def on_enable(self, msg: Bool):
        new_state = bool(msg.data)
        if new_state != self.enabled:
            self.enabled = new_state
            if self.enabled:
                self.get_logger().info(">>> CONTROLEUR ACTIVE")
                self.integral_err = 0.0
                self.integral_ang = 0.0
                self.in_deadband = False
                self.in_deadband_ang = False
                self.last_tick_time = None
            else:
                self.get_logger().info(">>> CONTROLEUR DESACTIVE")
                self._publish_zero()
                self.last_cmd_lin = 0.0
                self.last_cmd_ang = 0.0

    # ============ Boucle principale ============

    def tick(self):
        # Lecture des parametres (peuvent changer dynamiquement)
        self.publish_real = bool(self.get_parameter('publish_real_cmd').value)

        if not self.enabled:
            return

        now = self.get_clock().now()

        # dt
        if self.last_tick_time is None:
            dt = 1.0 / float(self.get_parameter('rate_hz').value)
        else:
            dt = (now - self.last_tick_time).nanoseconds * 1e-9
            dt = max(0.001, min(0.5, dt))

        # Timeout pose : si plus de pose recue -> freinage
        timeout = float(self.get_parameter('pose_timeout').value)
        pose_stale = (
            self.last_pose is None
            or self.last_pose_time is None
            or (now - self.last_pose_time).nanoseconds * 1e-9 > timeout
        )

        if pose_stale:
            self.get_logger().warn(
                "Pose obsolete, freinage vers 0",
                throttle_duration_sec=2.0)
            u_lin_final = self._apply_rate_limit(0.0, self.last_cmd_lin, dt)
            self._publish_cmd(u_lin_final, 0.0)
            self.last_tick_time = now
            return

        # ============ Mesure ============
        spot_x = self.last_pose.pose.position.x
        spot_y = self.last_pose.pose.position.y

        # ============ Erreurs ============
        target_dist = float(self.get_parameter('target_distance').value)
        # err_lin > 0 si Spot trop loin (le Summit doit AVANCER pour se rapprocher)
        err_lin = spot_x - target_dist

        # err_ang > 0 si Spot est a gauche (atan2 retourne positif a gauche)
        # On veut que le Summit TOURNE A GAUCHE (omega > 0) -> signe positif OK
        bearing = math.atan2(spot_y, spot_x)
        err_ang = bearing  # target = 0 (tag centre)

        # ============ Gains ============
        kp_lin = float(self.get_parameter('kp_lin').value)
        ki_lin = float(self.get_parameter('ki_lin').value)
        kp_ang = float(self.get_parameter('kp_ang').value)
        ki_ang = float(self.get_parameter('ki_ang').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        integ_max = float(self.get_parameter('integ_max').value)
        integ_ang_max = float(self.get_parameter('integ_ang_max').value)
        db_enter = float(self.get_parameter('deadband_enter').value)
        db_exit = float(self.get_parameter('deadband_exit').value)
        db_ang_enter = float(self.get_parameter('deadband_ang_enter').value)
        db_ang_exit = float(self.get_parameter('deadband_ang_exit').value)
        bearing_max = float(self.get_parameter('bearing_max').value)

        # ============ Deadband LINEAIRE avec hysteresis ============
        abs_err_lin = abs(err_lin)
        if not self.in_deadband and abs_err_lin < db_enter:
            self.in_deadband = True
            self.get_logger().info(
                f"Entree deadband LIN (err={err_lin*100:+.1f}cm < {db_enter*100:.0f}cm)",
                throttle_duration_sec=1.0)
        elif self.in_deadband and abs_err_lin > db_exit:
            self.in_deadband = False
            self.get_logger().info(
                f"Sortie deadband LIN (err={err_lin*100:+.1f}cm > {db_exit*100:.0f}cm)",
                throttle_duration_sec=1.0)

        # ============ Deadband ANGULAIRE avec hysteresis ============
        abs_err_ang = abs(err_ang)
        if not self.in_deadband_ang and abs_err_ang < db_ang_enter:
            self.in_deadband_ang = True
            self.get_logger().info(
                f"Entree deadband ANG (err={math.degrees(err_ang):+.1f}deg)",
                throttle_duration_sec=1.0)
        elif self.in_deadband_ang and abs_err_ang > db_ang_exit:
            self.in_deadband_ang = False
            self.get_logger().info(
                f"Sortie deadband ANG (err={math.degrees(err_ang):+.1f}deg)",
                throttle_duration_sec=1.0)

        # ============ Loi de commande LINEAIRE (IP sur l'erreur) ============
        if self.in_deadband:
            u_lin_raw = 0.0
            u_lin_sat = 0.0
        else:
            u_test = kp_lin * err_lin + ki_lin * self.integral_err

            saturate_high = u_test >= v_max and err_lin > 0
            saturate_low = u_test <= -v_max and err_lin < 0

            if not (saturate_high or saturate_low):
                self.integral_err += err_lin * dt

            self.integral_err = max(-integ_max, min(integ_max, self.integral_err))

            u_lin_raw = kp_lin * err_lin + ki_lin * self.integral_err
            u_lin_sat = max(-v_max, min(v_max, u_lin_raw))

        # ============ Loi de commande ANGULAIRE (IP sur l'erreur) ============
        if self.in_deadband_ang:
            u_ang_raw = 0.0
            u_ang_sat = 0.0
        else:
            u_test_ang = kp_ang * err_ang + ki_ang * self.integral_ang

            saturate_high_ang = u_test_ang >= w_max and err_ang > 0
            saturate_low_ang = u_test_ang <= -w_max and err_ang < 0

            if not (saturate_high_ang or saturate_low_ang):
                self.integral_ang += err_ang * dt

            self.integral_ang = max(-integ_ang_max, min(integ_ang_max, self.integral_ang))

            u_ang_raw = kp_ang * err_ang + ki_ang * self.integral_ang
            u_ang_sat = max(-w_max, min(w_max, u_ang_raw))

        # ============ Couplage : reduire le lineaire si bearing grand ============
        # factor = 1 quand bearing=0 (tag centre)
        # factor = 0 quand |bearing| >= bearing_max (tag tres decale)
        # En zone intermediaire, decroissance lineaire
        align_factor = max(0.0, 1.0 - abs(bearing) / bearing_max)
        u_lin_coupled = u_lin_sat * align_factor

        # ============ Rate limiter ============
        u_lin_final = self._apply_rate_limit(u_lin_coupled, self.last_cmd_lin, dt)
        u_ang_final = self._apply_rate_limit_ang(u_ang_sat, self.last_cmd_ang, dt)

        # ============ Publication ============
        self._publish_cmd(u_lin_final, u_ang_final)

        self._publish_float(self.pub_err, err_lin)
        self._publish_float(self.pub_integ, self.integral_err)
        self._publish_float(self.pub_cmd_raw, u_lin_raw)
        self._publish_float(self.pub_cmd_sat, u_lin_sat)
        self._publish_float(self.pub_cmd_lim, u_lin_final)

        self.last_tick_time = now

    # ============ Helpers ============

    def _apply_rate_limit(self, target, last, dt):
        rising = float(self.get_parameter('rising_accel').value)
        falling = float(self.get_parameter('falling_accel').value)
        delta = target - last
        max_delta = rising * dt if delta > 0 else falling * dt
        delta = max(-max_delta, min(max_delta, delta))
        return last + delta

    def _apply_rate_limit_ang(self, target, last, dt):
        rising = float(self.get_parameter('rising_alpha').value)
        falling = float(self.get_parameter('falling_alpha').value)
        delta = target - last
        max_delta = rising * dt if delta > 0 else falling * dt
        delta = max(-max_delta, min(max_delta, delta))
        return last + delta

    def _publish_cmd(self, v_lin, v_ang):
        cmd = Twist()
        cmd.linear.x = float(v_lin)
        cmd.angular.z = float(v_ang)

        # Toujours publier sur le topic debug
        self.pub_cmd_debug.publish(cmd)

        # Publier sur le vrai topic SEULEMENT si publish_real est True
        if self.publish_real:
            self.pub_cmd_real.publish(cmd)

        self.last_cmd_lin = v_lin
        self.last_cmd_ang = v_ang

    def _publish_zero(self):
        cmd = Twist()
        self.pub_cmd_debug.publish(cmd)
        if self.publish_real:
            self.pub_cmd_real.publish(cmd)

    def _publish_float(self, pub, value):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def _log_stats(self):
        if not self.enabled:
            self.get_logger().info(
                f"[STATS] DISABLED, publish_real={self.publish_real}")
            return

        if self.last_pose is None:
            self.get_logger().info("[STATS] ENABLED, no pose received yet")
            return

        spot_x = self.last_pose.pose.position.x
        spot_y = self.last_pose.pose.position.y
        target = float(self.get_parameter('target_distance').value)
        err = spot_x - target
        bearing_deg = math.degrees(math.atan2(spot_y, spot_x))

        mode = "REAL" if self.publish_real else "DEBUG"
        db_lin = "DB" if self.in_deadband else "  "
        db_ang = "DB" if self.in_deadband_ang else "  "

        self.get_logger().info(
            f"[STATS-{mode}] spot_x={spot_x:.2f}m err={err:+.2f} [{db_lin}] | "
            f"bearing={bearing_deg:+5.1f}deg [{db_ang}] | "
            f"integ=({self.integral_err:+.2f},{self.integral_ang:+.2f}) | "
            f"cmd=(lin={self.last_cmd_lin:+.2f}m/s ang={self.last_cmd_ang:+.2f}rad/s)")


def main():
    rclpy.init()
    node = IpControllerDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_zero()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
