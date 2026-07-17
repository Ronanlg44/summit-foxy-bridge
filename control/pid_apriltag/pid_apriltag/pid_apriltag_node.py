"""
Controleur PI (Proportionnel-Integral classique) du Summit XL
pour suivre le Spot via AprilTag.

- Vise un point projete derriere le Spot dans son axe de marche
- Si Spot immobile : vise directement le Tag 0 (pas de courbe artificielle)
- Anti-collision via LiDAR

"""

import math
import os
import yaml
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64


POSE_TOPIC = '/spot_pose_in_summit'
CMD_DEBUG_TOPIC = '/cmd_vel_debug'
CMD_REAL_TOPIC = '/summit_xl/robotnik_base_control/cmd_vel'
ENABLE_TOPIC = '/ip_enable'
LIDAR_TOPIC = '/summit_xl/front_laser/scan'

ERR_TOPIC = '/ip_debug/error_dist'
INTEG_TOPIC = '/ip_debug/integral'
CMD_RAW_TOPIC = '/ip_debug/cmd_raw'
CMD_SAT_TOPIC = '/ip_debug/cmd_saturated'
CMD_LIM_TOPIC = '/ip_debug/cmd_rate_limited'

HYSTERESIS_FACTOR = 1.5
PARAMS_FILE = '/opt/pid_params.yaml'

# Anti-collision LiDAR (non modifiables via IHM)
COLLISION_DIST_STOP = 0.5       # m
COLLISION_DIST_SLOW = 0.7       # m
COLLISION_CONE_DEG = 45.0       # deg
COLLISION_TIMEOUT = 0.5         # s

DEFAULT_PARAMS = {
    'target_distance': 1.5,
    'kp_lin': 1.804,
    'ki_lin': 0.817,
    'kp_ang': 1.60555042185557,
    'ki_ang': 0.621528632958943,
    'v_max': 0.3,
    'w_max': 0.5,
    'max_accel_lin': 1.5,
    'max_accel_ang': 2.0,
    'integ_max': 1.5,
    'integ_ang_max': 1.5,
    'deadband_lin': 0.03,
    'deadband_ang': 0.035,
    'bearing_max': 0.52,
    'pose_timeout': 0.5,
    'rate_hz': 20.0,
    'start_enabled': True,
    'publish_real_cmd': True,
    # Pure pursuit
    'pure_pursuit_enabled': True,
    'stationary_speed_threshold': 0.1,  # m/s
}


def load_params():
    if not os.path.exists(PARAMS_FILE):
        print(f"[PID] Fichier {PARAMS_FILE} absent, utilisation des defauts")
        return dict(DEFAULT_PARAMS)

    try:
        with open(PARAMS_FILE, 'r') as f:
            loaded = yaml.safe_load(f) or {}
        params = dict(DEFAULT_PARAMS)
        params.update(loaded)
        print(f"[PID] Parametres charges depuis {PARAMS_FILE}")
        return params
    except Exception as e:
        print(f"[PID] Erreur lecture {PARAMS_FILE} : {e}. Utilisation des defauts.")
        return dict(DEFAULT_PARAMS)


class PiControllerDebug(Node):

    def __init__(self):
        super().__init__('ip_apriltag_debug')

        loaded_params = load_params()
        for name, value in loaded_params.items():
            self.declare_parameter(name, value)

        self.last_pose: Optional[PoseStamped] = None
        self.last_pose_time: Optional[rclpy.time.Time] = None

        self.integral_err = 0.0
        self.integral_ang = 0.0
        self.last_tick_time: Optional[rclpy.time.Time] = None

        self.last_cmd_lin = 0.0
        self.last_cmd_ang = 0.0

        self.in_deadband = False
        self.in_deadband_ang = False

        # Anti-collision : etat interne
        self.last_scan: Optional[LaserScan] = None
        self.last_scan_time: Optional[rclpy.time.Time] = None
        self.min_dist_front = float('inf')

        # Pure pursuit : etat pour estimation vitesse Spot
        self.prev_spot_x = None
        self.prev_spot_y = None
        self.prev_pose_time = None
        self.spot_speed_ema = 0.0
        self.is_pursuing = False  # Etat courant : True si en mode pure pursuit

        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.publish_real = bool(self.get_parameter('publish_real_cmd').value)

        self.create_subscription(PoseStamped, POSE_TOPIC, self.on_pose, 10)
        self.create_subscription(Bool, ENABLE_TOPIC, self.on_enable, 10)
        self.create_subscription(LaserScan, LIDAR_TOPIC, self.on_scan, 10)

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

        kp_lin = self.get_parameter('kp_lin').value
        ki_lin = self.get_parameter('ki_lin').value
        target = self.get_parameter('target_distance').value
        pp_enabled = self.get_parameter('pure_pursuit_enabled').value
        self.get_logger().info("=" * 70)
        self.get_logger().info("PI Controller LIN + ANG (V7 - pure pursuit)")
        self.get_logger().info(f"  Cible distance     : {target:.2f} m")
        self.get_logger().info(f"  Gains LIN          : Kp = {kp_lin:.4f}, Ki = {ki_lin:.4f}")
        self.get_logger().info(f"  Pure pursuit       : {'ENABLED' if pp_enabled else 'DISABLED'}")
        self.get_logger().info(f"  Anti-collision     : stop < {COLLISION_DIST_STOP}m, slow < {COLLISION_DIST_SLOW}m")
        self.get_logger().info(f"  Cone avant         : +/- {COLLISION_CONE_DEG} deg")
        self.get_logger().info(f"  Etat initial       : {'ENABLED' if self.enabled else 'DISABLED'}")
        if self.publish_real:
            self.get_logger().warn("  /!\\ publish_real_cmd = True /!\\")
        self.get_logger().info("=" * 70)

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
                # Reset estimation vitesse
                self.prev_spot_x = None
                self.prev_spot_y = None
                self.prev_pose_time = None
                self.spot_speed_ema = 0.0
            else:
                self.get_logger().info(">>> CONTROLEUR DESACTIVE")
                self._publish_zero()
                self.last_cmd_lin = 0.0
                self.last_cmd_ang = 0.0

    def on_scan(self, msg: LaserScan):
        """Callback LiDAR : calcule la distance min dans le cone avant."""
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

        cone_rad = math.radians(COLLISION_CONE_DEG)
        n = len(msg.ranges)
        if n == 0:
            self.min_dist_front = float('inf')
            return

        min_dist = float('inf')
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if -cone_rad <= angle <= cone_rad:
                if r < min_dist:
                    min_dist = r
        self.min_dist_front = min_dist

    def _apply_anti_collision(self, v_lin):
        """Ralentit/arrete si obstacle devant. Recul non protege."""
        now = self.get_clock().now()
        if self.last_scan_time is None:
            self.get_logger().warn("Pas de scan LiDAR recu, freinage par securite",
                                   throttle_duration_sec=5.0)
            return 0.0
        age = (now - self.last_scan_time).nanoseconds * 1e-9
        if age > COLLISION_TIMEOUT:
            self.get_logger().warn(f"Scan LiDAR obsolete ({age:.1f}s), freinage",
                                   throttle_duration_sec=2.0)
            return 0.0

        if v_lin <= 0:
            return v_lin

        dist = self.min_dist_front

        if dist <= COLLISION_DIST_STOP:
            self.get_logger().warn(f"Obstacle a {dist:.2f}m : ARRET",
                                   throttle_duration_sec=1.0)
            return 0.0
        elif dist <= COLLISION_DIST_SLOW:
            factor = (dist - COLLISION_DIST_STOP) / (COLLISION_DIST_SLOW - COLLISION_DIST_STOP)
            return v_lin * factor
        else:
            return v_lin

    def _update_spot_speed(self, spot_x, spot_y):
        """Met a jour l'estimation EMA de la vitesse du Spot."""
        if self.prev_spot_x is not None and self.prev_pose_time is not None:
            dt_pose = (self.last_pose_time - self.prev_pose_time).nanoseconds * 1e-9
            if dt_pose > 0.001:
                dx = spot_x - self.prev_spot_x
                dy = spot_y - self.prev_spot_y
                instant_speed = math.sqrt(dx * dx + dy * dy) / dt_pose
                # EMA alpha=0.3 : lisse fort le bruit
                self.spot_speed_ema = 0.7 * self.spot_speed_ema + 0.3 * instant_speed

        self.prev_spot_x = spot_x
        self.prev_spot_y = spot_y
        self.prev_pose_time = self.last_pose_time

    def tick(self):
        self.publish_real = bool(self.get_parameter('publish_real_cmd').value)

        if not self.enabled:
            return

        now = self.get_clock().now()

        if self.last_tick_time is None:
            dt = 1.0 / float(self.get_parameter('rate_hz').value)
        else:
            dt = (now - self.last_tick_time).nanoseconds * 1e-9
            dt = max(0.001, min(0.5, dt))

        timeout = float(self.get_parameter('pose_timeout').value)
        pose_stale = (
            self.last_pose is None
            or self.last_pose_time is None
            or (now - self.last_pose_time).nanoseconds * 1e-9 > timeout
        )

        if pose_stale:
            self.get_logger().warn("Pose obsolete, freinage vers 0",
                                   throttle_duration_sec=2.0)
            u_lin_final = self._apply_rate_limit_lin(0.0, self.last_cmd_lin, dt)
            u_ang_final = self._apply_rate_limit_ang(0.0, self.last_cmd_ang, dt)
            self._publish_cmd(u_lin_final, u_ang_final)
            self.last_tick_time = now
            return

        spot_x = self.last_pose.pose.position.x
        spot_y = self.last_pose.pose.position.y

        # Estimation vitesse du Spot (pour differencier mobile/immobile)
        self._update_spot_speed(spot_x, spot_y)

        # Extraction yaw du Tag 0 depuis le quaternion
        q = self.last_pose.pose.orientation
        spot_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        target_dist = float(self.get_parameter('target_distance').value)
        pure_pursuit_enabled = bool(self.get_parameter('pure_pursuit_enabled').value)
        speed_threshold = float(self.get_parameter('stationary_speed_threshold').value)

        # ========== Calcul du point cible ==========
        # Convention Tag 0 : face regarde vers l'arriere du Spot (queue).
        # spot_yaw est l'orientation de la normale du Tag 0 dans le repere Summit.
        # Le Spot avance dans la direction OPPOSEE (spot_yaw + pi).
        # Point cible "derriere le Spot dans son sens de marche" =
        # position Tag 0 + target_dist dans la direction OU regarde le Tag 0.
        self.is_pursuing = (pure_pursuit_enabled
                            and self.spot_speed_ema > speed_threshold)

        if self.is_pursuing:
            # Spot mobile : viser un point projete derriere le Spot
            target_point_x = spot_x + target_dist * math.cos(spot_yaw)
            target_point_y = spot_y + target_dist * math.sin(spot_yaw)
            # Erreur = distance au point cible
            err_lin = math.sqrt(target_point_x ** 2 + target_point_y ** 2)
            bearing = math.atan2(target_point_y, target_point_x)
        else:
            # Spot immobile : viser le Tag 0 directement (comportement classique)
            err_lin = spot_x - target_dist
            bearing = math.atan2(spot_y, spot_x)

        err_ang = bearing

        # ========== Gains et parametres ==========
        kp_lin = float(self.get_parameter('kp_lin').value)
        ki_lin = float(self.get_parameter('ki_lin').value)
        kp_ang = float(self.get_parameter('kp_ang').value)
        ki_ang = float(self.get_parameter('ki_ang').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        integ_max = float(self.get_parameter('integ_max').value)
        integ_ang_max = float(self.get_parameter('integ_ang_max').value)

        db_lin_enter = float(self.get_parameter('deadband_lin').value)
        db_lin_exit = db_lin_enter * HYSTERESIS_FACTOR
        db_ang_enter = float(self.get_parameter('deadband_ang').value)
        db_ang_exit = db_ang_enter * HYSTERESIS_FACTOR
        bearing_max = float(self.get_parameter('bearing_max').value)

        # ========== Deadband + hysteresis ==========
        abs_err_lin = abs(err_lin)
        if not self.in_deadband and abs_err_lin < db_lin_enter:
            self.in_deadband = True
        elif self.in_deadband and abs_err_lin > db_lin_exit:
            self.in_deadband = False

        abs_err_ang = abs(err_ang)
        if not self.in_deadband_ang and abs_err_ang < db_ang_enter:
            self.in_deadband_ang = True
        elif self.in_deadband_ang and abs_err_ang > db_ang_exit:
            self.in_deadband_ang = False

        # ========== PI lineaire ==========
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

        # ========== PI angulaire ==========
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

        # ========== Couplage lineaire/angulaire ==========
        align_factor = max(0.0, 1.0 - abs(bearing) / bearing_max)
        u_lin_coupled = u_lin_sat * align_factor

        # ========== Anti-collision AVANT rate limiter ==========
        u_lin_coupled = self._apply_anti_collision(u_lin_coupled)

        # ========== Rate limiter ==========
        u_lin_final = self._apply_rate_limit_lin(u_lin_coupled, self.last_cmd_lin, dt)
        u_ang_final = self._apply_rate_limit_ang(u_ang_sat, self.last_cmd_ang, dt)

        self._publish_cmd(u_lin_final, u_ang_final)

        self._publish_float(self.pub_err, err_lin)
        self._publish_float(self.pub_integ, self.integral_err)
        self._publish_float(self.pub_cmd_raw, u_lin_raw)
        self._publish_float(self.pub_cmd_sat, u_lin_sat)
        self._publish_float(self.pub_cmd_lim, u_lin_final)

        self.last_tick_time = now

    def _apply_rate_limit_lin(self, target, last, dt):
        max_accel = float(self.get_parameter('max_accel_lin').value)
        max_delta = max_accel * dt
        delta = target - last
        delta = max(-max_delta, min(max_delta, delta))
        return last + delta

    def _apply_rate_limit_ang(self, target, last, dt):
        max_accel = float(self.get_parameter('max_accel_ang').value)
        max_delta = max_accel * dt
        delta = target - last
        delta = max(-max_delta, min(max_delta, delta))
        return last + delta

    def _publish_cmd(self, v_lin, v_ang):
        cmd = Twist()
        cmd.linear.x = float(v_lin)
        cmd.angular.z = float(v_ang)
        self.pub_cmd_debug.publish(cmd)
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
        pp_str = "PP" if self.is_pursuing else "  "

        if math.isinf(self.min_dist_front):
            coll_str = "coll=INF"
        else:
            coll_str = f"coll={self.min_dist_front:.2f}m"

        self.get_logger().info(
            f"[STATS-{mode}] spot_x={spot_x:.2f}m err={err:+.2f} [{db_lin}] | "
            f"bearing={bearing_deg:+5.1f}deg [{db_ang}] | "
            f"speed={self.spot_speed_ema:.2f}m/s [{pp_str}] | "
            f"integ=({self.integral_err:+.2f},{self.integral_ang:+.2f}) | "
            f"cmd=(lin={self.last_cmd_lin:+.2f}m/s ang={self.last_cmd_ang:+.2f}rad/s) | "
            f"{coll_str}")


def main():
    rclpy.init()
    node = PiControllerDebug()
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
