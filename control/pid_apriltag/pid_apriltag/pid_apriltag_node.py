"""
PID 2-DOF d'asservissement du Summit XL pour suivre le Spot.

Souscrit a /spot_target_pose (PoseStamped dans summit_xl_base_link) qui
contient la pose du Spot vue par le Summit. Publie /summit_xl/robotnik_base_control/cmd_vel
pour maintenir une distance cible et garder le Spot devant.

============================================================
STRUCTURE 2-DOF (Two-Degree-Of-Freedom PID)
============================================================
  u = Kp*(b*r - y) + Ki*INT(r - y)dt + Kd*d(c*r - y)/dt

Avec b et c les "setpoint weights" optimises par MATLAB.

============================================================
GAIN SCHEDULING SELON LA SOURCE DE PERCEPTION
============================================================
Le supervisor publie /perception_status au format "STATE conf=X.XX".
Selon l'etat (TAG_OK ou YOLO_TRACKING), on utilise des gains differents :

- TAG_OK         : latence vision ~200 ms, gains rapides (tunes pour Td=200ms)
- YOLO_TRACKING  : latence vision ~400 ms (YOLO CPU lourd), gains prudents
- LOST           : on garde les gains TAG (ne devrait pas etre utilise car pas de cmd publiee)
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String


# --------- Topics ---------
POSE_TOPIC = '/spot_target_pose'
CMD_VEL_TOPIC = '/summit_xl/robotnik_base_control/cmd_vel'
ENABLE_TOPIC = '/pid_enable'
PERCEPTION_STATUS_TOPIC = '/perception_status'


class PidApriltag(Node):

    def __init__(self):
        super().__init__('pid_apriltag')

        # ============ Parametres ROS 2 ============
        self.declare_parameter('target_distance', 1.07)

        # ----------- Gains pour TAG_OK (Td_vision ~ 200 ms) -----------
        self.declare_parameter('kp_lin_tag', 3.43)
        self.declare_parameter('ki_lin_tag', 2.10)
        self.declare_parameter('kd_lin_tag', 0.39)
        self.declare_parameter('b_lin_tag', 0.56)
        self.declare_parameter('c_lin_tag', 0.007)

        self.declare_parameter('kp_ang_tag', 2.00)
        self.declare_parameter('ki_ang_tag', 0.83)
        self.declare_parameter('kd_ang_tag', 0.44)
        self.declare_parameter('b_ang_tag', 0.49)
        self.declare_parameter('c_ang_tag', 0.025)

        # ----------- Gains pour YOLO_TRACKING (Td_vision ~ 400 ms) -----------
        # A re-tuner dans MATLAB avec H_vision = 0.4 s
        self.declare_parameter('kp_lin_yolo', 1.70)
        self.declare_parameter('ki_lin_yolo', 1.00)
        self.declare_parameter('kd_lin_yolo', 0.20)
        self.declare_parameter('b_lin_yolo', 0.56)
        self.declare_parameter('c_lin_yolo', 0.007)

        self.declare_parameter('kp_ang_yolo', 1.00)
        self.declare_parameter('ki_ang_yolo', 0.40)
        self.declare_parameter('kd_ang_yolo', 0.22)
        self.declare_parameter('b_ang_yolo', 0.49)
        self.declare_parameter('c_ang_yolo', 0.025)

        # ----------- Saturations -----------
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 0.8)
        self.declare_parameter('max_lin_accel', 0.5)
        self.declare_parameter('max_ang_accel', 1.0)

        # ----------- Deadbands -----------
        self.declare_parameter('deadband_dist', 0.10)
        self.declare_parameter('deadband_angle_deg', 3.0)

        # ----------- Securite -----------
        self.declare_parameter('pose_timeout', 0.5)
        self.declare_parameter('integral_max', 0.5)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('start_enabled', False)

        # ============ Etat interne ============
        self.last_pose: Optional[PoseStamped] = None
        self.last_pose_time: Optional[rclpy.time.Time] = None

        self.last_err_D_dist = 0.0
        self.last_err_D_ang = 0.0

        self.integral_dist = 0.0
        self.integral_ang = 0.0

        self.last_tick_time: Optional[rclpy.time.Time] = None

        self.last_cmd_lin = 0.0
        self.last_cmd_ang = 0.0

        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.perception_state = 'TAG_OK'

        # ============ I/O ============
        self.create_subscription(PoseStamped, POSE_TOPIC, self.on_pose, 10)
        self.create_subscription(Bool, ENABLE_TOPIC, self.on_enable, 10)
        self.create_subscription(
            String, PERCEPTION_STATUS_TOPIC, self.on_perception_status, 10)

        self.pub_cmd = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(2.0, self._log_stats)

        self.get_logger().info(
            f"PID 2-DOF AprilTag pret avec gain scheduling. "
            f"Demarrage {'ENABLED' if self.enabled else 'DISABLED'}. "
            f"Cible distance = {self.get_parameter('target_distance').value} m. "
            f"Mode initial = {self.perception_state}.")

    # ============ Callbacks ============

    def on_pose(self, msg: PoseStamped):
        self.last_pose = msg
        self.last_pose_time = self.get_clock().now()

    def on_enable(self, msg: Bool):
        new_state = bool(msg.data)
        if new_state != self.enabled:
            self.enabled = new_state
            if self.enabled:
                self.get_logger().info("PID ACTIVE.")
                self.integral_dist = 0.0
                self.integral_ang = 0.0
                self.last_tick_time = None
            else:
                self.get_logger().info("PID DESACTIVE.")
                self._publish_zero()
                self.last_cmd_lin = 0.0
                self.last_cmd_ang = 0.0

    def on_perception_status(self, msg: String):
        """Parse '/perception_status' au format 'STATE conf=X.XX'."""
        parts = msg.data.split()
        if not parts:
            return
        new_state = parts[0]
        if new_state in ('TAG_OK', 'YOLO_TRACKING', 'LOST'):
            if new_state != self.perception_state:
                self.get_logger().info(
                    f"Switch gains : {self.perception_state} -> {new_state}")
                self.perception_state = new_state

    # ============ Selection des gains ============

    def _get_current_gains(self):
        """Retourne les gains PID selon le mode de perception courant."""
        if self.perception_state == 'YOLO_TRACKING':
            suffix = 'yolo'
        else:
            suffix = 'tag'

        return {
            'kp_lin': float(self.get_parameter(f'kp_lin_{suffix}').value),
            'ki_lin': float(self.get_parameter(f'ki_lin_{suffix}').value),
            'kd_lin': float(self.get_parameter(f'kd_lin_{suffix}').value),
            'b_lin':  float(self.get_parameter(f'b_lin_{suffix}').value),
            'c_lin':  float(self.get_parameter(f'c_lin_{suffix}').value),
            'kp_ang': float(self.get_parameter(f'kp_ang_{suffix}').value),
            'ki_ang': float(self.get_parameter(f'ki_ang_{suffix}').value),
            'kd_ang': float(self.get_parameter(f'kd_ang_{suffix}').value),
            'b_ang':  float(self.get_parameter(f'b_ang_{suffix}').value),
            'c_ang':  float(self.get_parameter(f'c_ang_{suffix}').value),
        }

    # ============ Boucle principale ============

    def tick(self):
        if not self.enabled:
            return

        now = self.get_clock().now()

        if self.last_tick_time is None:
            dt = 1.0 / float(self.get_parameter('rate_hz').value)
        else:
            dt = (now - self.last_tick_time).nanoseconds * 1e-9
            dt = max(0.001, min(0.5, dt))

        timeout = float(self.get_parameter('pose_timeout').value)
        if (self.last_pose is None or self.last_pose_time is None
                or (now - self.last_pose_time).nanoseconds * 1e-9 > timeout):
            self.get_logger().warn(
                "Pose obsolete, freinage progressif vers 0",
                throttle_duration_sec=2.0)
            self._publish_with_rate_limit(0.0, 0.0, dt)
            self.last_tick_time = now
            return

        x = self.last_pose.pose.position.x
        y = self.last_pose.pose.position.y
        dist = math.hypot(x, y)
        bearing = math.atan2(y, x)

        target_dist = float(self.get_parameter('target_distance').value)
        target_ang = 0.0

        err_simple_dist = dist - target_dist
        err_simple_ang = bearing - target_ang
        db_dist = float(self.get_parameter('deadband_dist').value)
        db_ang = math.radians(float(self.get_parameter('deadband_angle_deg').value))

        g = self._get_current_gains()

        if abs(err_simple_dist) < db_dist and abs(err_simple_ang) < db_ang:
            self._publish_with_rate_limit(0.0, 0.0, dt)
            self.last_err_D_dist = dist - g['c_lin'] * target_dist
            self.last_err_D_ang = bearing - g['c_ang'] * target_ang
            self.last_tick_time = now
            return

        i_max = float(self.get_parameter('integral_max').value)

        # PID 2-DOF lineaire
        err_P_dist = dist - g['b_lin'] * target_dist
        err_I_dist = dist - target_dist
        err_D_dist = dist - g['c_lin'] * target_dist

        self.integral_dist += err_I_dist * dt
        self.integral_dist = max(-i_max, min(i_max, self.integral_dist))

        deriv_dist = (err_D_dist - self.last_err_D_dist) / dt

        v_lin_target = (g['kp_lin'] * err_P_dist
                        + g['ki_lin'] * self.integral_dist
                        + g['kd_lin'] * deriv_dist)

        # PID 2-DOF angulaire
        err_P_ang = bearing - g['b_ang'] * target_ang
        err_I_ang = bearing - target_ang
        err_D_ang = bearing - g['c_ang'] * target_ang

        self.integral_ang += err_I_ang * dt
        self.integral_ang = max(-i_max, min(i_max, self.integral_ang))

        deriv_ang = (err_D_ang - self.last_err_D_ang) / dt

        v_ang_target = (g['kp_ang'] * err_P_ang
                        + g['ki_ang'] * self.integral_ang
                        + g['kd_ang'] * deriv_ang)

        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)
        v_lin_target = max(-v_max, min(v_max, v_lin_target))
        v_ang_target = max(-w_max, min(w_max, v_ang_target))

        self._publish_with_rate_limit(v_lin_target, v_ang_target, dt)

        self.last_err_D_dist = err_D_dist
        self.last_err_D_ang = err_D_ang
        self.last_tick_time = now

    # ============ Helpers ============

    def _publish_with_rate_limit(self, v_lin_target, v_ang_target, dt):
        max_lin_accel = float(self.get_parameter('max_lin_accel').value)
        max_ang_accel = float(self.get_parameter('max_ang_accel').value)

        max_dv = max_lin_accel * dt
        max_dw = max_ang_accel * dt

        delta_lin = v_lin_target - self.last_cmd_lin
        delta_lin = max(-max_dv, min(max_dv, delta_lin))
        v_lin = self.last_cmd_lin + delta_lin

        delta_ang = v_ang_target - self.last_cmd_ang
        delta_ang = max(-max_dw, min(max_dw, delta_ang))
        v_ang = self.last_cmd_ang + delta_ang

        cmd = Twist()
        cmd.linear.x = v_lin
        cmd.angular.z = v_ang
        self.pub_cmd.publish(cmd)

        self.last_cmd_lin = v_lin
        self.last_cmd_ang = v_ang

    def _publish_zero(self):
        cmd = Twist()
        self.pub_cmd.publish(cmd)

    def _log_stats(self):
        if not self.enabled:
            self.get_logger().info(f"[STATS] DISABLED, mode={self.perception_state}")
            return

        if self.last_pose is None:
            self.get_logger().info(
                f"[STATS] ENABLED, mode={self.perception_state}, no pose yet")
            return

        x = self.last_pose.pose.position.x
        y = self.last_pose.pose.position.y
        dist = math.hypot(x, y)
        bearing_deg = math.degrees(math.atan2(y, x))
        target = float(self.get_parameter('target_distance').value)

        self.get_logger().info(
            f"[STATS] mode={self.perception_state}, "
            f"dist={dist:.2f}m (target={target:.2f}, err={dist-target:+.2f}), "
            f"bearing={bearing_deg:+.1f}deg, "
            f"cmd=({self.last_cmd_lin:+.2f}m/s, {self.last_cmd_ang:+.2f}rad/s)")


def main():
    rclpy.init()
    node = PidApriltag()
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
