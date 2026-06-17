"""
Envoie un echelon de cmd_vel propre pour l'identification systeme du Summit XL.

Selon le parametre `axis` :
- 'linear'  : echelon sur cmd_vel.linear.x  (m/s)
- 'angular' : echelon sur cmd_vel.angular.z (rad/s)

Sequence par defaut :
- warmup_duration s a 0 (warmup, etablit la connexion)
- step_duration s a step_value (echelon)
- cooldown_duration s a 0 (retour au repos)
- Termine et publie 0

Parametres ROS 2 :
  axis               (default 'linear')   'linear' ou 'angular'
  step_value         (default 0.2)        valeur de l'echelon (m/s ou rad/s)
  step_duration      (default 3.0)        duree de l'echelon en s
  warmup_duration    (default 1.0)        duree avant l'echelon en s
  cooldown_duration  (default 2.0)        duree apres l'echelon en s
  rate_hz            (default 50.0)       frequence de publication

Exemples :
  Lineaire 0.2 m/s :  ros2 run step_input step_input
  Angulaire 0.5 rad/s : ros2 run step_input step_input --ros-args \\
      -p axis:=angular -p step_value:=0.5

Topic publie : /summit_xl/robotnik_base_control/cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


CMD_VEL_TOPIC = '/summit_xl/robotnik_base_control/cmd_vel'


class StepInput(Node):

    def __init__(self):
        super().__init__('step_input')

        self.declare_parameter('axis', 'linear')        # 'linear' ou 'angular'
        self.declare_parameter('step_value', 0.2)
        self.declare_parameter('step_duration', 3.0)
        self.declare_parameter('warmup_duration', 1.0)
        self.declare_parameter('cooldown_duration', 2.0)
        self.declare_parameter('rate_hz', 50.0)

        self.axis = str(self.get_parameter('axis').value).lower()
        if self.axis not in ('linear', 'angular'):
            self.get_logger().error(
                f"axis doit etre 'linear' ou 'angular', recu : {self.axis}")
            raise ValueError(f"axis invalide : {self.axis}")

        self.v_step = float(self.get_parameter('step_value').value)
        self.t_warmup = float(self.get_parameter('warmup_duration').value)
        self.t_step = float(self.get_parameter('step_duration').value)
        self.t_cooldown = float(self.get_parameter('cooldown_duration').value)
        rate = float(self.get_parameter('rate_hz').value)

        self.t_total = self.t_warmup + self.t_step + self.t_cooldown
        self.t0 = self.get_clock().now()
        self.finished = False

        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        unit = 'm/s' if self.axis == 'linear' else 'rad/s'
        self.get_logger().info(
            f"step_input pret (axis={self.axis}). Sequence :\n"
            f"  [0, {self.t_warmup:.1f}]s : 0 {unit} (warmup)\n"
            f"  [{self.t_warmup:.1f}, {self.t_warmup + self.t_step:.1f}]s : "
            f"{self.v_step} {unit} (echelon)\n"
            f"  [{self.t_warmup + self.t_step:.1f}, {self.t_total:.1f}]s : 0 {unit} (cooldown)\n"
            f"Publie sur {CMD_VEL_TOPIC}")

    def tick(self):
        if self.finished:
            return

        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        cmd = Twist()
        if elapsed < self.t_warmup:
            value = 0.0
            phase = "warmup"
        elif elapsed < self.t_warmup + self.t_step:
            value = self.v_step
            phase = "echelon"
        elif elapsed < self.t_total:
            value = 0.0
            phase = "cooldown"
        else:
            # Securite finale : 0 sur les deux axes
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            self.get_logger().info("Sequence terminee. Robot a l'arret.")
            self.finished = True
            return

        # Applique sur l'axe choisi
        if self.axis == 'linear':
            cmd.linear.x = value
        else:
            cmd.angular.z = value

        self.pub.publish(cmd)

        # Log toutes les 0.5s
        if int(elapsed * 2) != int((elapsed - 1.0 / 50.0) * 2):
            unit = 'm/s' if self.axis == 'linear' else 'rad/s'
            self.get_logger().info(
                f"t={elapsed:.2f}s phase={phase} cmd={value:.2f} {unit}")


def main():
    rclpy.init()
    node = StepInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
