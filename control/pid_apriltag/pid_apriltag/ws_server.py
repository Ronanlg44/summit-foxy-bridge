import asyncio
import json
import math
import os
import time
import yaml
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterType

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool

import websockets


WS_HOST = "0.0.0.0"
WS_PORT = 8081

TARGET_NODE = 'ip_apriltag_debug'
PARAMS_FILE = '/opt/pid_params.yaml'

EXPOSED_PARAMS = [
    'v_max',
    'w_max',
    'max_accel_lin',
    'max_accel_ang',
    'deadband_lin',
    'deadband_ang',
    'kp_lin',
    'ki_lin',
    'kp_ang',
    'ki_ang',
    'target_distance',
]

POSE_TOPIC = '/spot_pose_in_summit'
CMD_DEBUG_TOPIC = '/cmd_vel_debug'
CMD_REAL_TOPIC = '/summit_xl/robotnik_base_control/cmd_vel'
ENABLE_TOPIC = '/ip_enable'


class WsBridgeNode(Node):

    def __init__(self):
        super().__init__('ws_bridge')

        self.last_pose_x = None
        self.last_pose_y = None
        self.last_pose_time = 0.0

        self.create_subscription(PoseStamped, POSE_TOPIC, self._on_pose, 10)

        self.pub_cmd_debug = self.create_publisher(Twist, CMD_DEBUG_TOPIC, 10)
        self.pub_cmd_real = self.create_publisher(Twist, CMD_REAL_TOPIC, 10)
        self.pub_enable = self.create_publisher(Bool, ENABLE_TOPIC, 10)

        self.cli_get = self.create_client(GetParameters, f'/{TARGET_NODE}/get_parameters')
        self.cli_set = self.create_client(SetParameters, f'/{TARGET_NODE}/set_parameters')

        self.get_logger().info("WsBridgeNode ready")

    def _on_pose(self, msg: PoseStamped):
        self.last_pose_x = msg.pose.position.x
        self.last_pose_y = msg.pose.position.y
        self.last_pose_time = time.time()

    def get_pose_snapshot(self):
        if self.last_pose_x is None:
            return None
        age = time.time() - self.last_pose_time
        if age > 1.0:
            return {"detected": False, "age_s": round(age, 1)}
        distance = math.sqrt(self.last_pose_x ** 2 + self.last_pose_y ** 2)
        bearing_deg = math.degrees(math.atan2(self.last_pose_y, self.last_pose_x))
        return {
            "detected": True,
            "spot_x": round(self.last_pose_x, 3),
            "spot_y": round(self.last_pose_y, 3),
            "distance_m": round(distance, 3),
            "bearing_deg": round(bearing_deg, 1),
            "age_s": round(age, 2),
        }

    def get_params(self, names=None):
        if names is None:
            names = EXPOSED_PARAMS

        if not self.cli_get.wait_for_service(timeout_sec=2.0):
            return {"error": "GetParameters service unavailable"}

        req = GetParameters.Request()
        req.names = names
        future = self.cli_get.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done() or future.result() is None:
            return {"error": "timeout"}

        result = future.result()
        params = {}
        for name, val in zip(names, result.values):
            if val.type == ParameterType.PARAMETER_DOUBLE:
                params[name] = val.double_value
            elif val.type == ParameterType.PARAMETER_INTEGER:
                params[name] = val.integer_value
            elif val.type == ParameterType.PARAMETER_BOOL:
                params[name] = val.bool_value
            else:
                params[name] = None
        return params

    def set_param(self, name, value, param_type='double'):
        if not self.cli_set.wait_for_service(timeout_sec=2.0):
            return {"error": "SetParameters service unavailable"}

        if param_type == 'bool':
            param = Parameter(name=name, value=bool(value)).to_parameter_msg()
        else:
            param = Parameter(name=name, value=float(value)).to_parameter_msg()

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.cli_set.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done() or future.result() is None:
            return {"error": "timeout"}

        result = future.result()
        if len(result.results) > 0 and result.results[0].successful:
            return {"ok": True, "name": name, "value": value}
        else:
            reason = result.results[0].reason if len(result.results) > 0 else "unknown"
            return {"error": f"set failed : {reason}"}

    def set_multiple_params(self, params_dict):
        if not self.cli_set.wait_for_service(timeout_sec=2.0):
            return {"error": "SetParameters service unavailable"}

        param_msgs = []
        for name, value in params_dict.items():
            if isinstance(value, bool):
                param_msgs.append(Parameter(name=name, value=bool(value)).to_parameter_msg())
            else:
                param_msgs.append(Parameter(name=name, value=float(value)).to_parameter_msg())

        req = SetParameters.Request()
        req.parameters = param_msgs

        future = self.cli_set.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if not future.done() or future.result() is None:
            return {"error": "timeout"}

        result = future.result()
        all_ok = all(r.successful for r in result.results)
        if all_ok:
            return {"ok": True, "count": len(param_msgs)}
        else:
            failures = [
                f"{name}: {r.reason}"
                for name, r in zip(params_dict.keys(), result.results)
                if not r.successful
            ]
            return {"error": f"set failed : {failures}"}

    def emergency_stop(self):
        zero = Twist()
        for _ in range(10):
            self.pub_cmd_debug.publish(zero)
            self.pub_cmd_real.publish(zero)
            time.sleep(0.05)
        enable_msg = Bool()
        enable_msg.data = False
        self.pub_enable.publish(enable_msg)
        self.set_param('publish_real_cmd', False, 'bool')
        return {"ok": True}

    def activate_pid(self):
        """Active le PID en mode reel avec menage prealable.

        Sequence :
        1. Desactive le PID (/ip_enable = false) : le node reset ses integrateurs
        2. Petit sleep pour laisser le node processer
        3. publish_real_cmd = true
        4. Reactive le PID (/ip_enable = true)
        """
        # 1. Reset : desactive proprement
        enable_msg = Bool()
        enable_msg.data = False
        self.pub_enable.publish(enable_msg)
        time.sleep(0.3)

        # 2. Passe en mode reel
        r = self.set_param('publish_real_cmd', True, 'bool')
        if 'error' in r:
            return r

        # 3. Active
        enable_msg = Bool()
        enable_msg.data = True
        self.pub_enable.publish(enable_msg)
        return {"ok": True}

    def save_params(self):
        all_names = EXPOSED_PARAMS + [
            'integ_max', 'integ_ang_max', 'bearing_max',
            'pose_timeout', 'rate_hz',
        ]
        params = self.get_params(all_names)
        if 'error' in params:
            return params

        params['publish_real_cmd'] = True
        params['start_enabled'] = True

        try:
            with open(PARAMS_FILE, 'w') as f:
                yaml.safe_dump(params, f, default_flow_style=False)
            return {"ok": True, "file": PARAMS_FILE}
        except Exception as e:
            return {"error": f"ecriture echouee : {e}"}

    def reload_params(self):
        if not os.path.exists(PARAMS_FILE):
            return {"error": f"fichier {PARAMS_FILE} inexistant"}

        try:
            with open(PARAMS_FILE, 'r') as f:
                loaded = yaml.safe_load(f) or {}
        except Exception as e:
            return {"error": f"lecture echouee : {e}"}

        to_apply = {k: v for k, v in loaded.items() if k in EXPOSED_PARAMS}
        return self.set_multiple_params(to_apply)


ros_node = None


async def handle_client(websocket):
    print(f"[WS] Client connecte : {websocket.remote_address}")

    async def send_pose_loop():
        while True:
            try:
                snap = ros_node.get_pose_snapshot()
                if snap is not None:
                    await websocket.send(json.dumps({"type": "pose", **snap}))
                await asyncio.sleep(0.2)
            except websockets.ConnectionClosed:
                return
            except Exception as e:
                print(f"[WS] Erreur envoi pose : {e}")
                return

    send_task = asyncio.create_task(send_pose_loop())

    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                action = data.get("action")

                if action == "get_params":
                    params = ros_node.get_params()
                    await websocket.send(json.dumps({"type": "params", **params}))

                elif action == "set_param":
                    name = data.get("name")
                    value = data.get("value")
                    result = ros_node.set_param(name, value)
                    await websocket.send(json.dumps({"type": "ack", "action": "set_param", **result}))

                elif action == "emergency_stop":
                    result = ros_node.emergency_stop()
                    await websocket.send(json.dumps({"type": "ack", "action": "emergency_stop", **result}))

                elif action == "activate_pid":
                    result = ros_node.activate_pid()
                    await websocket.send(json.dumps({"type": "ack", "action": "activate_pid", **result}))

                elif action == "save_params":
                    result = ros_node.save_params()
                    await websocket.send(json.dumps({"type": "ack", "action": "save_params", **result}))

                elif action == "reload_params":
                    result = ros_node.reload_params()
                    if 'ok' in result:
                        new_params = ros_node.get_params()
                        await websocket.send(json.dumps({"type": "params", **new_params}))
                    await websocket.send(json.dumps({"type": "ack", "action": "reload_params", **result}))

                else:
                    await websocket.send(json.dumps({"type": "error", "message": f"action inconnue : {action}"}))

            except json.JSONDecodeError:
                await websocket.send(json.dumps({"type": "error", "message": "JSON invalide"}))
            except Exception as e:
                await websocket.send(json.dumps({"type": "error", "message": str(e)}))

    except websockets.ConnectionClosed:
        pass
    finally:
        send_task.cancel()
        print(f"[WS] Client deconnecte : {websocket.remote_address}")


def ros_spin_thread():
    while rclpy.ok():
        rclpy.spin_once(ros_node, timeout_sec=0.1)


async def main_async():
    print(f"[WS] Serveur WebSocket sur ws://{WS_HOST}:{WS_PORT}")
    async with websockets.serve(handle_client, WS_HOST, WS_PORT):
        await asyncio.Future()


def main():
    global ros_node

    rclpy.init()
    ros_node = WsBridgeNode()

    spin_thread = Thread(target=ros_spin_thread, daemon=True)
    spin_thread.start()

    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
