"""
Extrait cmd_vel, odom ET spot_pose_in_summit d'un rosbag2 SQLite en CSV pour MATLAB.

Utilise sqlite3 + rclpy serialization (pas rosbag2_py qui n'est pas
toujours installe en Foxy).

Usage :
    python3 bag_to_csv.py <chemin_du_dossier_bag>

Produit : <dossier_bag>/ident.csv avec colonnes :
    time, cmd_lin, cmd_ang, odom_lin, odom_ang, spot_x, spot_y, spot_yaw

- cmd_lin    : commande cmd_vel.linear.x (m/s)
- cmd_ang    : commande cmd_vel.angular.z (rad/s)
- odom_lin   : vitesse mesuree odom.twist.linear.x (m/s)
- odom_ang   : vitesse mesuree odom.twist.angular.z (rad/s)
- spot_x     : position de Spot dans summit_xl_base_link (m, axe devant)
- spot_y     : position de Spot dans summit_xl_base_link (m, axe gauche)
- spot_yaw   : orientation de Spot autour de l'axe Z (rad)

"""

import sys
import csv
import sqlite3
import glob
from pathlib import Path

import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CMD_TOPIC = '/summit_xl/robotnik_base_control/cmd_vel'
ODOM_TOPIC = '/summit_xl/robotnik_base_control/odom'
POSE_TOPIC = '/spot_pose_in_summit'


def find_db_file(bag_dir: str) -> str:
    """Trouve le fichier .db3 dans le dossier du bag."""
    candidates = glob.glob(str(Path(bag_dir) / '*.db3'))
    if not candidates:
        print(f"Aucun .db3 trouve dans {bag_dir}")
        sys.exit(1)
    return candidates[0]


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extrait le yaw (rotation autour de Z) d'un quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def read_topic(db_path: str, topic_name: str, required: bool = True):
    """Lit les messages d'un topic depuis le bag SQLite.

    Si required=False et que le topic est absent, retourne (None, None).
    """
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    cur.execute("SELECT id, name, type FROM topics")
    topic_id = None
    msg_type_str = None
    for tid, name, type_ in cur.fetchall():
        if name == topic_name:
            topic_id = tid
            msg_type_str = type_
            break

    if topic_id is None:
        if not required:
            conn.close()
            return None, None
        print(f"Topic {topic_name} absent du bag.")
        cur.execute("SELECT name FROM topics")
        print("Disponibles :")
        for (n,) in cur.fetchall():
            print(f"  {n}")
        sys.exit(1)

    msg_type = get_message(msg_type_str)

    cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
        (topic_id,))

    times = []
    msgs = []
    for ts, data in cur.fetchall():
        msg = deserialize_message(bytes(data), msg_type)
        times.append(ts * 1e-9)
        msgs.append(msg)

    conn.close()
    return np.array(times), msgs


def main():
    if len(sys.argv) < 2:
        print("Usage : python3 bag_to_csv.py <dossier_du_bag>")
        sys.exit(1)

    bag_dir = sys.argv[1]
    db_path = find_db_file(bag_dir)
    print(f"Lecture du bag SQLite : {db_path}")

    # cmd_vel : requis
    t_cmd, cmd_msgs = read_topic(db_path, CMD_TOPIC, required=True)
    cmd_lin = np.array([m.linear.x for m in cmd_msgs])
    cmd_ang = np.array([m.angular.z for m in cmd_msgs])
    print(f"  cmd_vel : {len(cmd_msgs)} messages "
          f"({t_cmd[0]:.2f}s a {t_cmd[-1]:.2f}s)")
    print(f"    cmd_lin range : [{cmd_lin.min():.2f}, {cmd_lin.max():.2f}] m/s")
    print(f"    cmd_ang range : [{cmd_ang.min():.2f}, {cmd_ang.max():.2f}] rad/s")

    # odom : requis
    t_odom, odom_msgs = read_topic(db_path, ODOM_TOPIC, required=True)
    odom_lin = np.array([m.twist.twist.linear.x for m in odom_msgs])
    odom_ang = np.array([m.twist.twist.angular.z for m in odom_msgs])
    print(f"  odom    : {len(odom_msgs)} messages "
          f"({t_odom[0]:.2f}s a {t_odom[-1]:.2f}s)")
    print(f"    odom_lin range : [{odom_lin.min():.2f}, {odom_lin.max():.2f}] m/s")
    print(f"    odom_ang range : [{odom_ang.min():.2f}, {odom_ang.max():.2f}] rad/s")

    # spot_pose_in_summit : optionnel
    t_pose, pose_msgs = read_topic(db_path, POSE_TOPIC, required=False)
    has_pose = t_pose is not None and len(t_pose) > 0
    if has_pose:
        spot_x = np.array([m.pose.position.x for m in pose_msgs])
        spot_y = np.array([m.pose.position.y for m in pose_msgs])
        spot_yaw = np.array([
            quaternion_to_yaw(
                m.pose.orientation.x, m.pose.orientation.y,
                m.pose.orientation.z, m.pose.orientation.w)
            for m in pose_msgs
        ])
        print(f"  spot_pose: {len(pose_msgs)} messages "
              f"({t_pose[0]:.2f}s a {t_pose[-1]:.2f}s)")
        print(f"    spot_x range   : [{spot_x.min():.2f}, {spot_x.max():.2f}] m")
        print(f"    spot_y range   : [{spot_y.min():.2f}, {spot_y.max():.2f}] m")
        print(f"    spot_yaw range : [{spot_yaw.min():.2f}, {spot_yaw.max():.2f}] rad")
    else:
        print(f"  spot_pose: absent du bag (les colonnes seront vides)")

    # Resampling sur les timestamps cmd_vel
    odom_lin_rs = np.interp(t_cmd, t_odom, odom_lin)
    odom_ang_rs = np.interp(t_cmd, t_odom, odom_ang)

    if has_pose:
        spot_x_rs = np.interp(t_cmd, t_pose, spot_x)
        spot_y_rs = np.interp(t_cmd, t_pose, spot_y)
        # Le yaw : unwrap avant interp pour eviter sauts de 2pi, re-wrap apres
        spot_yaw_unwrapped = np.unwrap(spot_yaw)
        spot_yaw_rs = np.interp(t_cmd, t_pose, spot_yaw_unwrapped)
        spot_yaw_rs = np.arctan2(np.sin(spot_yaw_rs), np.cos(spot_yaw_rs))

    # Decale le temps pour partir de 0
    t_rel = t_cmd - t_cmd[0]

    # Ecriture CSV
    csv_path = Path(bag_dir) / 'ident.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'cmd_lin', 'cmd_ang', 'odom_lin', 'odom_ang',
            'spot_x', 'spot_y', 'spot_yaw',
        ])
        for i in range(len(t_rel)):
            row = [
                f"{t_rel[i]:.4f}",
                f"{cmd_lin[i]:.4f}",
                f"{cmd_ang[i]:.4f}",
                f"{odom_lin_rs[i]:.4f}",
                f"{odom_ang_rs[i]:.4f}",
            ]
            if has_pose:
                row.extend([
                    f"{spot_x_rs[i]:.4f}",
                    f"{spot_y_rs[i]:.4f}",
                    f"{spot_yaw_rs[i]:.4f}",
                ])
            else:
                row.extend(["", "", ""])
            writer.writerow(row)

    print(f"\nEcrit : {csv_path}")
    print(f"  {len(t_rel)} lignes, periode moyenne "
          f"{(t_rel[-1] / len(t_rel)) * 1000:.1f} ms")
    if has_pose:
        print(f"  Colonnes : time, cmd_lin, cmd_ang, odom_lin, odom_ang, "
              f"spot_x, spot_y, spot_yaw")
    else:
        print(f"  Colonnes : time, cmd_lin, cmd_ang, odom_lin, odom_ang, "
              f"(spot_* vides car topic absent)")


if __name__ == '__main__':
    main()
