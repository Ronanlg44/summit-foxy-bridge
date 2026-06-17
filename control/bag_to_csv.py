"""
Extrait cmd_vel et odom d'un rosbag2 SQLite en CSV pour MATLAB.

Utilise sqlite3 + rclpy serialization (pas rosbag2_py qui n'est pas
toujours installe en Foxy).

Usage :
    python3 bag_to_csv.py <chemin_du_dossier_bag>

Produit : <dossier_bag>/ident.csv avec colonnes :
    time, cmd_lin, cmd_ang, odom_lin, odom_ang

- cmd_lin : commande cmd_vel.linear.x (m/s)
- cmd_ang : commande cmd_vel.angular.z (rad/s)
- odom_lin : vitesse mesuree odom.twist.linear.x (m/s)
- odom_ang : vitesse mesuree odom.twist.angular.z (rad/s)

Le temps est en secondes, partant de 0 au debut du bag.
Les valeurs odom sont resamplees sur les timestamps de cmd_vel
(interpolation lineaire).

Pour identifier en MATLAB :
- G_lin(s) : input = cmd_lin, output = odom_lin
- G_ang(s) : input = cmd_ang, output = odom_ang
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


def find_db_file(bag_dir: str) -> str:
    """Trouve le fichier .db3 dans le dossier du bag."""
    candidates = glob.glob(str(Path(bag_dir) / '*.db3'))
    if not candidates:
        print(f"Aucun .db3 trouve dans {bag_dir}")
        sys.exit(1)
    return candidates[0]


def read_topic(db_path: str, topic_name: str):
    """Lit les messages d'un topic depuis le bag SQLite."""
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

    # cmd_vel : on extrait linear.x ET angular.z
    t_cmd, cmd_msgs = read_topic(db_path, CMD_TOPIC)
    cmd_lin = np.array([m.linear.x for m in cmd_msgs])
    cmd_ang = np.array([m.angular.z for m in cmd_msgs])
    print(f"  cmd_vel : {len(cmd_msgs)} messages "
          f"({t_cmd[0]:.2f}s a {t_cmd[-1]:.2f}s)")
    print(f"    cmd_lin range : [{cmd_lin.min():.2f}, {cmd_lin.max():.2f}] m/s")
    print(f"    cmd_ang range : [{cmd_ang.min():.2f}, {cmd_ang.max():.2f}] rad/s")

    # odom : on extrait twist.linear.x ET twist.angular.z
    t_odom, odom_msgs = read_topic(db_path, ODOM_TOPIC)
    odom_lin = np.array([m.twist.twist.linear.x for m in odom_msgs])
    odom_ang = np.array([m.twist.twist.angular.z for m in odom_msgs])
    print(f"  odom    : {len(odom_msgs)} messages "
          f"({t_odom[0]:.2f}s a {t_odom[-1]:.2f}s)")
    print(f"    odom_lin range : [{odom_lin.min():.2f}, {odom_lin.max():.2f}] m/s")
    print(f"    odom_ang range : [{odom_ang.min():.2f}, {odom_ang.max():.2f}] rad/s")

    # Resampling : interpolation odom sur les timestamps cmd_vel
    odom_lin_resampled = np.interp(t_cmd, t_odom, odom_lin)
    odom_ang_resampled = np.interp(t_cmd, t_odom, odom_ang)

    # Decale le temps pour partir de 0
    t_rel = t_cmd - t_cmd[0]

    # Ecriture CSV avec les 4 colonnes
    csv_path = Path(bag_dir) / 'ident.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'cmd_lin', 'cmd_ang', 'odom_lin', 'odom_ang'])
        for i in range(len(t_rel)):
            writer.writerow([
                f"{t_rel[i]:.4f}",
                f"{cmd_lin[i]:.4f}",
                f"{cmd_ang[i]:.4f}",
                f"{odom_lin_resampled[i]:.4f}",
                f"{odom_ang_resampled[i]:.4f}",
            ])

    print(f"\nEcrit : {csv_path}")
    print(f"  {len(t_rel)} lignes, periode moyenne "
          f"{(t_rel[-1] / len(t_rel)) * 1000:.1f} ms")


if __name__ == '__main__':
    main()
