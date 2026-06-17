# **Conception et intégration** d’une solution de **téléopération inter-version** pour le **Robotnik Summit XL** (**ROS 1 Kinetic** ↔ **ROS 2 Foxy**) avec module de **vision embarquée** assistée par **IA**.

Pont Docker qui relie le robot **Summit XL** (Robotnik, ROS Kinetic figé) à
des nœuds **ROS 2 Foxy** sur le PC. Permet de lire les capteurs du robot
depuis ROS 2 et de lui envoyer des commandes de vitesse. Une couche
**vision** (RealSense D435i + AprilTag + YOLO + supervisor) tourne en
parallèle pour le tracking visuel du Spot et la formulation des commandes
de vitesse.

Rédigé par Ronan Le Guenne : ronan.le-guenne@polytech-lille.net

---

## Sommaire

1. [Démarrage rapide](#démarrage-rapide)
2. [Architecture](#architecture)
3. [Prérequis](#prérequis)
4. [Topics disponibles](#topics-disponibles)
5. [Bridge sélectif (parameter_bridge)](#bridge-sélectif-parameter_bridge)
6. [Pilotage au clavier (teleop)](#pilotage-au-clavier-teleop)
7. [Vision (RealSense + AprilTag + YOLO + supervisor)](#vision-realsense--apriltag--yolo--supervisor)
8. [Choix d'architecture (résumé du diagnostic)](#choix-darchitecture-résumé-du-diagnostic)
9. [Dépannage](#dépannage)
10. [Évolutions prévues](#évolutions-prévues)
11. [Fichiers du projet](#fichiers-du-projet)

---

## Démarrage rapide

```bash
# 1. PC sur WiFi du robot (SSID : SXL00181120AA, mdp : R0b0tn1K)
ping -c 2 192.168.0.200           # doit repondre

# 2. Pipeline complet (bridge + vision + supervisor)
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
./launch_all.sh           # lance tout dans tmux dans le bon ordre
./launch_all.sh attach    # voir les logs (Ctrl+B puis N/P pour naviguer)
./launch_all.sh stop      # arrêter
```

Pour ne lancer que le pont (sans vision) :

```bash
docker compose run --rm bridge
# Puis dans un autre terminal :
docker compose run --rm shell
ros2 topic list | grep summit_xl
```

---

## Architecture

```
+-----------------+         +-------------------------------+
|   Summit XL HL  |  TCPROS |  Conteneur bridge             |
|   Kinetic       |<------->|  Noetic + Foxy                |
|   192.168.0.200 |         |  parameter_bridge sélectif    |
+-----------------+         +---------------+---------------+
                                            | DDS via wlo1
                                            v
                                +-----------+-----------+
                                |  Conteneurs Foxy      |
                                |  realsense, apriltag, |
                                |  refiner, pose_fuser, |
                                |  yolo, supervisor,    |
                                |  shell, teleop, rviz  |
                                +-----------------------+
                                          PC (192.168.0.219)
```

Trois éléments clés :
- Le robot reste **intouché** : on ne fait que lire son `roscore`.
- Le bridge utilise **`parameter_bridge`** (sélectif via YAML), pas
  `dynamic_bridge` (qui s'abonne à tous les topics ROS 2).
- Tous les processus ROS 2 utilisent **CycloneDDS** et **`ROS_DOMAIN_ID=0`**,
  avec une config commune via `cyclonedds.xml` (buffer réseau 64 MB).

---

## Prérequis

### Matériel et réseau

- Summit XL HL allumé, accessible en `192.168.0.200`
- PC sur le **point d'accès WiFi du robot** (SSID `SXL00181120AA`, mot de passe `R0b0tn1K`)

### Logiciel

- Docker 25+ et Docker Compose v2
- Aucun ROS 2 natif requis sur l'hôte

### Tuning kernel UDP (à faire à chaque reboot du PC, ou persister)

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=2147483647
```

### Vérifications session

```bash
ip addr show wlo1 | grep "inet "    # doit montrer 192.168.0.219/24
ping -c 2 192.168.0.200              # doit répondre

# Pour les outils graphiques (RViz, rqt) — une fois par session
xhost +local:docker
```

---

## Topics disponibles

Topics ROS 1 pontés vers ROS 2 :

| Topic ROS 2                                  | Type                    | Sens       |
|----------------------------------------------|-------------------------|------------|
| `/summit_xl/front_laser/scan`                | `sensor_msgs/LaserScan` | robot → PC |
| `/tf`, `/tf_static`                          | `tf2_msgs/TFMessage`    | robot → PC |
| `/summit_xl/robotnik_base_control/cmd_vel`   | `geometry_msgs/Twist`   | PC → robot |

Topics ROS 2 publiés par la couche vision : voir [`vision/README.md`](vision/README.md#topics-principaux).

---

## Bridge sélectif (parameter_bridge)

Migration de `dynamic_bridge` (qui pontait tous les topics ROS 2 vers ROS 1)
vers `parameter_bridge` (sélectif par YAML).

Configuration dans `bridge/topics.yaml` :

```yaml
topics:
  - { topic: /summit_xl/front_laser/scan, type: sensor_msgs/msg/LaserScan, queue_size: 10 }
  - { topic: /tf,        type: tf2_msgs/msg/TFMessage, queue_size: 100 }
  - { topic: /tf_static, type: tf2_msgs/msg/TFMessage, queue_size: 100 }
  - { topic: /summit_xl/robotnik_base_control/cmd_vel, type: geometry_msgs/msg/Twist, queue_size: 10 }

services_1_to_2: []
services_2_to_1: []
```

Les paramètres sont chargés par `entrypoint.sh` via `rosparam load` à la
racine du master ROS 1, puis lus par `parameter_bridge` via XmlRpc.

Pour ajouter un topic : éditer le YAML et relancer le service `bridge`.

**Note** : le bridge publie `/tf_static` en DURABILITY_VOLATILE, incompatible
avec les subscribers tf2 standard qui attendent TRANSIENT_LOCAL. Les TF
critiques du robot (notamment `summit_xl_front_laser_link`) sont donc
publiées localement par le service `tf_static` du projet vision. Voir
[`vision/README.md`](vision/README.md#goulot-détranglement-dds-et-contournements).

---

## Pilotage au clavier (teleop)

### Sécurité

- Robot **surélevé** (roues dans le vide) pour les premiers tests
- Bouton d'arrêt d'urgence à portée
- Désarmer l'arrêt d'urgence (tirer) seulement quand prêt

### Lancement

```bash
docker compose run --rm teleop
```

Touches : `i` avant, `,` arrière, `j`/`l` rotation, `k` stop, `q`/`z`
vitesse max ±10%. **Taper plusieurs `z` au démarrage** pour réduire la
vitesse à ~0.1 m/s avant tout essai.

L'à-coup au démarrage vient de l'absence de rampe d'accélération dans
`teleop_twist_keyboard`. Voir [Évolutions](#évolutions-prévues).

---

## Vision (RealSense + AprilTag + YOLO + supervisor)

Pipeline complet documenté dans [`vision/README.md`](vision/README.md) :

- **Driver D435i** (RGB + depth + IR) sur USB 3.0 direct au PC
- **AprilTag** (RGB ou IR) avec raffinement pose via depth (médiane 5×5)
- **Calibration extrinsèque** D435i et LiDAR sur Summit + 3 tags sur Spot
- **Pose Spot** via chaînage TF et fusion multi-tags
- **YOLO** (YOLOv8m ONNX + ByteTrack) comme repli quand tags non visibles,
  activé à la demande par le supervisor pour économiser le CPU
- **Supervisor** : machine d'états TAG_OK / YOLO_TRACKING / LOST avec
  fusion YOLO bearing + LiDAR distance

Particularités du pipeline final :
- Subscribers cam en mode **JPEG** (`/camera/color/image_raw/compressed`)
  pour contourner le goulot multi-subscribers Cyclone DDS Foxy
- YOLO **lazy + décimé** à 5 Hz quand actif, désactivé quand TAG_OK
- Plusieurs **patches QoS** appliqués (apriltag BEST_EFFORT, cyclonedds.xml,
  buffer kernel UDP). Détails et chronologie dans [`vision/README.md`](vision/README.md#goulot-détranglement-dds-et-contournements).

Sortie unifiée : `/spot_target_pose` + `/perception_status`.

---

## Choix d'architecture (résumé du diagnostic)

### Autres pistes écartées

- **`ros1_bridge` Kinetic ↔ Humble** : Humble n'a pas de paquets ROS 1
  en apt sur Ubuntu 22.04, conflits Python (3.8 vs 3.10) à la compilation
  
- **Zenoh** : bug de handshake TCPROS non corrigé côté ROS 1 Kinetic

- **Migration robot vers ROS 2** : plusieurs semaines, risque élevé sur
  robot de labo partagé

## Évolutions prévues

- **PID d'asservissement** linéaire/angulaire sur `/spot_target_pose`,
  avec adaptation selon `/perception_status`
- **Validation RJ45 + RPi4 embarqué** : passage en Ethernet pour éliminer
  les pertes WiFi et stabiliser DDS
- **Rampe d'accélération** sur les `cmd_vel` (`nav2_velocity_smoother`
  ou nœud custom) pour éliminer les à-coups teleop
- **Nav2 + AMCL** pour la navigation autonome
- **Évolutions vision** : voir [`vision/README.md`](vision/README.md#évolutions-prévues)

---

## Fichiers du projet

```
summit_foxy/
├── Dockerfile             # Image Noetic + Foxy + ros1_bridge + CycloneDDS + vision
├── docker-compose.yml     # Tous les services (CYCLONEDDS_URI partagé)
├── entrypoint.sh          # Lancement bridge avec attente master ROS 1
├── launch_all.sh          # Orchestration tmux de la chaîne complète
├── cyclonedds.xml         # Config DDS partagée (buffer 64 MB)
├── env.example            # ROBOT_IP, MY_IP, DOMAIN_ID
├── README.md              # Ce fichier
├── bridge/
│   └── topics.yaml        # Configuration parameter_bridge
├── vision/
│   ├── README.md          # Doc vision détaillée
│   ├── refiner/           # apriltag_refiner + pose_fuser
│   ├── apriltag/          # config tags YAML
│   ├── calibration/       # tf_static_launch.py
│   ├── supervisor/        # perception_supervisor
│   └── yolo/              # yolo_detector + modèle ONNX
└── rviz_configs/
    ├── realsense_d435i.rviz
    ├── apriltag_d435i.rviz
    ├── apriltag_ir_d435i.rviz
    └── SPOT_TRACKING.rviz
```

---

## Crédits

Diagnostic et mise au point : Ronan Le Guenne, mai-juin 2026.

Pont basé sur `ros1_bridge` (OSRF), CycloneDDS (Eclipse),
`teleop_twist_keyboard` (ROS community), `realsense2_camera` (Intel),
`apriltag` + `apriltag_ros` (AprilRobotics + Adlink-ROS),
YOLOv8 (Ultralytics) + ByteTrack via `supervision` (Roboflow).
