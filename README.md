# Pont ROS 1 Kinetic ↔ ROS 2 Foxy pour la téléopération du Summit XL avec vision embarquée

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
7. [Outils graphiques (RViz, rqt)](#outils-graphiques-rviz-rqt)
8. [Vision (RealSense + AprilTag + YOLO + supervisor)](#vision-realsense--apriltag--yolo--supervisor)
9. [Limitations connues](#limitations-connues)
10. [Dépannage](#dépannage)
11. [Choix d'architecture (résumé du diagnostic)](#choix-darchitecture-résumé-du-diagnostic)
12. [Évolutions prévues](#évolutions-prévues)

---

## Démarrage rapide

```bash
# 1. PC sur WiFi du robot (SSID : SXL00181120AA)
ping -c 2 192.168.0.200           # doit repondre

# 2. Lancer le pont (terminal 1)
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm bridge

# 3. Ouvrir un shell ROS 2 (terminal 2)
docker compose run --rm shell
ros2 topic list | grep summit_xl
ros2 topic echo /summit_xl/robotnik_base_hw/battery
```

Pour lancer la chaîne vision complète d'un coup, utiliser le script
orchestré décrit dans [`vision/README.md`](vision/README.md#démarrage) :

```bash
./launch_all.sh           # lance tout dans tmux dans le bon ordre
./launch_all.sh attach    # voir les logs
./launch_all.sh stop      # arrêter
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
                                |  (shell, teleop,      |
                                |   rviz, rqt,          |
                                |   realsense, apriltag,|
                                |   refiner, pose_fuser,|
                                |   yolo_detector,      |
                                |   supervisor)         |
                                +-----------------------+
                                          PC (192.168.0.219)
```

Trois éléments clés :
- Le robot reste **intouché** : on ne fait que lire son `roscore`.
- Le bridge utilise **`parameter_bridge`** (sélectif via YAML), pas
  `dynamic_bridge` (qui s'abonne à tous les topics ROS 2).
- Tous les processus ROS 2 utilisent **CycloneDDS** et **`ROS_DOMAIN_ID=0`**.

---

## Prérequis

### Matériel et réseau

- Summit XL HL allumé, accessible en `192.168.0.200`
- PC sur le **WiFi du robot** (SSID `SXL00181120AA`, mot de passe `R0b0tn1K`)
- **Pas eduroam** (multicast DDS bloqué), **VPN déconnecté**

### Logiciel

- Docker 25+ et Docker Compose v2
- Aucun ROS 2 natif requis sur l'hôte

### Vérifications session

```bash
ip addr show wlo1 | grep "inet "    # doit montrer 192.168.0.219/24
ping -c 2 192.168.0.200              # doit répondre

# Pour les outils graphiques (RViz, rqt) — une fois par session
xhost +local:docker
```

---

## Topics disponibles

Topics ROS 1 pontés vers ROS 2 (liste non exhaustive — voir bridge YAML pour
les actifs) :

| Topic ROS 2                                  | Type                    | Sens       |
|----------------------------------------------|-------------------------|------------|
| `/summit_xl/front_laser/scan`                | `sensor_msgs/LaserScan` | robot → PC |
| `/summit_xl/imu/data`                        | `sensor_msgs/Imu`       | robot → PC |
| `/summit_xl/robotnik_base_control/odom`      | `nav_msgs/Odometry`     | robot → PC |
| `/summit_xl/robotnik_base_hw/battery`        | `std_msgs/Float32`      | robot → PC |
| `/tf`, `/tf_static`                          | `tf2_msgs/TFMessage`    | robot → PC |
| `/summit_xl/robotnik_base_control/cmd_vel`   | `geometry_msgs/Twist`   | PC → robot |

Topics ROS 2 publiés par la couche vision : voir [`vision/README.md`](vision/README.md#topics-principaux).

---

## Bridge sélectif (parameter_bridge)

Migration de `dynamic_bridge` (qui pontait tous les topics ROS 2 vers ROS 1
automatiquement) vers `parameter_bridge` (sélectif par YAML).

Configuration dans `bridge/topics.yaml` :

```yaml
topics:
  - { topic: /summit_xl/front_laser/scan, type: sensor_msgs/msg/LaserScan, queue_size: 10 }
  - { topic: /tf,        type: tf2_msgs/msg/TFMessage,         queue_size: 100 }
  - { topic: /tf_static, type: tf2_msgs/msg/TFMessage,         queue_size: 100 }
  - { topic: /summit_xl/robotnik_base_control/cmd_vel, type: geometry_msgs/msg/Twist, queue_size: 10 }

services_1_to_2: []
services_2_to_1: []
```

Les paramètres sont chargés par `entrypoint.sh` via `rosparam load` à la
racine du master ROS 1, puis lus par `parameter_bridge` via XmlRpc.

Pour ajouter un topic : éditer le YAML et relancer le service `bridge`.

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

Touches : `i` avant, `,` arrière, `j`/`l` rotation, `k` stop, `q`/`z` vitesse
max ±10%. **Taper plusieurs `z` au démarrage** pour réduire la vitesse à ~0.1
m/s avant tout essai.

L'à-coup au démarrage vient de l'absence de rampe d'accélération dans
`teleop_twist_keyboard`. Voir [Évolutions](#évolutions-prévues).

---

## Outils graphiques (RViz, rqt)

Prérequis : `xhost +local:docker` une fois par session de bureau.

```bash
docker compose run --rm rviz      # visualisation 3D
docker compose run --rm rqt       # introspection, plot, console
```

Configs RViz prêtes dans `rviz_configs/` :
- `realsense_d435i.rviz` — cam seule
- `apriltag_d435i.rviz` — cam + TF tags
- `apriltag_ir_d435i.rviz` — mode IR
- `SPOT_TRACKING.rviz` — pipeline complet avec `/spot_target_pose`

**Limitations connues** :
- `RobotModel` non disponible (URDF côté Kinetic, non bridgé)
- `2D Goal Pose` publie sur `/goal_pose` mais aucun planificateur ne l'écoute

---

## Vision (RealSense + AprilTag + YOLO + supervisor)

Pipeline complet documenté dans [`vision/README.md`](vision/README.md) :

- **Driver D435i** (RGB + depth + IR) sur USB 3.0 direct au PC
- **AprilTag** (RGB ou IR) avec raffinement pose via depth (médiane 5×5)
- **Calibration extrinsèque** D435i sur Summit + 3 tags sur Spot
- **Pose Spot** via chaînage TF et fusion multi-tags
- **YOLO** (YOLOv8m ONNX + ByteTrack) comme repli quand tags non visibles
- **Supervisor** : machine d'états TAG_OK / YOLO_TRACKING / LOST avec fusion
  YOLO bearing + LiDAR distance

Sortie unifiée : `/spot_target_pose` + `/perception_status`.

---

## Limitations connues

### Bridge actif → cam D435i dégradée

Avec le bridge actif, `/camera/color/image_raw` passe de ~24 Hz à ~2-3 Hz,
alors que la cam est USB locale et ne transite pas par le bridge.

Vérifications effectuées qui n'expliquent pas le comportement :
- CPU non saturé (40 % sur 16 cœurs)
- `network_mode: host` partout
- `net.core.rmem_max` augmenté à 2 GB sans effet
- Pas de fragments UDP perdus (`netstat -s | grep Reasm` vide)
- `receive buffer errors` UDP n'augmentent plus après tuning kernel
- Allègement YAML bridge (11 → 4 topics) sans amélioration durable
- MultiThreadedExecutor du supervisor sans effet

Hypothèse non confirmée : limitation Cyclone DDS Foxy avec
coexistence multi-flux sur WiFi. À revalider en RJ45 + RPi4 embarqué.

### Débit côté Foxy plus bas que côté Kinetic

L'IMU publie à 50 Hz côté Kinetic mais arrive à ~4 Hz côté Foxy via le
bridge sur WiFi. Suffisant pour le pilotage, à garder à l'esprit pour des
applications temps réel.

### Bug d'ordre de démarrage DDS

Si le bridge démarre avant `perception_supervisor`, la subscription LiDAR
du supervisor reste muette. Le script `launch_all.sh` impose l'ordre.

### RViz : RobotModel et Nav2 absents

URDF et planificateurs Nav2 non installés. Squelette TF visible suffit pour
la plupart des besoins de debug.

---

## Dépannage

### `ros2 topic list` ne montre pas les topics `summit_xl`

À vérifier dans l'ordre :
1. Le bridge tourne (`docker ps` doit montrer son conteneur)
2. WiFi du robot actif (`ip addr show wlo1` doit montrer `192.168.0.219`)
3. VPN/Warp désactivé (`warp-cli status` : Disconnected)

### `ros2 topic echo` muet alors que `topic list` voit le topic

RMW ou domaine incohérent. Vérifier :
```bash
docker exec <container> printenv | grep -E "RMW|DOMAIN"
# attendu : RMW_IMPLEMENTATION=rmw_cyclonedds_cpp et ROS_DOMAIN_ID=0
```

### Le robot ne bouge pas malgré teleop

1. Arrêt d'urgence désarmé (tiré) ?
2. Bridge voit-il `Passing message from ROS 2 ... Twist to ROS 1` ?
3. Manette PS4 prise de contrôle ? Appuyer sur bouton PS pour libérer.

### RViz/rqt : `cannot connect to X server`

```bash
xhost +local:docker
```
À refaire à chaque session de bureau.

### `ros2-daemon` hôte squatte le port 7400

```bash
pkill -9 -f ros2-daemon
```

### Caméra ou AprilTag

Voir [`vision/README.md`](vision/README.md#dépannage).

---

## Choix d'architecture (résumé du diagnostic)

Trois architectures écartées avant d'arriver à la configuration actuelle :

1. **`ros1_bridge` Kinetic ↔ Jazzy** : 4 versions DDS d'écart, livraison
   impossible (type hash incompatible). → Repli sur Foxy.

2. **Foxy ↔ Foxy sur eduroam** : multicast DDS bloqué par le filtrage
   eduroam (`239.255.0.1:7400`). → Repli sur WiFi du robot.

3. **Foxy ↔ Foxy avec Fast DDS** : livraison KO entre conteneurs en
   `network_mode: host` sur le même hôte. → Repli sur CycloneDDS.

Autres pistes écartées :
- **`ros1_bridge` Kinetic ↔ Humble** : Humble n'a pas de paquets ROS 1
  en apt sur Ubuntu 22.04, conflits Python (3.8 vs 3.10) à la compilation
- **Zenoh** : bug de handshake TCPROS non corrigé côté ROS 1 Kinetic
- **Migration robot vers ROS 2** : plusieurs semaines de travail,
  risque élevé sur un robot de labo partagé

---

## Évolutions prévues

- **Validation RJ45 + RPi4 embarqué** : passage en Ethernet pour éliminer
  les pertes WiFi et valider que les limitations Cyclone DDS observées
  disparaissent
- **Rampe d'accélération** sur les `cmd_vel` (`nav2_velocity_smoother`
  ou nœud custom) pour éliminer les à-coups teleop
- **Affichage RobotModel** : monter `summit_xl_description` et lancer un
  `robot_state_publisher` côté Foxy
- **Nav2 + AMCL** pour la navigation autonome (le bouton 2D Goal Pose
  serait alors fonctionnel)
- **Migration Foxy → Humble** (Foxy EOL)
- **Lien Ethernet PC ↔ Summit** pour des applications temps réel
- **Évolutions vision** : voir [`vision/README.md`](vision/README.md#évolutions-prévues)

---

## Fichiers du projet

```
summit_foxy/
├── Dockerfile             # Image Noetic + Foxy + ros1_bridge + CycloneDDS + vision
├── docker-compose.yml     # Tous les services
├── entrypoint.sh          # Lancement bridge avec attente master ROS 1
├── launch_all.sh          # Orchestration tmux de la chaîne complète
├── env.example            # ROBOT_IP, MY_IP, DOMAIN_ID
├── README.md              # Ce fichier
├── bridge/
│   └── topics.yaml        # Configuration parameter_bridge
├── vision/
│   ├── README.md          # Doc vision détaillée
│   ├── refiner/
│   ├── apriltag/
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
