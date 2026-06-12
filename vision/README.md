# Perception : RealSense D435i + AprilTag + YOLO + supervisor

Vision embarquée pour le tracking du Spot par le Summit XL :
- driver D435i, détection AprilTag (RGB ou IR), raffinement de pose via depth
- chaînage TF pour pose Spot dans le repère Summit
- détection YOLO + LiDAR comme repli quand les tags ne sont pas visibles
- supervisor à machine d'états qui fusionne les sources

Rédigé par Ronan Le Guenne : ronan.le-guenne@polytech-lille.net

---

## Sommaire

1. [Vue d'ensemble](#vue-densemble)
2. [Démarrage](#démarrage)
3. [Topics principaux](#topics-principaux)
4. [Pipeline AprilTag](#pipeline-apriltag)
5. [Détection IR (faible lumière)](#détection-ir-faible-lumière)
6. [Pipeline YOLO + supervisor](#pipeline-yolo--supervisor)
7. [Bridge sélectif Kinetic↔Foxy](#bridge-sélectif-kineticfoxy)
8. [Compatibilité matérielle](#compatibilité-matérielle)
9. [Goulot d'étranglement DDS et contournements](#goulot-détranglement-dds-et-contournements)
10. [Limitations résiduelles](#limitations-résiduelles)
11. [Dépannage](#dépannage)
12. [Évolutions prévues](#évolutions-prévues)

---

## Vue d'ensemble

```
D435i (USB 3) ─► realsense ─► apriltag ─► refiner ─┐
                          │                        ├─► pose_fuser ─► /spot_pose_in_summit ──┐
                          │  (JPEG compressed)     │   (+ tf_static)                        │
                          │                        │                                        ▼
                          └────────► yolo_detector ──────────────────────────► perception_supervisor ─► /spot_target_pose
                                       ▲ /yolo_enable                              ▲                    /perception_status
Summit XL (ROS 1) ─► bridge ──────► /summit_xl/front_laser/scan ──────────────────┘
                          ──────► /tf, /tf_static, /cmd_vel
```

Tous les services tournent en ROS 2 Foxy, dans des conteneurs Docker séparés.
Le bridge Kinetic↔Foxy assure la passerelle bidirectionnelle vers le Summit.

Particularités du pipeline final :
- AprilTag et YOLO s'abonnent à `/camera/color/image_raw/compressed` (JPEG)
  au lieu du flux raw, pour contourner le goulot multi-subscribers DDS
- YOLO est activé/désactivé dynamiquement par le supervisor via
  `/yolo_enable` (CPU quasi nul quand le tag est visible)
- La TF `summit_xl_base_link → summit_xl_front_laser_link` est publiée
  localement par `tf_static`

---

## Démarrage

### Démarrage orchestré (recommandé)

```bash
./launch_all.sh           # lance tout dans une session tmux
./launch_all.sh attach    # attache la session (Ctrl+B puis N/P pour naviguer)
./launch_all.sh stop      # arrête tout
./launch_all.sh status    # état des services
```

L'ordre de démarrage est imposé :
realsense → tf_static → perception_supervisor → bridge → apriltag → refiner
→ pose_fuser → yolo_detector.

L'ordre `supervisor avant bridge` est nécessaire pour qu'une subscription
LiDAR du supervisor s'établisse correctement.

### Démarrage manuel (un service par terminal)

```bash
docker compose run --rm realsense
docker compose run --rm tf_static
docker compose run --rm perception_supervisor
docker compose run --rm bridge
docker compose run --rm apriltag
docker compose run --rm refiner
docker compose run --rm pose_fuser
docker compose run --rm yolo_detector
```

### Visualisation RViz

```bash
xhost +local:docker
docker compose run --rm rviz
# File → Open Config → /rviz_configs/SPOT_TRACKING.rviz
```

Note : RViz est gourmand sur le bus DDS et peut faire chuter les
fréquences des autres subscribers cam. À utiliser pour debug ponctuel,
pas en continu pendant le tracking.

---

## Topics principaux

| Topic                                       | Type                              | Producteur          |
|---------------------------------------------|-----------------------------------|---------------------|
| `/camera/color/image_raw`                   | `sensor_msgs/Image`               | realsense           |
| `/camera/color/image_raw/compressed`        | `sensor_msgs/CompressedImage`     | realsense (JPEG)    |
| `/camera/aligned_depth_to_color/image_raw`  | `sensor_msgs/Image`               | realsense           |
| `/apriltag_detections`                      | `apriltag_msgs/...`               | apriltag            |
| `/apriltag_detections_refined`              | `apriltag_msgs/...`               | refiner             |
| `/yolo_detections`                          | `vision_msgs/Detection2DArray`    | yolo_detector       |
| `/yolo_enable`                              | `std_msgs/Bool`                   | perception_supervisor |
| `/spot_pose_in_summit`                      | `geometry_msgs/PoseStamped`       | pose_fuser          |
| `/summit_xl/front_laser/scan`               | `sensor_msgs/LaserScan`           | bridge (depuis Summit) |
| `/spot_target_pose`                         | `geometry_msgs/PoseStamped`       | perception_supervisor |
| `/perception_status`                        | `std_msgs/String`                 | perception_supervisor |

---

## Pipeline AprilTag

### apriltag (détection)

Détecteur Adlink-ROS lancé avec `image_transport:=compressed` pour
s'abonner au flux JPEG. Configuration multi-IDs dans
`vision/apriltag/tags_36h11_filter.yaml` :

```yaml
tag_ids:    [0, 1, 2]
tag_frames: [dock_frame_0, dock_frame_1, dock_frame_2]
tag_sizes:  [0.08, 0.08, 0.08]
```

Patch appliqué au fork Adlink (dans le Dockerfile, `sed` sur
`AprilTagNode.cpp`) : QoS de la subscription image passe de
`rmw_qos_profile_default` (RELIABLE) à `rmw_qos_profile_sensor_data`
(BEST_EFFORT).

### refiner (raffinement par depth)

Lit la profondeur alignée au centre du tag (médiane sur patch 5×5), recalcule
la position 3D via pinhole inverse, conserve l'orientation PnP. Publie sur
`/apriltag_detections_refined`.

### tf_static (calibration extrinsèque)

Publie les TF statiques de calibration :
- `summit_xl_base_link → camera_link` (D435i sur Summit)
- `summit_xl_base_link → summit_xl_front_laser_link` (LiDAR sur Summit)
- `spot_base_link → tag_X_link` pour X ∈ {0, 1, 2}

La TF LiDAR est ajoutée ici (au lieu d'être reçue du `robot_state_publisher`
du Summit via le bridge) pour contourner un bug QoS du bridge sur
`/tf_static` (voir [Goulot d'étranglement DDS](#goulot-détranglement-dds-et-contournements)).

| Composant sur Summit XL | x (m) | y (m) | z (m) | Rotation |
|-------------------------|-------|-------|-------|----------|
| D435i (`camera_link`)   | 0.068 | 0.000 | 0.190 | aucune   |
| LiDAR Hokuyo            | 0.006 | 0.000 | 0.220 | aucune   |

| Tag sur Spot | Position (x, y, z) m | Yaw   | Position |
|--------------|----------------------|-------|----------|
| 0            | (-0.43, 0, 0.01)     | π     | arrière  |
| 1            | (0, 0.12, 0.01)      | π/2   | gauche   |
| 2            | (0, -0.12, 0.01)     | -π/2  | droite   |

Convention Adlink : axe `+X` du tag = sa normale (style ROS, pas la
convention AprilTag native `+Z`).

### pose_fuser (composition + fusion multi-tags)

Compose `T_summit_spot = T_summit_cam · T_cam_tag · T_tag_spot` via scipy.
Si plusieurs tags visibles : moyenne pondérée des positions en `1/Z²`,
orientation du tag le plus proche.

Vérification rapide :
```bash
ros2 run tf2_ros tf2_echo summit_xl_base_link spot_base_link
```

---

## Détection IR (faible lumière)

Services dédiés `realsense_ir` + `apriltag_ir`. Utilise le flux infrarouge
proche (~850 nm) pour les environnements sombres ou avec lumières visibles
aveuglantes.

```bash
docker compose run --rm realsense_ir
docker compose run --rm apriltag_ir
docker compose run --rm refiner       # partagé avec le mode RGB
```

Paramètre clé : `depth_module.emitter_enabled:=2` (alternance émetteur ON/OFF
frame à frame, pour ne pas polluer l'image IR avec la mire).

Ne pas lancer `realsense` et `realsense_ir` simultanément (une seule
ouverture USB possible).

---

## Pipeline YOLO + supervisor

### yolo_detector

Modèle YOLOv8m_SPOT exporté ONNX (opset 19, FP32, input 320×320, 1 classe).
ByteTrack (`supervision`) pour filtrer les faux positifs d'une frame.

Subscriber `/camera/color/image_raw/compressed` (JPEG) avec QoS
BEST_EFFORT. Décompression via `cv2.imdecode`.

**Activation à la demande** : yolo démarre désactivé (`enabled=False`).
Le supervisor publie `/yolo_enable` (Bool) pour activer/désactiver
dynamiquement. Quand désactivé, le callback ignore les frames sans
décompresser ni faire d'inférence (CPU quasi nul).

**Décimation** : quand activé, traite 1 frame sur 6 (cam à 30 Hz →
inférence à 5 Hz). Limite la charge CPU pour préserver les autres
services. Constante `FRAME_SKIP=6` modifiable dans le code (à tester sur RPI4).

Paramètres en dur : `conf=0.7, iou=0.1, max_det=1`.

### perception_supervisor

Machine à états à 3 états avec hystérésis :

| État          | Condition                       | yolo_enable | Publication            |
|---------------|---------------------------------|-------------|------------------------|
| TAG_OK        | pose tag fraîche (<1s)          | False       | pose tag forwardée     |
| YOLO_TRACKING | pas de tag, YOLO+LiDAR cohérent | True        | pose fusionnée         |
| LOST          | aucune source fraîche (>3s)     | True        | aucune                 |

Fusion YOLO+LiDAR :

1. YOLO publie une bbox dans le frame caméra optique
2. Calcul du bearing optique à partir du centre bbox et de `fx`
3. Transformation bearing optique → bearing laser via TF
4. Lecture du LiDAR dans cette direction : médiane sur 9 faisceaux adjacents
5. Validation temporelle : rejet si saut `> MAX_SPOT_SPEED × dt + 0.5 m`
6. Construction de la pose dans le repère `summit_xl_base_link`

Confiance YOLO : score continu [0, 1] qui croît à chaque détection
cohérente (+0.15 × score) et décroît si pas de détection (×0.85).

Sortie : `/spot_target_pose` (PoseStamped @ 20 Hz) + `/perception_status`.

---

## Bridge sélectif Kinetic↔Foxy

Migration de `dynamic_bridge` (qui s'abonnait à tous les topics ROS 2) vers
`parameter_bridge` (sélectif via YAML).

Topics pontés dans `bridge/topics.yaml` :
```yaml
topics:
  - { topic: /summit_xl/front_laser/scan, type: sensor_msgs/msg/LaserScan, queue_size: 10 }
  - { topic: /tf,        type: tf2_msgs/msg/TFMessage, queue_size: 100 }
  - { topic: /tf_static, type: tf2_msgs/msg/TFMessage, queue_size: 100 }
  - { topic: /summit_xl/robotnik_base_control/cmd_vel, type: geometry_msgs/msg/Twist, queue_size: 10 }
```

Les paramètres sont chargés au démarrage par `entrypoint.sh` via
`rosparam load` à la racine du master ROS 1, puis `ros2 run ros1_bridge
parameter_bridge` les lit via XmlRpc.

---

## Compatibilité matérielle

### USB 3.0/3.2 requis pour la D435i

En USB 2.1, le driver segfaulte après ~100 ms. Vérification :
```bash
lsusb -t                  # chercher 8086:0b3a sur un hub à 5000M+
```
Côté driver, vérifier dans les logs : `Device USB type: 3.2`.

### Firmware D435i testé

`05.17.00.10`. Warnings cosmétiques au démarrage (`hwmon command 0x80 ...
HW not ready`, `Depth stream start failure`) sans impact sur le
fonctionnement.

---

## Goulot d'étranglement DDS et contournements

### Le problème

Cyclone DDS Foxy ne supporte pas `iceoryx` (shared memory), disponible
seulement à partir de Galactic. Sans shared memory, chaque subscriber
reçoit sa propre copie complète des messages via UDP loopback en local.

Pour un flux cam 640×480 RGB à 30 Hz, chaque copie pèse ~900 KB par
frame, soit ~27 MB/s par subscriber. Avec 2 subscribers `apriltag` +
`yolo_detector` (+ topic hz/rviz/... en test), le thread DDS local sature, **toutes les fréquences
s'effondrent à 2 Hz**, y compris la cam que voient les nœuds.

### Le diagnostic complet (chronologique)

1. **Faux suspects écartés** : CPU, buffer kernel UDP, fragments
   réseau, `network_mode: host`, allègement YAML bridge, ordre de
   démarrage. Aucun n'expliquait la chute de fréquence.

2. **Vrai diagnostic** : le coupable est le nombre de subscribers DDS
   sur un gros topic. Cela vaut pour `apriltag + yolo_detector`, mais
   aussi pour `apriltag + rviz`, ou pour `apriltag + ros2 topic hz`
   lancé en parallèle. Le 2e subscriber décroche systématiquement.

3. **Mesure trompeuse** : utiliser `ros2 topic hz /camera/color/image_raw`
   pour mesurer la cam pendant qu'un nœud y est abonné fausse la
   mesure (le shell devient un 2e subscriber). La vraie fréquence
   reçue par le 1er subscriber se mesure indirectement, par exemple
   via la fréquence de `/apriltag_detections` (qu'apriltag publie à la
   cadence de réception de la cam).

### Le contournement adopté : compression JPEG

`realsense2_camera` publie en plus du flux raw un flux
`/camera/color/image_raw/compressed` (JPEG). Le plugin
`compressed_image_transport` est déjà installé dans l'image.

- Frame raw : ~900 KB → JPEG ~40 KB (22× moins de débit)
- AprilTag : lancé avec `image_transport:=compressed`
- YOLO : subscriber `CompressedImage` au lieu de `Image`, décompression
  via `cv2.imdecode`
- Impact qualité : négligeable pour la détection AprilTag/YOLO

Résultat : apriltag à 30 Hz et yolo à 5-10 Hz peuvent tourner ensemble.

### Autres contournements appliqués

- **Patch QoS apriltag** : subscription image passée en BEST_EFFORT
  via patch `sed` dans le Dockerfile (`rmw_qos_profile_default` →
  `rmw_qos_profile_sensor_data`). Permet de drop les frames non
  traitées au lieu de bloquer le publisher.
- **YOLO lazy + décimation** : décrit dans la section supervisor.
  Réduit le 2e subscriber à 5 Hz seulement quand nécessaire.

### Bug QoS sur `/tf_static` du bridge

`ros1_bridge` publie `/tf_static` en `DURABILITY_VOLATILE` alors que les
subscribers tf2 standard attendent `TRANSIENT_LOCAL`. Conséquence : les
TF du robot publiées par le `robot_state_publisher` côté Kinetic
n'arrivent jamais aux nœuds ROS 2.

Warning visible :
```
[WARN] New publisher discovered on this topic, requesting
incompatible QoS. Last incompatible policy: DURABILITY_QOS_POLICY
```

**Contournement** : la TF nécessaire au supervisor
(`summit_xl_base_link → summit_xl_front_laser_link`) est publiée
localement par le service `tf_static` du projet, en TRANSIENT_LOCAL.

---

## Limitations résiduelles

### RViz incompatible avec le pipeline en charge

Ouvrir RViz pendant que `apriltag` et `yolo_detector` tournent ajoute
un 3e subscriber sur le bus DDS, ce qui dégrade les fréquences. À
utiliser pour debug ponctuel hors run de tracking.

### CPU saturé sur PC dev quand YOLO actif

YOLOv8m FP32 à 5 Hz consomme ~50% CPU sur 16 cœurs. Tient sur PC dev
mais pas sur RPi4. Optimisations prévues : modèle YOLOv8n nano, baisse de la fréquence (déjà décimée à 5 Hz).

### Saut depth incohérent en mode tracking sans cam montée

Quand la cam n'est pas physiquement sur le robot, le LiDAR du Summit
et la cam ne sont pas alignés, donc les distances LiDAR dans la
direction du Spot ne correspondent pas à ce que voit YOLO. La
validation temporelle rejette ces poses :

```
[WARN] Saut depth incoherent : delta=7.07m > max=1.04m
```

À valider une fois la cam montée sur le robot.

---
### Détection AprilTag : AttributeError

Le message Adlink est minimaliste (`id`, `size`, `pose` seulement, pas de
`centre`, `corners`, `homography`). Vérifier la structure réelle :
```bash
ros2 interface show apriltag_msgs/msg/AprilTagDetection
```

---

## Évolutions prévues

- **Validation cam montée sur le robot** : aligner physiquement cam et
  LiDAR, valider la fusion YOLO+LiDAR en mouvement.
- **PID d'asservissement** linéaire/angulaire sur `/spot_target_pose`,
  avec adaptation de vitesse selon `/perception_status` (confiance).
- **Validation RJ45 + RPi4 embarqué** : passage en Ethernet pour
  éliminer les pertes WiFi.
- **Quantification YOLO INT8** pour pouvoir tenir sur RPi4 (4 cœurs ARM).
- **Filtrage temporel Kalman** sur la pose YOLO_TRACKING pour gérer les
  occlusions ponctuelles.
