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
9. [Limitations connues](#limitations-connues)
10. [Dépannage](#dépannage)
11. [Évolutions prévues](#évolutions-prévues)

---

## Vue d'ensemble

```
D435i (USB 3) ─► realsense ─► apriltag ─► refiner ─┐
                          │                        ├─► pose_fuser ─► /spot_pose_in_summit ──┐
                          │                        │   (+ tf_static)                        │
                          │                        │                                        ▼
                          └────────► yolo_detector ─────────────────────────────► perception_supervisor ─► /spot_target_pose
                                                                                       ▲                   /perception_status
Summit XL (ROS 1) ─► bridge ──────► /summit_xl/front_laser/scan ──────────────────────┘
                          ──────► /tf, /tf_static, /cmd_vel
```

Tous les services tournent en ROS 2 Foxy, dans des conteneurs Docker séparés.
Le bridge Kinetic↔Foxy assure la passerelle bidirectionnelle vers le Summit.

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

L'ordre `supervisor avant bridge` est nécessaire à cause d'un bug de
discovery DDS (voir [Limitations connues](#limitations-connues)).

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

---

## Topics principaux

| Topic                            | Type                              | Producteur          |
|----------------------------------|-----------------------------------|---------------------|
| `/camera/color/image_raw`        | `sensor_msgs/Image`               | realsense           |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`     | realsense           |
| `/apriltag_detections`           | `apriltag_msgs/...`               | apriltag            |
| `/apriltag_detections_refined`   | `apriltag_msgs/...`               | refiner             |
| `/yolo_detections`               | `vision_msgs/Detection2DArray`    | yolo_detector       |
| `/spot_pose_in_summit`           | `geometry_msgs/PoseStamped`       | pose_fuser          |
| `/summit_xl/front_laser/scan`    | `sensor_msgs/LaserScan`           | bridge (depuis Summit) |
| `/spot_target_pose`              | `geometry_msgs/PoseStamped`       | perception_supervisor |
| `/perception_status`             | `std_msgs/String`                 | perception_supervisor |

---

## Pipeline AprilTag

### apriltag (détection)

Détecteur Adlink-ROS (`tag_realsense.launch.py`). Configuration multi-IDs
dans `vision/apriltag/tags_36h11_filter.yaml` (monté en volume) :

```yaml
tag_ids:    [0, 1, 2]
tag_frames: [dock_frame_0, dock_frame_1, dock_frame_2]
tag_sizes:  [0.08, 0.08, 0.08]
```

### refiner (raffinement par depth)

Lit la profondeur alignée au centre du tag (médiane sur patch 5×5), recalcule
la position 3D via pinhole inverse, conserve l'orientation PnP. Publie sur
`/apriltag_detections_refined`.

Précision : ±3 mm à 30 cm, ±30 mm à 3 m (vs ±15 cm en PnP brut à 3 m).

### tf_static (calibration extrinsèque)

Publie les TF statiques de calibration :
- `summit_xl_base_link → camera_link` (D435i sur Summit)
- `spot_base_link → tag_X_link` pour X ∈ {0, 1, 2}

| Paramètre D435i sur Summit | Valeur     |
|----------------------------|-----------:|
| x                          | 0.206 m    |
| y                          | 0.000 m    |
| z                          | 0.136 m    |
| pitch                      | 22.84°     |

| Tag | Position (m) sur Spot | Yaw   | Position |
|-----|-----------------------|-------|----------|
| 0   | (-0.43, 0, 0.01)      | π     | arrière  |
| 1   | (0, 0.12, 0.01)       | π/2   | gauche   |
| 2   | (0, -0.12, 0.01)      | -π/2  | droite   |

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
Inférence ~10-15 Hz sur CPU.

Publie `/yolo_detections` (Detection2DArray) avec `id="spot_<tracker_id>"`.

Paramètres en dur dans le code : `conf=0.7, iou=0.1, max_det=1`.

### perception_supervisor

Machine à états à 3 états avec hystérésis :

| État          | Condition                       | Publication           |
|---------------|---------------------------------|------------------------|
| TAG_OK        | pose tag fraîche (<1s)          | pose tag forwardée     |
| YOLO_TRACKING | pas de tag, YOLO+LiDAR cohérent | pose fusionnée         |
| LOST          | aucune source fraîche (>3s)     | aucune                 |

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
  - { topic: /tf,        type: tf2_msgs/msg/TFMessage,         queue_size: 100 }
  - { topic: /tf_static, type: tf2_msgs/msg/TFMessage,         queue_size: 100 }
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

## Limitations connues

### Bridge actif → cam D435i dégradée

**Observation** : avec le bridge actif, `/camera/color/image_raw` passe de
~24 Hz à ~2-3 Hz, alors que la cam est USB locale et ne transite pas par
le bridge.

**Vérifications effectuées qui n'expliquent pas le comportement** :
- CPU non saturé (40 % sur 16 cœurs)
- `network_mode: host` partout
- `net.core.rmem_max` augmenté à 2 GB sans effet sur la cam
- Pas de fragments UDP perdus (`netstat -s | grep Reasm` vide)
- Pas d'augmentation des `receive buffer errors` UDP après tuning kernel
- Allègement du bridge YAML (11 → 4 topics) sans amélioration notable
- Test supervisor Multi-thread sans effet

### Bug d'ordre de démarrage DDS

Si le bridge démarre avant le supervisor, la subscription LiDAR du
supervisor reste muette. `launch_all.sh` impose l'ordre correct.
Origine non confirmée, probablement liée au discovery Cyclone DDS Foxy.

### Conflit QoS sur `/tf_static`

Warnings au démarrage :
```
[WARN] [ros_bridge]: New subscription discovered on this topic, requesting
incompatible QoS. Last incompatible policy: DURABILITY_QOS_POLICY
```

Bridge publie en `TRANSIENT_LOCAL`, certains subscribers attendent du
`VOLATILE`. Pas d'impact fonctionnel observé sur les transformations.

---

## Dépannage

### `topic hz` retourne du vide

Cause : le driver realsense est lazy (ne publie que si quelqu'un est abonné).
Solution : ouvrir RViz avec le topic concerné, ou lancer un autre subscriber.

### `realsense2_camera_node` exit code -11

Cause probable : câble USB 2.1 ou état firmware coincé.
Solution : vérifier `lsusb -t`, brancher la cam directement sur le PC
(pas via hub), redémarrer la cam (débrancher/rebrancher), reboot complet
si besoin (en cas de problème post-veille système).

### `ros2-daemon` hôte squatte le port 7400

Si `ros2 ...` est lancé directement sur l'hôte hors compose, le daemon
peut planter les containers (`general protection fault in libddsc.so`).
```bash
pkill -9 -f ros2-daemon
```

### Détection AprilTag : AttributeError

Le message Adlink est minimaliste (`id`, `size`, `pose` seulement, pas de
`centre`, `corners`, `homography`). Vérifier la structure réelle :
```bash
ros2 interface show apriltag_msgs/msg/AprilTagDetection
```

---

## Évolutions prévues

- **Validation RJ45 + RPi4 embarqué** : passage en Ethernet pour éliminer
  les pertes WiFi et valider que les limitations Cyclone DDS observées
  disparaissent.
- **PID** d'asservissement linéaire/angulaire sur
  `/spot_target_pose`, avec adaptation de vitesse selon
  `/perception_status` (confiance).
- **Migration `apriltag_ros` Adlink → christianrauch** pour accéder à un
  message plus riche (`corners`, `decision_margin`).
- **Migration Foxy → Humble** (Foxy EOL).
- **Filtrage temporel Kalman** sur la pose YOLO_TRACKING pour gérer les
  occlusions ponctuelles.
