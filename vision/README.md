# Perception : RealSense D435i + AprilTag

Cette partie du projet gère la **vision embarquée** : driver de la caméra
Intel RealSense D435i et détection AprilTag. Tout est conçu pour tourner en
parallèle du pont Kinetic↔Foxy, sans interférer avec lui — la caméra est en
USB direct sur le PC, et seules les commandes finales (futures `cmd_vel`)
traverseront le pont vers le Summit XL.

Rédigé par Ronan Le Guenne : ronan.le-guenne@polytech-lille.net

---

## Sommaire

1. [Vue d'ensemble](#vue-densemble)
2. [Topics publiés](#topics-publiés)
3. [Démarrage](#démarrage)
4. [Visualisation RViz](#visualisation-rviz)
5. [Test de détection AprilTag](#test-de-détection-april-tag)
6. [Compatibilité matérielle](#compatibilité-matérielle)
7. [Architecture interne](#architecture-interne)
8. [Dépannage](#dépannage)
9. [Choix techniques](#choix-techniques)
10. [Évolutions prévues](#évolutions-prévues)

---

## Vue d'ensemble

Deux services Docker indépendants, tous deux en ROS 2 Foxy :

```
                D435i (USB 3)
                    │
            ┌───────▼────────┐
            │  realsense     │  publie /camera/color/* (RGB)
            │  (driver)      │         /camera/aligned_depth_to_color/* (depth)
            │                │         /camera/imu (gyro + accel)
            └───────┬────────┘
                    │ DDS (CycloneDDS)
                    │
            ┌───────▼────────┐
            │  apriltag      │  souscrit à /camera/color/*
            │  (détecteur)   │  publie /apriltag_detections
            │                │         /tf (pose 6D des tags)
            └────────────────┘
```

Les deux services partagent le même `ROS_DOMAIN_ID=0` et `CycloneDDS`, hérités
du compose. Aucune configuration manuelle de l'utilisateur.

### Pourquoi cette séparation ?

- **Le driver realsense est lourd à démarrer** (~5 s avec `initial_reset`),
  on ne veut pas le redémarrer chaque fois qu'on touche apriltag.
- **apriltag est lazy** : il ne consomme que si la caméra publie. Pas de
  course de démarrage à gérer.
- **Découpler permet de remplacer apriltag** facilement (variante IR, autre
  détecteur, etc.) sans toucher au driver caméra.

---

## Topics publiés

### Côté RealSense (lazy — publient si on souscrit)

| Topic                                          | Type                          | Fréq.  | Usage                                |
|------------------------------------------------|-------------------------------|--------|--------------------------------------|
| `/camera/color/image_raw`                      | `sensor_msgs/Image`           | 30 Hz  | Flux RGB                             |
| `/camera/color/camera_info`                    | `sensor_msgs/CameraInfo`      | 30 Hz  | Intrinsèques RGB                     |
| `/camera/aligned_depth_to_color/image_raw`     | `sensor_msgs/Image`           | 30 Hz  | Depth aligné sur RGB                 |
| `/camera/aligned_depth_to_color/camera_info`   | `sensor_msgs/CameraInfo`      | 30 Hz  | Intrinsèques depth aligné            |
| `/camera/depth/image_rect_raw`                 | `sensor_msgs/Image`           | 30 Hz  | Depth brute (non alignée)            |
| `/camera/depth/color/points`                   | `sensor_msgs/PointCloud2`     | ~6 Hz  | Nuage de points 3D coloré            |
| `/camera/imu`                                  | `sensor_msgs/Imu`             | varie  | IMU 6 axes (gyro 200 Hz, accel 63 Hz)|
| `/camera/extrinsics/depth_to_color`            | `realsense2_camera_msgs/Extrinsics` | 1 fois | Calibration usine                |

**Note sur le lazy publisher** : `realsense2_camera` ne convertit et ne publie
les frames que si au moins un abonné existe. Conséquence : `ros2 topic hz`
seul peut retourner du vide tant qu'aucun visualiseur n'est connecté. C'est
une feature pour économiser CPU, pas un bug. Ouvrir RViz avec le topic
concerné suffit à le réveiller.

### Côté AprilTag

| Topic                              | Type                                      | Fréq.   | Usage                       |
|------------------------------------|-------------------------------------------|---------|-----------------------------|
| `/apriltag_detections`             | `apriltag_msgs/AprilTagDetectionArray`    | 30 Hz   | Liste des tags + pose 6D    |
| `/tf`                              | `tf2_msgs/TFMessage`                      | 30 Hz   | Transform de chaque tag     |

Le détecteur publie à 30 Hz même si aucun tag n'est visible (avec
`detections: []`). C'est utile pour vérifier que le pipeline tourne.

---

## Démarrage

### Procédure standard

**Pré-requis** : avoir cloné le repo et fait `docker compose build` au moins
une fois. La D435i branchée en USB 3 (voir [Compatibilité matérielle](#compatibilité-matérielle)).

**Terminal 1 — driver RealSense :**

```bash
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm realsense
```

Attendre la séquence :

```
[INFO] Device USB type: 3.2
[INFO] Resetting device...                          ← initial_reset, ~5 s
[INFO] Device with serial number ... was found.
[INFO] RealSense Node Is Up!
```

Les warnings `hwmon command 0x80 ... HW not ready` et `Depth stream start
failure` sont **cosmétiques** sur le firmware actuel — voir [Dépannage](#dépannage).

**Terminal 2 — détecteur AprilTag :**

```bash
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm apriltag
```

Attendre :

```
[INFO] Loaded node '/apriltag' in container '/apriltag/tag_container'
```

À ce stade, `/apriltag_detections` publie en continu.

**Terminal 3 — vérification :**

```bash
docker compose run --rm shell
ros2 topic hz /apriltag_detections     # attendu : ~30 Hz
```

## Visualisation RViz

Deux configurations prêtes dans `rviz_configs/` :

- **`realsense_d435i.rviz`** : RGB + depth + nuage de points. Sert à valider
  la caméra seule.
- **`apriltag_d435i.rviz`** : tout ce qui précède + TF des tags. Sert à
  valider la détection.

### Lancement

```bash
# Autorisation X11 (une fois par session de bureau)
xhost +local:docker

# Lancer RViz
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm rviz
```

Dans RViz : **File → Open Config → /rviz_configs/apriltag_d435i.rviz**

### Comprendre la vue 3D

Le Fixed Frame est `camera_color_optical_frame`, qui suit la **convention
optique ROS** (X droite, Y bas, Z avant). Dans RViz par défaut, l'axe Z
apparaît vers le haut de la scène : un tag à 30 cm devant la caméra
apparaît donc visuellement "au-dessus" de l'origine. C'est juste une
représentation, les valeurs numériques (`pose.position.z`) sont bien la
distance physique au sens caméra-tag.

---

## Test de détection AprilTag

### Générer un tag

Générateur en ligne : https://chaitanyantr.github.io/apriltag.html

### Vérification

Avec la cam pointée sur le tag à ~30-50 cm :

```bash
ros2 topic echo /apriltag_detections --once
```

Un message valide ressemble à :

```yaml
header:
  stamp: {sec: ..., nanosec: ...}
  frame_id: camera_color_optical_frame
detections:
- id: 0
  size: 0.08                       # taille du tag en mètres
  pose:
    pose:
      pose:
        position: {x: -0.02, y: -0.04, z: 0.27}     # tag à 27 cm
        orientation: {x: 0.46, y: 0.45, z: -0.50, w: 0.57}
```

### Critères de qualité d'une détection

- `id` : doit correspondre à celui généré
- `pose.position.z` : doit correspondre approximativement à la distance
  physique caméra-tag (±10% en RGB, ±2% au palier 3 avec depth)
- Pas de saut entre deux frames successives : la pose doit être stable à
  ~0.1 mm près sur la distance à tag fixe

---

## Compatibilité matérielle

### USB

**USB 3.0/3.2 requis** pour la combinaison RGB + depth (+ éventuellement IMU
ou IR). En USB 2.1, le bus sature et le driver segfaulte (`exit code -11`)
après une centaine de millisecondes.

Vérifier le type USB côté hôte :

```bash
lsusb -t
```

Chercher la D435i (`8086:0b3a`) sur un root hub à `5000M` ou plus. À
`480M` = USB 2.1 = câble ou port à changer.

Vérifier côté driver (dans les logs au démarrage de `docker compose run
--rm realsense`) :

- `Device USB type: 3.2` → OK
- `Device USB type: 2.1` → problème

### Firmware D435i

Firmware testé : `05.17.00.10`. Symptômes mineurs constatés :

- Warnings `hwmon command 0x80 ... HW not ready` au démarrage (cosmétique)
- Warnings `Depth stream start failure` quelques secondes après `Node Is
  Up!` (cosmétique — la depth publie correctement à 30 Hz une fois
  qu'un abonné se connecte)

Pour mettre à jour : `realsense-viewer` sur l'hôte (depuis Ubuntu via
`sudo apt install librealsense2-utils`).

---

## Architecture interne

### Image Docker

L'image `summit-kinetic-foxy-bridge:20.04` contient :

- Ubuntu 20.04
- ROS Noetic (pour le pont Kinetic)
- ROS 2 Foxy desktop
- `librealsense2` + `realsense2_camera` (en apt, depuis `packages.ros.org`)
- `apriltag` (lib C) + `apriltag_ros` (wrapper ROS 2) compilés depuis source
  dans `/opt/apriltag_ws/`

### Workspace AprilTag

Le wrapper `apriltag_ros` n'est pas en apt pour Foxy. On utilise le fork
**Adlink-ROS** branche `foxy-devel` qui fournit un launcher
`tag_realsense.launch.py` clé en main.

Build dans le Dockerfile :

```dockerfile
RUN mkdir -p /opt/apriltag_ws/src \
 && cd /opt/apriltag_ws/src \
 && git clone https://github.com/AprilRobotics/apriltag.git \
 && git clone https://github.com/Adlink-ROS/apriltag_ros.git -b foxy-devel \
 && cd /opt/apriltag_ws \
 && /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" \
 && rm -rf build log
```

**Note importante** : pas de `--symlink-install`. Avec cette option, colcon
crée des symlinks depuis `install/` vers `build/`. Le `rm -rf build` final
casserait alors tous les symlinks → erreurs `local_setup.bash not found`
au moment du `source`, alors que `ls` voit bien les fichiers.

### Sourcing dans les services

L'entrypoint `entrypoint.sh` source automatiquement :

1. `/opt/ros/foxy/setup.bash` (toujours)
2. `/opt/apriltag_ws/install/setup.bash` (si présent)

Le sourcing apriltag est aussi répliqué dans le service `shell` du compose,
indispensable pour que `ros2 topic echo /apriltag_detections` puisse
importer le module Python `apriltag_msgs`.

### Paramètres du driver realsense

Dans `docker-compose.yml`, le service `realsense` lance :

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=false \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30 \
  initial_reset:=true
```

| Paramètre                       | Valeur     | Raison                                          |
|---------------------------------|------------|-------------------------------------------------|
| `align_depth.enable`            | `true`     | Raffinement de pose                             |
| `pointcloud.enable`             | `false`    | On n'en a pas besoin pour le tracking, et désactivé évite des warnings `No stream match for pointcloud chosen texture Process - Color` |
| `depth_module.profile`          | `640x480x30` | Compromis qualité/bande passante                |
| `rgb_camera.profile`            | `640x480x30` | Idem                                            |
| `initial_reset`                 | `true`     | Reset hardware au démarrage, évite les états firmware coincés |

### Paramètres du détecteur apriltag

Le service `apriltag` lance le launcher Adlink :

```bash
ros2 launch apriltag_ros tag_realsense.launch.py \
  camera_name:=/camera/color \
  image_topic:=image_raw
```

Configuration par défaut : famille `tag36h11`, taille `0.08` m. Pour changer,
voir `apriltag_ros/config/tags.yaml` dans `/opt/apriltag_ws/src/apriltag_ros/`

---

## Dépannage

**Prévention** : toujours **un seul Ctrl-C** dans le terminal du driver.
Le second Ctrl-C envoie SIGKILL et est interdit. Le paramètre
`initial_reset:=true` (déjà actif dans le compose) limite le risque.

### `topic hz` retourne du vide alors que le driver tourne

**Cause** : le `realsense2_camera` ne publie que si quelqu'un est abonné
(lazy publisher). `ros2 topic hz` met du temps à démarrer son abonnement,
peut renvoyer du vide.

**Solution** :

- Lancer RViz avec un Display sur le topic concerné, puis le `topic
  hz` se met à fonctionner immédiatement

## Choix techniques

### Pourquoi Adlink-ROS/apriltag_ros et pas christianrauch/apriltag_ros ?

L'équipe officielle AprilRobotics ne fournit qu'un wrapper ROS 1. Pour ROS 2,
deux forks principaux :

- **christianrauch/apriltag_ros** (ROS 2 officiel, mais pas de branche Foxy
  dédiée — il faut patcher le `CMakeLists.txt` à la main).
- **Adlink-ROS/apriltag_ros** branche `foxy-devel` : fork de Rauch adapté
  à Foxy + launcher RealSense clé en main `tag_realsense.launch.py`.
---

## Évolutions prévues

### Raffinement de pose avec la profondeur

La pose 6D estimée par AprilTag via PnP est bonne en orientation mais
bruitée en distance (~5-10 cm à 2-3 m). La D435i fournit une distance
précise à ±1 % via la depth alignée. Plan :

1. Nœud Python qui souscrit à `/apriltag_detections` et
   `/camera/aligned_depth_to_color/image_raw`.
2. Pour chaque détection, lecture de la depth au centre du tag avec
   médiane sur un patch 5×5 pixels.
3. Remplacement de la composante Z de la pose par la valeur depth.
4. Publication d'une pose raffinée sur `/apriltag/pose_refined`.

### Détection en infrarouge (vision dans le noir / lumières aveuglantes)

Le palier actuel utilise le flux RGB. Pour du tracking en faible luminosité
ou en présence de lumières aveuglantes (LEDs intenses, projecteurs visibles),
basculer sur les caméras IR est plus robuste : l'émetteur IR actif de la
D435i éclaire la scène en proche infrarouge (~850 nm), filtré naturellement
par les sources visibles.

Service `apriltag_ir` à créer, qui souscrit à `/camera/infra1/image_rect_raw`
et `/camera/infra1/camera_info`. Penser à désactiver le projecteur de points
alternativement (`depth_module.emitter_enabled:=2`) pour qu'il ne pollue
pas la lecture des tags toutes les deux frames.
