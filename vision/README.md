# Perception : RealSense D435i + AprilTag

Cette partie du projet gère la **vision embarquée** : driver de la caméra
Intel RealSense D435i, détection AprilTag et raffinement de pose avec la
profondeur. Tout est conçu pour tourner en parallèle du pont Kinetic↔Foxy,
sans interférer avec lui — la caméra est en USB direct sur le PC, et seules
les commandes finales (futures `cmd_vel`) traverseront le pont vers le
Summit XL.

Rédigé par Ronan Le Guenne : ronan.le-guenne@polytech-lille.net

---

## Sommaire

1. [Vue d'ensemble](#vue-densemble)
2. [Topics publiés](#topics-publiés)
3. [Démarrage](#démarrage)
4. [Visualisation RViz](#visualisation-rviz)
5. [Test de détection AprilTag](#test-de-détection-april-tag)
6. [Raffinement de pose avec la profondeur](#palier-3--raffinement-de-pose-avec-la-profondeur)
7. [Compatibilité matérielle](#compatibilité-matérielle)
8. [Architecture interne](#architecture-interne)
9. [Calibration extrinsèque et pose du Spot](#calibration-extrinsèque-et-pose-du-spot)
9. [Dépannage](#dépannage)
10. [Choix techniques](#choix-techniques)
11. [Évolutions prévues](#évolutions-prévues)

---

## Vue d'ensemble

Trois services Docker indépendants, tous en ROS 2 Foxy :

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
            │  (détecteur)   │  publie /apriltag_detections (PnP brut)
            │                │         /tf (dock_frame)
            └───────┬────────┘
                    │
            ┌───────▼────────┐
            │  refiner       │  souscrit aux détections + depth alignée
            │  (raffinement) │  publie /apriltag_detections_refined
            │                │         /tf (tag_<id>_refined)
            └────────────────┘
```

Les trois services partagent le même `ROS_DOMAIN_ID=0` et `CycloneDDS`,
hérités du compose. Aucune configuration manuelle de l'utilisateur.

### Pourquoi cette séparation ?

- **Le driver realsense est lourd à démarrer** (~5 s avec `initial_reset`),
  on ne veut pas le redémarrer chaque fois qu'on touche apriltag ou refiner.
- **apriltag est lazy** : il ne consomme que si la caméra publie. Pas de
  course de démarrage à gérer.
- **Découpler permet de remplacer apriltag** facilement (variante IR, autre
  détecteur, etc.) sans toucher au driver caméra.
- **Le refiner est un nœud custom** Python facilement modifiable (volume
  monté côté hôte) — itération rapide pour ajuster la taille du patch
  depth, le filtrage temporel, etc.

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
| `/tf` (dock_frame)                 | `tf2_msgs/TFMessage`                      | 30 Hz   | Transform PnP brute         |

Le détecteur publie à 30 Hz même si aucun tag n'est visible (avec
`detections: []`). C'est utile pour vérifier que le pipeline tourne.

### Côté Refiner (palier 3)

| Topic                              | Type                                      | Fréq.   | Usage                                 |
|------------------------------------|-------------------------------------------|---------|---------------------------------------|
| `/apriltag_detections_refined`     | `apriltag_msgs/AprilTagDetectionArray`    | ~25 Hz  | Détections avec pose raffinée par depth |
| `/tf` (tag_\<id\>_refined)         | `tf2_msgs/TFMessage`                      | ~25 Hz  | Transform raffinée                    |

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

**Terminal 3 — raffinement (palier 3) :**

```bash
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm refiner
```

Attendre :

```
Starting >>> apriltag_refiner
Finished <<< apriltag_refiner [0.5s]
[INFO] Refiner pret. Souscrit a /apriltag_detections, ...
[INFO] Intrinseques recus : fx=607.8, fy=607.7, cx=323.3, cy=240.5
```

`/apriltag_detections_refined` publie maintenant à ~25 Hz dès qu'un tag
est visible.

**Terminal 4 — vérification :**

```bash
docker compose run --rm shell
ros2 topic hz /apriltag_detections           # attendu : ~30 Hz
ros2 topic hz /apriltag_detections_refined   # attendu : ~25 Hz
```

## Visualisation RViz

Deux configurations prêtes dans `rviz_configs/` :

- **`realsense_d435i.rviz`** : RGB + depth + nuage de points. Sert à valider
  la caméra seule.
- **`apriltag_d435i.rviz`** : tout ce qui précède + TF des tags. Sert à
  valider la détection et le raffinement (le Display TF affiche
  automatiquement à la fois `dock_frame` et `tag_<id>_refined` quand le
  refiner tourne).

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

Quand le refiner tourne, deux frames coexistent dans la vue 3D :

- **`dock_frame`** : pose PnP brute publiée par apriltag (nom imposé par la
  config par défaut du fork Adlink, orienté "docking")
- **`tag_<id>_refined`** : pose raffinée par le refiner

Le décalage entre les deux est visible à l'œil, et matérialise la valeur
ajoutée du palier 3.

---

## Test de détection AprilTag

### Générer un tag

Générateur en ligne : https://chaitanyantr.github.io/apriltag.html

### Vérification

Avec la cam pointée sur le tag à ~30-50 cm :

```bash
timeout 1 ros2 topic echo /apriltag_detections
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
  physique caméra-tag (±10% en PnP brut, ±2% au palier 3 avec depth)
- Pas de saut entre deux frames successives : la pose doit être stable à
  ~0.1 mm près sur la distance à tag fixe

---

## Raffinement de pose avec la profondeur

La pose 6D estimée par AprilTag via PnP est bonne en orientation mais
bruitée en distance (~5-10 cm à 2-3 m). La D435i fournit une distance
précise à ±1 % via la depth alignée. Le service `refiner` lit cette
depth au centre du tag détecté et reconstruit une pose 3D plus précise.

### Architecture

```
              /apriltag_detections (PnP brut)
                        │
                                          ▼
                   ┌─────────┐
/camera/depth ───► │ refiner │ ───► /apriltag_detections_refined
/camera/info  ───► │         │ ───► /tf (tag_<id>_refined)
                   └─────────┘
```

Le nœud refiner (`vision/refiner/apriltag_refiner/refiner_node.py`) :

1. Souscrit aux détections AprilTag, à l'image depth alignée et aux
   intrinsèques de la caméra
2. Pour chaque tag, projette sa position 3D PnP dans l'image (pinhole
   forward) pour obtenir le pixel central `(u, v)`
3. Lit la depth au pixel `(u, v)` avec une **médiane sur un patch 5×5**
   pour gommer le bruit et ignorer les pixels invalides
4. Reconstruit la position 3D du tag par **pinhole inverse** à partir de
   la depth lue : `X = (u - cx) · Z / fx` et `Y = (v - cy) · Z / fy`
5. Conserve l'orientation issue de PnP (qui n'est pas bruitée)
6. Publie le résultat sur `/apriltag_detections_refined` + une TF
   `tag_<id>_refined`

### Démarrage

Le refiner se lance après realsense et apriltag :

```bash
# Terminal 3 (après realsense et apriltag) :
docker compose run --rm refiner
```

Le service compile le package Python au démarrage (~5-10 sec via
`colcon build --symlink-install`), puis lance le nœud. Le code Python
est monté en volume depuis `vision/refiner/`, donc les modifications
côté hôte sont prises en compte au redémarrage sans rebuild d'image.

### Validation visuelle

Avec un tag36h11 ID 0 visible à la caméra :

```bash
# Comparaison brut vs raffiné :
timeout 1 ros2 topic echo /apriltag_detections | grep position -A 4
timeout 1 ros2 topic echo /apriltag_detections_refined | grep position -A 4

# Visualisation en RViz :
docker compose run --rm rviz
# File → Open Config → /rviz_configs/apriltag_d435i.rviz
# Les frames dock_frame (PnP brut) et tag_0_refined (raffinée) sont
# visibles simultanément dans la vue 3D, légèrement décalées en Z.
```

### Précision attendue

Plus la distance augmente, plus l'avantage du refiner se creuse :

| Distance | PnP brut (typique) | Refined (typique) |
|----------|--------------------:|-------------------:|
| 0.3 m    | ±3 cm               | ±3 mm              |
| 1.0 m    | ±5 cm               | ±10 mm             |
| 2.0 m    | ±10 cm              | ±20 mm             |
| 3.0 m    | ±15 cm              | ±30 mm             |

### Limitations connues

- **Sensible à la taille du tag déclarée** : si le tag à l'écran ou
  imprimé ne fait pas la taille déclarée dans la config apriltag, PnP
  se trompe sur la distance (proportionnellement au ratio des
  tailles). Le refiner corrige cette erreur, mais il est plus sain de
  toujours utiliser des tags à la taille déclarée.
- **Sensible aux obstacles entre la caméra et le tag** : si un objet
  est plus proche que le tag sur le pixel central projeté, la depth
  lira l'obstacle et pas le tag. À éviter (vue dégagée recommandée).
- **Pas de synchronisation stricte timestamp** : on prend la dernière
  depth reçue, décalage max ±16 ms à 30 Hz. À améliorer avec
  `message_filters.ApproximateTimeSynchronizer` si la latence devient
  problématique.
- **Patch 5×5 limite à grande distance** : à 3 m, le tag fait ~10 pixels
  de côté et un patch 5×5 occupe la moitié du tag. Au-delà, envisager
  un patch adaptatif basé sur la taille du tag détectée.

### Choix techniques refiner

- **Volume monté côté hôte** (et non dans le Dockerfile) pour itérer
  rapidement sur le code Python. Quand le palier sera figé pour
  l'embarqué, on déplacera le code dans le Dockerfile.
- **Médiane 5×5 plutôt que valeur pixel unique** : robuste aux trous
  (pixels invalides à 0 dans la depth) et aux outliers. Filtrage
  préalable des pixels hors plage utile (0.1 m à 6 m).
- **Pas de filtrage temporel pour l'instant** : la pose refined est
  déjà très stable frame à frame grâce au lissage spatial. Un Kalman
  1D pourrait être ajouté plus tard si besoin pour gérer les
  occlusions ponctuelles.
- **Python plutôt que C++** : le coût CPU est négligeable (quelques
  pourcents) et l'itération est beaucoup plus rapide.

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

Le code du refiner n'est **pas** dans l'image — il est monté en volume
depuis `vision/refiner/` au démarrage du service, et compilé à la volée.

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

### Workspace Refiner

Le package `apriltag_refiner` est monté en volume depuis l'hôte :

```yaml
volumes:
  - ./vision/refiner:/opt/refiner_ws/src/apriltag_refiner:rw
```

Au démarrage du service, le compose fait :

```bash
cd /opt/refiner_ws
colcon build --packages-select apriltag_refiner --symlink-install
source /opt/refiner_ws/install/setup.bash
ros2 run apriltag_refiner refiner
```

L'option `--symlink-install` est **OK ici** parce que le workspace n'est pas
détruit après le build (contrairement au Dockerfile où le `rm -rf build`
casserait les symlinks).

### Sourcing dans les services

L'entrypoint `entrypoint.sh` source automatiquement :

1. `/opt/ros/foxy/setup.bash` (toujours)
2. `/opt/apriltag_ws/install/setup.bash` (si présent)

Le sourcing apriltag est aussi répliqué dans le service `shell` du compose,
indispensable pour que `ros2 topic echo /apriltag_detections` puisse
importer le module Python `apriltag_msgs`.

Le service `refiner` ajoute en plus le sourcing de `refiner_ws/install/`
avant de lancer le nœud.

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
| `align_depth.enable`            | `true`     | Indispensable pour le palier 3 (refiner lit la depth alignée sur le RGB) |
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

Configuration par défaut : famille `tag36h11`, taille `0.08` m, frame
`dock_frame`. Pour changer, voir `apriltag_ros/cfg/tags_36h11_filter.yaml`
dans `/opt/apriltag_ws/src/apriltag_ros/`.

---

### Détection en infrarouge

**Statut** : services `realsense_ir` et `apriltag_ir` ajoutés au compose.

Pour du tracking en faible luminosité ou en présence de lumières
aveuglantes (LEDs intenses, projecteurs visibles), les caméras IR de
la D435i sont plus robustes que le RGB : l'émetteur IR actif éclaire
la scène en proche infrarouge (~850 nm), filtré naturellement par les
sources visibles.

**Architecture** : un mode = un service realsense + un service apriltag,
mais le refiner est partagé (consomme `/apriltag_detections` peu importe
le mode source).

| Mode jour            | Mode nuit / aveuglement |
|----------------------|--------------------------|
| `realsense`          | `realsense_ir`          |
| `apriltag`           | `apriltag_ir`           |
| `refiner`            | `refiner`               |
| Config RViz : `apriltag_d435i.rviz` | Config RViz : `apriltag_ir_d435i.rviz` |

**Démarrage en mode IR** :

```bash
# Terminal 1
docker compose run --rm realsense_ir

# Terminal 2
docker compose run --rm apriltag_ir

# Terminal 3 (identique au mode jour)
docker compose run --rm refiner
```

**Note** : ne pas lancer `realsense` et `realsense_ir` simultanément
(une seule connexion USB possible avec la cam).

**Paramètre clé** : `depth_module.emitter_enabled:=2` active le mode
alternance du projecteur IR — une frame sur deux ON (pour la depth),
une frame sur deux OFF (pour la détection IR sans pollution par la
mire). Malgré ce mode, la détection apriltag reste à 30 Hz constant
(le détecteur tolère bien les variations de la mire).

**Compatibilité refiner** : le refiner souscrit à
`/apriltag_detections` qui est le **même topic** dans les deux modes.
Les deux frames optiques `camera_color_optical_frame` et
`camera_infra1_optical_frame` étant physiquement proches (quelques mm),
le décalage est négligeable pour notre échelle de détection.
Pas d'adaptation nécessaire dans le refiner.


## Calibration extrinsèque et pose du Spot

Calculer en temps réel la pose du Spot dans le repère du Summit, par
chaînage TF, à partir d'un ou plusieurs tags détectés.

### Architecture

```
realsense → apriltag → refiner → pose_fuser → /spot_pose_in_summit
                          ↑              ↑
                      tf_static (calibration extrinsèque)
```

- **tf_static** publie 4 transformations statiques :
  - `summit_xl_base_link → camera_link` (D435i sur le Summit, mesure CAO)
  - `spot_base_link → tag_X_link` pour X ∈ {0, 1, 2} (tags sur le Spot)
- **pose_fuser** lit `/apriltag_detections_refined`, compose `T_cam_spot = T_cam_tag · T_tag_spot` via scipy, et publie la pose finale sur `/spot_pose_in_summit` + une TF `summit_xl_base_link → spot_base_link`.

### Calibration D435i sur le Summit

Mesurée dans la CAO OnShape (modèle "RB Summit XL HL") et validée au mètre :

| Paramètre | Valeur     |
|-----------|-----------:|
| x         | 0.206 m    |
| y         | 0.000 m    |
| z         | 0.136 m    |
| pitch     | 22.84° (0.3986 rad) |

La cam est fixée sur la face avant inclinée du Summit, centrée latéralement.

### Calibration des 3 tags sur le Spot

| Tag | Position (x, y, z) m | Rotation (yaw, pitch, roll) | Frame parent  |
|-----|----------------------|------------------------------|---------------|
| 0   | (-0.43, 0, 0.01)     | (π, 0, 0)                   | arrière       |
| 1   | (0, 0.12, 0.01)      | (π/2, 0, 0)                 | flanc gauche  |
| 2   | (0, -0.12, 0.01)     | (-π/2, 0, 0)                | flanc droit   |

Les rotations ont été déterminées expérimentalement après mesure de la convention de repère utilisée par le wrapper Adlink (qui aligne l'axe +X du tag avec sa normale, et non +Z comme la convention native AprilTag).

### Fusion multi-tags

Si plusieurs tags sont visibles, le `pose_fuser` :

- moyenne pondérée des positions par **1/Z²** (le tag le plus proche pèse plus)
- conserve l'orientation du tag le plus proche

Au log :
```
[INFO] Detections: [1, 0, 2] | Fusionne 3 estimations
```

### Démarrage

Après `realsense`, `apriltag` et `refiner`:

```bash
# Terminal 4 — TF statiques de calibration
docker compose run --rm tf_static

# Terminal 5 — fusion + publication pose Spot
docker compose run --rm pose_fuser
```

Vérification rapide :

```bash
ros2 run tf2_ros tf2_echo summit_xl_base_link spot_base_link
```

Avec un tag visible droit face à la cam, la rotation doit être proche de l'identité (`qw ≈ 1`).

### Configuration AprilTag (détection multi-IDs)

Le yaml `vision/apriltag/tags_36h11_filter.yaml` est monté en volume dans le service `apriltag`, ce qui permet de modifier les IDs détectés sans rebuild de l'image. Pour le tracking Spot :

```yaml
tag_ids: [0, 1, 2]
tag_frames: [dock_frame_0, dock_frame_1, dock_frame_2]
tag_sizes: [0.10, 0.09, 0.09]
```

### Pièges et résolutions

- **Conflit de double-parent sur `tag_X_link`** : le refiner publiait `camera_color_optical_frame → tag_X_link`, en conflit avec la TF statique `spot_base_link → tag_X_link`. Résolution : le refiner ne publie plus que sur le topic `/apriltag_detections_refined` ; la composition de transformation est faite par le `pose_fuser`.
- **Composition de quaternions sans librairie** : ajout de `python3-scipy` dans le Dockerfile, utilisation de `scipy.spatial.transform.Rotation`.
- **Convention de repère du tag détecté inattendue** : Adlink applique une convention "ROS-style" (axe +X = normale), différente de la convention AprilTag native (axe +Z = normale). Mesurée expérimentalement dans RViz, intégrée dans le calcul des rotations statiques des tags.



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

### Le refiner crash avec `AttributeError`

Le message `apriltag_msgs/AprilTagDetection` du fork Adlink est **minimal**
(seulement `id`, `size`, `pose`). Pas de champ `centre`, `corners` ni
`homography`. Le refiner contourne cette limitation en projetant la
position 3D PnP dans l'image pour obtenir le pixel central. Si tu vois
un `AttributeError` sur un champ du message, c'est probablement une
tentative d'utiliser un champ inexistant — vérifier la structure réelle :

```bash
ros2 interface show apriltag_msgs/msg/AprilTagDetection
```

### Le ros2-daemon hôte squatte le port 7400

Si tu lances `ros2 ...` directement sur ton hôte (en dehors du compose),
Jazzy démarre un daemon qui occupe le port 7400 et fait planter
CycloneDDS dans les containers.

**Symptôme** : `general protection fault in libddsc.so` dans dmesg, et
crash du node realsense à l'init.

**Solution** : tuer le daemon avant tout `docker compose run` :

```bash
ros2 daemon stop
# Ou directement :
pkill -9 -f ros2-daemon
```

---

## Choix techniques

### Pourquoi Adlink-ROS/apriltag_ros et pas christianrauch/apriltag_ros ?

L'équipe officielle AprilRobotics ne fournit qu'un wrapper ROS 1. Pour ROS 2,
deux forks principaux :

- **christianrauch/apriltag_ros** (ROS 2 officiel, mais pas de branche Foxy
  dédiée — il faut patcher le `CMakeLists.txt` à la main).
- **Adlink-ROS/apriltag_ros** branche `foxy-devel` : fork de Rauch adapté
  à Foxy + launcher RealSense clé en main `tag_realsense.launch.py`.

Adlink a été choisi pour la simplicité de mise en route. Migration vers
Rauch envisagée plus tard pour bénéficier du message complet
(`centre`, `corners`, `homography`, `decision_margin`).

---

## Évolutions possibles


### Migration vers christianrauch/apriltag_ros

Le fork Adlink fournit un message minimaliste (`AprilTagDetection` sans
`centre`, `corners`, etc.). Migrer vers le fork Rauch donnerait accès à
un message plus riche, permettant entre autres :

- Patch depth adaptatif basé sur la taille du tag détectée
- Détection de l'orientation par les corners directement
- Meilleur diagnostic (`decision_margin` accessible)

Le coût : un petit patch du `CMakeLists.txt` pour compatibilité Foxy.

### Filtrage temporel de la pose refinée

La pose refined est déjà très stable spatialement (médiane 5×5), mais
pour gérer les **occlusions ponctuelles** (jambes de Spot qui passent
devant un flanc, tag temporairement masqué), un filtre de Kalman 1D
sur la distance + lissage exponentiel sur l'orientation rendrait le
signal de commande plus continu. À ajouter dans le refiner ou dans un
nœud `tracker` distinct.
