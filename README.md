# FILDARIANE — Suivi Spot par Summit XL

Projet de stage CRIStAL/CNRS (Polytech Lille — parcours Systèmes Embarqués), avril–juillet 2026.
Le Summit XL suit le Spot en s'appuyant sur des AprilTags collés sur le corps
et les pattes du Spot. Toute la boucle vision + contrôle tourne à bord d'un
Raspberry Pi 4 dédié, embarqué sur le Summit et raccordé au réseau Wifi
propre du robot.

## Vue d'ensemble

- **Vision** : caméra Intel RealSense D435i, détection AprilTag 36h11,
  raffinement des poses avec l'image de profondeur, fusion multi-tags
  ancrée sur un tag de référence (Tag 0 sur la queue du Spot).
- **Contrôle** : PID à deux boucles (linéaire + angulaire) sur la pose du
  Spot exprimée dans le repère `summit_xl_base_link`. Anti-collision
  basée sur le LiDAR avant (freinage progressif + arrêt).
- **Bridge ROS1 ↔ ROS2** : pipeline en ROS 2 Foxy dans les conteneurs,
  ROS 1 Noetic natif côté Summit, `dynamic_bridge --bridge-all-topics`
  pour tout relier.
- **IHM** : interface web Flask + WebSocket sur le PC, permet de lancer
  la mission, régler le PID en direct, sauvegarder les gains dans un
  fichier YAML persistant sur la RPi4, et visualiser l'état système
  et le suivi.

## Structure des branches

- `main` : code source de développement (référence, avec IHM,
  outils d'identification et de calibration).
- `rpi4` : déploiement embarqué. Sync du contenu tournant réellement
  sur la RPi4, plus le dossier `summit_control_ui/` pour l'IHM lancée
  depuis le PC. Voir `README.rpi.md` pour le déploiement.

## Organisation du dépôt

- `bridge/` — configuration du bridge ROS1 ↔ ROS2 (cyclonedds, topics).
- `control/` — contrôle et identification :
  - `pid_apriltag/` — package ROS 2 du contrôleur PID (avec pure pursuit
    optionnel et anti-collision LiDAR).
  - `step_input/` — modèles Simulink d'identification du Summit.
  - `bag_to_csv.py`, `tuning_imc.m`, `ident_summit.m` — outils de tuning.
- `vision/` — pipeline vision :
  - `apriltag/` — configuration des tags (IDs, tailles, frames).
  - `refiner/` — raffinement profondeur + fusion multi-tags
    (`pose_fuser_node.py` ancre sur Tag 0).
  - `calibration/` — TF statiques (caméra, LiDAR, tags).
- `summit_control_ui/` — IHM Flask + WebSocket (voir README interne).
- `identification/` — scripts et données d'identification du Summit.
- `data/` — logs et bags de sessions passées.
- `Dockerfile.rpi`, `docker-compose.rpi.yml`, `launch_all.rpi.sh`,
  `env.rpi.example` — infrastructure de déploiement embarqué.

## Démarrage rapide

Consulter `README.rpi.md` pour le déploiement sur la RPi4 et
`summit_control_ui/README.md` pour l'IHM.

## Auteur

Ronan Le Guenne (Polytech Lille EIF), stage encadré par Gérald Dherbomez
au laboratoire CRIStAL (UMR CNRS 9189, Villeneuve-d'Ascq).
