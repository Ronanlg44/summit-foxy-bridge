# Déploiement embarqué (branche `rpi4`)

Ce README couvre uniquement la partie embarquée : ce qui tourne sur la
RPi4 attachée au Summit XL. La vue d'ensemble du projet est dans le
`README.md` de la branche `main`.

## Contenu de la branche

Sync du contenu réellement déployé sur la RPi4 (`~/summit-foxy-bridge/`)
plus le dossier `summit_control_ui/` pour l'IHM lancée depuis le PC.

## Prérequis

- RPi4 8GB avec Ubuntu 20.04 + Docker installé.
- RealSense D435i branchée en USB3.
- RPi4 connectée au Wifi du Summit (SSID `SXL00181120AA`, sans
  internet en fonctionnement).
- Summit sous ROS 1 Noetic (natif), IP `192.168.0.200`.
- PC de l'opérateur sur le même Wifi que la RPi4.

## Réseau

- Summit XL : `192.168.0.200`
- RPi4 (`summit-pi`) : `192.168.0.50`
- PC (opérateur) : DHCP sur le Wifi Summit

Fichier `env.rpi.example` : copier en `env.rpi` et adapter si besoin.

## Première installation

1. Cloner la branche `rpi4` sur la RPi4 :
   ```bash
   git clone -b rpi4 <url_repo> ~/summit-foxy-bridge
   ```

2. Construire l'image Docker embarquée (nécessite internet, prévoir
   ~90 min sur RPi4) :
   ```bash
   cd ~/summit-foxy-bridge
   docker build -f Dockerfile.rpi -t summit-rpi:latest .
   ```

3. Copier les configs :
   ```bash
   cp env.rpi.example env.rpi
   ```

4. Vérifier que `pid_params.yaml` existe (sinon les gains PID
   utiliseront les valeurs par défaut du code) :
   ```bash
   ls control/pid_apriltag/pid_params.yaml
   ```

## Lancement d'une mission

Depuis le PC via l'IHM (voir `summit_control_ui/README.md`), ou
directement sur la RPi4 :

```bash
cd ~/summit-foxy-bridge
./launch_all.rpi.sh mission
```

Le script démarre une session tmux avec :
- `realsense` — flux caméra + depth
- `tf_static` — TF caméra + LiDAR
- `bridge` — pont ROS1 ↔ ROS2 (dynamic_bridge)
- `apriltag` — détection AprilTag 36h11
- `refiner` — raffinement depth
- `pose_fuser` — fusion multi-tags ancrée sur Tag 0
- `pid_apriltag` — PID + WebSocket serveur

Arrêt propre :

```bash
./launch_all.rpi.sh stop
```

## Sécurités

- **Anti-collision LiDAR** intégrée au node PID : freinage progressif
  entre 0.8 m et 0.4 m devant, arrêt en dessous.
- **`publish_real_cmd`** par défaut à `false` dans `pid_params.yaml` :
  au démarrage, aucune commande n'est envoyée au vrai robot. Il faut
  activer explicitement le mode réel via l'IHM.
- **`pose_timeout`** de 0.5 s : freinage automatique si les
  détections AprilTag se perdent.

## Réglage à chaud

Les paramètres PID sont modifiables en direct via l'IHM (bouton
"SAUVEGARDER" écrit dans `pid_params.yaml`, chargé au prochain
démarrage du conteneur).

## Débogage rapide

État des conteneurs :
```bash
docker ps
```

Logs d'un conteneur :
```bash
docker logs -f $(docker ps --filter "name=pid_apriltag" -q)
```

Vérifier les topics ROS 2 :
```bash
docker compose -f docker-compose.rpi.yml run --rm shell
ros2 topic list
```

Vérifier la fréquence de la pose fusionnée :
```bash
ros2 topic hz /spot_pose_in_summit
```

## Notes

- La RPi4 n'a pas d'internet en Wifi Summit : toute modif de l'image
  Docker nécessite de repasser en Ethernet partagé depuis le PC.
- Charge CPU en régime normal : environ 5/8. Température CPU à
  surveiller au-delà de 70 °C.
