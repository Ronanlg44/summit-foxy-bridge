# Pont ROS 1 Kinetic ↔ ROS 2 Foxy pour la téléopération du Summit XL (SXL00-181120AA)

Pont Docker qui relie le robot **Summit XL** (Robotnik, ROS Kinetic figé) à
des nœuds **ROS 2 Foxy** tournant sur le PC. Permet de lire tous les capteurs
du robot depuis ROS 2 et de lui envoyer des commandes de vitesse. Une couche
**vision** (RealSense D435i + AprilTag) tourne en parallèle pour les
futurs paliers de tracking visuel.

Rédigé par Ronan Le Guenne : ronan.le-guenne@polytech-lille.net

---

## Sommaire

1. [Démarrage rapide](#démarrage-rapide)
2. [Architecture](#architecture)
3. [Prérequis](#prérequis)
4. [Procédure complète](#procédure-complète-à-chaque-session)
5. [Topics disponibles](#topics-disponibles)
6. [Pilotage au clavier (teleop)](#pilotage-au-clavier-teleop)
7. [Outils graphiques (RViz, rqt)](#outils-graphiques-rviz-rqt)
8. [vision (RealSense + AprilTag)](#vision-realsense--apriltag)
9. [Dépannage](#dépannage)
10. [Historique du diagnostic](#historique-du-diagnostic)
11. [Évolutions possibles](#évolutions-possibles)

---

## Démarrage rapide

```bash
# 1. PC sur WiFi du robot
ping -c 2 192.168.0.200           # doit repondre

# 2. Lancer le pont (terminal 1)
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose up

# 3. Ouvrir un shell ROS 2 (terminal 2)
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm shell
# Dedans :
ros2 topic list | grep summit_xl
ros2 topic echo /summit_xl/robotnik_base_hw/battery  #vérifier que la réception des infos
```

---

## Architecture

```
+-----------------+         +-------------------------------+
|                 |  TCPROS |  Conteneur summit_foxy        |
|   Summit XL HL  |<------->|  Noetic + Foxy + ros1_bridge  |
|   Kinetic       | unicast |  RMW = CycloneDDS             |
|   192.168.0.200 |         +---------------+---------------+
|                 |                         | DDS via wlo1
+-----------------+                         |
                                            v
                                +-----------+-----------+
                                |  Conteneurs Foxy      |
                                |  (shell, teleop,      |
                                |   rviz, rqt,          |
                                |   realsense, apriltag,|
                                |   noeuds de stage)    |
                                |  RMW = CycloneDDS     |
                                +-----------------------+
                                          PC (192.168.0.219)
```

**Trois éléments clés :**

- Le robot reste **intouché** : il tourne son ROS Kinetic d'origine, on ne fait
  que lire son `roscore`.
- Le pont (`summit_foxy`) cohabite Noetic et Foxy dans la même image Ubuntu
  20.04, tous deux en paquets `apt` (aucune compilation source fragile).
  L'image utilise `ros-foxy-desktop` : RViz, rqt, tf2 complet, ros2 bag…
  sont inclus.
- Tous les processus ROS 2 (pont + shells + nœuds) utilisent **CycloneDDS** et
  le **domaine 0**, sinon ils ne se voient pas.

---

## Prérequis

### Matériel et réseau

- Robot Summit XL HL (SXL00-181120AA) allumé, accessible en `192.168.0.200`.
- PC connecté au **WiFi du robot** (SSID `SXL00181120AA`, mot de passe
  `R0b0tn1K`). **Pas eduroam** — eduroam bloque le multicast DDS.
- **VPN déconnecté** pendant l'utilisation (interfère avec le
  DNS Docker et le routage).

### Logiciel sur le PC

- Docker 25+ et Docker Compose v2 (`docker compose`, pas `docker-compose`).
- Aucun ROS 2 natif n'est nécessaire : tout vit dans les conteneurs.

### Configuration robot (déjà faite, à vérifier en cas de problème)

Le `/etc/hosts` du robot contient une ligne pour le PC :

```bash
ssh summit@192.168.0.200       # mot de passe : R0b0tn1K
tail -3 /etc/hosts             # doit contenir "192.168.0.219 <hostname-pc>"
```

---

## Procédure complète à chaque session

### Étape 1 — Préparation

Sur le PC :

```bash
ip addr show wlo1 | grep "inet "    # doit montrer 192.168.0.219/24
ping -c 2 192.168.0.200              # doit repondre
```

Si une des vérifications échoue : règle-la avant d'aller plus loin

**Si tu comptes utiliser RViz, rqt ou tout autre outil graphique** dans un
conteneur, autorise une fois par session de bureau :

```bash
xhost +local:docker
```

Sans ça, les services `rviz` et `rqt` planteront avec `cannot connect to X
server`. À refaire à chaque redémarrage de session graphique.

### Étape 2 — Lancement du pont (terminal 1)

```bash
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose up
```

**Le pont est prêt** quand tu vois successivement :

```
[pont] Master ROS 1 joignable. Demarrage du pont Kinetic <-> Foxy.
created 1to2 bridge for topic '/summit_xl/...'
... (une cinquantaine de lignes)
[INFO] [ros_bridge]: Passing message from ROS 1 sensor_msgs/Imu to ROS 2 ...
```

Laisse ce terminal ouvert. C'est lui qui fait passer les messages dans les deux
sens.

### Étape 3 — Ouvrir un shell ROS 2 (terminal 2)

```bash
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm shell
```

Tu obtiens un prompt avec Foxy déjà sourcé. CycloneDDS et `ROS_DOMAIN_ID=0`
sont posés par le compose, tu n'as rien à exporter manuellement.

`--rm` : le conteneur s'efface automatiquement quand tu fais `exit`. Pas
d'accumulation de conteneurs zombies.

Tu peux ouvrir plusieurs shells en parallèle : un `compose run --rm shell` par
terminal.

### Étape 4 — Vérifier que tout marche

Dans le shell :

```bash
ros2 topic list                                        # liste des topics
ros2 topic echo /summit_xl/robotnik_base_hw/battery    # ~26 V
```

### Étape 5 — Arrêt propre

```bash
# Dans chaque shell : exit (les conteneurs --rm disparaissent)
# Dans le terminal du pont : Ctrl-C, puis :
docker compose down
```

---

## Topics disponibles

Une fois le pont actif, ces topics ROS 1 sont accessibles côté ROS 2 (liste
non exhaustive) :

| Topic ROS 2                                    | Type                  | Sens         |
|------------------------------------------------|-----------------------|--------------|
| `/summit_xl/imu/data`                          | `sensor_msgs/Imu`     | robot → PC   |
| `/summit_xl/front_laser/scan`                  | `sensor_msgs/LaserScan` | robot → PC |
| `/summit_xl/robotnik_base_control/odom`        | `nav_msgs/Odometry`   | robot → PC   |
| `/summit_xl/joy`                               | `sensor_msgs/Joy`     | robot → PC   |
| `/summit_xl/robotnik_base_hw/battery`          | `std_msgs/Float32`    | robot → PC   |
| `/summit_xl/robotnik_base_hw/emergency_stop`   | `std_msgs/Bool`       | robot → PC   |
| `/tf`, `/tf_static`                            | `tf2_msgs/TFMessage`  | robot → PC   |
| `/summit_xl/robotnik_base_control/cmd_vel`     | `geometry_msgs/Twist` | PC → robot   |

Liste complète : `ros2 topic list` dans un shell.

Les topics produits par la couche **vision** (RealSense + AprilTag)
sont documentés séparément dans [`vision/README.md`](vision/README.md).

**Note sur le débit** : côté Kinetic l'IMU publie à 50 Hz, mais CycloneDDS via
ce WiFi délivre (à première vue) autour de 4 Hz côté Foxy. Suffisant pour le pilotage et la
plupart des usages, à garder à l'esprit pour des applications temps réel.

---

## Pilotage au clavier (teleop)

### Sécurité avant tout

Pour tout premier test logiciel d'une commande motrice :

1. Robot **surélevé**, roues dans le vide (cale sous le châssis batterie).
2. Bouton d'**arrêt d'urgence physique** à portée de main.
3. Désarmer l'arrêt d'urgence (le tirer) seulement quand tu es prêt — sinon le
   robot ne réagira pas, c'est conçu comme ça.

### Lancement

Dans un nouveau terminal :

```bash
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm teleop
```

Le service `teleop` lance `teleop_twist_keyboard` en remappant son topic de
sortie vers `/summit_xl/robotnik_base_control/cmd_vel`.

### Touches

```
   u    i    o      i = avant
   j    k    l      , = arriere
   m    ,    .      j/l = rotation gauche/droite
                    k = STOP IMMEDIAT
q/z = vitesse max +/- 10%
w/x = lineaire seul +/- 10%
e/c = angulaire seul +/- 10%
```

**Au démarrage**, taper plusieurs fois `z` pour réduire la `speed` à ~0.1 m/s
avant tout autre essai. Sinon le robot démarre fort.

### À-coup au démarrage

Comportement normal de `teleop_twist_keyboard` qui publie la consigne d'un
seul coup, sans rampe. Plus la vitesse max est faible (`z`), plus l'à-coup est
faible. Pour une commande douce permanente, voir
[Évolutions possibles](#évolutions-possibles).

### Arrêt

`k` pour stopper, puis `Ctrl-C` pour quitter. Le conteneur s'efface tout seul.

---

## Outils graphiques (RViz, rqt)

Les deux services nécessitent que `xhost +local:docker` ait été lancé une
fois dans la session (voir [Procédure - Étape 1](#étape-1--préparation)).

### RViz : visualisation 3D

```bash
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm rviz
```

Configurations RViz prêtes à l'emploi dans `rviz_configs/` (chargeables via
**File → Open Config**) :

- `realsense_d435i.rviz` : flux caméra D435i (RGB, depth, nuage de points)
- `apriltag_d435i.rviz` : flux caméra + TF des tags détectés

Pour visualiser les capteurs du Summit XL via le pont, configuration manuelle :

- **Fixed Frame** : passer de `map` à `odom` ou `base_footprint`
- **Add → By topic → `/summit_xl/front_laser/scan` → LaserScan**
- **Add → TF** : squelette des frames du robot
- **Add → By topic → `/summit_xl/robotnik_base_control/odom` → Odometry**

**Limitation connue — RobotModel** : l'affichage du mesh 3D du robot
(`Add → RobotModel`) ne fonctionne **pas** dans ce montage. Cause : RViz a
besoin du paramètre `robot_description` (URDF) et des fichiers de mesh, qui
vivent côté Kinetic. `ros1_bridge` fait passer les **topics** et les
**services** mais pas le **Parameter Server** ROS 1. Les TF restent visibles
(squelette des frames), ce qui couvre la grande majorité des besoins de
visualisation.

**Limitation connue — 2D Goal Pose** : le bouton publie bien un
`PoseStamped` sur `/goal_pose`, mais aucun planificateur n'écoute ce topic
dans ce montage. Pour que le robot navigue vers un but, il faut un
**Nav2** (Foxy) ou utiliser le `move_base` Kinetic du robot via ses
propres services. Voir [Évolutions possibles](#évolutions-possibles).

### rqt : boîte à outils GUI

```bash
cd ~/Pro/Stage_CNRS/ROS/summit_foxy
docker compose run --rm rqt
```

`rqt` ouvre une fenêtre où tu peux empiler des plugins via le menu **Plugins**.
Les plus utiles au quotidien :

- **Topics → Topic Monitor** : voir tous les topics et leurs débits en direct.
- **Visualization → Plot** : tracer en temps réel n'importe quel champ d'un
  topic (ex: la vitesse linéaire publiée sur `/cmd_vel`).
- **Introspection → Node Graph** : graphe des nœuds et de leurs connexions.
- **Configuration → Dynamic Reconfigure** : modifier des paramètres à chaud
  (compatible ROS 2 si le nœud le supporte).
- **Logging → Console** : visionner les logs `/rosout` filtrés par niveau.

---

## vision (RealSense + AprilTag)

Le projet inclut une **couche vision** indépendante du pont : driver
Intel RealSense D435i et détecteur AprilTag, tous deux en ROS 2 Foxy. La
caméra est en USB direct sur le PC, donc latence minimale et pleine cadence
(30 Hz RGB + depth).

**Services Docker dédiés** :

```bash
docker compose run --rm realsense    # driver D435i (RGB + depth + IMU)
docker compose run --rm apriltag     # detecteur AprilTag36h11
```

**Documentation complète** : [`vision/README.md`](vision/README.md)
— topics publiés, paramètres, dépannage, choix techniques et évolutions
prévues (raffinement de pose avec depth, détection IR pour vision dans
le noir, calibration extrinsèque, asservissement visuel).

---

## Dépannage

### Le build échoue avec "Could not resolve host"

Cause : pas d'accès Internet (WiFi du robot).

Solution : bascule sur un WiFi avec Internet (eduroam, perso), puis relance `docker compose build`.

### `ros2 topic list` ne montre pas les topics `summit_xl`

Trois causes possibles, à vérifier dans l'ordre :

1. **Le pont ne tourne pas.** Vérifier `docker ps` : doit montrer
   `summit_foxy` en `Up`. Sinon relancer `docker compose up`.
2. **Mauvais WiFi.** `ip addr show wlo1 | grep inet` doit montrer
   `192.168.0.219`, pas `10.21.x.x` (eduroam).
3. **Warp actif.** `warp-cli status` doit dire `Disconnected`.

### `ros2 topic echo` muet alors que `topic list` montre les topics

Cause classique : RMW incohérent entre le pont et le shell, ou domaine
différent. Vérifier :

```bash
docker exec summit_foxy printenv | grep -E "RMW|DOMAIN"
# doit montrer RMW_IMPLEMENTATION=rmw_cyclonedds_cpp et ROS_DOMAIN_ID=0
```

Le service `shell` du compose pose ces variables automatiquement. Si tu lances
un `docker run` manuel, n'oublie pas `-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

### Le robot ne bouge pas malgré les commandes teleop

Vérifier dans l'ordre :

1. L'**arrêt d'urgence** physique est-il bien désarmé (tiré) ?
2. Côté pont, voit-on `Passing message from ROS 2 ... Twist to ROS 1` quand
   tu appuies sur une touche ? Si oui, la commande arrive — c'est côté robot
   que ça bloque.
3. La **manette PS4 Bluetooth** est-elle active ? Elle peut prendre la
   priorité sur le mux (normalement non). Appuyer sur le bouton PS pour la désactiver.

### RViz ou rqt ne s'ouvre pas (`cannot connect to X server`)

Cause : `xhost +local:docker` n'a pas été fait dans la session courante.

Solution : dans un terminal hôte, lance `xhost +local:docker`, puis relance
le service. À refaire à chaque redémarrage de session graphique.

### Souci spécifique à la caméra ou aux AprilTag

Voir la section dépannage dédiée dans
[`vision/README.md`](vision/README.md#dépannage).

### Conteneurs zombies à nettoyer

```bash
docker ps -a                    # liste tous les conteneurs
docker container prune -f       # supprime les conteneurs arretes
```

Les conteneurs lancés via `docker compose run --rm ...` ou `docker run --rm ...`
se nettoient automatiquement, mais d'anciens essais peuvent traîner.

---

## Historique du diagnostic

Pour comprendre **pourquoi** cette architecture précise (Noetic+Foxy en apt,
CycloneDDS, WiFi du robot) — et éviter de retomber dans les pièges.

### Mur 1 : l'écart de versions ROS 2 Foxy ↔ Jazzy

Première tentative : `ros1_bridge` Kinetic ↔ Jazzy (Jazzy natif sur le PC).

Résultat : le côté Kinetic marchait parfaitement (lecture `/summit_xl/joy`
prouvée en direct dans le conteneur), mais Jazzy ne recevait rien. Cause :
quatre versions majeures de ROS 2 d'écart entre Foxy (le seul ROS 2 qui
s'installe en apt à côté de Noetic) et Jazzy. Les protocoles DDS ont été
refondus entre les deux, le `type hash` ne correspond pas, la livraison est
impossible. Pas un réglage : une incompatibilité structurelle.

**Décision** : abandonner Jazzy, faire tourner tout ROS 2 en Foxy (pont et
nœuds de stage). Pas de Jazzy nulle part dans la chaîne.

### Mur 2 : eduroam bloque le multicast DDS

Deuxième tentative : Foxy ↔ Foxy sur eduroam. Découverte OK (`topic list` plein),
livraison KO (`topic hz` muet).

Cause révélée par les logs CycloneDDS : eduroam place le PC sur le sous-réseau
`10.21.x.x` partagé avec des centaines d'autres machines et filtre le multicast
`239.255.0.1:7400` utilisé par DDS pour la découverte des canaux de données.
Conséquences : impossible d'établir le canal unicast de livraison entre les
participants DDS.

**Décision** : se placer sur le **WiFi du robot** (`192.168.0.0/24`, réseau
privé propre où le multicast passe), et accepter qu'on n'a plus accès à
Internet en mode exécution (donc préparer toutes les images à l'avance).

### Mur 3 : Fast DDS ne livre pas entre conteneurs sur le même hôte

Troisième tentative : Foxy ↔ Foxy sur le WiFi du robot, RMW par défaut
(Fast DDS). Toujours « découverte OK, livraison KO ».

Cause : Fast DDS a des difficultés à établir le canal unicast de livraison
entre deux participants tournant dans deux conteneurs distincts en
`network_mode: host`, même sur la même machine.

**Décision** : basculer sur **CycloneDDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)
côté pont et côté shells/nœuds. CycloneDDS gère mieux ces scénarios. Les
données passent dès le premier essai.

### Architectures écartées et pourquoi

- **`ros1_bridge` Kinetic ↔ Humble** : Humble n'a pas de paquets ROS 1 en apt
  sur Ubuntu 22.04. Il faudrait compiler `ros1_bridge` depuis les sources, ce
  qui crée des conflits Python (Noetic = 3.8, Humble = 3.10). Build long,
  fragile, mal documenté.
- **Zenoh (`zenoh-bridge-ros1` + `zenoh-bridge-ros2dds`)** : le côté ROS 2
  fonctionne, mais le côté ROS 1 (basé sur la bibliothèque Rust `rosrust`) a un
  bug de handshake TCPROS connu et non corrigé avec les masters Kinetic
  (erreur `Data field '*' within header mismatched`). Limitation reconnue par
  les mainteneurs du plugin Zenoh-ROS1.
- **Migration du robot vers ROS 2** : plusieurs semaines de travail, haut
  risque pour un robot de labo partagé, stack Robotnik entièrement spécifique
  à Kinetic.

---

## Évolutions possibles

### Lisser les commandes de vitesse

L'à-coup au démarrage du teleop vient de l'absence de rampe d'accélération.
Solution propre : interposer un `nav2_velocity_smoother` ou un nœud custom
qui s'abonne au `cmd_vel` brut et republie une version rampée.

### Afficher le mesh du robot dans RViz

Cloner `summit_xl_common` (qui contient `summit_xl_description` avec l'URDF
et les meshes), le monter dans le conteneur Foxy, et lancer un
`robot_state_publisher` côté Foxy qui charge ce URDF. RViz pourra alors
afficher `RobotModel`.

### Navigation autonome (Nav2)

Le clic "2D Goal Pose" dans RViz produit bien un message `PoseStamped`, mais
aucun planificateur ne l'écoute. Pour de la navigation autonome, installer
Nav2 dans un conteneur Foxy : carte, AMCL pour la localisation, planners
global et local, et publication sur `/summit_xl/robotnik_base_control/cmd_vel`.

### Migrer vers Humble plus tard

Si le projet doit durer, Foxy étant en fin de vie, prévoir une bascule sur
Humble. Stratégie : continuer à utiliser le pont actuel Kinetic ↔ Foxy
comme passerelle, et chaîner un `domain_bridge` ou Zenoh ROS 2 entre Foxy
et Humble.

### Améliorer la stabilité réseau

Le WiFi entre PC et robot est moyen (latence variable, std-dev de 0.5 s sur
les flux haute fréquence). Pour des applications temps réel, prévoir un lien
Ethernet entre le PC et le routeur du robot (port WAN du panneau arrière du
Summit), ou un PC compagnon directement à bord.

### Évolutions de la couche vision

Voir la section dédiée dans [`vision/README.md`](vision/README.md#évolutions-prévues) :
raffinement de pose avec depth, détection IR, calibration extrinsèque,
asservissement visuel PID, et couche sécurité par fusion LiDAR.

---

## Fichiers du projet

```
summit_foxy/
├── Dockerfile            # Image Noetic + Foxy desktop + ros1_bridge + CycloneDDS + vision
├── docker-compose.yml    # Services : bridge, shell, teleop, rviz, rqt, realsense, apriltag
├── entrypoint.sh         # Lancement du pont avec attente TCP du master ROS 1
├── env.example           # Variables : ROBOT_IP, MY_IP, DOMAIN_ID
├── .env                  # Copie locale de env.example (non versionnée)
├── README.md             # Ce fichier
├── vision/
│   └── README.md         # Doc dédiée RealSense + AprilTag
└── rviz_configs/
    ├── realsense_d435i.rviz
    └── apriltag_d435i.rviz
```

---

## Crédits

Diagnostic et mise au point : Ronan Le Guenne, mai 2026.

Pont basé sur `ros1_bridge` (Open Source Robotics Foundation), CycloneDDS
(Eclipse), `teleop_twist_keyboard` (ROS community), `realsense2_camera`
(Intel), et `apriltag` + `apriltag_ros` (AprilRobotics + Adlink-ROS).
