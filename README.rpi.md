# Summit RPi4 - Branche `rpi4`

Deploiement autonome du pipeline Summit-Spot tracking sur Raspberry Pi 4
embarquee sur le robot.

## Architecture

```
PC (developpement)         ssh        RPi4 (192.168.0.50)       ethernet
    -------                          ----------------------       ------>
                                     Docker container :
                                     - vision (Foxy)              Summit XL
                                     - bridge ROS2 <-> ROS1       (192.168.0.200)
                                     - pid_controller (Foxy)      ROS 1 Kinetic
```

La RPi4 est totalement autonome une fois lancee : pas besoin du PC pour la mission.

## Materiel RPi4

- Raspberry Pi 4 Model B Rev 1.5 (BCM2835, Cortex-A72)
- 8 Go RAM, carte SD 16 Go
- Ubuntu Server 20.04.6 LTS ARM64
- Docker 28.x
- Ethernet vers le Summit (eth0 = 192.168.0.50)
- D435i sur USB 3.0
- Hostname : `summit-pi`

## Acces SSH (depuis le PC)

```bash
# Double-saut par le Summit (le seul moyen aujourd'hui)
ssh summit@192.168.0.200       # Summit
ssh summit@192.168.0.50        # RPi4

# Ou avec ~/.ssh/config configure (recommande) :
ssh rpi4
```

Exemple `~/.ssh/config` :
```
Host summit
    HostName 192.168.0.200
    User summit

Host rpi4
    HostName 192.168.0.50
    User summit
    ProxyJump summit
```

## Premier deploiement

### 1. Activer du swap (pour la compilation initiale)

```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
free -h     # verifier
```

### 2. Cloner la branche rpi4

```bash
cd ~
git clone -b rpi4 https://github.com/Ronanlg44/summit-foxy-bridge.git
cd summit-foxy-bridge
```

### 3. Preparer la config

```bash
cp env.rpi.example .env
nano .env                     # adapter si besoin
```

### 4. Copier le modele YOLO (depuis le PC)

Depuis ton PC :
```bash
scp ~/Pro/Stage_CNRS/.../best.onnx rpi4:~/summit-foxy-bridge/data/yolo/
```

### 5. Build l'image (~45-60 min sur RPi4)

```bash
docker build -f Dockerfile.rpi -t summit-rpi:latest .
```

Ou build cross-platform sur le PC pour aller plus vite :
```bash
# Sur le PC (necessite docker buildx + qemu)
docker buildx build --platform linux/arm64 \
    -f Dockerfile.rpi \
    -t summit-rpi:latest \
    --load .

# Exporter
docker save summit-rpi:latest | gzip > summit-rpi.tar.gz

# Copier sur la RPi4
scp summit-rpi.tar.gz rpi4:~/

# Sur la RPi4
gunzip -c ~/summit-rpi.tar.gz | docker load
```

## Modes de fonctionnement

### Mode mission (production)

Lance vision + bridge + PID. Le robot suit le Spot automatiquement.

```bash
./launch_all.rpi.sh mission
```

### Mode identification

Pour mesurer la latence vision et re-tuner les PID.

```bash
./launch_all.rpi.sh ident
```

Puis dans le shell de debug :
```bash
cd /workspace/identification
ros2 bag record -o ident_$(date +%H%M%S) \
    /summit_xl/robotnik_base_control/cmd_vel \
    /summit_xl/robotnik_base_control/odom \
    /spot_target_pose
```

### Mode debug

Inspection seule (pas de commandes envoyees au Summit).

```bash
docker compose -f docker-compose.rpi.yml --profile debug up
```

## Arret

```bash
./launch_all.rpi.sh stop
```

## TODO restant (a remplir une fois testes)

- [ ] Verifier que `ros-foxy-realsense2-camera` ARM64 fonctionne
- [ ] Sinon, compiler `librealsense2` depuis sources dans le Dockerfile
- [ ] Adapter le `command:` de `vision_stack` avec le bon launch file
- [ ] Tester la connectivite RPi4 -> Summit (ROS_MASTER_URI)
- [ ] Mesurer Td_vision_apriltag et Td_vision_yolo sur RPi4 (Methode A)
- [ ] Re-tuner les PID dans MATLAB avec les vraies latences mesurees
- [ ] Mettre a jour les gains dans le code Python
- [ ] Tester sur le robot en mode mission

## Depannage

### Build OOM killed
Verifier le swap (`free -h`), augmenter a 6-8 Go si besoin.

### RealSense non detectee dans le container
```bash
# Verifier sur l'hote
lsusb | grep Intel
# Doit afficher : Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i

# Dans le container, ajouter privileged: true et volume /dev:/dev (deja fait)
```

### Multicast DDS ne marche pas
Verifier que l'interface est bien `eth0` dans `cyclonedds.xml` et `.env`.

### Le bridge n'atteint pas le Summit
```bash
# Tester la connectivite
ping 192.168.0.200

# Tester ROS 1
docker exec -it bridge bash -c "source /opt/ros/noetic/setup.bash && rostopic list"
```
