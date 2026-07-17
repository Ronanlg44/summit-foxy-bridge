# Summit Control UI

Interface web pour piloter le pipeline FILDARIANE (Summit XL + Spot + AprilTag)
depuis un PC utilisateur, via SSH vers la RPi4 embarquee.
Aucune installation cote RPi4. Tout tourne sur le PC.

## Prerequis

- **PC** : Linux ou macOS avec `python3 >= 3.9`
- **Reseau** : etre connecte au Wifi du Summit (`SXL00181120AA` ou equivalent)
- **RPi4** : accessible en SSH (`summit@192.168.0.50` par defaut)

## Installation

1. Copier ce dossier `summit_control_ui/` sur ton PC (n'importe ou).
2. Editer `config.yaml` si l'IP de la RPi4 est differente.
3. Rendre le script executable :
```bash
   chmod +x start.sh
```

## Lancement

```bash
./start.sh
```

Le script :
1. Cree un environnement virtuel Python (`.venv/`) si absent.
2. Installe les dependances (`flask`, `paramiko`, `pyyaml`).
3. Demande le mot de passe SSH `summit@192.168.0.50`.
4. Lance Flask sur `http://127.0.0.1:8080`.

Ouvre ensuite ton navigateur sur cette adresse.

## Utilisation

### Section Mission

- **START MISSION** : lance `./launch_all.rpi.sh mission` sur la RPi4 (pipeline complet + PID).
- **IDENTIFICATION** : lance `./launch_all.rpi.sh ident` (pipeline complet + step_input).
- **STOP TOUT** : arrete la session tmux et les containers via `docker compose down`.
- **NETTOYER DOCKER** : kill force + rm de tous les containers du projet
  (necessaire car `docker compose run --rm` cree des containers one-off que
  `down` ne cible pas). A utiliser quand STOP TOUT laisse des containers actifs.
- **Status** : affiche l'etat des fenetres tmux et des containers.

### Section Services individuels

Chaque bouton **Start** lance un service via `docker compose run --rm <service>`.
Si la session tmux `summit_rpi` existe deja, le service est ajoute dans une nouvelle
fenetre tmux. Sinon, il est lance en arriere-plan (nohup).

Le bouton **Stop** arrete le container correspondant.

### Section Controleur PI

- **Activer / Desactiver PI** : publie sur `/ip_enable` (std_msgs/Bool).
- **DEBUG / REEL** : bascule le parametre `publish_real_cmd` du node PID.
  - DEBUG : les commandes sont publiees sur `/cmd_vel_debug` (robot immobile).
  - REEL : les commandes vont sur le vrai topic Summit (robot bouge).

### Section Etat systeme

Rafraichi toutes les 5 secondes :
- Temperature CPU RPi4
- Load average 1 min
- RAM utilisee
- Espace disque
- Etat session tmux `summit_rpi`
- Liste des containers Docker actifs

## Limites connues

- **Perte de contact quand le Summit s'eloigne** : l'IHM depend d'un lien SSH
  permanent avec la RPi4. Une fois le robot parti en mission autonome, le Wifi
  du Summit sort de portee et l'interface affiche des timeouts. Le pipeline
  continue de tourner sur la RPi4 en autonomie ; l'IHM sert au demarrage et
  a l'arret de la mission, pas au suivi temps reel.
- **Latence Wifi elevee (100-300 ms typique)** : les actions dans l'IHM
  prennent 1-3 secondes a s'executer. Normal.

## Depannage

**Le script demande le mot de passe puis dit "Connexion SSH impossible" :**
- Verifier que ton PC est connecte au Wifi Summit.
- `ping 192.168.0.50` doit repondre.
- L'IP de la RPi4 peut avoir change, editer `config.yaml`.

**SSH marche mais l'IHM timeout sur les commandes :**
- Le Wifi du Summit est instable ou la RPi4 est surchargee. Verifier avec
  `ping -c 5 192.168.0.50` : si les latences depassent 500 ms ou perte de
  paquets, se rapprocher du Summit.
- Si la RPi4 elle-meme ne repond plus (SSH classique bloque aussi) : la
  redemarrer physiquement.

**STOP TOUT laisse des containers actifs :**
- Cliquer sur NETTOYER DOCKER pour forcer le kill des containers `run --rm`.


## Contact

Interface developpee par Ronan Le Guenne (ronan.le-guenne@polytech-lille.net) dans le cadre du stage FILDARIANE (CRIStAL / Polytech Lille), Avril/Juillet 2026.
