# bamboo4WD_V4_WSEsp32 sous linorobot2 (ROS 2 Humble)

Fiche du robot **bamboo4WD_V4_WSEsp32** et procédure reproductible pour mettre en route sa
stack ROS 2 (« stack V4 ») sur le RPi embarqué : synchro du code Windows → RPi, bascule
depuis l'ancienne stack, build de l'image Docker, vérification via le MCP.

> Objectifs du chantier : (1) navigation autonome (SLAM + Nav2), (2) pilotage manette
> Bluetooth, (3) diffusion caméra H.264. Ce document couvre l'**infrastructure** (lots L0–L1
> faits) ; le déroulé fonctionnel lot par lot est dans le plan de projet.

---

## 1. Description matérielle

| Élément | Détail |
|---|---|
| Châssis | 4 roues, entraînement différentiel (2 côtés G/D) |
| Carte de commande | **Waveshare General Driver for Robots** — ESP32-WROOM-32, double pont **TB6612FNG** (4 moteurs), **2 voies d'encodeur** seulement (1 par côté), **INA219** (tension/courant batterie), IMU **QMI8658C** + magnétomètre **AK09918C** |
| Transport ESP32 ↔ RPi | **USB série**, protocole **MAVLink** (dialecte `bamboo`, PAS micro-ROS). Firmware `esp32_bamboo` env MAVLink (`ConnectorMavlink`, sysid=2) |
| Calculateur embarqué | **Raspberry Pi 4** sous **DietPi** (aarch64), IP LAN **192.168.1.164** |
| Télémètre laser | **YDLidar** USB (pont **CP2102**, VID:PID `10c4:ea60`) |
| Caméra | Webcam **H.264** USB |
| Manette | Manette **Bluetooth** (XInput) |

### Ce que publie / souscrit le firmware (établi par lecture du code)

- **Publie** : `BAMBOO_WHEEL_STATE` (42001, vx/vy/wz SI), `ATTITUDE` (#30, roll/pitch/yaw),
  `SYS_STATUS` (#1, tension), `BAMBOO_ENCODERS` (42003, `counts[4]`), `BAMBOO_MAG` (42005),
  `HEARTBEAT`.
- **Souscrit** : `BAMBOO_CMD_VEL` (42004) → cinématique différentielle + PID **exécutés sur la
  carte**.
- **Ne calcule PAS** la pose x, y, θ → à intégrer côté nœud ROS (odométrie encodeurs).
- `WHEEL_CPR/CIRC/APB` et `CAR_TYPE` sont **read-only / echo** (cinématique figée à la
  compilation) ; les valeurs d'amorçage du firmware sont absurdes (`WHEEL_DIAMETER=0.8`,
  `LR_WHEELS_DISTANCE=1.3`) ou sans source (`COUNTS_PER_REV=2114`, qui ne correspond à aucun
  produit WaveShare) → **calibration obligatoire côté ROS**.
- La géométrie de référence vit désormais dans **un seul fichier**,
  `bamboo_base/config/robots/bamboo4WD_V4_WSEsp32.yaml`, aux valeurs constructeur du kit
  *UGV Rover* (`WHEEL_D 0.0800`, `ONE_CIRCLE_PLUSES 1650`, `TRACK_WIDTH 0.172`, relevées dans
  `waveshareteam/ugv_base_general` → `General_Driver/ugv_config.h`, mainType 02). Elles restent
  **à confirmer par la mesure**.
- `MOTOR3_ENCODER == MOTOR1`, `MOTOR4 == MOTOR2` : `counts[4]` ne porte que **2 infos** (G/D).

> ⚠️ **Ambiguïté CP2102** : l'ESP32 (carte Waveshare) **et** le YDLidar sont tous deux des
> ponts CP2102 `10c4:ea60`. La règle udev bakée dans l'image (`Dockerfile`, stage `hardware`)
> crée `/dev/ydlidar` pour **tout** `10c4:ea60` → collision si les deux sont branchés. À
> distinguer par **numéro de série** (`ATTRS{serial}`). Voir §7.

---

## 2. Topologie logicielle (stack V4)

- **Fork** : `github.com/brag00n/linorobot2` et `.../linorobot2_hardware`, branche
  **`humble_develop_bamboov4`** (les deux dépôts).
- **Deux dépôts côte à côte** obligatoires : le `docker-compose.yaml` bind-monte `..`
  (→ `linorobot2`) et `../../linorobot2_hardware`, et le codec hôte importe `firmware/` +
  `tools/` par chemin relatif.
- **Docker** : `linorobot2/docker/` — Dockerfile multi-stage (`base` → `hardware`/`build`,
  + stages indépendants `rosbridge` et `foxglovebridge`), piloté par `docker/.env`.
- **Isolation V2 / V4** : `COMPOSE_PROJECT_NAME=bamboov4_humble` namespace images,
  conteneurs et réseau bridge → **aucune collision** avec l'ancienne stack V2
  (`bamboov2_humble`, sous `/root/workspace/bamboo4WD_V2.humble/...`).
  **On ne fait PAS tourner les deux en même temps** : on stoppe V2 avant de lancer V4
  (V4 reprend les ports 9090 / 8765).
- **Réseau bridge** (`linorobot2.network`), **pas** `network_mode: host` (resté commenté).
  Empiriquement le bridge suffit à la découverte DDS, à Foxglove et à rosbridge sur ce robot.
- **Deux ponts d'observation** (cohabitent) :
  - **`rosbridge`** (rosbridge_suite, WebSocket JSON **:9090**) → consommé par le MCP
    `ros2-analysis` (client WebSocket **côté Windows**, dans `.mcp.json`).
  - **`foxglove_bridge`** (**:8765**) → client Foxglove Studio de l'utilisateur.

```
Windows (poste dev)                         RPi 192.168.1.164 (DietPi, Docker)
  ┌───────────────────┐   ws://…:9090        ┌──────────────────────────────┐
  │ MCP ros2-analysis │◀────────────────────▶│ rosbridge  :9090             │
  │ (tools/.venv)     │                       │ foxglove_bridge :8765        │
  └───────────────────┘   ws://…:8765         │ (+ nœuds ROS de la stack V4) │
  Foxglove Studio ◀───────────────────────────┤ réseau bridge bamboov4_humble│
                                               └──────────────────────────────┘
```

### Fichiers clés

| Fichier | Rôle |
|---|---|
| `linorobot2/docker/.env` | `DEPLOY=hardware`, `COMPOSE_PROJECT_NAME=bamboov4_humble`, `ROBOT_BASE=4wd`, `LASER_SENSOR=ydlidar`, `ROS_DISTRO=humble`, `BASE_GIT_LINOROBOT2=brag00n` |
| `linorobot2/docker/docker-compose.yaml` | services `rosbridge`, `foxglovebridge`, `base` + dérivés (`bringup.real`, `slam.real`, `navigation.real`, …) |
| `linorobot2/docker/sync_to_rpi.sh` | synchro rsync Windows → RPi (§4) |
| `linorobot2_hardware/.mcp.json` | déclare `ros2-analysis` (env `ROS2_BRIDGE_URL=ws://192.168.1.164:9090`) |
| `linorobot2_hardware/tools/robot_control/mcp/ros2.py` | code du serveur MCP `ros2-analysis` (client rosbridge) |

---

## 3. Cycle de vie du code (Windows ↔ RPi)

1. **INIT** (une fois, sur le RPi) : `git clone` des 2 dépôts côte à côte (§4.1).
2. **ITÉRATION** : éditer sous Windows → synchro rsync/scp → tester sur le RPi.
   - Nœud Python (source bind-montée) : **relancer le nœud** suffit, pas de rebuild.
   - YAML / launch : relancer le nœud.
   - deps / Dockerfile : `docker compose build`.
   - Paquet C++ (ydlidar, `bamboo_interfaces`) : `colcon build` dans le conteneur.
3. **FIN DE PHASE** : commit + push depuis Windows, puis sur le RPi remettre le working tree
   exactement sur le remote :
   ```bash
   git fetch origin && git reset --hard origin/humble_develop_bamboov4
   ```
   puis re-test final « propre ».

---

## 4. Procédure reproductible

Prérequis : SSH sans mot de passe vers le RPi via la clé **`~/.ssh/id_rpi_grovepi`** (nom non
standard → à passer explicitement, il n'y a ni agent ni `~/.ssh/config`). `docker` sur le RPi
exige `sudo` (l'utilisateur `dietpi` n'est pas dans le groupe `docker`) ; les commandes sous
`/root/...` (stack V2) exigent aussi `sudo`.

### 4.1 INIT — clone git sur le RPi (une seule fois)

```bash
ssh -i ~/.ssh/id_rpi_grovepi dietpi@192.168.1.164 '
  B=~/prj_robotique/Bamboo4WD_V4; mkdir -p "$B" && cd "$B"
  git clone -b humble_develop_bamboov4 https://github.com/brag00n/linorobot2.git          linorobot2
  git clone -b humble_develop_bamboov4 https://github.com/brag00n/linorobot2_hardware.git linorobot2_hardware
'
```

### 4.2 SYNCHRO Windows → RPi (à chaque itération)

Depuis un shell **bash** disposant de `rsync` + `ssh` :

```bash
bash linorobot2/docker/sync_to_rpi.sh
```

Le script pousse les 2 dépôts (exclut `.git/`, `tools/.venv/`, `data/faces/`, artefacts de
build), sans `--delete` (protège les cartes côté RPi), et utilise la clé
`~/.ssh/id_rpi_grovepi`.

> ⚠️ **`rsync` peut manquer** : il est absent de Git Bash, et WSL peut être cassé sur le poste.
> **Repli** pour de petits deltas — pousser les fichiers modifiés au `scp` :
> ```bash
> scp -i ~/.ssh/id_rpi_grovepi linorobot2/docker/.env linorobot2/docker/docker-compose.yaml \
>     dietpi@192.168.1.164:/home/dietpi/prj_robotique/Bamboo4WD_V4/linorobot2/docker/
> ```

### 4.3 BASCULE V2 → V4 (stopper l'ancienne stack)

```bash
ssh -i ~/.ssh/id_rpi_grovepi dietpi@192.168.1.164 'sudo -n bash -lc "
  cd /root/workspace/bamboo4WD_V2.humble/docker/script/linorobot2/docker
  docker compose stop           # libère 9090 / 8765 ; tient tant que le démon tourne
"'
```

> Les services V2 ont `restart: always` → un `stop` manuel est honoré, mais un **reboot** les
> relancerait. Pour un basculement durable, désactiver l'auto-start V2 (non fait à ce stade).

### 4.4 BUILD de l'image base V4

Vérifier l'espace disque d'abord (l'image `hardware` pèse ~3,5 GB) ; prune **conservateur** si
besoin (n'enlève que le dangling + le cache, **pas** les images V2 taguées) :

```bash
ssh -i ~/.ssh/id_rpi_grovepi dietpi@192.168.1.164 '
  df -h / | tail -1
  sudo -n docker image prune -f && sudo -n docker builder prune -f
'
```

Lancer le build (long : clone GitHub + `colcon` + micro-ROS + `install_sensors.bash`).
En tâche de fond avec `nohup` + log :

```bash
ssh -i ~/.ssh/id_rpi_grovepi dietpi@192.168.1.164 'sudo -n bash -lc "
  cd /home/dietpi/prj_robotique/Bamboo4WD_V4/linorobot2/docker
  nohup docker compose build base > /tmp/v4_base_build.log 2>&1 &
"'
# suivre : ssh … 'tail -f /tmp/v4_base_build.log'
```

Produit l'image `bamboov4_humble-linorobot2-hardware:humble`. Les images `rosbridge` /
`foxglovebridge` **ne sont pas** namespacées (`linorobot2-rosbridge:humble`,
`linorobot2-foxglovebridge:humble`) → **réutilisées telles quelles**, aucun rebuild.

### 4.5 DÉMARRER la stack V4 (bring-up minimal L1)

```bash
ssh -i ~/.ssh/id_rpi_grovepi dietpi@192.168.1.164 'sudo -n bash -lc "
  cd /home/dietpi/prj_robotique/Bamboo4WD_V4/linorobot2/docker
  docker compose up -d rosbridge foxglovebridge
  docker compose ps
"'
```

Crée le réseau isolé `bamboov4_humble_linorobot2.network`, expose 9090 et 8765.

### 4.6 VÉRIFIER depuis Windows (MCP)

Après un **reload de Claude Code** (pour charger `ros2-analysis` avec la bonne URL), les outils
MCP lisent le graphe en direct : `ros2_status`, `topics`, `nodes`, `echo <topic>`.

Test « talker » de bout en bout (publie un topic bidon dans le conteneur, puis echo depuis
Windows) :

```bash
# côté RPi : publier /chatter (ros2cli présent dans l'image, demo_nodes non requis)
ssh -i ~/.ssh/id_rpi_grovepi dietpi@192.168.1.164 'sudo -n bash -lc "
  cd /home/dietpi/prj_robotique/Bamboo4WD_V4/linorobot2/docker
  docker compose exec -T -d rosbridge bash -lc \
    \"source /opt/ros/humble/setup.bash && exec ros2 topic pub /chatter std_msgs/msg/String {data:\ hello} -r 2\"
"'
# côté Windows : MCP echo /chatter  → doit renvoyer {\"data\": \"hello\"}
# nettoyage : pkill -f \"topic pub\" dans le conteneur
```

Résultat attendu : nœuds `rosbridge_websocket` / `rosapi` / `foxglove_bridge` visibles ;
`echo /chatter` renvoie le message ; Foxglove Studio se connecte sur 8765.

---

## 5. Paquets ROS 2 à créer (rappel plan)

| Paquet | Rôle |
|---|---|
| `bamboo_interfaces` | `msg/WheelRpm.msg`, `Esp32Status.msg` |
| `bamboo_base` | `esp32_mavlink_driver.py` (pont MAVLink, unique propriétaire du série) |
| `bamboo_bringup` | launch + YAML (driver, caméra, bringup intégré) |
| `bamboo_description` | xacro + meshes adaptés de `waver_description` (Apache-2.0, attribution conservée) |

Réutilisés : `linorobot2_base` (EKF), `linorobot2_bringup`, `linorobot2_navigation`.
Le driver importe le codec hôte : `from robot_control.communication.Esp32ComSerial import Esp32ComSerial`
(nécessite `pip install -e tools` dans l'env ROS, et l'arbre `firmware/` présent).

---

## 6. Caméra & diffusion vidéo (stratégie, performance, impact)

### 6.1 Stratégie — trois chemins, une seule webcam

La webcam UVC (05a3:9331) encode en **matériel** à 640×480 : MJPG@30, YUYV@30, **H264@30**
(vérifié `v4l2-ctl`). Contrainte UVC : **un seul format actif à la fois** sur `/dev/video0`.
On combine donc trois chemins, les deux « lourds » étant **à la demande** pour cohabiter :

| Chemin | Rôle | Format sur `/dev/video0` | Ré-encodage RPi |
|---|---|---|---|
| `camera.real` (v4l2_camera + web_video_server) | `/image_raw` pour ROS/RViz/SLAM + vue HTTP :8080 | YUYV → **rgb8** | oui (conversion + libx264/VP8) |
| `camera.h264` (MediaMTX + ffmpeg) — **étape 1, fait** | diffusion distante 30 fps (RTSP/HLS/WebRTC) | **H264** natif | **non** (`-c:v copy`) |
| gscam2 — **étape 2, à faire** | `/image_raw/compressed` 30 fps bas coût | **MJPG** natif | non (passthrough) |

### 6.2 Le goulot des ~12 fps (chemin ROS `camera.real`)

`v4l2_camera` 0.6.2 ne capture que des formats **bruts** → YUYV, puis **conversion logicielle
YUYV→rgb8** (« possibly slow conversion »). `web_video_server` **ré-encode** ensuite rgb8→JPEG/
H264 (libx264) à la volée. Ces deux étapes logicielles **saturent les 4 cœurs du RPi4**
(load ~4.3) → cadence plafonnée à **~12 fps**. Un node C++ n'y changerait rien : le coût est la
conversion de pixels + libx264, déjà en C++. **La seule issue = éviter conversion et ré-encodage**
en servant le flux **déjà compressé en matériel** par la caméra (d'où les étapes 1 et 2).

> ⚠️ Le « H.264 noir » dans le navigateur n'était **pas** une panne caméra : `web_video_server`
> renvoie le H264 en `video/mp4` fragmenté, illisible en navigation directe (il faut MSE/RTSP).
> MJPEG et VP8/VP9 sont lisibles en `<img>`/`<video>`. C'est ce qui a motivé MediaMTX (RTSP/HLS/
> WebRTC = lecture H.264 native).

### 6.3 Métrique FPS — `camera_fps_node.py`

Node rclpy autonome (`linorobot2_bringup/scripts/`, lancé par **chemin absolu** depuis le
bind-mount, comme `camera.launch.py` ; arg launch `enable_fps_metric`, défaut `true`). Publie
deux `std_msgs/Float32` (types **standard** → visibles via MCP rosbridge, contrairement aux
messages custom) :

- **`/camera/capture_fps`** — cadence de `/image_raw` (souscription `sensor_data`, on **date**
  l'arrivée sans désérialiser l'image). C'est un **plancher** : un abonné Python sur le RPi4
  sous-compte (rgb8 640×480 ≈ 920 Ko/msg, désérialisation lente + `best_effort` qui *drop*).
- **`/camera/stream_fps`** — cadence **réelle** en sortie de `web_video_server`, mesurée en
  comptant les marqueurs de début JPEG (SOI `0xFFD8`) sur le flux MJPEG HTTP.

**Échantillonnage périodique** (important pour l'impact CPU) : le node **n'ouvre pas** le flux
MJPEG en permanence — un flux ouvert force `web_video_server` à ré-encoder en continu. Il ouvre
`stream_measure_s` (3 s) toutes les `stream_sample_period_s` (30 s), mesure, **ferme**, et
republie la valeur jusqu'à l'échantillon suivant. `/dev/video0` et le CPU respirent entre deux
mesures.

### 6.4 Performance & impact mesurés

- **Étape 1 (MediaMTX H.264)** : ffprobe rapporte `h264 640×480 30/1` ; ffmpeg en `-c:v copy` →
  **ré-encodage nul**, load RPi **~0.8** pendant la diffusion. Source à la demande
  (`runOnDemandCloseAfter: 15s`) → `/dev/video0` **libre** quand personne ne regarde, donc
  cohabite avec MJPG→ROS.
- **Chemin ROS `camera.real`** : ~12 fps, load ~4.3 (saturation). À réserver à ROS/SLAM ; pour
  la vue distante fluide, préférer l'étape 1.
- **Métrique** : le client MJPEG permanent d'origine ajoutait une charge CPU constante (SSH
  ralenti, ping LAN ~300 ms). L'échantillonnage périodique **supprime** cette charge continue.

### 6.5 URLs de diffusion (étape 1)

`rtsp://<rpi>:8554/cam` (VLC) · `http://<rpi>:8888/cam` (HLS) · `http://<rpi>:8889/cam` (WebRTC).

### 6.6 Empaquetage (Dockerfile / compose)

- `camera.real` + métrique : `ros-$ROS_DISTRO-{v4l2-camera,web-video-server,image-transport-plugins}`
  (stage `hardware`) ; le node FPS n'ajoute **aucune** dépendance (stdlib + rclpy déjà présents).
- `camera.h264` : image **`bluenviron/mediamtx:latest-ffmpeg`** (hors stack ROS) + `docker/mediamtx.yml`.
- Étape 2 (gscam2) : gstreamer + build colcon de gscam2 — bloc **commenté** dans le Dockerfile,
  à décommenter une fois l'étape validée.

---

## 7. État d'avancement

- **L0** — protocole hôte (`mav/bamboo.py` généré, `tools/pyproject.toml`) : **fait** (banc).
- **L1** — conteneur Docker + MCP `ros2-analysis` lecture seule via rosbridge : **fait et
  vérifié sur le robot réel** (topics / nodes / echo / ros2_status OK depuis Windows).
- **L2** — YDLidar : **fait et vérifié** (`/scan` ~8 Hz, 600 pts, baud 128000).
- **L3** — caméra : **fait** (v4l2_camera YUYV→rgb8 + web_video_server :8080). **Métrique FPS**
  (`/camera/capture_fps`, `/camera/stream_fps`) et **diffusion H.264 distante 30 fps** (MediaMTX,
  étape 1) : faits (voir §6). Étape 2 gscam2 (MJPG→ROS 30 fps) : à faire.
- **L4/L5** — `bamboo_interfaces` + driver ESP32 **lecture seule** : faits (télémétrie validée).
- **L6+** — actuation + calibration (roues surélevées), manette, URDF/TF, EKF, SLAM, navigation :
  **à faire** (barrière L6 non franchie, choix utilisateur).

---

## 8. Points durs / caveats

1. **Double CP2102** : ESP32 et YDLidar = même `10c4:ea60`. Distinguer par `ATTRS{serial}`
   (règle udev dédiée `/dev/ydlidar` vs `/dev/esp32`), ne pas se fier au `/dev/ttyUSB0` en dur.
   Laisser `Esp32ComSerial` (DTR/RTS désassertés) ouvrir le port ESP32 pour éviter un reset.
2. **Calibration obligatoire** : placeholders firmware absurdes → `counts_per_rev`,
   `wheel_diameter_m`, `wheel_separation_m` en paramètres ROS (pas de reflash).
3. **QoS via rosbridge** : `/scan` et `/imu` sont *best-effort/sensor_data* ; rosbridge souscrit
   par défaut en *reliable* → **0 message** si la QoS n'est pas précisée dans l'op `subscribe`.
4. **Timestamps** : horloge du nœud à la réception (l'horloge ESP32 `time_boot_ms` n'est pas
   synchronisée).
5. **Clé YDLidar** : `resolution_fixed` (YAML) vs `fixed_resolution` (launch) — aligner sur le
   driver réellement installé.
6. **`DO_SET_SERVO` UNSUPPORTED** : pas de pan/tilt MAVLink → pas de suivi actionné promis.
7. **micro-ROS inutile** ici (transport = MAVLink série + driver Python) : ne pas s'appuyer sur
   l'agent micro-ROS installé par le stage `hardware`.
