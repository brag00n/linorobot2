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
  `BAMBOO_MOTOR_RPM` (42006, `rpm[4]` + `rpm_req[4]`, microcode ≥ 0.3.0), `HEARTBEAT`.
- **Réglage du PID** : `BAMBOO_MOTOR_RPM` porte les deux nombres que la boucle embarquée
  compare réellement — le RPM mesuré (encodeur, après filtrage carte) et le RPM demandé
  (sortie de la cinématique embarquée). Le driver les republie en
  `sensor_msgs/JointState` sur **`/joint_states`** (mesure) et **`/req_states`** (consigne),
  quatre articulations dans l'ordre M1..M4, `velocity` **en RPM** (écart assumé à la
  convention rad/s, pour comparer aux relevés de BambooV2) et `position` en radians.
  Comme le driver publie `/joint_states`, la description se lance avec
  `publish_joints:=false`.
- **Critère « suit le PID », VALIDÉ le 2026-09-23** (roues surélevées, gains
  `kp/ki/kd = 0,600 / 0,300 / 0,500` d'origine, **aucun réglage nécessaire**, cpr 2100,
  Ø 0,080 m). Banc : `bamboo_base/bench/pid_step.py`.

  | `linear.x` | consigne | `t_conv(± 10 %)` M1 / M2 | régime M1 / M2 | crête-à-crête | indice d'oscillation |
  |---|---|---|---|---|---|
  | 0,10 m/s | 23,87 RPM | 0,60 / 0,67 s | 24,03 ±0,53 (+0,6 %) / 23,85 ±0,63 (−0,1 %) | 2,3 / 3,5 | 0,43 / 0,60 |
  | 0,15 m/s | 35,81 RPM | 0,67 / 0,57 s | 34,91 ±1,06 (−2,5 %) / 35,00 ±1,12 (−2,3 %) | 3,6 / 3,6 | 0,29 / 0,29 |
  | 0,25 m/s | 59,68 RPM | 0,67 / 0,67 s | 59,62 ±0,54 (−0,1 %) / 59,63 ±0,49 (−0,1 %) | 2,1 / 2,1 | 0,50 / 0,38 |

  Convergence à **0,6–0,7 s** partout, donc sous la seconde ; erreur statique ≤ 2,5 % ;
  pas d'oscillation entretenue. **Piège de mesure à connaître** : un premier essai à
  0,10 m/s sur une marche de 3 s a donné `t_conv` = 1,00 / 1,73 s. Ce n'était pas une rampe
  lente mais **une excursion de bruit tardive** — à cette vitesse la bande ± 10 %
  (± 2,39 RPM) est du même ordre que le bruit crête-à-crête du RPM encodeur, et le critère
  (« dernier instant hors bande ») retient donc le dernier pic. La même marche allongée à
  5 s retombe à 0,60 / 0,67 s. **Conséquence pratique** : sous 0,15 m/s, mesurer sur au
  moins 5 s. L'indice d'oscillation élevé (0,4–0,8) à basse vitesse dit la même chose :
  c'est du bruit de quantification, pas du pompage — le lire **avec** la crête-à-crête,
  jamais seul.
- **Freinage, MESURE AU SOL le 2026-09-23** (`linear.x = 0,15` → ~35 RPM, banc
  `bamboo_base/bench/brake_stop.py`). Deux chemins d'arrêt, qui ne coûtent pas la même
  chose et qu'il faut mesurer séparément :

  | chemin | `cmd_vel_timeout_s` | t₁₀₀ % arrêt | distance après coupure |
  |---|---|---|---|
  | `Twist(0,0)` explicite | — | 0,77 s | **3,9 cm** |
  | flux coupé (watchdog) | 0,50 s | 1,14 s | 10,5 cm |
  | flux coupé (watchdog) | **0,25 s** | 1,13 s | **6,8 cm** |
  | flux coupé (watchdog) | 0,15 s | 0,70 s | 4,7 cm |

  Le critère du plan (« freinage < 0,5 s ») **n'est pas tenu, et ne peut pas l'être sans
  reflash** : le plancher est une **queue de freinage de ~0,7 s**, même avec un watchdog
  quasi nul. ⚠️ Ce n'est **pas** une roue libre, contrairement à ce que ce document affirmait
  avant T18 (2026-09-23) : consigne nulle et roue encore en mouvement, le PID embarqué
  **contre-pilote** — `spin()` appelle `forward()`/`reverse()` avec une PWM réelle, et
  l'intégrale amplifie l'effort tant que l'erreur persiste. Il est exact que l'ESP32 ne
  **court-circuite pas** le TB6612FNG (`USE_SHORT_BRAKE` n'est défini nulle part), mais il
  freine bel et bien par le moteur. La queue s'explique donc par l'autorité du PID à bas RPM,
  la bande morte `PWM_MIN` et la résolution du RPM en fin de course. Candidat firmware si
  besoin : frein par court-circuit (les deux entrées du pont au même niveau) sur consigne
  nulle, qui s'ajouterait au freinage PID au lieu de le remplacer.

  En revanche le critère en **temps** est trompeur ici : la queue de freinage est à 3-6 RPM,
  donc elle coûte des **centimètres**. La grandeur à retenir est la distance : **6,8 cm**
  avec le réglage retenu. D'où `cmd_vel_timeout_s` passé de 0,50 à **0,25 s** : à 25 Hz de
  `/joy` cela tolère encore 6 trames manquées (robuste au jitter du bridge Docker) et gagne
  3,7 cm. 0,15 s gagnerait 2,1 cm de plus pour 3 trames de marge seulement — non retenu.
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
- **L6** — actuation : **armée en permanence** depuis le 2026-10-03 (décision utilisateur après
  réparation de l'attache moteur M1), seul verrou = appui long sur B de la manette.
- **Lot QUALIBRATION** (2026-10-03) — modes du robot, calibration du glissement en rotation,
  compensation dans les drivers (§9) : code livré, compensation à ZÉRO (comportement d'origine),
  contrôle IMU validé au repos ; calibration au sol à faire.
- **Suite** — URDF/TF, EKF (vx roues + vyaw IMU), SLAM, navigation : **à faire**.

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

---

## 9. Modes du robot, calibration du glissement, compensation de rotation (lot QUALIBRATION)

### 9.1 Le problème

Au sol, sur l'essai du 2026-10-03 :
- les roues ont compté **4205° de rotation** pour **1972° réels** mesurés par l'IMU, soit un rapport de **2,13** ;
- le cap final était de −1141° selon les roues, contre −13° réels ;
- en ligne droite, l'écart restait dans ±6 %.

C'est le **patinage d'un châssis à 4 roues fixes (skid-steer)** : pour tourner, les roues doivent riper. Une consigne `angular.z` n'est donc tenue qu'à moitié, et l'odométrie croit que le robot tourne deux fois plus qu'il ne le fait.

Le défaut vaut quel que soit le producteur de `cmd_vel` : manette, navigation, ou les deux. La correction est donc placée **dans le driver**, en aval de tous les producteurs.

### 9.2 Choix d'architecture : System Modes et arbres de comportement

- **micro-ROS/system_modes** : un mode y est un jeu de valeurs de paramètres par nœud, avec les services `change_mode` / `get_mode` / `get_available_modes` et un sujet latché `mode_request_info`.
  - Le dépôt n'est **plus maintenu**.
  - Il suppose des nœuds **lifecycle**, et aucun des nôtres ne l'est.
  - Décision : on **reprend ses conventions sans le paquet**, dans `bamboo_mode_manager`.
- **Arbres de comportement (py_trees)** : ils servent **à l'intérieur** d'un mode, pour enchaîner des comportements avec garde et repli.
  - Aujourd'hui : la séquence de calibration.
  - Demain : la navigation (Nav2 est lui-même construit en BT).
  - Ils ne servent pas à choisir le mode : c'est une machine à états simple, pilotée par l'opérateur.

| Élément | Où | Rôle |
|---|---|---|
| `bamboo_mode_manager` | `bamboo_control`, conteneur `bamboo_control` | Tient le mode. Modèle dans `config/modes.yaml`. Services `/bamboo/change_mode`, `/bamboo/get_mode`, `/bamboo/get_available_modes`. Sujets latchés `/bamboo/mode` et `/bamboo/mode_request_info`. |
| `bamboo_teleop` | idem | **Demande** le mode (croix) et le **lit**. Les touches dépendent du mode et du groupe (`keymap.*` de `joy_bamboo.yaml`). Publie `/teleop/state`. |
| `MotionCompensator` | `bamboo_base/motion_compensation.py`, dans les **deux** drivers | Corrige la consigne et l'odométrie. Publie `/motion_compensation/state`. |
| `bamboo_calibration` | `bamboo_base/calibration_node.py`, conteneur `bamboo_calibration` (image `behaviors`) | Action `/bamboo_calibration/calibrate`, arbre py_trees. |
| HUD | `overlay_node` | Panneau mode / groupe / touches, et bandeau `CALIB n/N`. |
| OLED de l'ESP32 | firmware ≥ 0.4.0, `BAMBOO_HMI_TEXT` | Ligne `Ready <MODE>`. |

### 9.3 La manette en modes

**Contrôles globaux, non réaffectables :**

| Contrôle | Effet |
|---|---|
| **B court** | STOP. Annule aussi une calibration en cours. |
| **B long** | Armer / désarmer la traction. |
| **Stick gauche** | Conduite. Le pousser **annule** une calibration en cours. |
| **Croix gauche/droite** | Mode précédent / suivant. Les modes indisponibles sont sautés. |
| **Croix haut/bas** | Groupe d'actions du mode. |

**Touches par mode et par groupe :**

| Mode | Groupe | Touches |
|---|---|---|
| CONDUITE | vitesse | ZL/ZR = plafond de vitesse, −/+ = plancher |
| TRACKING | vision | X = suivi (X long = recentrage), Y = reconnaissance, A = acquisition, L = tracker, R = prédiction, ZL/ZR = plafond |
| TRACKING | cible | A = détecteur, Y = apprentissage, L/R = taille de la cible, ZL/ZR = plafond |
| CALIB | mesure | **A long = lancer**, X = contrôle IMU seul, Y = arcs on/off, R = variante suivante |
| CALIB | essai | A = appliquer le candidat **en RAM**, X = revenir au fichier, Y = PI de lacet on/off |
| NAV | exploration | A = exploration auto marche / arrêt, **A long = cartographier** (complète la carte courante), L / R = gate précédente / suivante, X = explorer la gate sélectionnée, X long = sauver la carte, Y long = arrêter, **+ long = nouvelle carte vierge**, + / − = carte enregistrée suivante / précédente (forcée), **− long = carte courante à la corbeille** (`maps/_corbeille/`), ZR long = miroir |
| NAV | navigation | **A long = naviguer** (carte courante), L / R = destination précédente / suivante, A = aller à la destination sélectionnée, Y long = nouvelle destination, **− long = supprimer la destination sélectionnée**, X long = retour au départ, Y = annuler le but, + long = relocaliser, + / − = carte enregistrée suivante / précédente (forcée) |

Une carte de touches fausse est **refusée au démarrage** : touche, action ou groupe inconnu, ou B réaffecté. La manette se replie alors sur la conduite seule, et le diagnostic `bamboo_teleop: modes` passe en ERROR avec la raison.

### 9.4 Les constantes et leur rangement

**Taux de glissement en lacet** : `s = 1 − ω_IMU / ω_roues`, compris entre 0 et 1.
- La consigne est corrigée en `ω' = ω / (1 − s)` (feed-forward).
- L'odométrie est corrigée en `ω = ω_roues · (1 − s)`.
- **s = 0 donne le comportement d'origine.**
- s est rangé dans une table |v| × |ω|, indexée sur la vitesse **réelle** du corps :
  - lignes = `v_bins_mps`, colonnes = `w_bins_rps` ;
  - interpolation bilinéaire, valeurs bornées hors de la table.
- Option : un **PI de vitesse de lacet** sur le gyro (`yaw_rate_kp/ki/i_max`). Il tient aussi la ligne droite.

**L'APB de la carte n'est pas modifié** : l'ESP32 l'utilise à la fois pour sa consigne et pour le `wz` qu'il renvoie.

Le rangement se fait dans le **fichier canonique du robot**, bloc `/**`, ce qui le rend visible de tous ses nœuds :
- **une entrée `calib.<variante>` par configuration matérielle** (roues, sol, écartement) ;
- la variante active est donnée par `hardware_variant` ;
- `origine` = tout à zéro, c'est la valeur livrée ;
- `essai` = variante de travail, toujours déclarée, où la calibration pousse un candidat **en RAM** ;
- une variante peut surcharger `wheel_separation_m` si le matériel change l'écartement ;
- `imu_gz_sign` = signe du gyro z, **à établir à l'essai** (un signe faux ferait diverger le PI ; la calibration le détecte).

### 9.5 Procédure de calibration

**Avant de commencer :**
- robot **au sol** ;
- **batterie chargée** (au moins 11,4 V, réglé dans `bamboo_base/config/calibration.yaml`) ;
- pour les arcs, environ **2 m × 2 m dégagés**.

**Déroulé :**
1. Croix → mode **CALIB**, groupe *mesure*. R choisit la variante, c'est-à-dire le nom du candidat.
2. **X** : contrôle IMU seul, **sans mouvement** (10 s au repos ; seuils |biais| < 0,005 rad/s et dérive < 0,5° en 10 s). Validé au repos le 2026-10-03 : biais −0,0001 rad/s, dérive −0,14°.
3. **B long** pour armer, puis **A long** pour lancer.
   - Paliers de rotation sur place : ω_cmd = 0,6 / 1,2 / 2,0 / 3,0 rad/s aux roues, dans les deux sens, avec la compensation neutralisée (`motion_compensation_bypass`).
   - Puis, si Y les a laissés actifs, les arcs : v = 0,2 / 0,4 × ω = 0,6 / 1,5.
   - Durée : environ 1 min 30 en rotations seules, environ 2 min 30 avec les arcs.
   - Le HUD et l'OLED affichent `CALIB n/N`.
4. **Pour arrêter à tout moment** : B, B long, le stick gauche, un changement de mode, ou la sortie du mode par le gestionnaire. Le chien de garde du driver (0,25 s) arrête le robot si le nœud meurt.
5. **Résultat** : un **candidat** `calib_candidat_<variante>_<date>.yaml` et son rapport `.json`, écrits **à côté du fichier canonique**. Le candidat est **rejeté** dans quatre cas :
   - robot soulevé (le gyro ne voit rien alors que les roues tournent) ;
   - signe du gyro inversé ;
   - asymétrie entre les sens supérieure à 15 % ;
   - s hors de [0, 1[.
6. Groupe *essai* : **A** essaie le candidat en RAM, **Y** essaie le PI, **X** revient au fichier.
7. Après validation, **recopier** le bloc du candidat sous `calib:` du fichier canonique et poser `hardware_variant`. Le nœud n'écrit **jamais** dans le fichier canonique.

**Sans manette :**
```
ros2 action send_goal /bamboo_calibration/calibrate bamboo_interfaces/action/Calibrate "{variant: essai, include_arcs: false, imu_check_only: true}"
```

### 9.6 Vérifier

- `/motion_compensation/state` : consigne reçue et consigne envoyée, s appliqué, `w_wheels` brut, `gz`. Avec la variante `origine`, `identity: true`.
- `/imu/data` : `angular_velocity` est rempli (covariance 0,0004) depuis `ATTITUDE.yawspeed`.
- Diagnostic :
  - `<driver>: compensation` ;
  - `<driver>: imu` (`gz_rad_s`) ;
  - `bamboo_teleop: modes` ;
  - `bamboo_mode_manager: modes du robot`.

## 10. Navigation autonome (lot NAV)

### 10.1 Vue d'ensemble

```
<robot>_base/config/<robot>.yaml   sensors: / lidar: / navigation:      -> parametres
<robot>_base/config/layout.yaml    wiring: lidar / navigation / control -> noms de sujets
<robot>_base/config/navigation.yaml  Nav2 propre au robot (RPP, costmaps, AMCL)

driver.lidarD500 (bamboo_lidar)  ldlidar_stl_ros2 --/scan--> scan_monitor (diag `lidar`)
navigation (bamboo_navigation)   EKF (roues vx + gyro vyaw) --/odom + TF odom-->
                                 nav_supervisor : exploration -> slam_toolbox (cartographie)
                                                  navigation  -> slam_toolbox (localisation) + Nav2
                                                  (carte COURANTE gardee, relocalisation maison)
                                 Nav2 (RPP, velocity_smoother) --/cmd_vel_nav-->
bamboo_control                   cmd_vel_mux : teleop 100 > calib 90 > nav 10 --/cmd_vel--> driver
```

Les deux conteneurs sont **à la demande** (`restart: "no"`) : rien ne démarre au boot tant que la navigation n'est pas validée.

```
docker compose --profile lidar up -d driver.lidarD500
docker compose --profile nav   up -d navigation
```

### 10.2 Lidar D500 (LDROBOT STL-19P, famille LD19)

- UART 230 400 bauds par un CP2102 — même VID:PID que l'ESP32. La règle udev se fait donc **par numéro de série** (`0001`) → `/dev/lidar` (`docker/udev/99-bambooWS.rules`). Les anciennes règles VID:PID du `Dockerfile`, qui envoyaient tout CP2102 sur `/dev/ydlidar`, sont supprimées.
- Driver `ldlidar_stl_ros2`, figé sur un commit, cloné par `docker/fetch_sources.sh` dans `third_party/` (non versionné), compilé par `rosbuild.sh`.
- `bamboo_lidar/launch/lidar.launch.py` lit le bloc `lidar:` du fichier canonique (modèle, port, débit, trame, sens, recadrage, portée) et le groupe `wiring.lidar` (`scan`).
- **Pose** : `lidar.pose_x/y/z/yaw` est lue par `description.launch.py` et passée au xacro. Mesurée le 2026-10-10 : 4 cm en avant du centre, centré, 17 cm de haut, **tourné de +90°**.
- Diagnostic `lidar` (`scan_monitor`) : cadence, points valides, portée, sujet muet.

### 10.3 `bamboo_navigation`

`launch/navigation.launch.py` (arguments `robot`, `robot_config`, `sim`, `use_sim_time`, `map`) écrit des yaml **patchés** dans `/tmp` (sujets du wiring, vitesses bornées de `navigation:`, `use_sim_time`), puis lance :

| Nœud | Rôle |
|---|---|
| `ekf_filter_node` | Fusion **vx des roues + vyaw du gyro** (le lacet des roues est ignoré : glissement). Publie `/odom` et la TF `odom → base_footprint`. |
| Nav2 | controller (RPP, `use_rotate_to_heading`) → velocity_smoother → `/cmd_vel_nav` ; planner, smoother, behavior_server (spin/backup directement sur `/cmd_vel_nav`), bt_navigator, waypoint_follower. |
| `nav_supervisor` | Carte courante, sous-modes, slam_toolbox (sous-processus), gates, destinations, relocalisation, sauvegardes, état, diagnostic. |
| lifecycle manager | navigation, `autostart: false` : c'est le superviseur qui démarre Nav2. map_server et AMCL ne sont plus lancés (2026-10-13). |

**Sous-modes** (un seul actif), tous sur la **carte courante**, gardée de l'un à l'autre :
- `exploration` : slam_toolbox en **cartographie**. La carte courante se **complète** : son graphe est rechargé à la pose du robot. `exploration new` (ou `new_map`) repart d'une carte vierge. On conduit à la manette, ou l'exploration automatique va de gate en gate ;
- `navigation` : slam_toolbox en **localisation** sur le graphe de la carte courante, Nav2 actif, buts vers les destinations ;
- `idle` : tout est arrêté ; la carte reste la carte courante, sur le disque.

En passant d'exploration à navigation (ou l'inverse), la pose courante est transmise : pas de relocalisation, pas de saut.

**Commandes** — service unique `/nav_supervisor/command` (`bamboo_interfaces/srv/NavCommand`, liste complète dans le `.srv`) :
`exploration [new]`, `navigation [carte]`, `idle`, `new_map`, `load_map <carte>`, `save_map`, `auto_explore`, `explore_gate [n]`, `select gate+|gate-|dest+|dest-|gate n|dest n|map+|map-`, `dest_add [nom]`, `dest_del [n|nom]`, `dest_rename <n|nom> <nouveau>`, `goto [n|nom|home|next]`, `relocalize [global]`, `cancel`, `list`, `events [n]`.

**État** : `/navigation/state` (`NavState`, latché) — sous-mode, carte, but, distance restante, reprises, **localisation** (`loc_state`, accord scan/carte `map_match`), **sélection**, **gates** et **destinations** (tableaux, pour la mini-carte), capteurs manquants. Lu par le HUD (carte NAV, mini-carte), la manette et le MCP. Le détail part aussi en JSON sur `/navigation/objects`.

**Diagnostic `navigation`** : il n'exige **que les capteurs déclarés** dans `sensors:` du fichier canonique. Lidar absent ou muet → ERROR « navigation impossible ». En navigation : localisation perdue ou en cours de relocalisation, ou carte douteuse → WARN.

### 10.3.1 La carte est un dossier

`<robot>_base/maps/<carte>/` (`map_io.MapStore`) :

| Fichier | Contenu |
|---|---|
| `slam.posegraph` + `slam.data` | graphe de poses de slam_toolbox (`SerializePoseGraph`) : ce qui permet de **compléter** la carte et de s'y **localiser** |
| `map.pgm` + `map.yaml` | l'image (format map_server) : MCP, HUD, relocalisation |
| `meta.yaml` | destinations, gates, départ, dernière pose |
| `events.jsonl` | **historique**, une ligne JSON par événement (sous-modes, sauvegardes, gates, destinations, relocalisations, décalages) |

- **Carte courante** : son nom est dans `maps/COURANTE`, et elle est reprise au démarrage.
- **Sauvegarde** :
  - graphe + image + meta toutes les `save_period_s` (60 s) **si le robot a bougé** ;
  - à chaque événement : gate atteinte, destination, changement de sous-mode, arrêt, sortie du mode NAV, fin d'exploration ;
  - toute écriture passe par un fichier temporaire renommé, donc une coupure laisse l'ancienne version ou la nouvelle.
- **Anciennes cartes** (fichiers à plat d'avant le 2026-10-13) : listées « ancienne ». `load_map` les **importe** en dossier (image, destinations, gates), mais sans graphe de poses : on ne peut ni les compléter, ni y naviguer.

### 10.3.2 Gates et destinations

- **Gates** = les frontières connu / inconnu, **numérotées** et gardées dans la meta. Une gate garde son numéro tant qu'elle reste à moins de 0,5 m. Statuts :
  - `ouverte` : à explorer ;
  - `en_cours` : visée en ce moment ;
  - `atteinte` : le robot y est allé ;
  - `exploree` : la zone est connue ;
  - `abandonnee` : Nav2 a échoué, ou le but n'avançait plus ;
  - `inatteignable` : aucun chemin.
- **Exploration automatique** : elle va de gate ouverte en gate ouverte, après avoir vérifié qu'un chemin existe, puis revient au départ. Une gate **désignée** (`explore_gate`, ou L / R puis X à la manette) passe avant les autres. Une nouvelle session redonne leur chance aux gates en échec.
- **Destinations** : `p1`, `p2`… (renommables au MCP). Les numéros ne sont jamais réutilisés.

### 10.3.3 Relocalisation et contrôle « carte = réalité » (`relocalize.py`)

slam_toolbox ne sait pas faire de **localisation globale** : il lui faut une pose approchée. Quand la pose est inconnue (démarrage, robot déplacé, décalage), le superviseur la cherche lui-même sur l'image de la carte :
- **score** d'une pose = champ de vraisemblance (distance au mur le plus proche) × part des rayons qui ne **traversent** aucun mur de la carte avant leur impact ;
- recherche **locale** autour de la dernière pose connue, puis **globale** sur toutes les cellules libres × 32 caps, puis affinage ;
- la pose est **acceptée** si son score atteint `reloc_accept` et devance de `reloc_margin` la meilleure hypothèse **distincte** ;
- en cas d'**ambiguïté**, le robot fait des rotations de 90° (au plus `reloc_turns`, traction armée seulement) et cumule les scores ;
- la pose trouvée est donnée à slam_toolbox : `map_start_pose` au démarrage, `/initialpose` en cours de navigation.

**Contrôle continu en navigation**, toutes les 2 s :
- accord sous `match_ok` : avertissement « carte modifiée ? » ;
- sous `match_lost` pendant 5 s : le robot est **perdu**. Le but est mis en pause, le robot est relocalisé (localement, puis globalement), puis le but **reprend**.

Les seuils sont à caler sur de vrais scans pris à des poses connues (essai G4).

**Plusieurs cartes** (2026-10-13) : si le robot n'est pas reconnu sur la carte courante, en entrant en exploration ou en navigation, ou s'il est perdu en navigation, les **autres cartes** à graphe de poses sont essayées une à une, la plus récente d'abord (`reloc_other_maps_max` = 10). Parmi les cartes qui le reconnaissent et dont les scores sont voisins, la plus récente l'emporte, et elle devient la carte courante. Un but en cours n'est pas repris. À la manette, **+ / −** (appuis courts) parcourent les cartes enregistrées : la carte choisie est **forcée** 2 s après le dernier appui (relocalisation sur elle seule).

### 10.4 Arbitrage des consignes

`bamboo_control/cmd_vel_mux` reprend la sémantique et la configuration de twist_mux (image non reconstruite : plusieurs heures, disque à 90 %). Les noms viennent de `wiring.control`, les priorités de `config/cmd_vel_mux.yaml` :

| Entrée | Priorité | Délai |
|---|---|---|
| manette `/cmd_vel_teleop` | 100 | 0,5 s |
| calibration `/cmd_vel_calib` | 90 | 0,5 s |
| navigation `/cmd_vel_nav` | 10 | 0,5 s |

Pousser le stick reprend **toujours** la main ; la navigation repart 0,5 s après le relâchement. B / B long et le chien de garde du driver (0,25 s) restent en vigueur.

### 10.5 Mode NAV

Prérequis : `node:/nav_supervisor` **et** `param:has_lidar` (poussé par `control.launch.py` depuis `sensors.lidar`). À l'entrée, la détection de visages est coupée ; à la sortie, le but en cours est annulé. Touches : voir §9.3.

### 10.6 Simulateur (`linorobot2_gazebo`)

Le plugin Gazebo publie les mêmes sujets que le vrai robot (`odom/unfiltered`, `/scan` en trame `laser`, `imu/data`) et consomme `cmd_vel`.

```
docker compose up bringup.simu.nav     # gazebo.launch.py ekf:=false robot:=${ROBOT}
docker compose up navigation.simu      # navigation.launch.py robot:=${ROBOT} sim:=true
```

- `gazebo.launch.py` gagne `ekf` (défaut `true`, comportement amont) et `robot` (description du robot simulé).
- Avec `sim:=true`, le launch de navigation lance aussi `cmd_vel_mux`, que le vrai robot tient dans `bamboo_control`.
- Gazebo tourne sur le PC, pas sur le RPi.

### 10.7 MCP

| Outil | Serveur | Effet |
|---|---|---|
| `map` | ros2-analysis | GetMap (slam_toolbox, dans les deux sous-modes) → taille, part explorée, pose du robot, localisation, gates (statut, numéro), destinations, **PNG** à lire en image. |
| `nav_state` | ros2-analysis | Résumé de `/navigation/state` (localisation, sélection, gates, destinations). |
| `waypoints` | ros2-analysis | Cartes, destinations et gates (commande `list`). |
| `nav_events` | ros2-analysis | Historique de la carte courante (commande `events`). |
| `call_service /nav_supervisor/command` | ros2-action | Toutes les autres commandes. |

Le rosbridge monte `ws_install` en lecture seule et le source : sans cela, il ne connaît pas les types `bamboo_interfaces`.

### 10.8 Essais

| Essai | Vérification |
|---|---|
| Na ✅ | `/dev/esp32` et `/dev/lidar` distincts, `/scan` ≈ 10 Hz, orientation et sens validés à l'obstacle |
| Nb | TF `odom → base_footprint`, `/odom` suit l'IMU sur 360°, manette à travers le multiplexeur |
| Nc | carte d'une pièce, sauvée par X long, visible par l'outil `map` |
| Nd | localisation, deux points, aller à 0,2 m/s, reprise au stick, retour au départ |
| Ne | obstacle sur le trajet : contournement ou reprise |
| Nf | lidar débranché : mode NAV indisponible, diagnostic `navigation` en ERROR |
| Nsim | cartographie et points dans `playground.world` |
| G0 ✅ (partiel) | relocalisation hors robot : 80 poses simulées sur les 10 vraies cartes, aucune pose fausse acceptée. Un vrai scan, robot immobile : une seule carte acceptée. Reste à la refaire à une pose **connue** |
| G1 | carte vierge, exploration auto, sauvegarde périodique visible dans le dossier ; arrêt brutal → rien de perdu |
| G2 | compléter : carte rechargée, cartographie reprise sans murs dédoublés ; numéros des gates stables ; gate désignée explorée en premier |
| G3 | exploration → navigation → exploration sans perte ni saut ; destinations ajoutées, supprimées, atteintes |
| G4 | robot posé ailleurs : localisation globale (avec rotations si ambiguë), puis navigation vers une destination |
| G5 | robot soulevé et déplacé en navigation → pause, relocalisation, reprise ; meuble déplacé → simple avertissement |
