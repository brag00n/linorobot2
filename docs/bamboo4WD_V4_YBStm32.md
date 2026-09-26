# bamboo4WD_V4_YBStm32 (bambooSTM32YB) sous linorobot2 (ROS 2 Humble)

Fiche de **ce robot-ci**. Pendant de [`bamboo4WD_V4_WSEsp32.md`](bamboo4WD_V4_WSEsp32.md), et il faut
lire les deux comme deux machines distinctes, pas comme deux variantes d'une même : elles n'ont ni la
même carte, ni le même nombre d'encodeurs réels, ni les mêmes capacités d'actuation.

> ⚠️ **Règle de périmètre, à ne pas éroder.** La **caméra motorisée** appartient à **ce robot**. Elle
> n'est pas une option de BambooWS : la carte de BambooWS répond `MAV_RESULT_UNSUPPORTED` à
> `DO_SET_SERVO` (`ConnectorMavlink.cpp:450`) **tout en renvoyant un ACK**, donc le pan/tilt MAVLink y
> est impossible — et impossible **silencieusement**, ce qui est pire.

> 🔶 **État de validation, annoncé d'emblée.** Les paquets ROS de la chaîne de tracking sont **écrits**
> et un seul essai est **validé sur robot** : **T10 de bambooSTM32YB** (groupe vidéo seul, flux brut à
> 30 fps). **Aucun servo n'a jamais été branché** : l'essai **T1** (tension du rail servo au
> multimètre) est un **prérequis non négociable**. Tout ce qui touche au mouvement de la caméra est
> donc ici du **contrat**, pas du mesuré. Les valeurs non mesurées sont marquées
> `<< NON MESURÉ >>` — elles le sont aussi dans le fichier canonique, et c'est là qu'il faut les
> corriger.

**Numérotation des essais** : la série `Tn` de ce robot est **homonyme** de celle de BambooWS. Toujours
citer le robot — « T10 de bambooSTM32YB », jamais « T10 » nu.

---

## 1. La carte, et ce qui la distingue

Fiche matérielle détaillée : [`hardware_yahboom_ros_control_board_v3.md`](../../linorobot2_hardware/docs/hardware_yahboom_ros_control_board_v3.md).
Ici, seulement ce qui change la façon de piloter ce robot.

| | bambooSTM32YB (Yahboom V3.0) | BambooWS (WaveShare ESP32) |
|---|---|---|
| MCU | STM32F103RCT6 | ESP32-WROOM-32 |
| sysid MAVLink | **1** | 2 |
| Pont série | CH340 `1a86:7523` @115200 | CP2102 `10c4:ea60` |
| Encodeurs | **4 réels** | 2 (voies 3/4 **recopiées**) |
| IMU | MPU9250, **avec magnétomètre** → cap absolu | AK09918C + QMI8658C |
| Servos PWM | **4 voies** | **aucune** |
| PID | **par moteur** | commun aux 4 |
| Persistance | **flash** | aucune |
| Courant batterie | non | oui (INA219) |

Deux conséquences pratiques du tableau, plus importantes que le tableau lui-même :

1. **Le CH340 et le CP2102 ne se disputent rien.** Les deux robots peuvent coexister sur le même hôte
   sans arbitrage de périphérique. En revanche le CP2102 est **partagé avec le lidar**, qui a le même
   VID:PID → désambiguïsation par **numéro de série**, jamais par ordre d'énumération.
2. **Le port série de cette carte a un propriétaire UNIQUE.** Avant de démarrer `driver.stm32`, arrêter
   `robot_controlv3` **et** le serveur MCP `robot-action`, qui l'ouvrent tous les deux. Et
   réciproquement. C'est la cause la plus fréquente d'un driver qui « ne voit pas la carte ».

---

## 2. Source unique de vérité : le fichier canonique

`bamboo_base/config/robots/bamboo4WD_V4_YBStm32.yaml` porte **toute** la physique de cette machine et
rien d'autre. Les **faits d'hôte** (port, baud, `/dev/videoN`, ports TCP) n'y sont **pas** : ils vivent
dans les YAML de groupe et dans les profils `robots/*.json` du dépôt firmware, parce qu'ils diffèrent
légitimement d'une machine à l'autre alors que la géométrie, non.

Format ROS natif, donc utilisable tel quel :

```bash
ros2 param load /stm32_mavlink_driver \
  install/bamboo_base/share/bamboo_base/config/robots/bamboo4WD_V4_YBStm32.yaml
```

| Clé | Valeur | Provenance |
|---|---|---|
| `counts_per_rev` | **1320.0** | microcode : 30 (réducteur) × 11 (fentes) × 4 (quadrature). À re-confirmer par **T3**. |
| `wheel_diameter_m` | 0.0685 | `FOURWHEEL_CIRCLE_MM 215.2 / π`. `<< pied à coulisse dû >>` |
| `wheel_separation_m` | 0.165 | `<< NON MESURÉ >>` — voir l'avertissement ci-dessous |
| `motor_max_rpm` | 330.0 | nominal moteur |
| `motor_operating_voltage` / `..._power_max_voltage` | 7.4 / **12.6** | l'écart **est** le risque, cf. §6 |
| `pid_kp/ki/kd` | 4 × [0.8 / 0.06 / 0.5] | **par moteur**, capacité propre à cette carte |
| `car_type` | 4.0 | FOURWHEEL |
| `enable_cmd_vel` | **false** | règle de sécurité, cf. §6 |

> ⚠️ **Le piège de conversion de ce robot.** Le firmware ne connaît **ni la voie ni l'empattement**,
> seulement leur **demi-somme** `FOURWHEEL_APB = 164.555` mm — la seule combinaison dont sa cinématique
> a besoin. On ne peut donc **pas** en déduire `wheel_separation_m` : 164,555 mm de demi-somme vaut pour
> une voie de 0,165 et un empattement de 0,164 **comme** pour 0,200 et 0,129. Le mètre ruban, ou
> l'erreur d'angle sur un 360° au sol, sont les seules sources. Le `<< NON MESURÉ >>` n'est pas de la
> prudence de façade : la valeur est réellement indéterminée.

> ℹ️ **Ce que la carte arrondit.** Le stockage flash quantifie circonférence et APB en **dixièmes de
> mm** (entiers 16 bits). Écrire une valeur plus fine ici ne la conserve pas : la carte l'arrondit.

**Écart à résoudre, consigné** : ce fichier donne 330 RPM là où
`hardware_yahboom_ros_control_board_v3.md:102` dit 275, et les valeurs de géométrie persistées relevées
en flash (circ 204,2 / APB 117,5) ne sont pas celles du microcode (215,2 / 164,555). Aucune des deux
divergences n'est tranchée — ne pas « harmoniser » l'une sur l'autre sans mesure.

---

## 3. Les paquets, et l'axe qu'ils servent

Deux axes de variabilité, à ne jamais confondre : un **robot** est une machine, un **contrôleur** est
une carte. Un robot compose un ou plusieurs contrôleurs.

| Paquet | Axe | Rôle |
|---|---|---|
| `bamboo4WD_V4_YBStm32_base` | **robot** | bringup : déclare la **composition** de cette machine. Aucun nœud. |
| `bamboo_controler_YBStm32v3` | **contrôleur** | le nœud `stm32_mavlink_driver` + `config/capabilities.yaml` |
| `bamboo_base` | commun | squelette de driver, résolveur VID:PID, `capability_check`, fichiers canoniques |
| `bamboo_video` | groupe | acquisition + mux + diffusion HTTP. **Autonome.** |
| `bamboo_videotracking` | groupe | Tracking / FaceRecog / Overlay / ServoCam (C++ composé) + apprentissage (Python) |
| `bamboo_control` | groupe | manette — **lot V5, pas encore écrit** |
| `bamboo_interfaces` | commun | `ServoCmd`, `ModeCmd`, `TrackState`, `TrackingStats`, `Stm32Status`, `WheelRpm`, … |

### Une seule bascule de robot

`docker/.env` porte `ROBOT=bamboo4WD_V4_YBStm32`. Compose le lit et le propage en `robot:=` aux launch,
qui le transmettent **tel quel** à tous les groupes → fichier canonique, géométrie, PID, table
`servo_axes`. C'est la **seule** ligne à changer.

> ⚠️ **Limite connue, et ce n'est pas un bug à contourner** : compose ne sait **pas** conditionner un
> service à la **valeur** d'une variable (`profiles:` est **additif**, pas restrictif). `ROBOT` choisit
> le canonique, mais c'est l'**opérateur** qui démarre le service de contrôleur de **son** robot
> (`driver.stm32` ici, `driver.real` pour BambooWS).

### Capacités : déclarées dans un fichier, vérifiées avant tout nœud

Chaque module de contrôleur livre `config/capabilities.yaml`. Ce n'est **pas** un fichier de paramètres
ROS (pas de `/**: ros__parameters:`) : il est lu par du code de **launch**, avant qu'aucun nœud
n'existe.

`bamboo_base.capability_check` confronte la table `servo_axes` du canonique aux capacités des
contrôleurs visés et **échoue en nommant l'axe** si la carte déclare `servo_pwm: 0` ou si la voie est
hors plage. Il tourne en **première action** du bringup, **même tracking désactivé** — un axe mal
déclaré est une erreur de configuration, pas une conséquence d'un drapeau. Il ne lit que des fichiers :
il n'ouvre **jamais** le port série mono-propriétaire.

```bash
ros2 run bamboo_base capability_check bamboo4WD_V4_YBStm32   # une ligne par axe
```

Ce que ce contrôle achète : sur BambooWS, un axe de servo partirait dans le vide **avec un ACK**. Le
défaut serait invisible du logiciel.

---

## 4. Servos : une abstraction par topic, pas par carte

La commande des servomoteurs **ne vient pas « de la carte STM32 »**. Elle circule sur un contrat ROS que
*n'importe quel* contrôleur porteur de la capacité peut honorer.

| Élément | Rôle |
|---|---|
| `/servo/cmd` (`ServoCmd`) | consigne **absolue** d'un axe **nommé** |
| `/servo/nudge` (`Nudge`) | incrément relatif |
| `/servo/state` (`ServoCmd`) | position rendue — **ombre d'hôte**, cf. l'avertissement |
| `servo_axes` (canonique) | `nom d'axe → (contrôleur, voie, bornes, repos)` — **seul** endroit qui lie un axe à une carte |

Le producteur adresse un axe **par son nom** (`pan`, `tilt`), jamais un numéro de voie ni une carte :
c'est l'**exécuteur** qui résout via `servo_axes`. Déplacer les SG90 sur une autre carte — second STM32,
Teensy, PCA9685 — ne change **que** `servo_axes`.

Deux conséquences à ne pas contourner : le **groupe tracking ne dépend d'aucun contrôleur** (il tourne
même sans exécuteur, ce qui rend le banc hors robot possible), et un **axe sans exécuteur doit être
bruyant, pas muet** (§3).

> ⚠️ **`/servo/state` est une ombre, pas une mesure.** Il n'existe **aucun retour de position matériel** :
> aucun appelant de `PwmServo_Get_Angle`, pas de `SERVO_OUTPUT_RAW`, table PARAM arrêtée à l'index 18. Un
> décalage mécanique restera **invisible du logiciel**, et un `COMMAND_ACK` ne prouve **pas** que l'angle
> a été appliqué — le firmware **jette en silence** un angle > 180° en répondant `ACCEPTED`.
> Loi d'impulsion, pour mémoire : `(angle × 11 + 500) / 10` µs.

---

## 5. Chaîne vidéo et tracking

Deux groupes, **découplés par construction** — exigence explicite de ce chantier : on doit pouvoir
arrêter le tracking sans couper le flux.

```
/dev/video0 --gscam2--> /video/raw/compressed --> mux --> /video/stream/compressed --> HTTP :8080
   (MJPG)                        |                 ^
                                 |                 | /videotracking/enriched/compressed
                                 +--> groupe tracking
```

`bamboo_videotracking` **s'enregistre** sur `bamboo_video` (`/video/register_overlay`) pour récupérer le
flux et le rendre **enrichi**. Tant qu'aucun client ne s'est enregistré, le mux sert le **brut** —
mesuré à T10, `stream_source` = `raw`, passthrough **sans perte** (30,001 en entrée, 29,989–30,000 en
sortie). Aucun `depends_on` dans un sens ni dans l'autre.

Les quatre étages du chemin chaud vivent dans **un seul processus** (`component_container_mt` +
intra-process) : la `cv::Mat` décodée une fois circule **par pointeur**. Hors composition, chaque saut
coûterait 921 Ko recopiés par trame — la mémoire partagée DDS étant coupée par l'entrypoint.
**Test de non-régression** : si un `sensor_msgs/Image` apparaît entre `tracking_node`, `facerecog_node`
et `overlay_node`, la composition **n'a pas pris** et tout le gain du C++ est perdu.

> ⚠️ **`transient_local` (latché) est incompatible avec l'intra-process.** rclcpp refuse **à la
> construction**, et les **quatre** composables mouraient sur **une seule** entité fautive. Le correctif
> est **par entité** (`SubscriptionOptions` + `IntraProcessSetting::Disable`), pas par nœud, pour que les
> images restent en zero-copy. Contraint **toute** future entité latchée du groupe.

### URLs de diffusion — le piège qui coûte une heure

`web_video_server` **exige `&type=ros_compressed`** quand seul le topic `/compressed` est publié. L'URL
nue s'abonne à l'`Image` **brute**, qui n'existe pas, et **attend jusqu'au timeout sans rien écrire ni
renvoyer d'erreur** (symptôme mesuré à T10 : 0 octet sur le snapshot, 22 octets sur le flux).

```
http://<rpi>:8080/stream?topic=/video/stream&type=ros_compressed
http://<rpi>:8080/snapshot?topic=/video/stream&type=ros_compressed
http://<rpi>:8080/stream?topic=/videotracking/enriched&type=ros_compressed
```

La **racine** `http://<rpi>:8080/` liste les topics servis **et donne les deux variantes d'URL** : à
consulter en premier en cas de doute.

> ⚠️ **Contrainte UVC, matérielle et non négociable** : `/dev/video0` ne sert **qu'un format à la fois**.
> `bamboo_video`, `camera.mjpg`, `camera.real` et `camera.h264` sont donc **mutuellement exclusifs** — et
> `bamboo_video` partage en plus le port 8080 avec `camera.mjpg`. Corollaire : **`h264` et tracking sont
> incompatibles**, en `h264` le flux ne passe pas par ROS.

### Deux OpenCV dans l'image, et pourquoi

L'image porte **4.5.4** (apt, `/usr/lib/aarch64-linux-gnu`) **et 4.10.0** compilé depuis les sources
(`/usr/local`). Motif mesuré : **4.5.4 ne sait pas lire `face_detection_yunet_2023mar.onnx`** — six
tailles d'entrée sondées, les six échouent. Ce n'est pas une contrainte de dimension mais un
**millésime de modèle** : 4.5.4 attend le graphe `2022mar`.

Les SONAME sont versionnés (`.so.405` vs `.so.410`) → cohabitation sans risque **sur le disque**. Ce qui
est interdit, c'est de **mélanger les deux dans un processus** :

- `bamboo_videotracking` (aucune dépendance à `cv_bridge`) se lie à **4.10 seul** ;
- `bamboo_video` (`web_video_server` + `cv_bridge`, compilé contre 4.5.4) reste sur **4.5.4 seul**.

`bamboo_videotracking/CMakeLists.txt` **trace** la version retenue et **échoue à la configuration** sous
4.8 : avec deux arbres dans l'image, un repli silencieux donnerait un build « réussi » et un modèle
toujours illisible à l'exécution. Bindings Python **volontairement OFF**.

---

## 6. Sécurité — l'ordre n'est pas négociable

1. **T1 d'abord : mesurer V+ du header servo au multimètre**, carte alimentée, moteurs arrêtés. Le rail
   servo de cette carte n'est **pas publié**, et la carte annonce aussi des servos de **bus série**
   (famille 6–12 V chez Yahboom). Brancher un SG90 (4,8–6 V) sur un rail 6–12 V le détruit **au premier
   contact**. Aucun SG90 ne se branche avant ce relevé.
2. **SG90 sur BEC 5 V dédié** pris sur le pack, **fil de signal seul** vers la carte, masse commune.
   Retenu **même si T1 donne 5 V** : deux SG90 en butée, c'est ~1,4 A impulsionnel dans le régulateur
   qui alimente le STM32.
3. **Course bornée 8°–172° côté hôte** — **jamais 17–178**. À 180° le SG90 force en butée en continu
   (~700 mA, bourdonnement, pignons plastique) : panne classique d'une caméra restée des minutes en
   position.
4. **`enable_cmd_vel: false` partout.** Le pack 12,6 V arrive **brut** sur des moteurs 7,4 V nominaux,
   les constantes PWM n'étant pas mises à l'échelle. **Servos oui, moteurs non**, jusqu'aux essais de
   motorisation.
5. **Ne jamais alimenter en 8,6–9,5 V** : arrêt **latchant** en 2 s, reset obligatoire. Seuils 9,6 V bas
   / 13,0 V haut.
6. **Un seul propriétaire du série** (§1).
7. **`bamboo_videotracking` hors du profil `boot`** jusqu'à validation du tracking en boucle fermée : ce
   groupe est le **seul** à commander les servos, et un robot qui bouge la tête tout seul au démarrage
   n'est pas acceptable avant d'avoir mesuré les butées.

---

## 7. Exploitation

### Les deux chemins, et ils ne font pas le même travail

**Exploitation — un groupe par conteneur** (politique de redémarrage et `colcon build` propres à
chacun ; c'est ce découpage qui permet d'arrêter le tracking sans couper la vidéo) :

```bash
cd /home/dietpi/prj_robotique/Bamboo4WD_V4/linorobot2/docker
sudo -n docker compose --profile boot up -d          # bamboo_video + driver.stm32
sudo -n docker compose up -d bamboo_videotracking    # à armer À LA MAIN (§6.7)
```

**Banc / mise au point — un seul processus**, la définition lisible de « tous les nœuds du robot » :

```bash
ros2 launch bamboo4WD_V4_YBStm32_base bringup.launch.py
ros2 launch bamboo4WD_V4_YBStm32_base bringup.launch.py enable_tracking:=true
ros2 launch bamboo4WD_V4_YBStm32_base bringup.launch.py enable_video:=false   # carte seule
```

Les deux chemins incluent les **mêmes** launch de groupe : il n'y a pas deux configurations à maintenir.
Et ils sont **exclusifs** — le service `bringup.stm32` et les trois services de groupe se disputeraient
le CH340 et `/dev/video0`.

Défauts du bringup, et pourquoi ils sont prudents : `enable_driver` / `enable_description` /
`enable_video` à **true** ; `enable_tracking` **false** (§6.7) ; `enable_control` **false** (paquet du
lot V5) ; `enable_cmd_vel` **false** (§6.4). `enable_control:=true` avant l'existence du paquet fait
**échouer le launch en le nommant** — comportement voulu, pas un bug.

> ⚠️ Un paquet `ament_cmake` **doit être construit** : la commande de chaque service fait son
> `colcon build --packages-select`. Un `colcon build` **nu** tenterait `bamboo_videotracking` et
> échouerait — `--packages-select` est **essentiel**. Et les fichiers neufs sous bind-mount ne sont pas
> liés dans `install/share` sans repasser colcon.

### Topics du contrôleur

Publiés : `odom/unfiltered`, `imu/data`, `mag`, `battery`, `wheel_rpm`, `stm32_status`, `joint_states`,
`req_states`, `servo/state`.
Souscrits / services : `servo/cmd`, `servo/nudge`, `save_board_config` (cette carte **a** la persistance
flash), et `cmd_vel` **uniquement** si `enable_cmd_vel` est armé.

> ⚠️ **QoS et rosbridge** : `imu/data` et `mag` publient en `sensor_data` (best-effort), donc sont
> **illisibles par rosbridge**, qui est `reliable`. Un `echo` vide n'y est pas la preuve d'une absence de
> données.

### Périphériques

Les règles udev sont au dépôt (`docker/udev/99-bambooSTM32YB.rules`) mais **pas encore posées sur le
RPi** — d'où `BAMBOO_CAM=/dev/video0` par défaut dans `.env` et non `/dev/bamboocam`. Après installation
des règles, basculer `.env`. Le driver, lui, se passe des symlinks : il résout par scan **VID:PID**
(`ros2 run bamboo_base hw_resolve`).

---

## 8. État d'avancement

| Élément | État |
|---|---|
| Fichier canonique + module de contrôleur | écrits |
| `bamboo_video` seul, flux brut 30 fps à `:8080` | ✅ **T10 validé** |
| `bamboo_videotracking` — compilation et composition | 🔶 le paquet compile, les 4 composables se construisent ; relevés à refaire sous 4.10 |
| OpenCV 4.10 dans l'image | 🔶 en construction |
| Bringup mono-processus | écrit, **jamais lancé** |
| `capability_check` + capacités déclarées | écrits, **jamais exécutés** |
| Servos | ❌ **rien branché** — T1 dû |
| Manette (`bamboo_control`) | ❌ lot V5, paquet inexistant (les paquets apt, eux, sont dans l'image) |
| Mesure de performance (banc reproductible) | écrit, **aucune mesure** |
| Démarrage automatique | non posé |

**Objectifs chiffrés — cibles, pas mesures** : capture 30 fps · détection ≥ 10 fps à `det_width 320` ·
latence bout en bout < 150 ms · flux annoté ≥ 10 fps. C'est le banc qui tranchera, pas l'intuition.

---

## 9. Dettes et pièges, en clair

1. **Duplication assumée** du code vision entre `tools/robot_controlv3/` et `bamboo_videotracking/`
   (choix explicite : copier dans les paquets ROS). Une correction d'algorithme devra être portée **deux
   fois** — c'est consigné des deux côtés.
2. **Le RPi4 peut ne pas tenir la cible** : détection + reconnaissance + ré-encodage JPEG sur 4 cœurs
   avec `gscam2` en parallèle. Cadence dégradée acceptée ; le banc tranche.
3. **`data/faces/` n'est pas versionné** (visages de personnes réelles) → **volume nommé**
   `bamboo_faces`, hors sauvegarde Git. Un bind-mount sur l'arbre de travail le ferait apparaître dans
   `git status` à chaque acquisition.
4. **`np.savez` doit rester NON compressé** : contrat dur avec le lecteur C++ `gallery.cpp`.
5. **`--symlink-install` hérite du mode du fichier source** là où `install(PROGRAMS)` mettrait 755 en
   copiant → un script d'entrée à 644 donne `executable not found on the libexec directory`. Et un
   fichier installé par `install(PROGRAMS)` puis lancé **directement** n'a **pas de paquet parent** →
   imports **absolus** obligatoires dans les scripts d'entrée.
6. **Pas de `twist_mux`** : dès Nav2, joy et Nav2 publieraient sur `/cmd_vel` sans arbitrage.
7. **Trois modules de contrôleur sur quatre ne sont pas testés** : `Teensy` et `DSGrovePiP` n'ont aucun
   robot monté (leur launch **refuse de démarrer**, délibérément), `WSEsp32` appartient à un chantier en
   pause et n'est qu'une **enveloppe** du driver resté dans `bamboo_base`.
8. **Pièges GrovePi+, si elle rejoint ce robot** : **ne jamais l'enficher sur le header RPi** (RESET sur
   BCM8 → carte bloquée), **UART uniquement sur le port Grove SERIAL**, et son **baud capteurs diffère
   du baud de contrôle** — confusion déjà commise une fois.
9. **Nommage des paquets** : colcon avertit à chaque build que les majuscules
   (`bamboo_controler_YBStm32v3`, `bamboo4WD_V4_*_base`, …) ne suivent pas ses conventions. Choix
   assumé, **conservé**.
