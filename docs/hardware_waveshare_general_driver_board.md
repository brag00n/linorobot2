# Carte WSEsp32 — WaveShare « General Driver for Robots » (Rev1.2)

Fiche matérielle de référence de la **carte de contrôle** du robot `bamboo4WD_V4_WSEsp32`.

- **Nom constructeur** : WaveShare *General Driver for Robots*, révision **Rev1.2**.
- **Contrôleur** : **ESP32-WROOM-32UE** (Wi-Fi + BLE + ESP-NOW, connecteur d'antenne IPEX1).
- **Rôle projet** : pont télémétrie/actionnement MAVLink vers le RPi4 (voir
  [bamboo4WD_V4_WSEsp32.md](bamboo4WD_V4_WSEsp32.md)). Dev firmware sous Arduino/PlatformIO.
- **Sources** : wiki officiel WaveShare *General Driver for Robots* + lecture du schéma
  officiel Rev1.2 (sept. 2026). Les pinouts marqués **(schéma)** proviennent du schéma, pas
  du wiki (qui ne publie pas le détail broche à broche).

> ⚠️ Deux points structurants pour ce robot, hérités du choix TB6612FNG :
> - **4 moteurs entraînés mais 2 voies d'encodeur seulement** (1 par côté). L'odométrie n'a
>   donc que 2 mesures réelles (gauche/droite).
> - **2× CP2102 avec le même VID:PID `10C4:EA60`** → désambiguïsation par n° de série (udev),
>   pas par VID:PID.

---

## 1. Caractéristiques générales

| Élément | Valeur |
|---|---|
| Contrôleur | ESP32-WROOM-32UE (Wi-Fi 2.4 GHz, Bluetooth, ESP-NOW) |
| Antenne | connecteur **IPEX1** (n° 2), antenne externe |
| Alimentation d'entrée | **DC 7–13 V** (compatible batterie **2S / 3S** Li-ion) |
| Régulateur 5 V | **DC-DC MP8759**, rail principal `NL5V`, **5 V / 5 A** (schéma) |
| Régulateur 3,3 V | **AMS1117-3.3** (schéma) |
| Pont moteur | **TB6612FNG** (double pont-H, 2 canaux → A et B) |
| Servos série | bus **ST3215** (半duplex série, jusqu'à 253 servos, ~5 A max) |
| Monitoring alim | **INA219** (tension **ET** courant, I2C) |
| IMU | **QMI8658** (6 axes) + **AK09918C** (magnétomètre 3 axes) = « 9 axes » |
| Interfaces USB | **2× CP2102** (`10C4:EA60`) : un ESP32-UART, un données LIDAR |
| Extension hôte | **header 40 broches** (Raspberry Pi / Jetson / Sunrise X3) |
| Stockage | slot **carte TF/SD** |
| Circuit de flash | **auto-download** (EN/BOOT automatiques, pas de manip bouton) |
| Dimensions | **65 × 65 mm**, trous de fixation 49 × 58 mm (Ø 3 mm) |
| Interface de flash | Type-C |

---

## 2. Liste des ports et connecteurs (repères silkscreen)

Numéros = repères de la nomenclature « onboard resource » du wiki.

| N° | Nom / fonction | Type connecteur |
|---|---|---|
| 2 | Connecteur antenne Wi-Fi | IPEX1 |
| 3 | Interface **LIDAR** (adaptateur radar) | PH2.0 4P (= **H7**, schéma) |
| 4 | Extension **I2C** (OLED / capteurs) | header I2C |
| 5 | Bouton **Reset** (redémarre l'ESP32) | bouton |
| 6 | Bouton **Download** (mode flash au boot) | bouton |
| 7 | Circuit régulateur **5 V** (alim hôte RPi/Jetson) | — |
| 8 | Connecteur **Type-C « LIDAR »** (données lidar → hôte) | USB Type-C |
| 9 | Connecteur **Type-C « USB »** (UART ESP32, upload) | USB Type-C |
| 10 | **Entrée alimentation** DC 7–13 V | **XH2.54** |
| 11 | INA219 (monitoring V + I) | puce I2C |
| 12 | Interrupteur **Power ON/OFF** | switch |
| 13 | Interface **servo bus ST3215** | connecteur série servo |
| 14 | **Moteur groupe B**, sans encodeur | PH2.0 **2P** |
| 15 | **Moteur groupe A**, avec encodeur | PH2.0 **6P** |
| 16 | **Moteur groupe A**, sans encodeur | PH2.0 **2P** |
| 17 | **Moteur groupe B**, avec encodeur | PH2.0 **6P** |
| 18 | AK09918C (compas 3 axes) | puce I2C |
| 19 | QMI8658 (IMU 6 axes) | puce I2C |
| 20 | TB6612FNG (pilote moteurs) | puce |
| 21 | Circuit de contrôle servo série | — |
| 22 | Slot **carte TF/SD** (logs, config Wi-Fi) | micro-SD |
| 23 | **Header 40 broches** (extension RPi / Sunrise X3) | 2×20 2,54 mm |
| 24 | Header 40 broches (accès broches hôte) | 2×20 2,54 mm |
| 25 | CP2102 — UART↔USB **données radar/LIDAR** | puce |
| 26 | CP2102 — UART↔USB **communication ESP32** | puce |
| 27 | Circuit **auto-download** (EN/BOOT auto) | — |

> ℹ️ La carte **ne gère pas** les servos PWM MG996R / MG90S ; servo PWM recommandé = WP90.
> Le pilotage servo se fait via le **bus série ST3215** (n° 13), hors périmètre initial du
> projet (`DO_SET_SERVO` = UNSUPPORTED côté firmware MAVLink).

---

## 3. Détail des fils par port

### 3.1 Entrée alimentation — n° 10 (XH2.54)

- **DC 7–13 V** : 2 fils **V+ / GND**.
- Alimente directement le **bus servo ST3215** et les **moteurs** (non régulé), et via
  MP8759 le rail **5 V** (hôte) + AMS1117 le **3,3 V** (logique).
- Commuté par l'interrupteur **Power ON/OFF** (n° 12).

### 3.2 Moteurs avec encodeur — n° 15 (groupe A) & 17 (groupe B) — PH2.0 6P

Ordre des 6 fils (schéma, à confirmer visuellement carte en main) :

| Broche | Signal | Rôle |
|---|---|---|
| 1 | **M+** | phase moteur A |
| 2 | **M−** | phase moteur B |
| 3 | **VCC** | alim encodeur (3,3 V) |
| 4 | **GND** | masse encodeur |
| 5 | **C1** | voie encodeur A |
| 6 | **C2** | voie encodeur B |

### 3.3 Moteurs sans encodeur — n° 14 (groupe B) & 16 (groupe A) — PH2.0 2P

- 2 fils **M+ / M−** uniquement (phases moteur).
- **Câblés en parallèle** sur le même canal TB6612 que le moteur à encodeur du même groupe →
  suivent la consigne du moteur voisin, **pas d'odométrie propre**.
- Conséquence : différentiel 4 moteurs (2 par côté) mais **2 encodeurs** (G/D) seulement.

### 3.4 Interface LIDAR — n° 3 = connecteur **H7** (PH2.0 4P) *(schéma)*

| Broche | Signal | Détail |
|---|---|---|
| 1 | **GND** | masse |
| 2 | **5V** | rail principal `NL5V` (MP8759 5 V-5 A), **toujours actif** (EN tiré au + par R4 1,5 MΩ, non commuté GPIO) |
| 3 | **GND** | masse |
| 4 | **CP_RX** | data lidar → carte, via **R15 1 kΩ** série → **RXD du CP2102 U3** |

- Le CP2102 U3 (n° 25) **n'utilise que RXD** (pas de TX) → chaîne mono-directionnelle
  `H7.CP_RX → CP2102 U3 → Type-C « LIDAR » (n° 8) → USB hôte`.
- **Pas de fusible par port** : limite partagée = 5 A interne du DC-DC. RPi ~2–3 A + lidar
  ~0,4 A → marge OK.
- Convient parfaitement à un **lidar mono-canal auto-tournant** (YDLidar X2 : data sortante
  seule). Câbler **par fonction** (le connecteur du lidar ≠ H7) : `lidar VCC→5V`, `GND→GND`,
  `Tx→CP_RX`. Data en **TTL 3,3 V**. Voir [bamboo4WD_V4_WSEsp32.md](bamboo4WD_V4_WSEsp32.md).

### 3.5 Type-C « LIDAR » — n° 8 & Type-C « USB » — n° 9

- **n° 9 « USB »** : UART de l'ESP32 (via CP2102 n° 26) → **upload firmware + comm** ; c'est
  le port série exposé au RPi pour le pont MAVLink. Auto-download (EN/BOOT) actif → DTR/RTS
  doivent rester **désassertés** à l'ouverture (sinon reset ESP32).
- **n° 8 « LIDAR »** : sortie des données lidar (via CP2102 n° 25). Ports USB **device** vers
  le RPi. Les deux CP2102 = même `10C4:EA60` → distinguer par **n° de série** (règle udev).

### 3.6 Servo bus ST3215 — n° 13

- Connecteur série demi-duplex : **VIN (7–13 V non régulé) / GND / Signal série**.
- Jusqu'à 253 servos chaînés ; ~5 A max total (≈ 5 servos typiques).

### 3.7 Extension I2C — n° 4

- **VCC (3,3 V) / GND / SDA / SCL**. Bus partagé avec INA219 (n° 11), QMI8658 (n° 19),
  AK09918C (n° 18). Sert aux OLED ou capteurs additionnels.

### 3.8 Header 40 broches — n° 23/24

- Brochage **compatible Raspberry Pi** (2×20, 2,54 mm) : 5 V, 3,3 V, GND, GPIO, I2C, SPI,
  UART. Fournit le **5 V** au RPi depuis le rail `NL5V` (même net que le lidar). Reçoit le RPi
  ou une carte hôte compatible (Jetson Nano, Sunrise X3 Pi).

---

## 4. Chaînes fonctionnelles utiles au projet

- **Rail 5 V** : `XH2.54 (7–13 V) → MP8759 → NL5V (5 V/5 A)` alimente : header 40P (RPi),
  H7 (lidar), 2× CP2102, AMS1117 (3,3 V). Toujours actif dès mise sous tension.
- **Odométrie** : seuls C1/C2 des connecteurs 6P (n° 15/17) remontent → **2 voies G/D**.
- **Série ESP32 ↔ RPi** : Type-C « USB » (n° 9) → CP2102 (n° 26) → ESP32. Auto-download →
  DTR/RTS=False à l'ouverture.

## Voir aussi

- [bamboo4WD_V4_WSEsp32.md](bamboo4WD_V4_WSEsp32.md) — intégration ROS2 / linorobot2 du robot.
- [hardware_waveshare_ups_module_3s.md](hardware_waveshare_ups_module_3s.md) — alim 5 V du RPi/lidar.
