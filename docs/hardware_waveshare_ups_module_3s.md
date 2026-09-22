# UPS WS — WaveShare « UPS Module 3S »

Fiche matérielle de référence du module d'alimentation sans coupure (UPS) retenu pour
alimenter le **RPi4** (et le lidar) du robot `bamboo4WD_V4_WSEsp32` en **5 V propre**.

- **Nom constructeur** : WaveShare *UPS Module 3S*.
- **Rôle projet** : source **5 V / 5 A** unique pour le RPi + lidar, prise **en amont** du
  polyfuse USB du RPi → corrige le brown-out USB historique ; masse commune automatique ;
  isolé du bruit moteur/carte ; suivi batterie par I2C ; arrêt propre.
- **Source** : wiki officiel WaveShare *UPS Module 3S*.

> ⚠️ **Charge uniquement par le port DC5521 en 12,6 V / 2 A.** On **ne peut PAS** charger par
> le port 5 V. Utiliser exclusivement le chargeur fourni (ripple maîtrisé).

---

## 1. Caractéristiques générales

| Élément | Valeur |
|---|---|
| Batterie | **3× 18650 Li-ion en série (3S)**, non fournies (~11,1 V nom / 12,6 V pleine) |
| Entrée de charge | **12,6 V / 2 A** (connecteur **DC5521**) |
| Sortie régulée principale | **5 V / 5 A** (continu) |
| Sortie régulée auxiliaire | **3,3 V / 300 mA** |
| Sortie tension batterie | passthrough (~11,1–12,6 V) sur port XH2.54 |
| Monitoring | **INA219** (tension / courant / puissance, **I2C**) |
| Protections | surcharge, décharge profonde, surintensité, court-circuit, inversion + équilibrage |
| Dimensions | **60 × 93 mm**, trous Ø 3,0 mm |
| Isolation | plaque acrylique à monter **obligatoirement** (composants exposés au dos) |

---

## 2. Liste des ports et connecteurs

| Nom | Type | Fonction |
|---|---|---|
| **Entrée charge** | **DC5521** (barrel femelle) | charge **12,6 V / 2 A** (seul port de charge) |
| **Sortie batterie** | **XH2.54** mâle | tension batterie brute 12,6 V / 2 A (sortie) |
| **Sortie 5 V** | **Type-C** mâle | sortie 5 V |
| **Header de sortie** | header broches | 5 V, GND, 3V3, SDA, SCL (voir §3) |
| **Switch** | bouton | interrupteur de sortie (ON/OFF) |
| **BOOT** | bouton | active la puce de protection si pas de sortie au 1ᵉʳ montage batterie |
| **LED charge** | indicateur | rouge = charge, vert = pleine, clignote ≈ presque pleine, éteint = court-circuit possible |
| **LED inversion** | indicateur | allumée = une batterie est montée à l'envers |

---

## 3. Détail des fils par port

### 3.1 Header de sortie (le port utilisé côté projet)

Le header expose les rails de sortie + le bus I2C de l'INA219. Broches confirmées par le wiki
(tableau de raccordement Raspberry Pi) :

| Broche | Signal | Détail |
|---|---|---|
| **5V** | +5 V | sortie régulée 5 A → **même net que la broche 5 V du RPi** |
| **GND** | masse | masse commune (obligatoire : données référencées masse) |
| **3V3** | +3,3 V | sortie auxiliaire 300 mA |
| **SDA** | I2C data | INA219 (monitoring batterie) |
| **SCL** | I2C clock | INA219 |

> L'ordre exact des broches sur le connecteur n'est pas publié broche à broche par le wiki
> (à relever carte en main). L'adresse I2C de l'INA219 n'est pas documentée sur le wiki ;
> le code de démonstration utilise `INA219.py` (scanner le bus : `i2cdetect`).

### 3.2 Raccordement Raspberry Pi (recommandé projet)

| UPS | → | RPi (header 40P) |
|---|---|---|
| 5V | → | 5V (broche 2 ou 4) |
| GND | → | GND (broche 6) |
| SDA | → | SDA1 (GPIO2, broche 3) |
| SCL | → | SCL1 (GPIO3, broche 5) |

- Alimenter le RPi par le rail 5 V du header **tape en amont du polyfuse USB** → règle le
  brown-out. Ne **pas** double-alimenter (ne pas garder aussi l'alim USB-C du RPi).
- **Masse commune** : la broche GND relie automatiquement toutes les masses (UPS, RPi,
  lidar). Indispensable pour toute liaison de données.

### 3.3 Entrée de charge DC5521

- 2 conducteurs : **centre = +12,6 V**, **manchon = GND**.
- Charge **uniquement** ici, avec le chargeur 12,6 V / 2 A fourni.

---

## 4. Intégration robot (architecture retenue)

```
Chargeur 12,6V/2A ──DC5521──▶ [ UPS Module 3S ] ──5V/5A──┬──▶ RPi4 (header 5V/GND)
        3× 18650 (3S) ────────────────────────┘         └──▶ Lidar 5V (+ GND commun)
                                               └──I2C(SDA/SCL)──▶ RPi (batterie %)
```

- Rend **inutiles** le hub 12 V Transcend, le convertisseur buck et la batterie 7,4 V
  envisagés précédemment.
- Isole le RPi/lidar du **bruit moteur** (rail séparé de la carte WSEsp32).
- Permet un **arrêt propre** (le RPi lit le % batterie via INA219 et peut se couper avant
  décharge profonde).

## Sécurité

- Monter la **plaque acrylique isolante** avant usage.
- Vérifier le **sens des batteries** (LED inversion) avant charge ; ne jamais charger une
  batterie inversée.
- Chargeur fourni uniquement ; pas de mélange anciennes/neuves ; remplacer en fin de vie.

## Voir aussi

- [hardware_waveshare_general_driver_board.md](hardware_waveshare_general_driver_board.md) — carte de contrôle WSEsp32.
- [bamboo4WD_V4_WSEsp32.md](bamboo4WD_V4_WSEsp32.md) — intégration ROS2 du robot.
