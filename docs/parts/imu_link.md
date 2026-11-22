# IMU QMI8658C (6 axes)

> Fiche de maintenance de la part `imu_link`.

## Rôle
Gyroscope et accéléromètre de la [carte ESP32](part:esp32_board). Donne l'attitude (roulis, tangage, cap) utilisée par le filtre de la navigation et la compensation de rotation. Le cap vient du gyroscope ; un compas existe sur la carte ([magnétomètre](part:magnetometre)), sans rôle établi dans ce fichier.

## Références
| | |
|---|---|
| Modèle | QMI8658C |
| Position | origine du châssis (pose estimée, non mesurée) |
| Orientation | corrigée côté logiciel (`imu_gz_sign` du fichier canonique) |

## Mesures
La tâche `esp32_mavlink_driver: imu` publie roulis, tangage et cap en degrés, l'âge de la dernière mesure (`imu_age`) et la vitesse de rotation `gz`. Un âge qui grandit signifie que la liaison série est en cause avant l'IMU.

## Action : recalibrer le gyroscope
La carte moyenne le biais pendant environ 2 secondes. **Le robot doit rester strictement immobile** : un mouvement fausse le zéro et le cap dérive ensuite. L'action est refusée si la traction est armée.

## Pannes connues
- **Attitude figée ou en retard** : vérifier d'abord la [liaison série](part:cp2102_esp32) (même cause).
- **Cap qui dérive** : recalibrer le gyroscope, robot posé et immobile.
