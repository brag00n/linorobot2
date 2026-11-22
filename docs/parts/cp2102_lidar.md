# Adaptateur USB-série CP2102 du lidar

> Fiche de maintenance de la part `cp2102_lidar`.

## Rôle
Pont USB vers série entre le [hub USB](part:hub_usb) du RPi et le [lidar](part:laser). Il apparaît sous `/dev/lidar` (règle udev du dépôt).

## Références
| | |
|---|---|
| Modèle | CP2102 |
| Identifiant USB | `10c4:ea60` |
| Numéro de série | `0001` (numéro générique des CP2102 bon marché) |

Le numéro de série est le même sur beaucoup d'adaptateurs de ce type. La règle udev distingue le lidar de l'ESP32 par ce numéro : **si l'adaptateur est remplacé, relever le nouveau numéro** (`udevadm info`) et mettre à jour `99-bambooWS.rules`.

## Vérifier
- `ls /dev/lidar` : doit exister.
- `lsusb | grep 10c4:ea60` : **deux** lignes attendues (le lidar et l'[ESP32](part:cp2102_esp32)). Une seule : le lidar n'est pas vu par le hub.

## Pannes connues
- **INC-2026-11-18** : adaptateur débranché, `/dev/lidar` absent. Revient en quelques secondes au rebranchement.

## À ne pas faire
Pas de réinitialisation logicielle du hub : il est partagé avec l'ESP32 et couperait aussi la liaison avec la carte.
