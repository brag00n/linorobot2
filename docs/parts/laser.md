# Lidar LDROBOT STL-19P (LD19, Waveshare D500)

> Fiche de maintenance de la part `laser`. Les valeurs viennent de `parts.yaml` ; les pistes de résolution de l'incident en cours s'affichent **sous** cette fiche.

## Rôle
Mesure les distances autour du robot (360°, portée annoncée 12 m). Il alimente `/scan`, donc la cartographie, la localisation et l'évitement d'obstacles de la pile de navigation. **Sans lidar, la navigation est déclarée impossible** : elle refuse de démarrer plutôt que de rouler à l'aveugle.

## Références
| | |
|---|---|
| Fabricant | LDROBOT |
| Modèle | STL-19P (famille LD19), vendu par Waveshare sous le nom D500 |
| Liaison | série 230400 bauds, via le pont USB [adaptateur du lidar](part:cp2102_lidar) |
| Pose | 4 cm en avant, 17 cm de haut, rotation de +90° (mesurée) |

## Dépendances
- Dépend de l'[adaptateur USB-série du lidar](part:cp2102_lidar), lui-même derrière le [hub USB](part:hub_usb) du [RPi](part:rpi).
- Entre dans la [pile de navigation](part:navigation).

## Démarrage à la demande
Le lidar **ne démarre pas au boot**. Il est lancé avec la pile de navigation (mode NAV). Un lidar arrêté parce que la pile est éteinte n'est **pas une panne** : le diagnostic dit alors « arrêté », pas « aucun scan reçu ».

## Pannes connues
- **INC-2026-11-18** : lidar débranché, `/dev/lidar` absent, deux erreurs de diagnostic pour une seule panne. Le pilote **ne reprend pas seul** après un rebranchement : il faut relancer `driver.lidarD500`.
- Après la relance, **relocaliser** (R long à la manette) : la localisation passe à « inconnue » quand le scan disparaît.

## À ne pas faire
- Ne **jamais** réinitialiser un port USB par logiciel : le hub est partagé avec la carte ESP32.
- Ne pas chercher le défaut dans la navigation quand `scan_monitor` est en erreur : la cause est ici.

Voir aussi le manuel de dépannage, incident INC-2026-11-18.
