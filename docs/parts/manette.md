# Manette Bluetooth Under Control 2919

> Fiche de maintenance de la part `manette`.

## Rôle
Pilote le robot et, avec le **bouton B**, arme et désarme la traction. C'est aussi l'arrêt d'urgence de la navigation (B court : arrêt et annulation du but).

## Références
| | |
|---|---|
| Modèle | Under Control 2919 (tierce, de type Switch) |
| Mode | XInput (appairage : Home + X) |
| Liaison | Bluetooth, via l'[adaptateur Bluetooth](part:adaptateur_bluetooth) du [RPi](part:rpi) |

## Dépendances
Dépend de l'[adaptateur Bluetooth](part:adaptateur_bluetooth). Sans manette, le CC peut piloter la traction (appui maintenu), mais l'arrêt d'urgence passe alors par Échap dans le CC.

## Pannes connues
- **Manette absente** : éteinte, hors de portée, ou non appairée. Elle est **liée** : une fois rallumée, elle se **réattache seule**. Aucune action n'est nécessaire côté robot.

## Dans la vue 3D
La manette n'a pas de pièce sur le robot : son état est affiché **sur l'adaptateur Bluetooth**.
