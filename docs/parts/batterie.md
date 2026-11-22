# Pack batterie Li-ion 3S1P (UPS 3 × 18650)

> Fiche de maintenance de la part `batterie`.

## Rôle
Alimente **tout le robot** : la [carte ESP32](part:esp32_board) (7 à 13 V), les moteurs via le [pont en H](part:tb6612fng) et le [RPi](part:rpi). Une panne ici est une cause amont de presque tout le reste.

## Références
| | |
|---|---|
| Type | UPS 3S1P, 3 cellules 18650 |
| Capacité idéale | 2,6 Ah |
| Capacité mesurée | **0,88 Ah** (2026-10-22) : pack usé |
| Résistance interne idéale | 0,15 Ω |

## Seuils de tension
- **Avertissement** sous 10,5 V ; **erreur** sous 9 V ; surtension au-dessus de 13 V.
- Recharger avant toute mission quand la tension passe sous 10,5 V.

## Mesures
La tâche de diagnostic `esp32_mavlink_driver: batterie` publie la tension, le courant, l'état du chargeur (« en charge »), la charge estimée et la tension à vide. Le capteur est l'[INA219](part:ina219).

## Pannes connues
Aucun incident réel enregistré. À surveiller : capacité mesurée très inférieure à la capacité idéale, donc autonomie réelle courte. Le Wi-Fi décroche avant les autres fonctions quand la batterie est basse (vers 10,1 V).

## Remplacement
Pack à remplacer si l'autonomie ne suffit plus (capacité mesurée 0,88 Ah contre 2,6 Ah). Référence exacte du pack : **à relever** sur le matériel.
