# Journal des essais BambooWS

Un essai = une ligne, un **numéro** définitif. On ne renumérote jamais : un essai
invalidé garde son numéro et le dit. Règle d'exploitation : **aucun déplacement du
robot sans go explicite de l'utilisateur**.

Colonne « réel » = mesure physique de l'utilisateur (ruban, pied à coulisse) ; les
autres colonnes sont lues sur le graphe ROS.

## 2026-09-23 — Lot 4.3, critère « suit le PID » (roues SURÉLEVÉES)

Banc `pid_step.py`, gains d'origine `kp/ki/kd = 0,600 / 0,300 / 0,500`, cpr 2100.

| № | essai | consigne | `t_conv(±10 %)` M1/M2 | régime M1/M2 | verdict |
|---|---|---|---|---|---|
| T1 | `pid_step 0.10 3` | 23,87 RPM | 1,00 / 1,73 s | 24,03 / 23,85 | **trompeur** — fenêtre trop courte, cf. T4 |
| T2 | `pid_step 0.15 3` | 35,81 RPM | 0,67 / 0,57 s | 34,91 (−2,5 %) / 35,00 (−2,3 %) | OK |
| T3 | `pid_step 0.25 3` | 59,68 RPM | 0,67 / 0,67 s | 59,62 (−0,1 %) / 59,63 (−0,1 %) | OK |
| T4 | `pid_step 0.10 5` | 23,87 RPM | 0,60 / 0,67 s | 24,03 (+0,6 %) / 23,85 (−0,1 %) | OK — invalide la lecture de T1 |

**Conclusion** : critère tenu (< 1 s, ≤ 2,5 % d'erreur), **aucun réglage de gains
nécessaire**. T1 contre T4 fixe la règle de mesure : sous 0,15 m/s, marche de 5 s
minimum, sinon une excursion de bruit tardive se lit comme une rampe lente.

## 2026-09-23 — Lot 4.2, freinage (robot AU SOL)

Banc `brake_stop.py`, `linear.x = 0,15` → ~35 RPM à la coupure.

| № | chemin d'arrêt | `cmd_vel_timeout_s` | t arrêt | distance |
|---|---|---|---|---|
| T5 | `Twist(0,0)` explicite | — | 0,80 s | non mesurée (banc sans distance) |
| T6 | flux coupé | 0,50 s | 1,10 s | non mesurée |
| T7 | `Twist(0,0)` explicite | — | 0,77 s | **3,9 cm** |
| T8 | flux coupé | 0,50 s | 1,14 s | 10,5 cm |
| T9 | flux coupé | 0,15 s | 0,70 s | 4,7 cm |
| T10 | flux coupé | **0,25 s** | 1,13 s | **6,8 cm** |

**Conclusion** : critère « < 0,5 s » **non tenable sans reflash** — le plancher est la
roue libre (~0,7 s), l'ESP32 met la PWM à zéro sans court-circuiter le TB6612FNG.
Retenu : `cmd_vel_timeout_s = 0,25 s` (6,8 cm, et 6 trames `/joy` manquées tolérées
à 25 Hz). La distance est la grandeur honnête, pas le temps.

## 2026-09-23 — Lot 4.4, calibration au sol

Banc `odom_calib.py` (`ligne`), cible 2,00 m à 0,15 m/s. Diamètre de roue **confirmé
à 80 mm au pied à coulisse** par l'utilisateur.

| № | odom | réel (ruban) | rotation parasite | état |
|---|---|---|---|---|
| T11 | 2,1116 m | **193,05 cm** | −3,85° | valide |
| T12 | — (diagnostic `odom_vs_counts 0.15 3`) | — | — | ratio odom/comptages = **1,0079** |
| T13 | 2,1198 m | — | −2,93° | **INVALIDE** : robot déplacé à la main avant mesure |
| T14 | 2,1274 m | à confirmer | −24,98° | mesure réelle à rattacher |

**En cours d'analyse.** T12 est le résultat décisif : l'odométrie intégrée en vitesse
ne s'écarte que de +0,8 % des comptages d'encodeur bruts, donc le calcul est sain. Avec
le diamètre confirmé à 80 mm, l'écart T11 (odom 211 cm pour 193 cm réels, +9,4 %) n'est
**ni** un défaut de calcul **ni** un défaut de géométrie : les roues tournent bien
l'équivalent de 211 cm, le châssis n'avance que de 193 cm. Reste à trancher entre
glissement et écrasement de pneu — et à confirmer la reproductibilité.

⚠️ La rotation parasite de T14 (−25°) est d'un autre ordre que T11/T13 (−3°) : à
expliquer avant d'exploiter T14 comme mesure de distance.
