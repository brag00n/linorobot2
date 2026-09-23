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

### Suite du 2026-09-23 — T15 à T20 : la cause était mécanique

| № | nature | résultat | état |
|---|---|---|---|
| T15 | 2 m au sol, `odom_calib.py` | odom 2,1182 m / réel corde 191,26 cm (AB 191, BC 10) → **+10,75 %** | valide |
| T16 | poussée main 2 m, fenêtre fixe | comptages `[0,0,0,0]` sur 70 s | **INVALIDE** : fenêtre mal posée par moi, geste hors fenêtre |
| T16b | poussée main, bornes détectées capteurs | 204 tics / 2,4 cm — c'est le **repositionnement** du robot, pas la poussée | **INVALIDE** : `SEUIL_TICS=20` trop sensible, `STABILITE_S=4` trop court |
| T17 | 10 tours main, roues en l'air | rien enregistré | **INVALIDE** : banc déployé mais pas lancé au « go » |
| T18 | résistance au toucher, gains PID 0,6/0,3/0,5 puis **1e-6** | M2 résistait avec **effort croissant**, libre à 1e-6 ; M1/M3/M4 libres dans les deux cas | valide |
| T19 | discriminant encodeur/moteur via M3 témoin | non exécuté (sans objet après T18) | abandonné |
| T20 | comptages bruts pendant T18 phase C | **M1 = 0 tic, M2 = +3733 tics** sur les mêmes gestes | valide |

**T18 — le PID embarqué contre-pilote toute rotation manuelle, en permanence.** `moveBase()`
n'a **pas** de `return` anticipé ([firmware.cpp:322-392](../../../linorobot2_hardware/firmware/esp32_bamboo/src/firmware.cpp#L322-L392)) : la branche « no activity » remet la
consigne à zéro puis l'exécution continue jusqu'à `spin(pid.compute(0, rpm_mesuré))`. Avec
une consigne nulle et une mesure non nulle, l'erreur est non nulle et le moteur s'oppose.
L'effort **croît** parce que l'intégrale n'est remise à zéro que si consigne **et** erreur
sont nulles ([pid.cpp:28-32](../../../linorobot2_hardware/firmware/esp32_bamboo/lib/pid/pid.cpp#L28-L32)). `enable_cmd_vel false` ne protège pas : il ne coupe que
l'abonnement ROS, la boucle 10 Hz de la carte tourne quand même.

→ **T16, T16b et T17 étaient condamnés d'avance.** Tout essai à la main exige d'annuler
les gains d'abord (`ros2 param set … 1e-6` ; `0.0` est refusé par `_POSITIVE_PARAMS`), ce qui
donne `spin(0)` → `brake()` → PWM 0 → roue libre, `USE_SHORT_BRAKE` n'étant défini nulle part.

**Correction d'une affirmation de ce dépôt** : « l'ESP32 met la PWM à zéro, il ne
court-circuite pas le TB6612FNG » est **exacte** ; l'inférence « donc pas de frein actif sans
reflash » est **fausse**, car le PID contre-pilote par `forward()`/`reverse()` avec une PWM
réelle. Les ~0,7 s de roue libre mesurés à T6-T10 se produisent donc **malgré** un freinage
actif ; leur cause est ailleurs (autorité du PID à bas RPM, bande morte `PWM_MIN`, résolution
du RPM en fin de course). À corriger dans `config/esp32_driver.yaml` et
`docs/bamboo4WD_V4_WSEsp32.md`.

**T20 puis inspection physique — la vraie cause.** L'encodeur M1 ne compte pas quand on
tourne la roue à la main, alors que M2 compte sur le même geste. L'odométrie prouvant que M1
comptait pendant T11/T15 (elle intègre `0.5*(v_l+v_r)` sur deux index distincts, donc un M1
muet aurait fait **sous**-estimer de moitié), la piste devenait la liaison roue↔moteur.
Inspection utilisateur : **l'attache de fixation du moteur M1 est cassée.** Le jeu de levier
fait, sous le poids du robot, toucher le disque d'odométrie contre la carcasse et bloque sa
rotation. Pièce à remplacer.

⚠️ **Conséquences, à traiter avant toute reprise :**
1. **T11, T14 et T15 sont suspects** : la casse n'est pas datée, et elle ne fausse la voie M1
   qu'**au sol**, c'est-à-dire exactement dans ces trois essais. Les +10 % ne sont plus un
   résultat acquis. **Rien n'a été écrit dans le fichier canonique** — la correction de
   diamètre / d'échelle d'odométrie restait gelée, et c'est ce qui nous évite d'avoir gravé
   un artefact.
2. **T14 (−25° de cap)** redevient explicable : une roue avant gauche qui traîne ou se bloque
   par intermittence tire le robot. Candidat, pas conclusion.
3. **T12 survit** : il compare l'odom intégrée aux comptages bruts, deux chemins nourris par
   les **mêmes** encodeurs — un blocage de M1 affecte les deux identiquement.
4. **`cpr = 2100` survit aussi** : mesuré au tachymètre optique, roues en l'air.
5. **Pas de roulage au sol avant réparation** : c'est la charge qui crée le blocage.

**Après réparation, dans cet ordre** : T21 = 10 tours main par roue, gains à 1e-6, roues en
l'air → `cpr` exact des deux voies et preuve que M1 compte à la main ; T22 = reprise des 2 m
au sol sur une base saine ; puis seulement décider où placer une éventuelle correction.

---

## 2026-09-23 — PHASE EN PAUSE : attente de pièces mécaniques

**Le lot 4.4 (calibration au sol) est suspendu**, décision utilisateur, en attente d'une
**attache de moteur N20** de remplacement pour M1. Ce n'est pas un blocage logiciel : le
diagnostic est clos, la cause est l'attache cassée qui laisse le disque d'odométrie toucher la
carcasse sous la charge.

**État du robot au moment de la pause**
- M1 démonté / attache cassée. **Ne pas faire rouler au sol** : c'est le poids qui crée le
  blocage, et tout essai au sol produirait des chiffres faux.
- Gains PID laissés à **1e-6 en SRAM** par T18. Sans effet durable : aucune persistance sur
  ESP32 (`MAV_CMD_PREFLIGHT_STORAGE` renvoie `UNSUPPORTED`), et le driver repousse
  0,600 / 0,300 / 0,500 depuis le fichier canonique à chaque connexion. **Vérifier la relecture
  carte (`board_pid_kp/ki/kd`) au redémarrage** avant tout essai motorisé.
- `enable_cmd_vel` toujours **false**. RPi éteint par l'utilisateur.
- Bancs déployés dans le conteneur, à `/tmp/push_odom.py` et `/tmp/cpr_manual.py` : volatils,
  à redéployer après reboot.

**Rien n'a été écrit dans le fichier canonique.** `counts_per_rev`, `wheel_diameter_m` et
`wheel_separation_m` restent aux valeurs d'avant les essais au sol. Aucune correction de
+10 % n'a été gravée, et c'est volontaire.

**`cpr = 2100` est désormais une valeur déduite, plus une mesure.** Le moteur étant un N20 et
l'encodeur lu en demi-quadrature ([encoder.h:56](../../../linorobot2_hardware/firmware/esp32_bamboo/lib/encoder/encoder.h#L56)) :
`7 impulsions/tour × 2 × 150 (réducteur) = 2100`. Les ±4 % de dispersion du tachymètre optique
étaient du bruit autour d'un entier exact. **À confirmer sur les marquages du moteur démonté**
(un réducteur 1:75 donnerait 1050 et invaliderait tout le calcul d'odométrie).

**À la reprise, dans cet ordre**
1. Remonter M1, vérifier que le disque ne touche plus, et **contrôler les trois autres
   attaches** — même pièce, même âge.
2. Relever les marquages moteur/encodeur → confirmer ou corriger `cpr = 2100`.
3. **T21** — gains à 1e-6, roues en l'air, 10 tours main par roue : `cpr` exact des deux voies,
   et preuve que M1 compte à la main (ce qu'il ne faisait pas à T20).
4. Restaurer les gains, roues à l'arrêt, avec relecture carte.
5. **T22** — reprise des 2 m au sol sur base saine. T11, T14 et T15 ne sont **pas** rejouables :
   ils restent au journal comme suspects, sans être renumérotés.
6. Alors seulement, décider où placer une éventuelle correction d'échelle — et **pas** dans
   `wheel_diameter_m`, qui alimente aussi l'URDF/TF et la cinématique de la carte.
7. Reste ouvert et indépendant : la seconde moitié du lot 4.4, la validation de
   `wheel_separation_m` (0,125 m, toujours `<< A CONFIRMER >>`) par un 360°.
