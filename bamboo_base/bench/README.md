# bench/ — bancs de mesure du PID embarqué (roues **surélevées**)

Deux scripts rclpy jetables-mais-versionnés, qui servent le lot 4.3 du chantier
BambooWS : régler le PID de la carte sans reflash. Ils ne sont **pas** des nœuds du
paquet (pas d'entry point, pas de launch) : on les exécute à la main, le temps d'un
essai, et c'est voulu — un banc qui démarre tout seul est un banc qui actionne les
moteurs sans qu'on l'ait demandé.

Ils mesurent en lisant les **deux** topics ajoutés pour ça :
`/req_states` (consigne vue par le PID) et `/joint_states` (mesure vue par le PID),
tous deux en RPM. Rien n'est recalculé côté hôte.

## Ce que chacun fait

| Script | Rôle |
|---|---|
| `pid_trace.py` | trace brute consigne/mesure, ligne par ligne — pour *regarder* |
| `pid_step.py` | échelon + analyse chiffrée : `t_conv` à ± tol %, erreur de régime, écart-type, crête-à-crête, indice d'oscillation — pour *décider* |

`pid_step.py` dédoublonne les échantillons répétés : le driver publie à ~30 Hz mais la
carte n'émet qu'à 10 Hz, donc trois lignes identiques sont **une** mesure. Les compter
trois fois gonflerait l'indice d'oscillation et écraserait l'écart-type.

## Usage

Les deux tournent **dans le conteneur** `driver.real`, qui est le seul à tenir le port
série et à avoir l'overlay ROS :

```bash
# depuis le RPi
sudo docker cp bamboo_base/bench/pid_step.py bamboov4_humble-driver.real-1:/tmp/
sudo docker exec bamboov4_humble-driver.real-1 bash -lc \
  'source /root/linorobot2_ws/install/setup.bash && python3 /tmp/pid_step.py 0.15 5.0 10'
# arguments : linear.x (m/s), durée de l'échelon (s), tolérance (%)
```

## Sécurité, dans cet ordre

1. **Roues surélevées** — ces scripts publient sur `/cmd_vel` pour de vrai.
2. `enable_cmd_vel` est à `false` par défaut : il faut l'armer explicitement
   (`ros2 param set /esp32_mavlink_driver enable_cmd_vel true`) et le **désarmer après**.
3. Chaque script publie une consigne nulle avant et après l'échelon, et garantit le STOP
   dans un `finally`. La consigne nulle d'avant n'est pas décorative : l'intégrale du PID
   embarqué n'est remise à zéro que si consigne **et** erreur sont nulles, donc deux essais
   enchaînés sans ce passage par zéro traînent le résidu du précédent.
