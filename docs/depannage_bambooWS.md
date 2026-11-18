# Manuel de dépannage de BambooWS (`bamboo4WD_V4_WSEsp32`)

Ce manuel dit **comment reconnaître une panne, remonter à sa cause, la corriger et vérifier**. Il s'appuie sur la nomenclature des parts
(`bamboo4WD_V4_WSEsp32_base/config/parts.yaml`, voir `bamboo4WD_V4_WSEsp32.md` §6.15), qui donne pour chaque part son parent, ses dépendances et ses tâches de diagnostic.
Il se complète **à chaque incident réel** : une panne n'entre ici que si elle a été vue et résolue, avec ce qui a marché et ce qui n'a pas marché.

## 1. Méthode (valable pour toute panne)

1. **Lire l'arbre de diagnostic** (CC : Vitaux › Santé ; MCP : `diagnostics` avec `problems_only`). Noter **toutes** les tâches en erreur ou en alerte, pas seulement la première.
2. **Chercher la cause racine, pas le symptôme.** Plusieurs erreurs simultanées ont souvent **une seule** cause en amont. Dans `parts.yaml`, suivre `depends_on` vers l'amont : la racine est la part la plus
   en amont qui est elle-même en défaut. La webcam gelée du 2026-11-14 montrait quatre services en erreur pour une seule webcam ; le lidar absent du 2026-11-18 montrait deux erreurs (`scan_monitor`, `nav_supervisor`) pour un seul câble.
3. **Chercher le matériel avant le logiciel** quand la cause est « absent » ou « muet » : `lsusb`, `ls /dev/<nom>`, `dmesg -T | tail`. Le diagnostic mesure des *symptômes logiciels* ; il ne voit pas un câble.
4. **Ne pas réinitialiser un port USB par logiciel** : le hub interne du RPi est **partagé** par l'ESP32, le lidar et la webcam. Un reset couperait le pilote moteur. Un débranchement/rebranchement à la main ne touche que la pièce visée.
5. **Garder la traction désarmée** pendant tout dépannage (B long à la manette, ou Échap au CC). Ne redémarrer `bamboo_control`, `driver.esp32` ou `robotdesc` qu'à traction désarmée.
6. **Vérifier la correction avec la même mesure que celle qui a révélé la panne**, et regarder aussi ce qui dépend de la part réparée (colonne « impact »).
7. **Noter l'incident** dans la section 3 de ce manuel (symptôme, cause, résolution, ce qui ne s'est pas passé comme prévu).

## 2. Commandes de base

| Besoin | Commande (depuis le PC) |
|---|---|
| État des services | MCP `ros2-orchestrator` : `services`, `status` |
| Journal d'un service | MCP `logs` (nom du service) ; ou `ssh … docker logs --tail 50 bamboov4_humble-<service>-1` |
| Redémarrer un service | MCP `restart` (avec `recreate=true` pour recréer le conteneur) |
| Périphériques USB | `ssh … 'lsusb; ls -l /dev/esp32 /dev/lidar /dev/bamboocam'` |
| Messages noyau | `ssh … 'dmesg -T | tail -40'` |
| Traction armée ? | MCP `echo` sur `/teleop/state` : champs `armed` et `armed_latched` |
| Arbre de santé | MCP `diagnostics` (`problems_only=true`, `values=true` pour le détail) |

Clé SSH du robot : `~/.ssh/id_rpi_grovepi` (nom non standard : la passer avec `-i`).

## 3. Journal des incidents résolus

### INC-2026-11-18 : lidar introuvable (`/dev/lidar` absent)

| | |
|---|---|
| **Date** | 2026-11-18 |
| **Part racine** | `cp2102_lidar` (adaptateur USB-série du lidar), sous `laser` |
| **Impact** | `laser` (plus de `/scan`), puis `navigation` (impossible sans lidar). Aucun effet sur l'ESP32, la vidéo, le suivi ni la manette |
| **Cause réelle** | le lidar était **débranché** (essai volontaire). Le bus USB ne montrait plus qu'un seul CP210x (l'ESP32) |

**Symptômes, dans l'ordre où ils sont apparus**

1. `ls /dev/lidar` : « No such file or directory » ; `lsusb` : un seul `10c4:ea60` au lieu de deux.
2. Journal du conteneur `driver.lidarD500` : « serial is not open » puis « ldlidar node start is fail » puis **« process has died, exit code 1 »**. Le conteneur reste pourtant « running ».
3. `/scan` : « Unknown topic », aucun publieur.
4. Diagnostic : `scan_monitor: lidar` en **ERROR** (« aucun scan reçu (driver arrêté ? lidar débranché ?) ») et `nav_supervisor: navigation` en **ERROR** (« navigation impossible : lidar (/scan muet) »). Sous-mode `idle`, « PAS prêt ».
5. La pile NAV continue de **consommer ~65 % d'un cœur** pour rien : slam_toolbox et Nav2 attendent un `/scan` qui ne vient pas.

**Ce qui n'a PAS été touché** : le pilote ESP32 (lui aussi en CP2102 sur le même hub), la webcam, la traction (restée désarmée). La navigation a **refusé** de démarrer sans lidar au lieu de rouler à l'aveugle : c'est le comportement voulu.

**Résolution**

1. **Rebrancher le lidar.** `/dev/lidar` réapparaît en quelques secondes (`-> ttyUSB1`, 2 CP210x sur le bus).
2. **Constater que le driver ne reprend PAS tout seul.** Le nœud `ldlidar_stl_ros2_node` est mort et le conteneur n'a **aucune politique de redémarrage** (`restart: no`, 0 redémarrage) : `/scan` reste sans publieur même après le rebranchement. *C'est le piège de cet incident.*
3. **Redémarrer le service à la main** : MCP `restart driver.lidarD500` (ou `docker restart bamboov4_humble-driver.lidarD500-1`). Pas besoin de `recreate`.
4. **Vérifier** : `/scan` à ~9,8 Hz, 460 points valides, portée 0,22 à 3,3 m ; journal « Actual BaudRate reported:230400 » ; `scan_monitor` et `nav_supervisor` repassent à OK.
5. **Relancer la localisation** : sous-mode `navigation` puis `relocalize` (R long à la manette). Relocalisé en 2,3 s sur la carte (score 0,70, accord scan/carte 0,51).

**Durée de la panne** : environ 4 minutes (débranché → rebranché → driver relancé → scan rétabli).

**Leçons**

- **Le diagnostic distingue bien « lidar absent » de « pile arrêtée à la demande »** : le message est différent (« aucun scan reçu » contre « arrêté, pile NAV éteinte »). Ne pas confondre.
- **Pas de redémarrage automatique du driver lidar** : volontaire (service à la demande), mais cela veut dire qu'un câble qui se débranche puis se rebranche **exige une relance manuelle**. Si l'on veut une reprise automatique, il faut un `restart: on-failure` pour ce service, ou que le nœud réessaie l'ouverture du port ; ce n'est pas fait.
- **Les deux erreurs côté diagnostic sont une seule panne.** Ne pas chercher un défaut dans la navigation : tout vient du lidar.
- **Après relance du lidar, il faut relocaliser** : la localisation passe à « inconnue » quand le scan disparaît.

**Fiche de résolution résumée**

```
SYMPTOME  scan_monitor + nav_supervisor en ERROR, /scan muet
1. ls /dev/lidar                       (absent ?)   -> cable USB / adaptateur CP2102 (serie 0001)
2. lsusb | grep 10c4:ea60              (1 seul ?)   -> le lidar n'est pas vu par le hub
3. brancher / rebrancher le lidar      (pas de reset logiciel du hub : partage avec l'ESP32)
4. restart driver.lidarD500            (le driver ne reprend pas seul)
5. verifier /scan ~10 Hz ; relocaliser (R long)
```

### INC-2026-11-14 : webcam gelée (plus aucune image)

| | |
|---|---|
| **Date** | 2026-11-14 |
| **Part racine** | `camera_link` (webcam USB ARC International 05a3:9331) |
| **Impact** | `bamboo_video`, `camera_h264`, `bamboo_videotracking` (quatre services en erreur pour une seule webcam), plus la vidéo du CC et son garde-fou |
| **Cause réelle** | firmware de la webcam **figé** : toujours visible sur le bus, mais ne répond plus aux requêtes |

**Symptômes** : `/video/raw/compressed` à 0 message ; `gscam` « Failed to pause stream » (FATAL) ; ffmpeg « Invalid data found » ; noyau : `uvcvideo … Failed to initialize the device (-5)`, `GET_DEF UVC control 12 … -32`, délais (`-110`).
La vidéo du CC était noire pendant 21 h sans que le garde-fou n'agisse (traction désarmée).

**Fausse piste écartée** : la décimation vidéo à 15 img/s ne l'expliquait pas (config revenue à l'origine, même échec).

**Résolution** : débrancher puis **rebrancher la webcam à la main** ; `dmesg` montre la ré-énumération (« Found UVC 1.00 device HD Web Camera ») ; recréer `bamboo_video` ; `/video/raw/compressed` repart (21,8 Hz au premier relevé).
**Ne pas** réinitialiser le port USB par logiciel (hub partagé).

**Leçon** : le diagnostic désignait `gscam_node` comme « arrêté » alors qu'il tournait ; la cause était matérielle et n'est visible que par `dmesg` et `lsusb`.

## 4. Pannes connues sans incident réel (à compléter quand elles arrivent)

Ces pistes sont écrites dans `parts.yaml` (champ `resolution`). Elles viennent de la lecture du code et de la documentation, **pas** d'un dépannage réel : à tenir pour des hypothèses jusqu'à confirmation.

| Part | Symptôme | Piste |
|---|---|---|
| `cp2102_esp32` | tâche « liaison série » en erreur | contrôler le **câble** USB avant de condamner le CP2102 (un moteur qui tourne alimente le VBUS sans D+/D−) |
| `batterie` | tension sous 10,5 V (alerte) ou 9 V (erreur) | recharger ; capacité mesurée 0,88 Ah contre 2,6 Ah idéal : pack usé |
| `motor_fl` (M1) | encodeur M1 muet | contrôler le jeu entre disque d'odométrie et carcasse (cause de la panne de septembre 2026, réparée le 2026-10-03) |
| `rpi` | sous-tension ou CPU saturée | alimentation 5 V ; couper le suivi (ZL + ZR) ou la pile NAV |

## 5. Comment ajouter un incident

Copier le modèle de la section 3 : date, part racine, impact, cause réelle, symptômes **dans l'ordre**, ce qui n'a **pas** été touché, résolution numérotée, durée, leçons, fiche résumée.
Ensuite, **mettre à jour `resolution` de la part concernée dans `parts.yaml`** : une piste démentie par l'incident doit être corrigée, pas seulement complétée.
