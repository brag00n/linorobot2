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
2. **Constater que le driver ne reprend PAS tout seul.** Le nœud `ldlidar_stl_ros2_node` est mort et rien ne le relance : `/scan` reste sans publieur même après le rebranchement. Le conteneur est en `restart: no` (0 redémarrage), **mais ce n'est pas la seule raison** : le conteneur n'est jamais « tombé », car `scan_monitor`, lancé par le même `launch`, a continué de tourner. Même avec `restart: on-failure`, Docker n'aurait rien relancé (voir 5.1). *C'est le piège de cet incident.*
3. **Redémarrer le service à la main** : MCP `restart driver.lidarD500` (ou `docker restart bamboov4_humble-driver.lidarD500-1`). Pas besoin de `recreate`.
4. **Vérifier** : `/scan` à ~9,8 Hz, 460 points valides, portée 0,22 à 3,3 m ; journal « Actual BaudRate reported:230400 » ; `scan_monitor` et `nav_supervisor` repassent à OK.
5. **Relancer la localisation** : sous-mode `navigation` puis `relocalize` (R long à la manette). Relocalisé en 2,3 s sur la carte (score 0,70, accord scan/carte 0,51).

**Durée de la panne** : environ 4 minutes (débranché → rebranché → driver relancé → scan rétabli).

**Leçons**

- **Le diagnostic distingue bien « lidar absent » de « pile arrêtée à la demande »** : le message est différent (« aucun scan reçu » contre « arrêté, pile NAV éteinte »). Ne pas confondre.
- **Pas de redémarrage automatique du driver lidar** : volontaire (service à la demande), mais cela veut dire qu'un câble qui se débranche puis se rebranche **exige une relance manuelle**. Une reprise automatique n'est pas faite. **Un simple `restart: on-failure` du conteneur ne suffirait pas** (le conteneur reste en vie grâce à `scan_monitor`) : la correction qui répond au cas observé est un `respawn` du nœud du driver dans `lidar.launch.py`. Voir les améliorations proposées, 5.1.
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

## 5. Améliorations proposées

Cette section **propose** ; elle ne décrit rien d'existant. Chaque entrée dit : le problème réel qui la motive (un incident), ce qu'on ferait, ce que cela coûte ou risque, ce qui a été vérifié et ce qui ne l'a pas été.
Quand une amélioration est faite, la noter dans le journal des incidents (section 3) avec son effet mesuré, puis la retirer d'ici.

**Statut** : `proposée` (rien n'est fait), `validée` (acceptée, à faire), `faite` (dans le dépôt et déployée), `rejetée` (écartée, avec la raison).

### 5.1 Reprise automatique du driver quand un périphérique revient

| | |
|---|---|
| **Statut** | proposée |
| **Motif** | INC-2026-11-18 : après le rebranchement du lidar, `/scan` reste muet tant qu'on ne relance pas le driver à la main |
| **Cause technique vérifiée** | le conteneur `driver.lidarD500` lance `ros2 launch bamboo_lidar lidar.launch.py`, qui démarre **deux** nœuds : le driver `ldlidar_stl_ros2_node` et `scan_monitor`. Quand le driver meurt (port absent), `scan_monitor` continue : le processus principal du conteneur **ne sort jamais**, et Docker n'a rien à relancer |

**Trois corrections possibles, qui ne se valent pas :**

| Option | Principe | Avantage | Limite |
|---|---|---|---|
| **A. `restart: on-failure` sur le conteneur** | Docker relance le conteneur quand son processus principal sort avec un code non nul | une ligne dans le compose | **ne répare pas seule le cas réel** : `scan_monitor` garde le conteneur en vie, donc il n'est jamais « en échec ». Elle ne servirait que si l'échec du driver arrêtait tout le lancement |
| **B. `respawn=True` sur le nœud du driver**, dans `lidar.launch.py` | le lanceur ROS relance **le nœud seul** quand il meurt, après un délai (`respawn_delay`) | corrige exactement le défaut observé, sans redémarrer le conteneur. **Vérifié** : `launch` 1.0.14 (installé) accepte `respawn` et `respawn_delay` | lidar resté débranché : le nœud est relancé **en boucle** (journal bruyant, un peu de CPU) : prévoir un délai de plusieurs secondes |
| **C. B + A** | respawn du nœud, plus `on-failure` pour le cas où le lanceur lui-même tombe | double protection | deux mécanismes à comprendre |

**Recommandation** : **B**. Seule option qui répare le cas observé. Ne pas se contenter de A : elle donnerait l'impression d'avoir corrigé sans rien changer.
**Non testé** : le respawn avec un lidar débranché plusieurs minutes (charge, bruit du journal) ; la réouverture effective du port par `ldlidar` après rebranchement sans autre geste ; l'interaction avec l'arrêt à la demande (un `docker stop` de l'hôte arrête le conteneur entier, le respawn n'y change rien).
**Reste manuel même avec B** : relocaliser (R long), car la localisation passe à « inconnue » quand le scan disparaît (voir 5.4).

### 5.2 Un diagnostic qui dit « la reprise n'a pas eu lieu »

| | |
|---|---|
| **Statut** | proposée |
| **Motif** | pendant les 4 minutes de l'incident, le diagnostic affichait « aucun scan reçu (driver arrêté ? lidar débranché ?) » : vrai, mais **identique avant et après le rebranchement**. Rien ne disait « le lidar est revenu, le driver est mort, relancez-le » |
| **Ce qu'on ferait** | dans `scan_monitor`, croiser **trois faits** : `/dev/lidar` existe ? le processus `ldlidar` tourne ? `/scan` publie ? Trois messages distincts : « lidar débranché » ; **« lidar présent mais driver arrêté : relancer driver.lidarD500 »** ; « lidar et driver présents mais pas de scan » |
| **Intérêt** | transforme un symptôme en **instruction** : c'est le chemin de résolution qu'on cherche à automatiser |
| **Vérifié** | le conteneur `driver.lidarD500` voit `/dev/lidar` réapparaître au rebranchement (`/dev` est monté en entier : `/dev/lidar -> ttyUSB1` observé dans le conteneur le 2026-11-18) |
| **Risque** | faible (lecture de `/dev` et de la liste des processus) ; bon candidat pour commencer |

### 5.3 Regrouper les symptômes sous une cause racine, en direct

| | |
|---|---|
| **Statut** | proposée (étape 2 du chantier « nomenclature des parts ») |
| **Motif** | lidar débranché : **deux** erreurs (`scan_monitor`, `nav_supervisor`) pour une panne. Webcam gelée : **quatre** services en erreur pour une webcam. Le diagnostic actuel ne regroupe pas |
| **Ce qu'on ferait** | un nœud en lecture seule qui lit `/diagnostics_agg`, rattache chaque tâche en défaut à une part (`parts.yaml`), calcule **racines** et **impact** (`parts_model.Graph.incidents` existe et est testé), puis publie un JSON `/bamboo/incidents` |
| **Règles pour ne pas mentir** | un service **arrêté à la demande** (suivi, lidar, navigation) n'est pas un incident ; une tâche en défaut **sans part** est signalée, jamais ignorée ; un défaut doit durer deux relevés avant de devenir incident |
| **Risque** | charge (à mesurer, objectif moins de 1 % d'un cœur) ; PyYAML à vérifier dans l'image de diagnostic (reconstruction : plusieurs heures) |

### 5.4 Proposer la relocalisation quand le scan revient

| | |
|---|---|
| **Statut** | proposée |
| **Motif** | INC-2026-11-18 : après la relance du lidar, la localisation restait « inconnue » jusqu'à un `relocalize` manuel (R long) |
| **Ce qu'on ferait** | quand `/scan` redevient publié, en sous-mode `navigation` avec une carte courante : afficher « scan rétabli, localisation inconnue : relocaliser ? ». **Proposer, pas lancer** |
| **Pourquoi pas automatique** | une relocalisation **fait tourner le robot** si la recherche locale échoue (rotations de recherche). Le robot ne doit pas bouger de lui-même après une panne : décision de l'opérateur |
| **Non vérifié** | le comportement du superviseur quand le scan revient tout seul (observé seulement avec une relance manuelle) |

### 5.5 Une pile NAV qui s'arrête quand le lidar disparaît

| | |
|---|---|
| **Statut** | proposée |
| **Motif** | pendant l'incident, slam_toolbox et Nav2 ont continué de consommer **~65 % d'un cœur** pour attendre un `/scan` qui ne venait pas, sur un robot déjà à ~56 % de CPU global |
| **Ce qu'on ferait** | le superviseur arrête la pile après N minutes sans scan (réglable, par exemple 5), en le disant ; elle redémarre à la demande |
| **Risque** | une pile qui s'arrête seule peut surprendre : **jamais pendant un but actif**, seulement à l'arrêt. À valider avec l'opérateur |

### 5.6 Voir un périphérique USB disparaître avant le symptôme logiciel

| | |
|---|---|
| **Statut** | proposée |
| **Motif** | les deux incidents (webcam gelée, lidar absent) sont d'abord apparus comme des erreurs logicielles ; la cause était matérielle et n'est visible que par `lsusb` et `dmesg`. Le diagnostic ne regarde pas le bus |
| **Ce qu'on ferait** | une tâche « périphériques USB » dans `bamboo_health` : compare ce que `lsusb` voit à ce que `parts.yaml` attend (webcam `05a3:9331`, deux `10c4:ea60`, `/dev/esp32`, `/dev/lidar`) et remonte les **événements noyau** utiles (`uvcvideo … Failed to initialize`, `usb … disconnect`) |
| **Gain** | la webcam gelée de 21 h aurait été signalée dès l'erreur noyau, et non par l'absence d'image |
| **Risque** | droits de lecture de `dmesg` depuis le conteneur : à vérifier. **Ne jamais réinitialiser un port** (hub partagé) |

### 5.7 Un chemin de résolution **affiché**, pas seulement écrit

| | |
|---|---|
| **Statut** | proposée |
| **Motif** | les pistes de résolution existent dans `parts.yaml` mais il faut les chercher à la main |
| **Ce qu'on ferait** | dans le CC, un panneau « Incidents » : racine, conséquences, **étapes numérotées** tirées de `resolution`, bouton « copier la commande ». **Le panneau n'exécute rien** : l'opérateur reste maître |
| **Risque** | ne pas ajouter de bouton « réparer » : un service redémarré à tort, traction armée, peut être dangereux |

### 5.8 Compléter la nomenclature par les pièces inconnues

| | |
|---|---|
| **Statut** | proposée |
| **Motif** | quatre parts sont encore « à renseigner » (`base_link`, `oled`, `hub_usb`, `adaptateur_bluetooth`), cinq autres « déduites » (dont les quatre moteurs) |
| **Ce qu'on ferait** | relever les références sur le matériel (étiquettes, factures) : elles conditionnent le **remplacement** d'une pièce défaillante |
| **Qui** | l'opérateur : je ne peux pas les deviner |

## 6. Comment ajouter un incident

Copier le modèle de la section 3 : date, part racine, impact, cause réelle, symptômes **dans l'ordre**, ce qui n'a **pas** été touché, résolution numérotée, durée, leçons, fiche résumée.
Ensuite, **mettre à jour `resolution` de la part concernée dans `parts.yaml`** : une piste démentie par l'incident doit être corrigée, pas seulement complétée.
