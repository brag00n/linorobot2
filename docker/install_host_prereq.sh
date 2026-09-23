#!/usr/bin/env bash
# install_host_prereq.sh — prerequis SYSTEME du robot, a poser une fois sur l'hote.
#
# POURQUOI SUR L'HOTE, contrairement a tout le reste : ce script configure le multiplexage
# des broches du SoC (/boot/config.txt) et l'etat d'une GPIO. Aucun conteneur ne peut le
# faire, meme privilegie : la fonction d'une broche est decidee par le firmware au boot,
# avant tout espace utilisateur. C'est l'unique exception assumee a la regle "on n'installe
# rien sur le RPi" -- et elle n'installe d'ailleurs aucun paquet.
#
# Usage :
#   bash install_host_prereq.sh            # DIAGNOSTIC SEUL : dit ce qui cloche, n'ecrit rien
#   sudo bash install_host_prereq.sh --apply   # applique les correctifs
#
# Idempotent : relancable sans effet si tout est deja conforme.
set -uo pipefail

APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1

# Compteurs de fin : ce qui reste a faire, et si un reboot est du.
todo=0
reboot_required=0

say()  { echo "$*"; }
ok()   { echo "  [ok]     $*"; }
warn() { echo "  [A FAIRE] $*"; todo=$((todo + 1)); }
act()  { echo "  [applique] $*"; }

need_root() {
    [ "$(id -u)" = "0" ] || { echo "ERREUR: --apply demande les droits root (sudo)." >&2; exit 2; }
}
[ "$APPLY" = "1" ] && need_root

# --- identification de l'hote -----------------------------------------------------------
# On ne corrige que ce qu'on reconnait : appliquer un reglage de brochage RPi sur une autre
# machine serait au mieux inutile, au pire nuisible.
MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || true)"
IS_RPI=0; case "$MODEL" in *"Raspberry Pi"*) IS_RPI=1 ;; esac
IS_DIETPI=0; [ -f /boot/dietpi.txt ] && IS_DIETPI=1

# Bookworm a deplace le fichier ; DietPi le garde a la racine de /boot.
CONFIG_TXT=""
for c in /boot/firmware/config.txt /boot/config.txt; do
    [ -f "$c" ] && { CONFIG_TXT="$c"; break; }
done

say "=== hote ==="
say "  modele      : ${MODEL:-inconnu}"
say "  distribution: $([ "$IS_DIETPI" = 1 ] && echo "DietPi $(sed -n 's/^G_DIETPI_VERSION_CORE=//p' /boot/dietpi/.version 2>/dev/null).$(sed -n 's/^G_DIETPI_VERSION_SUB=//p' /boot/dietpi/.version 2>/dev/null)" || echo "autre (pas DietPi)")"
say "  config.txt  : ${CONFIG_TXT:-absent}"
say ""

# ========================================================================================
# 1. UART du RPi au repos — CONDITION POUR ECRIRE DANS LA CARTE ESP32
# ========================================================================================
# Le header 40 broches de la General Driver partage le net de l'UART de l'ESP32 : la broche
# GPIO14 de l'hote est reliee au RXD de la carte. Avec `enable_uart=1`, GPIO14 est en
# fonction ALT "UART TXD", donc une sortie push-pull MAINTENUE A L'ETAT HAUT -- meme si
# rien n'emet. Elle ecrase alors le CP2102 du port Type-C, qui ne peut plus tirer ce RXD
# vers le bas : l'aller hote -> carte est mort, tandis que le retour carte -> hote reste
# parfait (10 Hz de telemetrie). D'ou un symptome tres trompeur, mesure le 2026-09-22 :
#   - esptool : "Download mode successfully detected, but getting no sync reply:
#               The serial TX path seems to be down."  (DTR/RTS passent, eux, par l'USB)
#   - aucun cmd_vel ni PARAM_SET n'aboutit, alors que la carte a l'air en pleine forme.
#
# D'ou vient le reglage : ni du firmware RPi, ni de la console serie. Sur un Pi 4
# (WiFi/BT integre) l'UART primaire est le mini-UART ttyS0, DESACTIVE par defaut cote
# firmware ; c'est DietPi qui injecte `enable_uart=1` (dietpi-set_hardware). Et DietPi
# dissocie les deux reglages : la CONSOLE serie peut etre coupee
# (CONFIG_SERIAL_CONSOLE_ENABLE=0, console=tty1 seul, serial-getty@* masques) alors que le
# PERIPHERIQUE UART reste actif. "Rien n'emet sur le port serie" ne veut donc pas dire
# "la ligne est libre" -- c'est exactement ce qui a fait perdre une session de diagnostic.
#
# Rien sur ce robot n'utilise l'UART du RPi : l'ESP32 et la carte capteurs passent par USB.
say "=== 1. UART de l'hote (prerequis a toute ecriture dans l'ESP32) ==="

if [ "$IS_RPI" != "1" ]; then
    ok "hote non-RPi : pas de brochage a liberer, rien a faire"
elif [ -z "$CONFIG_TXT" ]; then
    warn "aucun config.txt trouve : verifier manuellement que l'UART de l'hote est au repos"
else
    # Etat declare. Une ligne commentee ou absente = defaut firmware = au repos sur Pi 4.
    uart_line="$(grep -nE '^[[:blank:]]*enable_uart=' "$CONFIG_TXT" | tail -n1)"
    # Drapeau : le brochage est-il DEJA au repos de facon durable ? Si oui, le soulagement
    # sysfs plus bas n'a plus d'objet -- le reclamer serait un faux positif.
    uart_at_rest=0
    if [ -z "$uart_line" ]; then
        ok "enable_uart non declare dans $CONFIG_TXT (defaut firmware : UART au repos)"
        uart_at_rest=1
    elif echo "$uart_line" | grep -q 'enable_uart=0'; then
        ok "enable_uart=0 dans $CONFIG_TXT ($uart_line)"
        uart_at_rest=1
    else
        say "  enable_uart actif : $CONFIG_TXT:$uart_line"
        if [ "$APPLY" = "1" ]; then
            # Sauvegarde UNE SEULE FOIS : on ne veut pas ecraser l'original a la 2e passe.
            [ -f "$CONFIG_TXT.bamboo.bak" ] || cp -p "$CONFIG_TXT" "$CONFIG_TXT.bamboo.bak"
            # Meme operation que le G_CONFIG_INJECT de DietPi (remplacement par regex), en
            # evitant de sourcer ses 2000 lignes de globals depuis un script tiers.
            sed -i -E 's/^[[:blank:]]*enable_uart=.*/enable_uart=0   # BambooWS: GPIO14 partage le RXD de l ESP32 (cf. install_host_prereq.sh)/' "$CONFIG_TXT"
            if grep -qE '^enable_uart=0' "$CONFIG_TXT"; then
                act "enable_uart=0 ecrit dans $CONFIG_TXT (original : $CONFIG_TXT.bamboo.bak)"
                reboot_required=1
            else
                warn "l'ecriture de enable_uart=0 a echoue : editer $CONFIG_TXT a la main"
            fi
        else
            warn "enable_uart=1 : relancer avec --apply, ou dietpi-config > Advanced Options > Serial/UART"
        fi
    fi

    # --- soulagement immediat, sans reboot -----------------------------------------------
    # Exporter GPIO14 en entree la retire de la fonction UART tout de suite. NON PERSISTANT
    # (perdu au reboot), mais c'est ce qui permet de flasher la carte dans la minute sans
    # redemarrer le robot. Apres un reboot avec enable_uart=0, cette etape est inutile.
    # Le reglage de config.txt ne vaut qu'au boot suivant : on confirme donc l'etat REEL par la
    # presence du noeud. On teste /dev/serial0 et LUI SEUL, car c'est l'alias firmware du port
    # expose sur le header -- la seule chose qui puisse contester GPIO14.
    # NE PAS y ajouter /dev/ttyAMA0 : piege verifie le 2026-09-23, en activant la radio BT du
    # § 2. ttyAMA0 est le PL011, que le firmware route vers le MODEM BT (alias /dev/serial1)
    # des que `disable-bt` tombe -- il reapparait donc, sans rapport avec le header. Le tester
    # faisait reclamer la liberation de GPIO14 sur un hote pourtant conforme. Le noyau le dit
    # d'ailleurs lui-meme au boot : "uart-pl011 fe201000.serial: there is not valid maps for
    # state default", c'est-a-dire aucune broche assignee cote header.
    if [ "$uart_at_rest" = "1" ] && [ ! -e /dev/serial0 ]; then
        ok "aucun UART sur le header (/dev/serial0 absent) : GPIO14 est libre, rien a soulager"
        [ -e /dev/ttyAMA0 ] && say "  (/dev/ttyAMA0 present = PL011 pris par le modem BT, cf. § 2 : sans effet ici)"
    elif [ -d /sys/class/gpio ]; then
        cur=""
        [ -r /sys/class/gpio/gpio14/direction ] && cur="$(cat /sys/class/gpio/gpio14/direction 2>/dev/null)"
        if [ "$cur" = "in" ]; then
            ok "GPIO14 deja libere en entree (sysfs) : la voie TX vers la carte est ouverte"
        elif [ "$APPLY" = "1" ]; then
            [ -e /sys/class/gpio/gpio14 ] || echo 14 > /sys/class/gpio/export 2>/dev/null
            if [ -w /sys/class/gpio/gpio14/direction ] \
               && echo in > /sys/class/gpio/gpio14/direction 2>/dev/null; then
                act "GPIO14 libere en entree (immediat, non persistant)"
            else
                warn "liberation sysfs de GPIO14 impossible : le reboot reste necessaire"
            fi
        else
            warn "GPIO14 non liberee : --apply l'ouvre tout de suite, sans reboot"
        fi
    fi

    # Informatif : une console serie active reemettrait vraiment sur la ligne. Sur ce robot
    # elle est deja coupee -- mais son etat ne dit RIEN de enable_uart (cf. plus haut), donc
    # on l'affiche sans jamais en deduire que la ligne est libre.
    gettys="$(systemctl is-enabled serial-getty@ttyAMA0.service serial-getty@ttyS0.service 2>/dev/null | tr '\n' ' ')"
    say "  pour information, console serie : ${gettys:-etat indetermine} (independant de enable_uart)"
fi

say ""

# ========================================================================================
# 2. Radio Bluetooth du RPi — CONDITION POUR APPAIRER LA MANETTE
# ========================================================================================
# DietPi coupe la radio BT au niveau FIRMWARE par `dtoverlay=disable-bt`. Consequence : bluez
# est bien installe et le service `bluetooth` bien active, mais il n'a aucun adaptateur a
# piloter -- il demarre et meurt (`inactive (dead)`), et `hciconfig` repond
# "Can't open HCI socket.: Address family not supported by protocol". Aucun `systemctl` n'y
# changera rien : l'overlay est applique avant l'espace utilisateur, d'ou la presence de ce
# reglage ici et non dans install_bambooWS.sh.
#
# MAIS L'OVERLAY N'EST QUE LA PREMIERE DE TROIS COUCHES, constat du 2026-09-23 : l'avoir
# neutralise puis redemarre ne donnait TOUJOURS aucun `hci0`. DietPi empile en effet
#   1. `dtoverlay=disable-bt` dans config.txt (firmware) ;
#   2. `/etc/modprobe.d/dietpi-disable_bluetooth.conf`, qui blackliste six modules
#      (bluetooth, hci_uart, btbcm, bnep, rfcomm, hidp) -- donc meme overlay retire, rien ne
#      se charge ;
#   3. l'absence de `pi-bluetooth`, qui fournit `hciuart.service` : sans lui PERSONNE n'attache
#      le modem BCM au PL011, donc pas d'adaptateur meme modules charges.
# D'ou la DELEGATION a la routine DietPi ci-dessous plutot qu'un demontage couche par couche :
# elle traite les trois, et surtout elle laisse DietPi COHERENT avec lui-meme -- un
# `dietpi-config` ou une mise a jour ulterieure ne rejouera pas la desactivation par-dessus
# notre bricolage. Elle n'exige pas de reboot (modprobe + `systemctl start hciuart` suffisent),
# mais elle installe deux paquets sur l'hote : seule exception a la regle "on n'installe rien
# sur le RPi", inevitable puisque l'appairage est hote et que le conteneur ne voit qu'un
# /dev/input/js* deja appaire.
#
# INTERACTION AVEC LE § 1, contre-intuitive : sur un Pi 4 la BT integree est branchee sur
# l'UART materiel PL011 (ttyAMA0), et `disable-bt` sert justement a liberer ce PL011. Le
# reactiver ne rend donc PAS l'UART au header : avec la BT active, le port expose sur
# GPIO14/15 redevient le mini-UART ttyS0, qui reste eteint puisque `enable_uart=0` (§ 1).
# Les deux reglages sont donc compatibles -- radio BT active ET GPIO14 au repos -- alors que
# la lecture naive ("la BT utilise l'UART") laisse croire a un conflit.
#
# CE QU'ON PERD EN REACTIVANT LA BT, explicitement (car le compromis est reel sur Pi 4) :
#   1. L'UART DE QUALITE sur le header. Rendre la BT au PL011 laisse au header le mini-UART,
#      dont le debit est derive de l'horloge coeur du VPU : d'ou l'avertissement de DietPi
#      lui-meme en tete de ce fichier -- "enable_uart=1 will enforce core_freq=250 on RPi
#      models with onboard WiFi" -- qui BRIDE LE GPU pour stabiliser le baud. Sans objet ici :
#      `enable_uart=0`, rien sur ce robot n'utilise l'UART du header (ESP32 et carte capteurs
#      passent par USB), donc on ne perd un port qu'on n'emploie pas et on evite au passage le
#      bridage d'horloge.
#      => NE PAS "recuperer" le PL011 par `dtoverlay=miniuart-bt` : ce fameux compromis
#         deplace la BT sur le mini-UART, et c'est alors LE LIEN DE LA MANETTE dont le baud
#         depend de l'horloge coeur -- appairage capricieux, et core_freq a epingler. On veut
#         l'inverse de ce chantier.
#   2. La BANDE 2,4 GHz. La BT et le WiFi 2,4 GHz partagent la puce radio (CYW43455) : une
#      manette active ampute le debit d'un WiFi en 2,4 GHz, donc le flux camera H.264 et
#      rosbridge. Sans objet ici aussi, VERIFIE le 2026-09-23 : wlan0 est associe a 5180 MHz
#      (canal 36, 5 GHz), bande disjointe de la BT. Le controle ci-dessous le reverifie, car
#      un repli de l'AP en 2,4 GHz degraderait la video des que la manette emet -- panne
#      deroutante s'il faut la diagnostiquer sans connaitre ce couplage.
say "=== 2. Radio Bluetooth (prerequis a l'appairage de la manette) ==="

if [ "$IS_RPI" != "1" ]; then
    ok "hote non-RPi : pas d'overlay a retirer"
elif [ -z "$CONFIG_TXT" ]; then
    warn "aucun config.txt trouve : verifier manuellement que la radio BT est active"
else
    DIETPI_HW=/boot/dietpi/func/dietpi-set_hardware
    BT_BLACKLIST=/etc/modprobe.d/dietpi-disable_bluetooth.conf

    # ETAT REEL, seul juge : un adaptateur enumere, ou non. Tout le reste n'est qu'une cause.
    if [ -d /sys/class/bluetooth ] && [ -n "$(ls -A /sys/class/bluetooth 2>/dev/null)" ]; then
        ok "adaptateur BT present : $(ls /sys/class/bluetooth | tr '\n' ' ')"
    else
        # Les trois couches, enoncees pour que le diagnostic explique la panne au lieu de la
        # constater. Aucune ne suffit seule.
        grep -qE '^[[:blank:]]*dtoverlay=disable-bt' "$CONFIG_TXT" \
            && say "  couche 1/3 : dtoverlay=disable-bt actif dans $CONFIG_TXT (firmware)"
        [ -f "$BT_BLACKLIST" ] \
            && say "  couche 2/3 : modules blacklistes par $BT_BLACKLIST"
        dpkg-query -s pi-bluetooth >/dev/null 2>&1 \
            || say "  couche 3/3 : pi-bluetooth absent -> pas de hciuart, modem BCM non attache"

        if [ "$APPLY" != "1" ]; then
            warn "radio BT inactive : relancer avec --apply (delegue a dietpi-set_hardware)"
        elif [ -x "$DIETPI_HW" ]; then
            act "activation par la routine DietPi ($DIETPI_HW bluetooth enable)"
            if "$DIETPI_HW" bluetooth enable >/dev/null 2>&1; then
                # La routine ne fait qu'`enable hciuart` (sans --now) : au premier passage
                # l'unite attend /dev/serial1, qui n'apparait qu'avec les regles udev du
                # paquet fraichement installe. Un `start` explicite evite d'exiger un reboot.
                systemctl start hciuart >/dev/null 2>&1
                # bluetoothd a demarre AVANT que bthelper ne corrige l'adresse du controleur :
                # `bluetoothctl show` annonce alors AA:AA:AA:AA:AA:AA alors que le noyau lit la
                # vraie adresse. Un redemarrage du demon les raccorde (constat 2026-09-23).
                systemctl start "bthelper@hci0" >/dev/null 2>&1
                systemctl restart bluetooth    >/dev/null 2>&1
                if [ -n "$(ls -A /sys/class/bluetooth 2>/dev/null)" ]; then
                    act "adaptateur BT en service : $(ls /sys/class/bluetooth | tr '\n' ' ')"
                else
                    warn "toujours aucun adaptateur : un reboot devrait finir de l'attacher"
                    reboot_required=1
                fi
            else
                warn "la routine DietPi a echoue : rejouer dietpi-config > Advanced Options > Bluetooth"
            fi
        else
            # Hors DietPi : on ne recree pas sa routine, on dit quoi faire. L'installation de
            # paquets a l'aveugle sur un hote inconnu ferait plus de degats que de service.
            warn "hote sans dietpi-set_hardware : retirer dtoverlay=disable-bt, $BT_BLACKLIST,"
            say  "       installer pi-bluetooth, puis 'systemctl enable --now hciuart bluetooth'"
        fi
    fi

    # Coexistence 2,4 GHz (cf. point 2 ci-dessus). Informatif : on ne reconfigure pas l'AP du
    # foyer depuis un script de robot, mais taire le couplage coute une session de diagnostic
    # video le jour ou l'AP bascule de bande.
    wifi_freq="$(sudo -n wpa_cli -i wlan0 status 2>/dev/null | sed -n 's/^freq=//p' | head -n1)"
    if [ -z "$wifi_freq" ]; then
        say "  bande WiFi indeterminee : si l'AP est en 2,4 GHz, la manette amputera le flux video"
    elif [ "$wifi_freq" -ge 5000 ] 2>/dev/null; then
        ok "WiFi en 5 GHz ($wifi_freq MHz) : bande disjointe de la BT, aucun partage de debit"
    else
        warn "WiFi en 2,4 GHz ($wifi_freq MHz) : la BT partage cette bande -> debit video reduit"
        say "       passer l'AP/le RPi en 5 GHz avant de compter sur la camera pendant le pilotage"
    fi
fi

say ""
say "=== bilan ==="
if [ "$todo" = "0" ]; then
    say "  hote conforme."
else
    say "  $todo point(s) restant(s) : relancer avec 'sudo bash $0 --apply'."
fi
if [ "$reboot_required" = "1" ]; then
    # Un SEUL reboot pour les deux reglages : ils vivent dans le meme fichier, applique une
    # fois au boot. D'ou leur regroupement ici plutot qu'un script par sujet.
    say "  REBOOT REQUIS pour appliquer les changements de $CONFIG_TXT (brochage UART et/ou"
    say "  radio BT). La liberation sysfs de GPIO14 tient jusque-la. Verifier ensuite :"
    say "    docker compose --profile tools run --rm -e CHECK_ONLY=1 flash.esp32   # voie TX"
    say "    hciconfig -a                                                         # radio BT"
fi
exit 0
