# Point d'acces Wi-Fi du robot (service compose `ap.rpi`) : hostapd + dnsmasq (DHCP seul) + NAT,
# plus l'INTERFACE WEB RaspAP (lighttpd + PHP) pour regler SSID / mot de passe / clients.
# Image hors stack ROS. Reseau de l'hote : le conteneur pilote ap0 (interface virtuelle creee a
# partir de wlan0) sans jamais toucher a wlan0, qui reste client de la box.
#
# Pourquoi pas l'image officielle raspap-docker : elle embarque systemd, exige --privileged et un
# cgroup en ecriture, et RaspAP y gere dhcpcd/wpa_supplicant -- il couperait la liaison avec la
# box. Ici seule l'interface PHP de RaspAP est reprise (telechargee, jamais copiee dans le depot :
# GPL-3.0), les services sont pilotes par entrypoint.sh, et `systemctl` est une cale
# (systemctl-shim.sh) qui relaie les ordres de l'interface au superviseur.
#
# iptables-nft : l'hote (DietPi, Debian 12) utilise le backend nft ; les deux backends ne
# partagent pas leurs regles, il faut le meme que l'hote pour que le NAT soit vu.
FROM debian:bookworm-slim

# Version de l'interface RaspAP (tag GitHub). A relire a chaque montee de version : les
# constantes de config.php et les commandes du sudoers amont peuvent bouger.
ARG RASPAP_VERSION=3.5.9

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        hostapd dnsmasq-base iw iproute2 iptables procps sudo net-tools \
        ca-certificates curl lighttpd php8.2-cgi php8.2-cli \
 && update-alternatives --set iptables /usr/sbin/iptables-nft \
 && update-alternatives --set ip6tables /usr/sbin/ip6tables-nft \
 && rm -rf /var/lib/apt/lists/*

# --- Interface web : sources amont, puis reglages propres au robot --------------------------
RUN mkdir -p /var/www/html \
 && curl -fsSL "https://github.com/RaspAP/raspap-webgui/archive/refs/tags/${RASPAP_VERSION}.tar.gz" \
      | tar xz --strip-components=1 -C /var/www/html \
 && cp /var/www/html/config/config.php /var/www/html/includes/config.php \
 # L'interface vise ap0 (et non wlan0, qui est le client de la box). Les pages qui touchent au
 # client Wi-Fi, au reseau de l'hote et a l'extinction sont coupees : le sudoers ci-dessous ne
 # les autoriserait de toute facon pas, mais une page qui echoue en silence est pire qu'absente.
 && sed -i \
      -e "s/\('RASPI_WIFI_AP_INTERFACE', *\)'wlan0'/\1'ap0'/" \
      -e "s/\('RASPI_WIFICLIENT_ENABLED', *\)true/\1false/" \
      -e "s/\('RASPI_NETWORK_ENABLED', *\)true/\1false/" \
      -e "s/\('RASPI_SYSTEM_ENABLED', *\)true/\1false/" \
      -e "s/\('RASPI_VNSTAT_ENABLED', *\)true/\1false/" \
      -e "s/\('RASPI_PLUGINS_ENABLED', *\)true/\1false/" \
      /var/www/html/includes/config.php \
 # Gardes : si une constante amont change de forme, le build echoue au lieu de produire une
 # interface qui vise wlan0 ou garde des pages dangereuses.
 && grep -Eq "RASPI_WIFI_AP_INTERFACE', *'ap0'" /var/www/html/includes/config.php \
 && grep -Eq "RASPI_WIFICLIENT_ENABLED', *false" /var/www/html/includes/config.php \
 && grep -Eq "RASPI_NETWORK_ENABLED', *false" /var/www/html/includes/config.php \
 && grep -Eq "RASPI_SYSTEM_ENABLED', *false" /var/www/html/includes/config.php \
 # Routage de l'interface a la racine du site (le script d'installation amont remplace
 # /REPLACE_ME par le chemin web ; ici le chemin est vide).
 && sed 's#/REPLACE_ME##g' /var/www/html/config/50-raspap-router.conf \
      > /etc/lighttpd/conf-available/50-raspap-router.conf \
 && lighty-enable-mod fastcgi-php \
 && ln -s ../conf-available/50-raspap-router.conf /etc/lighttpd/conf-enabled/50-raspap-router.conf \
 && cp /etc/lighttpd/lighttpd.conf /etc/lighttpd/lighttpd.conf.orig \
 && chown -R www-data:www-data /var/www/html

# --- Etat persistant : un seul volume /state, les chemins de RaspAP y pointent ---------------
RUN rm -rf /etc/raspap /etc/hostapd /etc/dnsmasq.d \
 && ln -s /state/raspap /etc/raspap \
 && ln -s /state/hostapd /etc/hostapd \
 && ln -s /state/dnsmasq.d /etc/dnsmasq.d

COPY systemctl-shim.sh /usr/bin/systemctl
COPY sudoers /etc/sudoers.d/090_raspap-bamboo
COPY entrypoint.sh /entrypoint.sh
RUN chmod 0755 /usr/bin/systemctl /entrypoint.sh \
 && chmod 0440 /etc/sudoers.d/090_raspap-bamboo \
 && visudo -cf /etc/sudoers.d/090_raspap-bamboo

ENTRYPOINT ["/entrypoint.sh"]
