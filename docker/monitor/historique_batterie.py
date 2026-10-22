#!/usr/bin/env python3
"""Historique de DECHARGE de la batterie et de sante du RPi, un fichier CSV, une ligne par seconde.

Demande utilisateur du 2026-10-17 : laisser le robot se decharger et enregistrer en direct le
niveau, la tension et l'intensite, pour tracer la courbe de decharge ET savoir ce qui lache en
premier (le Wi-Fi, semble-t-il) afin de fixer les niveaux d'alerte (OK / WARN / ERROR).

A LANCER SUR L'HOTE (python3 standard, aucune dependance). Il faut l'hote pour deux raisons :
le Wi-Fi n'est visible que depuis lui (les conteneurs sont en reseau pont), et un enregistreur
qui vit dans un conteneur disparait avec lui. La batterie arrive par un tube `docker exec`
vers batt_stream.py, relance seul si le conteneur redemarre.

Chaque ligne est ECRITE ET FORCEE SUR LA CARTE SD (fsync) : a la coupure, la derniere ligne est
celle de la derniere seconde, ce qui est justement la plus utile.

Colonnes : voir COLS. `evenement` note les changements (Wi-Fi perdu / retrouve, changement de
point d'acces, telemetrie batterie muette, sous-tension du RPi, throttling).

    python3 historique_batterie.py                # fichier decharge_<date>.csv, 1 ligne/s
    python3 analyse_decharge.py <fichier.csv>     # courbe resumee, premiers evenements
"""
import argparse
import collections
import csv
import datetime
import json
import os
import re
import signal
import subprocess
import threading
import time

CONTAINER = "bamboov4_humble-driver.esp32-1"
STREAM = "/root/linorobot2_ws/src/linorobot2/docker/monitor/batt_stream.py"
THROTTLED = "/sys/devices/platform/soc/soc:firmware/get_throttled"
COLS = ["epoch", "heure", "v_V", "i_A", "pct", "statut", "age_batt_s", "ping_ms",
        "perte30s_pct", "wifi_dBm", "wifi_qualite", "wifi_debit_Mbps", "wifi_bssid",
        "temp_C", "freq_MHz", "charge_cpu", "mem_dispo_Mo", "sous_tension", "throttled",
        "evenement"]


class BatteryFeed(threading.Thread):
    """Dernier message /battery, lu sur le tube docker exec ; relance si le tube se ferme."""

    def __init__(self, container):
        super().__init__(daemon=True)
        self.container = container
        self.last, self.proc, self.stop = None, None, False
        self.lock = threading.Lock()

    def run(self):
        cmd = ["docker", "exec", "-i", self.container, "bash", "-c",
               "source /opt/ros/humble/setup.bash; "
               "export FASTRTPS_DEFAULT_PROFILES_FILE=/root/.shm_off.xml; "
               "exec python3 -u %s" % STREAM]
        while not self.stop:
            try:
                self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                             stderr=subprocess.DEVNULL, text=True)
                for line in self.proc.stdout:
                    try:
                        d = json.loads(line)
                    except ValueError:
                        continue
                    with self.lock:
                        self.last = d
                self.proc.wait()
            except OSError:
                pass
            time.sleep(3.0)

    def get(self):
        with self.lock:
            return dict(self.last) if self.last else None


class PingProbe(threading.Thread):
    """Ping de la passerelle une fois par seconde ; perte glissante sur 30 s."""

    def __init__(self, target):
        super().__init__(daemon=True)
        self.target = target
        self.hist = collections.deque(maxlen=30)
        self.rtt, self.fail_run, self.stop = None, 0, False
        self.lock = threading.Lock()

    def run(self):
        while not self.stop:
            t0 = time.time()
            rtt = None
            try:
                r = subprocess.run(["ping", "-c", "1", "-W", "1", self.target],
                                   capture_output=True, text=True, timeout=3)
                m = re.search(r"time=([0-9.]+) ms", r.stdout)
                rtt = float(m.group(1)) if m else None
            except (OSError, subprocess.TimeoutExpired):
                pass
            with self.lock:
                self.rtt = rtt
                self.hist.append(rtt is not None)
                self.fail_run = 0 if rtt is not None else self.fail_run + 1
            time.sleep(max(0.0, 1.0 - (time.time() - t0)))

    def get(self):
        with self.lock:
            loss = 100.0 * (1.0 - sum(self.hist) / len(self.hist)) if self.hist else None
            return self.rtt, loss, self.fail_run


def gateway():
    try:
        out = subprocess.run(["ip", "route", "show", "default"], capture_output=True,
                             text=True, timeout=3).stdout
        return re.search(r"via ([0-9.]+)", out).group(1)
    except (OSError, AttributeError, subprocess.TimeoutExpired):
        return "192.168.1.254"


def read(path, default=""):
    try:
        with open(path, encoding="ascii") as fh:
            return fh.read().strip()
    except OSError:
        return default


def hwmon_volt_alarm():
    for d in sorted(os.listdir("/sys/class/hwmon")) if os.path.isdir("/sys/class/hwmon") else []:
        base = os.path.join("/sys/class/hwmon", d)
        if read(os.path.join(base, "name")) == "rpi_volt":
            return read(os.path.join(base, "in0_lcrit_alarm"), "")
    return ""


def wifi_proc():
    """(qualite, dBm) de /proc/net/wireless ; ('', '') si wlan0 n'y est pas."""
    for line in read("/proc/net/wireless").splitlines():
        if "wlan0" in line:
            f = line.split()
            try:
                return float(f[2].rstrip(".")), float(f[3].rstrip("."))
            except (IndexError, ValueError):
                pass
    return "", ""


def wifi_iw():
    """(bssid, debit Mb/s) de iwconfig ; bssid vaut 'perdu' si la carte n'est plus associee."""
    try:
        out = subprocess.run(["/sbin/iwconfig", "wlan0"], capture_output=True, text=True,
                             timeout=3).stdout
    except (OSError, subprocess.TimeoutExpired):
        return "", ""
    ap = re.search(r"Access Point: ([0-9A-Fa-f:]{17}|Not-Associated)", out)
    rate = re.search(r"Bit Rate[=:]([0-9.]+) Mb/s", out)
    bssid = ap.group(1) if ap else ""
    return ("perdu" if bssid == "Not-Associated" else bssid), (rate.group(1) if rate else "")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dir", default=os.path.expanduser("~/historique_batterie"))
    ap.add_argument("--container", default=CONTAINER)
    ap.add_argument("--target", default=None, help="adresse a pinger (defaut : passerelle)")
    ap.add_argument("--period", type=float, default=1.0)
    args = ap.parse_args()

    os.makedirs(args.dir, exist_ok=True)
    name = "decharge_%s.csv" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(args.dir, name)
    link = os.path.join(args.dir, "decharge_en_cours.csv")
    try:
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(name, link)
    except OSError:
        pass
    with open(os.path.join(args.dir, "monitor.pid"), "w", encoding="ascii") as fh:
        fh.write(str(os.getpid()))

    target = args.target or gateway()
    batt, ping = BatteryFeed(args.container), PingProbe(target)
    batt.start()
    ping.start()
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_a: stop.set())
    print("historique -> %s (ping %s, pid %d)" % (path, target, os.getpid()), flush=True)

    prev = {"bssid": None, "wifi_ok": True, "batt_ok": True, "uv": None, "thr": None,
            "st": None}
    iw_t, bssid, rate = 0.0, "", ""
    out = open(path, "w", newline="", encoding="ascii", buffering=1)
    w = csv.writer(out)
    w.writerow(COLS)
    nxt = time.time()
    while not stop.is_set():
        now = time.time()
        events = []
        if now - iw_t >= 5.0:
            bssid, rate = wifi_iw()
            iw_t = now
        quality, dbm = wifi_proc()
        rtt, loss, fail_run = ping.get()
        b = batt.get()
        age = (now - b["t"]) if b else None
        uv = hwmon_volt_alarm()
        thr = read(THROTTLED)

        if fail_run >= 3 and prev["wifi_ok"]:
            events.append("wifi_perdu")
            prev["wifi_ok"] = False
        elif fail_run == 0 and not prev["wifi_ok"]:
            events.append("wifi_retour")
            prev["wifi_ok"] = True
        if bssid and bssid != prev["bssid"]:
            if prev["bssid"] is not None:
                events.append("bssid:%s" % bssid)
            prev["bssid"] = bssid
        if age is None or age > 5.0:
            if prev["batt_ok"]:
                events.append("batt_muette")
                prev["batt_ok"] = False
        elif not prev["batt_ok"]:
            events.append("batt_retour")
            prev["batt_ok"] = True
        if uv != prev["uv"] and prev["uv"] is not None:
            events.append("sous_tension_rpi=%s" % uv)
        prev["uv"] = uv
        if thr != prev["thr"] and prev["thr"] is not None:
            events.append("throttled=%s" % thr)
        prev["thr"] = thr
        if b and b.get("st") != prev["st"]:
            if prev["st"] is not None:
                events.append("statut=%s" % b.get("st"))
            prev["st"] = b.get("st")

        load = read("/proc/loadavg").split()
        mem = re.search(r"MemAvailable:\s+(\d+)", read("/proc/meminfo"))
        temp = read("/sys/class/thermal/thermal_zone0/temp")
        freq = read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        w.writerow([
            "%.1f" % now, datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S"),
            "" if not b else b["v"], "" if not b else b["i"],
            "" if not b or b.get("pct") is None else b["pct"], "" if not b else b.get("st"),
            "" if age is None else "%.1f" % age, "" if rtt is None else rtt,
            "" if loss is None else "%.0f" % loss, dbm, quality, rate, bssid,
            "" if not temp else "%.1f" % (int(temp) / 1000.0),
            "" if not freq else int(freq) // 1000, load[0] if load else "",
            "" if not mem else int(mem.group(1)) // 1024, uv, thr, ";".join(events)])
        out.flush()
        os.fsync(out.fileno())                 # la derniere seconde avant la coupure compte
        nxt += args.period
        stop.wait(max(0.0, nxt - time.time()))
    out.close()
    batt.stop = True
    ping.stop = True
    if batt.proc:
        batt.proc.terminate()


if __name__ == "__main__":
    main()
