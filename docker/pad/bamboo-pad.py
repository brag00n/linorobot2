#!/usr/bin/env python3
"""bamboo-pad.py -- assistant d'appairage de la manette Bluetooth, cote HOTE.

POURQUOI CE DETOUR : le portail (HMI :8000) vit dans un conteneur sans acces au Bluetooth de
l'hote, et lui donner D-Bus ou Docker reviendrait a lui donner le RPi. Meme schema que
docker/ondemand : le conteneur DEPOSE une demande, l'hote l'execute ici.
    /run/bamboo-pad/pair.want    `start <id>` ou `cancel <id>`, ecrit par /cgi-bin/pad
    /run/bamboo-pad/pair.state   JSON (etape, consigne, journal, resultat), ecrit ICI

Lance par bamboo-pad.path (systemd) a chaque demande. Recette = celle qui marche (notes du
2026-09-30) : scan BR/EDR CONTINU (`scan bredr`, le LE ne voit pas cette manette), la manette
est allumee PENDANT le scan, scan ARRETE avant `pair` (un scan concurrent le fait echouer).

N'oublie QUE les appareils de classe manette (Icon input-gaming ou nom *pad/controller*) :
jamais un telephone ni une enceinte.
    journalctl -u bamboo-pad      cat /run/bamboo-pad/pair.state
"""
import glob
import json
import os
import re
import subprocess
import sys
import time

DIR = os.environ.get("BAMBOO_PAD_DIR", "/run/bamboo-pad")
WANT = os.path.join(DIR, "pair.want")
STATE = os.path.join(DIR, "pair.state")
SCAN_S = int(os.environ.get("BAMBOO_PAD_SCAN_S", "75"))
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]|\r")
MAC = r"[0-9A-F]{2}(?::[0-9A-F]{2}){5}"
STEPS = [
    ("forget", "Oubli des anciens appairages de manette"),
    ("scan", "Recherche de la manette (Bluetooth classique)"),
    ("pair", "Appairage"),
    ("trust", "Confiance (reconnexion automatique)"),
    ("connect", "Connexion"),
    ("verify", "Verification (manette vue par le noyau)"),
]
HOWTO = ("Eteins la manette, puis rallume-la en maintenant Home + X (mode XInput) jusqu'a ce que "
         "les voyants se stabilisent. Home + X est a refaire A CHAQUE appairage : apres une "
         "partie sur la Switch, la manette est repassee en mode Switch.")


class Cancelled(Exception):
    pass


class Failed(Exception):
    def __init__(self, message, hint=""):
        super().__init__(message)
        self.hint = hint


S = {"id": "", "step": "idle", "title": "", "instruction": "", "message": "", "hint": "",
     "log": [], "steps": [], "device": None, "result": None, "remaining": None, "ts": 0}


def push():
    S["ts"] = int(time.time())
    tmp = STATE + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(S, fh)
    os.chmod(tmp, 0o644)
    os.replace(tmp, STATE)


def log(msg):
    S["log"] = (S["log"] + [time.strftime("%H:%M:%S ") + msg])[-40:]
    print("bamboo-pad:", msg, flush=True)
    push()


def mark(step, status):
    for s in S["steps"]:
        if s["id"] == step:
            s["status"] = status


def begin(step, title, instruction=""):
    mark(step, "doing")
    S.update(step=step, title=title, instruction=instruction, remaining=None)
    push()


def want():
    try:
        v = open(WANT).read().split()
    except OSError:
        return "", ""
    return (v[0], v[1]) if len(v) >= 2 else ("", "")


def check_cancel(rid):
    verb, i = want()
    if verb == "cancel" or (verb == "start" and i != rid):
        raise Cancelled()


def bt(*args, timeout=20):
    try:
        r = subprocess.run(["bluetoothctl", *args], capture_output=True, text=True, timeout=timeout)
        return ANSI.sub("", r.stdout + r.stderr)
    except subprocess.TimeoutExpired:
        return ""


def info(mac):
    d = {}
    for line in bt("info", mac, timeout=8).splitlines():
        m = re.match(r"\s*([A-Za-z]+):\s*(.*)$", line)
        if m and m.group(1) not in d:
            d[m.group(1)] = m.group(2).strip()
    return d


def devices():
    return {m.group(1): m.group(2) for m in re.finditer(
        r"^Device (%s) (.*)$" % MAC, bt("devices", timeout=8), re.M)}


def is_pad(d, name=""):
    return d.get("Icon") == "input-gaming" or bool(
        re.search(r"gamepad|controller|joystick|manette", d.get("Name", name), re.I))


# ---------------------------------------------------------------- etapes
def forget(rid):
    begin("forget", "Preparation", "Suppression des anciens appairages de manette (les autres "
          "appareils Bluetooth ne sont pas touches).")
    for mac, name in devices().items():
        check_cancel(rid)
        if is_pad(info(mac), name):
            bt("remove", mac)
            log("oublie : %s (%s)" % (name, mac))
    mark("forget", "done")


def scan(rid):
    begin("scan", "Allume la manette maintenant", HOWTO)
    end = time.time() + SCAN_S
    proc = subprocess.Popen(["bluetoothctl", "--timeout", str(SCAN_S), "scan", "bredr"],
                            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    log("scan Bluetooth classique lance (%d s)" % SCAN_S)
    seen = {}
    try:
        while time.time() < end:
            check_cancel(rid)
            S["remaining"] = end - time.time()
            push()
            for mac, name in devices().items():
                if time.time() < seen.get(mac, 0):
                    continue
                d = info(mac)
                if is_pad(d, name):
                    log("manette vue : %s (%s)" % (d.get("Name", name), mac))
                    mark("scan", "done")
                    return mac, d
                seen[mac] = time.time() + 4         # reevalue plus tard : la classe arrive apres
            time.sleep(1)
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
    raise Failed("Aucune manette vue en %d s." % SCAN_S,
                 "Verifie que la manette est en mode XInput (Home + X, voyants stables) et a moins "
                 "de 2 m. Si elle est aussi appairee a un autre appareil a portee (PC, console), "
                 "coupe son Bluetooth.")


def link(rid, mac, d):
    name = d.get("Name", mac)
    S["device"] = {"name": name, "mac": mac}
    time.sleep(1)                                    # scan arrete : laisse bluez se calmer
    begin("pair", "Appairage de %s" % name)
    out = bt("pair", mac, timeout=45)
    if "Pairing successful" not in out and info(mac).get("Paired") != "yes":
        log("pair : " + (out.strip().splitlines() or ["pas de reponse"])[-1])
        raise Failed("L'appairage a ete refuse.",
                     "Eteins la manette, rallume-la avec Home + X et relance l'assistant.")
    mark("pair", "done")
    check_cancel(rid)
    begin("trust", "Confiance")
    bt("trust", mac)
    if info(mac).get("Trusted") != "yes":
        raise Failed("Impossible de marquer la manette de confiance.")
    mark("trust", "done")
    begin("connect", "Connexion")
    for n in range(1, 4):
        check_cancel(rid)
        out = bt("connect", mac, timeout=25)
        if info(mac).get("Connected") == "yes":
            break
        log("connexion %d/3 : %s" % (n, (out.strip().splitlines() or ["pas de reponse"])[-1]))
        time.sleep(2)
    else:
        raise Failed("La manette est appairee mais ne se connecte pas.",
                     "Eteins-la et rallume-la avec Home + X, puis relance l'assistant.")
    mark("connect", "done")


def input_block(mac):
    try:
        text = open("/proc/bus/input/devices").read()
    except OSError:
        return "", ""
    for blk in text.split("\n\n"):
        if mac.lower() in blk.lower():
            nm = re.search(r'N: Name="(.*)"', blk)
            hd = re.search(r"H: Handlers=(.*)", blk)
            return (nm.group(1) if nm else ""), (hd.group(1).strip() if hd else "")
    return "", ""


def verify(rid, mac, t0):
    begin("verify", "Verification")
    end = time.time() + 12
    while time.time() < end and not glob.glob("/dev/input/js*"):
        check_cancel(rid)
        time.sleep(1)
    d = info(mac)
    js = sorted(glob.glob("/dev/input/js*"))
    in_name, handlers = input_block(mac)
    vp = re.search(r"v([0-9A-Fa-f]{4})p([0-9A-Fa-f]{4})", d.get("Modalias", ""))
    vidpid = ("%s:%s" % (vp.group(1), vp.group(2))).upper() if vp else ""
    xinput = vidpid == "045E:02E0"
    if not js:
        mark("verify", "fail")
        raise Failed("Appairee et connectee, mais le noyau n'a cree aucun /dev/input/js*.",
                     "La manette est sans doute en mode Switch et non XInput : eteins-la, "
                     "rallume-la avec Home + X et relance.")
    mark("verify", "done")
    rows = [["Nom", d.get("Name", "")], ["Adresse", mac],
            ["Identifiant USB", vidpid or d.get("Modalias", "?")],
            ["Mode", "XInput (manette Xbox 360)" if xinput else "autre que XInput"],
            ["Appairee", d.get("Paired", "?")], ["Liee (bonded)", d.get("Bonded", "?")],
            ["De confiance", d.get("Trusted", "?")], ["Connectee", d.get("Connected", "?")],
            ["Peripherique", ", ".join(js) + ((" · " + handlers) if handlers else "")],
            ["Nom cote noyau", in_name or "?"], ["Duree", "%d s" % (time.time() - t0)]]
    warnings = []
    if d.get("Bonded") != "yes":
        warnings.append("Appairage non lie (Bonded: no) : la manette ne se reconnectera pas seule "
                        "(AlwaysPairable de /etc/bluetooth/main.conf ?).")
    if not xinput:
        warnings.append("Identifiant USB different de 045E:02E0 : le mapping des boutons du "
                        "robot suppose le mode XInput.")
    S["result"] = {"rows": rows, "warnings": warnings}


def finish(step, message="", hint=""):
    S.update(step=step, message=message, hint=hint, remaining=None, instruction="")
    log({"done": "termine", "error": "echec : " + message, "cancelled": "annule"}[step])


def run(rid):
    S.update(id=rid, step="forget", title="Preparation", instruction="", message="", hint="",
             log=[], device=None, result=None, remaining=None,
             steps=[{"id": i, "label": lab, "status": "todo"} for i, lab in STEPS])
    push()
    t0 = time.time()
    try:
        forget(rid)
        mac, d = scan(rid)
        link(rid, mac, d)
        verify(rid, mac, t0)
        finish("done", "Manette appairee.")
    except Cancelled:
        for s in S["steps"]:
            if s["status"] == "doing":
                s["status"] = "todo"
        finish("cancelled", "Appairage annule.")
    except Failed as e:
        for s in S["steps"]:
            if s["status"] == "doing":
                s["status"] = "fail"
        finish("error", str(e), e.hint)
    except Exception as e:                                          # noqa: BLE001
        for s in S["steps"]:
            if s["status"] == "doing":
                s["status"] = "fail"
        finish("error", "Erreur interne : %r" % (e,))


def main():
    if not os.path.isdir(DIR):
        print("bamboo-pad: %s absent" % DIR)
        return 0
    try:
        prev = json.load(open(STATE))
    except (OSError, ValueError):
        prev = {}
    last = prev.get("id", "") if prev.get("step") in ("done", "error", "cancelled") else ""
    for _ in range(20):                      # une demande arrivee PENDANT un tour : on reboucle
        verb, rid = want()
        if verb != "start" or rid == last:
            break
        last = rid
        run(rid)
    return 0


if __name__ == "__main__":
    sys.exit(main())
