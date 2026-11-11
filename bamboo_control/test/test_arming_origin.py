"""Origine de l'armement : quatre cas, deux surveillances (2026-11-11). Teste la REGLE de _onGuardStop et de la perte de manette en
extrayant leur code du teleop (le noeud complet exige ROS) et en l'executant sur un faux objet. Si le texte de ces fonctions change
de forme, le test echoue ici plutot que sur le robot."""
import os
import re
import sys
import textwrap
import types

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

SRC = os.path.join(os.path.dirname(__file__), "..", "bamboo_control", "teleop_node.py")


def _method(name):
    text = open(SRC, encoding="utf-8").read().replace("\r\n", "\n")
    m = re.search(r"\n    def %s\(self, msg\):\n(.*?)(?=\n    def |\Z)" % re.escape(name), text, re.S)
    assert m, "methode %s introuvable dans teleop_node.py" % name
    return textwrap.dedent("    def %s(self, msg):\n%s" % (name, m.group(1)))


class Log:
    def __init__(self):
        self.lines = []

    def error(self, m):
        self.lines.append(("error", m))

    def warning(self, m):
        self.lines.append(("warning", m))

    warn = warning

    def info(self, m):
        self.lines.append(("info", m))


class Fake:
    """Le strict necessaire de bamboo_teleop pour _onGuardStop."""

    def __init__(self, on, latched, origin):
        self._motionOn, self._armLatched, self._armOrigin = on, latched, origin
        self.calls = []
        self._navCli = types.SimpleNamespace(service_is_ready=lambda: True)
        self.log = Log()

    def get_logger(self):
        return self.log

    def _stopMotion(self):
        self.calls.append("stop")

    def _nav(self, c):
        self.calls.append("nav:" + c)

    def _calibCancel(self, why):
        self.calls.append("calib:" + why)

    def _publishState(self):
        self.calls.append("publish")

    def _rumble(self, n):
        self.calls.append("rumble")


def _guard_stop(on, latched, origin):
    ns = {}
    exec(_method("_onGuardStop"), ns)                      # noqa: S102 - code du depot, extrait tel quel
    f = Fake(on, latched, origin)
    ns["_onGuardStop"](f, types.SimpleNamespace(data=True))
    return f


@pytest.mark.parametrize("latched,origin,disarms", [
    (False, "cc", True),          # Armee depuis le CC            -> desarmee par la perte du CC
    (True, "cc", False),          # Armee verrouillee depuis le CC -> jamais
    (False, "manette", False),    # Armee depuis la manette        -> ne depend pas du CC
    (True, "manette", False),     # Armee verrouillee manette      -> jamais
])
def test_the_cc_guard_only_disarms_a_simple_cc_arming(latched, origin, disarms):
    f = _guard_stop(True, latched, origin)
    if disarms:
        assert f._motionOn is False and f._armOrigin == "" and f._armLatched is False
        assert f.calls[0] == "stop" and "nav:cancel_all" in f.calls
    else:
        assert f._motionOn is True                          # toujours arme
        assert f.calls == []                                # ni arret, ni annulation du but : la mission continue
        assert any("rien n'est coupe" in m for _l, m in f.log.lines)


def test_nothing_happens_when_the_traction_is_already_off():
    f = _guard_stop(False, False, "")
    assert f.calls == [] and f._motionOn is False


def test_the_stop_goes_first_when_the_cc_arming_is_cut():
    f = _guard_stop(True, False, "cc")
    assert f.calls.index("stop") < f.calls.index("nav:cancel_all") < f.calls.index("publish")


def test_the_loss_of_the_gamepad_does_not_disarm_a_cc_arming():
    """Lecture directe de la condition : l'armement CC simple ne depend pas de la manette."""
    text = open(SRC, encoding="utf-8").read().replace("\r\n", "\n")
    m = re.search(r"silence > self\._disarmOnLoss and \\\n\s+not self\._armLatched and self\._armOrigin != \"cc\":", text)
    assert m, "la condition de perte de manette doit exclure l'armement venu du CC"


def test_every_arming_path_records_its_origin():
    text = open(SRC, encoding="utf-8").read().replace("\r\n", "\n")
    assert text.count('self._armOrigin = "manette"') + text.count('"manette"') >= 2        # B long simple et verrouille
    assert '"cc"' in text and 'self._armOrigin = ""' in text                             # armement CC et desarmement
    assert text.count("self._armOrigin") >= 8
