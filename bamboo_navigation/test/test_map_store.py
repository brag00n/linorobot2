"""MapStore (cartes-dossiers), destinations et gates, sans ROS."""
import os
import sys
import tempfile

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

yaml = pytest.importorskip("yaml")
from bamboo_navigation import map_io  # noqa: E402


def _store():
    return map_io.MapStore(tempfile.mkdtemp())


def test_create_never_overwrites_and_current_survives():
    s = _store()
    a = s.create(name="salon")
    b = s.create(name="salon")
    assert (a, b) == ("salon", "salon_2")
    assert s.current() == ""
    s.set_current(b)
    assert map_io.MapStore(s.dir).current() == "salon_2"       # relu par un autre objet
    assert [e["ev"] for e in s.events(a)] == ["creation"]
    assert {m["name"]: m["kind"] for m in s.list()} == {"salon": "image", "salon_2": "image"}


def test_list_order_is_creation_not_modification():
    s = _store()
    old = s.create(name="ancienne")
    meta = s.load_meta(old)
    meta["created"] = "2026-10-01T10:00:00"
    s.save_meta(old, meta)
    new = s.create(name="recente")
    meta = s.load_meta(old)                      # l'ancienne est MODIFIEE en dernier
    s.save_meta(old, meta)
    assert [m["name"] for m in s.list()] == [new, old]


def test_current_pointing_to_a_missing_map_is_empty():
    s = _store()
    s.set_current("disparue")
    assert s.current() == ""


def test_grid_and_posegraph_commit():
    s = _store()
    n = s.create(name="c")
    data = [-1] * 4 + [0] * 4 + [100] * 4
    s.write_grid(n, 4, 3, 0.05, (-1.0, -2.0, 0.0), data)
    w, h, res, origin, back = map_io.read_map(s.path(n, "map.yaml"))
    assert (w, h, origin[:2], back) == (4, 3, (-1.0, -2.0), data)
    assert abs(res - 0.05) < 1e-9
    assert not s.commit_posegraph(n)                 # slam_toolbox n'a rien ecrit
    for ext in (".posegraph", ".data"):
        with open(s.posegraph_tmp(n) + ext, "wb") as fh:
            fh.write(b"x")
    assert s.commit_posegraph(n) and s.has_posegraph(n)
    assert not os.path.exists(s.posegraph_tmp(n) + ".data")
    assert s.list()[0]["kind"] == "carte"
    assert not [f for f in os.listdir(s.path(n)) if f.endswith(".tmp")]


def test_backup_keeps_the_last_versions():
    s = _store()
    n = s.create(name="c")
    assert s.backup(n) == ""                         # rien encore a copier
    for k in range(7):
        with open(s.path(n, "slam.posegraph"), "w") as fh:
            fh.write("v%d" % k)
        s.backup(n, keep=5)
    hist = s.history(n)
    assert len(hist) == 5
    with open(os.path.join(hist[0], "slam.posegraph")) as fh:
        assert fh.read() == "v6"                     # la plus recente en tete
    with open(os.path.join(hist[-1], "slam.posegraph")) as fh:
        assert fh.read() == "v2"


def test_trash_moves_the_map_out_of_the_list():
    s = _store()
    a, b = s.create(name="garde"), s.create(name="jetee")
    s.set_current(b)
    dst = s.trash(b)
    assert os.path.isfile(os.path.join(dst, "meta.yaml"))       # recuperable
    assert [m["name"] for m in s.list()] == [a]
    assert s.current() == ""
    with pytest.raises(FileNotFoundError):
        s.trash(b)


def test_events_skip_a_truncated_last_line():
    s = _store()
    n = s.create(name="c")
    s.append_event(n, "gate", n=3, x=1.23456)
    with open(s.path(n, "events.jsonl"), "a", encoding="utf-8") as fh:
        fh.write('{"t": "2026-10-13T')                # coupure en pleine ecriture
    ev = s.events(n)
    assert [e["ev"] for e in ev] == ["creation", "gate"] and ev[1]["x"] == 1.235


def test_destinations_numbering_never_reuses_a_number():
    meta = map_io.new_meta("c")
    map_io.dest_add(meta, 1, 2, 0)
    map_io.dest_add(meta, 3, 4, 0, "cuisine")
    assert [d["name"] for d in meta["destinations"]] == ["p1", "cuisine"]
    assert map_io.dest_del(meta, "1")["name"] == "p1"
    assert map_io.dest_add(meta, 5, 6, 0)["name"] == "p3"
    assert map_io.dest_rename(meta, "p3", "porte")["n"] == 3
    assert map_io.dest_find(meta, "porte")["x"] == 5.0
    assert map_io.dest_del(meta, "absente") is None


def test_cycle():
    items = [{"n": 2}, {"n": 5}, {"n": 9}]
    assert map_io.cycle(items, None, +1) == 2 and map_io.cycle(items, None, -1) == 9
    assert map_io.cycle(items, 9, +1) == 2 and map_io.cycle(items, 2, -1) == 9
    assert map_io.cycle([], 3, +1) is None


def test_gates_keep_their_number_and_close_when_known():
    meta = map_io.new_meta("c")
    live = map_io.gates_update(meta, [{"x": 1.0, "y": 0.0, "size_m": 0.6},
                                      {"x": 4.0, "y": 0.0, "size_m": 0.4}])
    assert [g["n"] for g in live] == [1, 2]
    # Le bord de l'inconnu recule de 0,3 m : meme gate, position suivie.
    live = map_io.gates_update(meta, [{"x": 1.3, "y": 0.0, "size_m": 0.6},
                                      {"x": 4.0, "y": 0.1, "size_m": 0.4}])
    assert [g["n"] for g in live] == [1, 2] and map_io.gate_find(meta, 1)["x"] == 1.3
    # La gate 1 disparait (zone connue), une nouvelle apparait : numero 3.
    map_io.gate_find(meta, 2)["status"] = "en_cours"
    live = map_io.gates_update(meta, [{"x": 8.0, "y": 0.0, "size_m": 0.5}])
    assert [g["n"] for g in live] == [3]
    assert map_io.gate_find(meta, 1)["status"] == "exploree"
    assert map_io.gate_find(meta, 2)["status"] == "en_cours"   # son but decidera


def test_gates_reopen_for_a_new_session():
    meta = map_io.new_meta("c")
    map_io.gates_update(meta, [{"x": 0, "y": 0}, {"x": 2, "y": 0}, {"x": 4, "y": 0}])
    for n, st in ((1, "inatteignable"), (2, "abandonnee"), (3, "atteinte")):
        map_io.gate_find(meta, n)["status"] = st
    map_io.gates_reopen(meta)
    assert [g["status"] for g in meta["gates"]] == ["ouverte", "ouverte", "atteinte"]


def test_import_legacy_flat_map():
    s = _store()
    map_io.write_map(os.path.join(s.dir, "vieille"), 4, 3, 0.05, (0.0, 0.0, 0.0), [0] * 12)
    map_io.save_waypoint(s.dir, "vieille", "depart", 1, 2, 0.5)
    map_io.save_waypoint(s.dir, "vieille", "porte", 3, 4, 0.0)
    map_io.save_frontier_log(s.dir, "vieille", [{"n": 1, "x": 0.5, "y": 0.5,
                                                 "s": "inatteignable"},
                                                {"n": 2, "x": 1.5, "y": 0.5, "s": "en_cours"}])
    assert [m["kind"] for m in s.list()] == ["ancienne"]
    s.import_legacy("vieille")
    meta = s.load_meta("vieille")
    assert meta["home"] == [1.0, 2.0, 0.5]
    assert [d["name"] for d in meta["destinations"]] == ["porte"]
    assert [g["status"] for g in meta["gates"]] == ["inatteignable", "ouverte"]
    assert meta["gate_seq"] == 2
    assert [m["kind"] for m in s.list()] == ["image"]          # plus listee en double
    assert s.has_image("vieille") and not s.has_posegraph("vieille")
