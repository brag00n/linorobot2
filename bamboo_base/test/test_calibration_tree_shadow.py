"""Aucun attribut d'une feuille py_trees ne doit masquer une methode de Behaviour.

Un `self.tick = 0.0` a deja fait echouer la decharge moteur avec « 'float' object is not
callable » : py_trees appelle `tick()` et trouvait un nombre. rclpy n'etant pas disponible hors
conteneur, on lit les affectations `self.<nom> =` dans le source.
"""
import ast
import os

import py_trees

SRC = os.path.join(os.path.dirname(__file__), "..", "bamboo_base", "calibration_node.py")


def test_no_attribute_shadows_behaviour_method():
    reserved = {n for n in dir(py_trees.behaviour.Behaviour) if not n.startswith("__")}
    tree = ast.parse(open(SRC, encoding="utf-8").read())
    bad = []
    for cls in [c for c in ast.walk(tree) if isinstance(c, ast.ClassDef)]:
        if not cls.bases:
            continue
        for node in ast.walk(cls):
            if isinstance(node, ast.Attribute) and isinstance(node.ctx, ast.Store) \
                    and isinstance(node.value, ast.Name) and node.value.id == "self" \
                    and node.attr in reserved:
                bad.append((cls.name, node.attr, node.lineno))
    assert not bad, bad
