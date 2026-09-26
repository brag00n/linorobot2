r"""FaceTrainer - Pipeline d'apprentissage (enrolement) du jeu de visages.

« Apprentissage » = enrolement SFace : on ne re-entraine aucun reseau, on calcule
et range les embeddings de reference (galerie) des personnes a partir des lots
d'images acquis. Fonctions pures sur le systeme de fichiers `faces/`, appelables
depuis le noeud roslite (dans un thread worker) ou en CLI (offline).

Arborescence (voir plan / FaceRecognizer) :
    faces/
      identified/<id_pred>-<name|unknown>/ gallery.npz + <id_lot>/{learning/,*.jpg}
      unknown/<id_lot>/*.jpg

Deroule (mode 3 de la specification) :
  1. Scanner unknown/<id_lot> ayant assez d'images (>= min_imgs) = « gros lots ».
  2. Split 50/50 : deplacer 50% dans <id_lot>/learning/ (enrolement), garder 50%
     en holdout (validation).
  3. Enroler = embeddings du sous-lot learning/ (moyenne = signature du lot).
  4. Valider : taux de reconnaissance du holdout contre le sous-lot learning.
  5. Cross-test : regrouper les lots (moyennes) dont la similarite cosinus depasse
     le seuil = meme personne ; rattacher aussi aux personnes DEJA identifiees.
  6. Petits lots (< min_imgs) : rattaches a une personne s'ils la reconnaissent,
     SUPPRIMES si > 80% de leurs images ne reconnaissent personne (garbage).
  7. Pour chaque personne : identified/<id_pred>-unknown/ (id_pred sequentiel) neuf
     ou existant, y ranger les lots membres, (re)generer gallery.npz.

Un verrou fichier (faces/.train.lock) evite deux passes concurrentes. Chaque
etape est journalisee via `log(event, **fields)` (visible dans MCP robot-analysis).

RECOPIE VERBATIM de `robot_control/interaction/FaceTrainer.py`, au seul import
pres (`.face_recognizer` au lieu de `.FaceRecognizer`). Le hook `log()` est ce qui
alimente les lignes de FEEDBACK de l'action ROS `TrainFaces` -- ne pas le retirer.
Duplication assumee : voir __init__.py.
"""
import os
import shutil
import time

import cv2
import numpy as np

from .face_recognizer import FaceRecognizer

_IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")


def _images(d):
    """Liste triee des images d'un dossier (non recursif)."""
    if not os.path.isdir(d):
        return []
    return sorted(os.path.join(d, f) for f in os.listdir(d)
                  if f.lower().endswith(_IMG_EXT))


class _UnionFind:
    """Union-find minimal pour regrouper les lots « meme personne »."""

    def __init__(self, items):
        self._p = {k: k for k in items}

    def find(self, x):
        while self._p[x] != x:
            self._p[x] = self._p[self._p[x]]
            x = self._p[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self._p[ra] = rb

    def groups(self):
        g = {}
        for k in self._p:
            g.setdefault(self.find(k), []).append(k)
        return list(g.values())


class FaceTrainer:
    """Execute une passe d'enrolement sur `faces/`. Voir docstring module."""

    def __init__(self, faces_dir, recognizer=None, cos_thr=0.363,
                 min_imgs=10, recog_min_ok=0.6, garbage_frac=0.8, log=None):
        self.faces_dir = faces_dir
        self.rec = recognizer or FaceRecognizer(faces_dir=faces_dir, cos_thr=cos_thr)
        self.cos_thr = float(cos_thr)
        self.min_imgs = int(min_imgs)          # « assez d'images » pour enroler un lot
        self.recog_min_ok = float(recog_min_ok)  # taux holdout mini pour un lot « bon »
        self.garbage_frac = float(garbage_frac)   # > ce taux non reconnu => petit lot poubelle
        self._log = log or (lambda *a, **k: None)
        self.unknown_dir = os.path.join(faces_dir, "unknown")
        self.identified_dir = os.path.join(faces_dir, "identified")
        self.lock_path = os.path.join(faces_dir, ".train.lock")

    # --- verrou --------------------------------------------------------------
    def _acquireLock(self):
        os.makedirs(self.faces_dir, exist_ok=True)
        try:
            fd = os.open(self.lock_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.write(fd, str(time.time()).encode())
            os.close(fd)
            return True
        except FileExistsError:
            return False

    def _releaseLock(self):
        try:
            os.remove(self.lock_path)
        except OSError:
            pass

    # --- embeddings ----------------------------------------------------------
    def _embedImages(self, paths):
        """Embeddings des images DEJA alignees (112x112) d'une liste de chemins."""
        out = []
        for p in paths:
            img = cv2.imread(p)
            if img is None:
                continue
            if img.shape[0] != 112 or img.shape[1] != 112:
                img = cv2.resize(img, (112, 112))
            try:
                out.append(self.rec.feature(img))
            except Exception:
                continue
        return out

    def _nextIdPred(self):
        """Prochain id_pred sequentiel (max existant + 1, base 1)."""
        mx = 0
        if os.path.isdir(self.identified_dir):
            for name in os.listdir(self.identified_dir):
                idp, _ = FaceRecognizer.parseIdentifiedName(name)
                if idp is not None:
                    mx = max(mx, idp)
        return mx + 1

    # --- pipeline ------------------------------------------------------------
    def run(self):
        """Execute une passe complete. Retourne un dict de synthese."""
        if not self.rec.loaded:
            ok, msg = self.rec.ready()
            if not ok:
                self._log("train_error", msg=msg)
                return {"ok": False, "error": msg}
            self.rec.build()
        if not self._acquireLock():
            self._log("train_skip", reason="locked")
            return {"ok": False, "error": "train already running (.train.lock)"}
        try:
            return self._run()
        finally:
            self._releaseLock()

    def _run(self):
        t0 = time.time()
        self._log("train_start", faces_dir=self.faces_dir, min_imgs=self.min_imgs)
        self.rec.reload()

        big, small = [], []
        if os.path.isdir(self.unknown_dir):
            for lot in sorted(os.listdir(self.unknown_dir)):
                d = os.path.join(self.unknown_dir, lot)
                imgs = _images(d)
                if not imgs:
                    continue
                (big if len(imgs) >= self.min_imgs else small).append((lot, d, imgs))

        # --- 1-4 : enroler + valider les gros lots ---------------------------
        enrolled = {}   # lot -> {"dir","mean","embs","rate","n"}
        for lot, d, imgs in big:
            learn_dir = os.path.join(d, "learning")
            os.makedirs(learn_dir, exist_ok=True)
            # split 50/50 (deterministe : 1 image sur 2) -> learning/ + holdout
            learn_src = imgs[::2]
            holdout = imgs[1::2]
            for p in learn_src:
                dst = os.path.join(learn_dir, os.path.basename(p))
                try:
                    shutil.move(p, dst)
                except Exception:
                    pass
            learn_paths = _images(learn_dir)
            embs = self._embedImages(learn_paths)
            if not embs:
                self._log("train_lot_skip", lot=lot, reason="no_embeddings")
                continue
            mean = np.mean(np.stack(embs), axis=0)
            hold_embs = self._embedImages(holdout)
            rate = (np.mean([1.0 if max(self.rec.cosine(h, e) for e in embs)
                             >= self.cos_thr else 0.0 for h in hold_embs])
                    if hold_embs else 1.0)
            enrolled[lot] = {"dir": d, "mean": mean, "embs": embs,
                             "rate": float(rate), "n": len(imgs)}
            self._log("train_lot", lot=lot, n=len(imgs),
                      learn=len(learn_paths), holdout=len(holdout),
                      rate=round(float(rate), 3),
                      good=bool(rate >= self.recog_min_ok))

        # --- 5 : regrouper gros lots + personnes existantes (union-find) -----
        keys = list(enrolled.keys())
        # cle speciale par personne deja identifiee : ("id", id_pred)
        existing = {}
        for id_pred, entry in self.rec.gallery.items():
            embs = entry["embeddings"]
            if embs is not None and len(embs):
                existing[("id", id_pred)] = np.mean(np.stack(embs), axis=0)
        uf = _UnionFind(keys + list(existing.keys()))
        allkeys = keys + list(existing.keys())
        means = {k: enrolled[k]["mean"] for k in keys}
        means.update(existing)
        for i in range(len(allkeys)):
            for j in range(i + 1, len(allkeys)):
                a, b = allkeys[i], allkeys[j]
                if self.rec.cosine(means[a], means[b]) >= self.cos_thr:
                    uf.union(a, b)

        # --- 6 : petits lots -> rattachement ou suppression ------------------
        # references disponibles = moyennes (gros lots + personnes existantes)
        small_assign = {}   # lot -> group representative key (or None)
        for lot, d, imgs in small:
            embs = self._embedImages(imgs)
            if not embs:
                continue
            # pour chaque image : meilleure personne de reference
            votes = {}
            reco = 0
            for e in embs:
                best_k, best_c = None, -1.0
                for k, m in means.items():
                    c = self.rec.cosine(e, m)
                    if c > best_c:
                        best_k, best_c = k, c
                if best_c >= self.cos_thr:
                    reco += 1
                    votes[best_k] = votes.get(best_k, 0) + 1
            reco_frac = reco / len(embs)
            if votes:
                winner = max(votes, key=votes.get)
                small_assign[lot] = (d, imgs, embs, winner)
                self._log("train_small_assign", lot=lot, n=len(imgs),
                          reco=round(reco_frac, 3))
            elif (1.0 - reco_frac) > self.garbage_frac:
                # trop peu d'images ET > 80% non reconnues -> poubelle
                try:
                    shutil.rmtree(d)
                    self._log("train_small_delete", lot=lot, n=len(imgs),
                              reco=round(reco_frac, 3))
                except OSError:
                    pass
            else:
                self._log("train_small_keep", lot=lot, n=len(imgs),
                          reco=round(reco_frac, 3))

        # --- 7 : materialiser les personnes ----------------------------------
        os.makedirs(self.identified_dir, exist_ok=True)
        persons = 0
        for group in uf.groups():
            big_members = [k for k in group if k in enrolled]
            existing_members = [k for k in group if k in existing]
            if not big_members:
                continue   # groupe purement « existant » : rien de neuf
            # cible : personne existante du groupe si presente, sinon nouveau id_pred
            if existing_members:
                id_pred = existing_members[0][1]
                name = self.rec.gallery[id_pred]["name"]
                target = self._identifiedPath(id_pred, name)
                gallery_embs = list(self.rec.gallery[id_pred]["embeddings"])
            else:
                id_pred = self._nextIdPred()
                name = "unknown"
                target = os.path.join(self.identified_dir, f"{id_pred}-{name}")
                gallery_embs = []
            os.makedirs(target, exist_ok=True)

            # ranger les gros lots membres + accumuler les embeddings de reference
            for lot in big_members:
                info = enrolled[lot]
                self._moveLot(info["dir"], target, lot)
                gallery_embs.extend(info["embs"])
            # rattacher les petits lots assignes a ce groupe
            for lot, (d, imgs, embs, winner) in list(small_assign.items()):
                if uf.find(winner) == uf.find(group[0]):
                    self._moveLot(d, target, lot)
                    gallery_embs.extend(embs)
                    del small_assign[lot]

            FaceRecognizer.saveGallery(os.path.join(target, "gallery.npz"),
                                       np.stack(gallery_embs))
            persons += 1
            self._log("train_person", id_pred=id_pred, name=name,
                      dir=os.path.basename(target),
                      lots=len(big_members), refs=len(gallery_embs))

        self.rec.reload()
        summary = {"ok": True, "big_lots": len(big), "small_lots": len(small),
                   "enrolled": len(enrolled), "persons": persons,
                   "elapsed_s": round(time.time() - t0, 2)}
        self._log("train_done", **summary)
        return summary

    # --- helpers systeme de fichiers ----------------------------------------
    def _identifiedPath(self, id_pred, name):
        """Chemin du dossier identified d'une personne existante (retrouve le suffixe)."""
        if os.path.isdir(self.identified_dir):
            for d in os.listdir(self.identified_dir):
                idp, _ = FaceRecognizer.parseIdentifiedName(d)
                if idp == id_pred:
                    return os.path.join(self.identified_dir, d)
        return os.path.join(self.identified_dir, f"{id_pred}-{name}")

    def _moveLot(self, src_dir, target_person_dir, lot):
        """Deplace unknown/<lot> sous identified/<person>/<lot> (ecrase si deja la)."""
        dst = os.path.join(target_person_dir, lot)
        if os.path.abspath(src_dir) == os.path.abspath(dst):
            return
        if os.path.exists(dst):
            shutil.rmtree(dst, ignore_errors=True)
        try:
            shutil.move(src_dir, dst)
        except Exception as e:
            self._log("train_move_error", lot=lot, err=str(e))
