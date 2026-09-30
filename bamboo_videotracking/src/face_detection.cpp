// face_detection.cpp - implementation, cf. l'en-tete pour l'arbitrage detecteur/tracker.

#include "bamboo_videotracking/face_detection.hpp"

#include <algorithm>
#include <cmath>

namespace bamboo_videotracking
{

namespace
{
// Noms de fichiers du depot firmware, bind-monte dans le conteneur. Ils sont ICI et pas en
// parametre ROS : ce sont des noms de modeles, pas des reglages, et un modele substitue
// change la signification de tous les seuils.
constexpr const char * kYunetFile = "face_detection_yunet_2023mar.onnx";
constexpr const char * kRes10Model = "res10_300x300_ssd_iter_140000_fp16.caffemodel";
constexpr const char * kRes10Proto = "deploy.prototxt";
constexpr const char * kHaarFile = "haarcascade_frontalface_default.xml";
/// Repli systeme pour Haar : libopencv-dev installe les cascades ici. On cherche d'abord
/// dans models_dir pour qu'un banc puisse figer sa propre version.
constexpr const char * kHaarSystem =
  "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";

/// Entree du reseau res10, figee par son prototxt : la changer ne le rend pas plus precis,
/// elle le rend faux.
constexpr int kRes10Size = 300;

std::string join(const std::string & dir, const char * name)
{
  if (dir.empty()) { return std::string(name); }
  if (dir.back() == '/') { return dir + name; }
  return dir + "/" + name;
}
}  // namespace

// ---------------------------------------------------------------------------
// Geometrie

double FaceDetection::iou(const cv::Rect & a, const cv::Rect & b)
{
  const int inter = (a & b).area();
  const int uni = a.area() + b.area() - inter;
  if (uni <= 0) { return 0.0; }
  return static_cast<double>(inter) / static_cast<double>(uni);
}

bool FaceDetection::degenerate(const cv::Rect & r)
{
  // Teste EN PREMIER partout, parce que les autres criteres divisent par l'aire. Le seuil
  // de 4 px n'est pas cosmetique : MIL rend parfois une boite de 1 px de haut quand il
  // perd la cible, et une telle boite passerait tous les tests de ratio.
  return r.width < 4 || r.height < 4;
}

double FaceDetection::interFrac(const cv::Rect & r, int w, int h)
{
  if (r.area() <= 0) { return 0.0; }
  const cv::Rect in = r & cv::Rect(0, 0, w, h);
  return static_cast<double>(in.area()) / static_cast<double>(r.area());
}

bool FaceDetection::implausible(const cv::Rect & r, int w, int h) const
{
  // D'abord les deux gardes ABSOLUES du prototype (FaceDetection._implausible), et dans
  // cet ordre : le rapport d'aire ne protege de rien quand l'aire de REFERENCE est deja
  // fausse -- une boite gonflee des l'ancrage donne un ref_area_ enorme, donc un rapport
  // eternellement sage, et le verrou suit l'arriere-plan sans qu'aucun seuil ne bronche.
  if (r.width > w * 0.9 || r.height > h * 0.9) { return true; }
  if (static_cast<double>(r.area()) > cfg_.max_area_frac * static_cast<double>(w) * h) {
    return true;
  }
  // Puis la vraisemblance RELATIVE au verrou : un visage proche et un visage lointain sont
  // tous deux plausibles, mais un visage qui QUADRUPLE d'aire en quelques trames ne l'est
  // pas -- c'est un tracker qui vient d'avaler l'arriere-plan.
  if (ref_area_ > 0.0) {
    const double ratio = static_cast<double>(r.area()) / ref_area_;
    if (ratio > cfg_.max_grow || ratio < cfg_.area_ratio_min) { return true; }
  }
  return interFrac(r, w, h) < cfg_.inter_frac_min;
}

// ---------------------------------------------------------------------------
// Construction

bool FaceDetection::build(
  const std::string & models_dir, const DetectConfig & cfg, std::string & err)
{
  models_dir_ = models_dir;
  cfg_ = cfg;
  err.clear();

  // ViT : refus EXPLICITE, jamais un repli. L'OpenCV 4.5.4 de Humble ne porte pas
  // cv::TrackerVit (arrive en 4.7) ; accepter le mode en servant du MIL ferait croire au
  // banc V6.3 qu'il mesure ViT.
  if (cfg_.track_mode == "vit") {
    err = "track_mode 'vit' indisponible : cv::TrackerVit absent d'OpenCV " CV_VERSION
      " (requiert >= 4.7). Modes disponibles : none, mil.";
    return false;
  }

  if (cfg_.detector == "yunet") {
    const std::string path = join(models_dir_, kYunetFile);
    // Taille d'entree provisoire : setInputSize() la corrige a chaque trame, la detection
    // devant suivre det_width sans reconstruire le detecteur.
    yunet_ = cv::FaceDetectorYN::create(
      path, "", cv::Size(cfg_.det_width, cfg_.det_width),
      static_cast<float>(cfg_.det_conf), 0.3f, 5000);
    if (yunet_.empty()) {
      err = "YuNet illisible : " + path;
      return false;
    }
  } else if (cfg_.detector == "dnn") {
    const std::string model = join(models_dir_, kRes10Model);
    const std::string proto = join(models_dir_, kRes10Proto);
    try {
      res10_ = cv::dnn::readNetFromCaffe(proto, model);
    } catch (const cv::Exception & e) {
      err = std::string("res10 illisible : ") + e.what();
      return false;
    }
    if (res10_.empty()) {
      err = "res10 illisible : " + model;
      return false;
    }
  } else if (cfg_.detector == "haar") {
    if (!haar_.load(join(models_dir_, kHaarFile)) && !haar_.load(kHaarSystem)) {
      err = std::string("cascade Haar illisible : ni ") + join(models_dir_, kHaarFile) +
        " ni " + kHaarSystem;
      return false;
    }
  } else {
    err = "detecteur inconnu : '" + cfg_.detector + "' (attendu haar | dnn | yunet)";
    return false;
  }
  return true;
}

bool FaceDetection::reconfigure(const DetectConfig & cfg, std::string & err)
{
  // On reconstruit tout plutot que de trier ce qui a change : le cout est une fois par
  // changement de mode, et un tri partiel serait exactement le genre de code ou un champ
  // oublie produit un etat incoherent invisible.
  const DetectConfig old = cfg_;
  if (!build(models_dir_, cfg, err)) {
    cfg_ = old;   // la configuration precedente reste EN VIGUEUR, le noeud continue de suivre
    return false;
  }
  releaseLock("switch");
  return true;
}

void FaceDetection::releaseLock(const std::string & reason)
{
  locked_ = false;
  lost_reason_ = reason;
  cur_score_ = 0.0;
  cur_landmarks_.clear();
  ref_area_ = 0.0;
  miss_streak_ = 0;
  tracker_.release();
}

bool FaceDetection::initTracker(const cv::Mat & frame, const cv::Rect & box)
{
  if (cfg_.track_mode == "none") {
    tracker_.release();
    return true;   // mode detecteur seul : le verrou existe, l'interpolation non
  }
  const cv::Rect safe = box & cv::Rect(0, 0, frame.cols, frame.rows);
  if (degenerate(safe)) { return false; }
  tracker_ = cv::TrackerMIL::create();
  try {
    tracker_->init(frame, safe);
  } catch (const cv::Exception &) {
    tracker_.release();
    return false;
  }
  // AIRE DE REFERENCE posee ICI, a l'instant du verrou, et nulle part ailleurs : c'est
  // l'etalon du test de vraisemblance. La rafraichir a chaque recalage laisserait une
  // derive lente passer, chaque trame etant plausible par rapport a la precedente.
  ref_area_ = static_cast<double>(safe.area());
  return true;
}

void FaceDetection::tickFps(double now_s)
{
  ++fps_count_;
  if (fps_win_t0_ <= 0.0) { fps_win_t0_ = now_s; return; }
  const double el = now_s - fps_win_t0_;
  if (el >= 1.0) {
    det_fps_ = static_cast<double>(fps_count_) / el;
    fps_count_ = 0;
    fps_win_t0_ = now_s;
  }
}

// ---------------------------------------------------------------------------
// Detection

std::vector<cv::Rect> FaceDetection::detect(
  const cv::Mat & frame, std::vector<Landmarks> & landmarks, std::vector<float> & scores)
{
  det_boxes_.clear();
  det_landmarks_.clear();
  det_rows_.release();
  landmarks.clear();
  scores.clear();
  if (frame.empty()) { return det_boxes_; }

  const int fw = frame.cols;
  const int fh = frame.rows;

  // Reduction AVANT detection : c'est le seul levier de cadence qui ne change pas
  // l'algorithme. INTER_AREA et pas INTER_LINEAR, comme le prototype : en reduction c'est
  // un moyennage de bloc, donc il ne cree pas les artefacts d'aliasing qui font halluciner
  // un detecteur sur les textures fines (grillage, rayures de vetement).
  cv::Mat small;
  double scale = 1.0;
  if (cfg_.det_width > 0 && fw > cfg_.det_width) {
    scale = static_cast<double>(fw) / static_cast<double>(cfg_.det_width);
    const int sh = static_cast<int>(std::lround(fh / scale));
    cv::resize(frame, small, cv::Size(cfg_.det_width, sh), 0, 0, cv::INTER_AREA);
  } else {
    small = frame;
  }

  if (cfg_.detector == "yunet" && !yunet_.empty()) {
    yunet_->setInputSize(small.size());
    cv::Mat rows;
    yunet_->detect(small, rows);
    if (!rows.empty()) {
      // ON GARDE LES 15 COLONNES BRUTES, remises a l'echelle de la trame PLEINE. Les
      // colonnes 4..13 sont les 5 points de repere qu'alignCrop de SFace exige ; les jeter
      // pour ne garder que la boite couterait la partie "alignement" de la reconnaissance,
      // et le detecteur les a deja calcules gratuitement.
      det_rows_ = cv::Mat(rows.rows, 15, CV_32F, 0.0f);
      for (int i = 0; i < rows.rows; ++i) {
        const float * r = rows.ptr<float>(i);
        float * d = det_rows_.ptr<float>(i);
        for (int c = 0; c < 14; ++c) {
          d[c] = static_cast<float>(r[c] * scale);
        }
        d[14] = r[14];   // le score n'est PAS une longueur : pas de mise a l'echelle
        const cv::Rect box(
          static_cast<int>(std::lround(d[0])), static_cast<int>(std::lround(d[1])),
          static_cast<int>(std::lround(d[2])), static_cast<int>(std::lround(d[3])));
        Landmarks lm;
        lm.reserve(5);
        for (int k = 0; k < 5; ++k) {
          lm.emplace_back(d[4 + 2 * k], d[5 + 2 * k]);
        }
        det_boxes_.push_back(box);
        det_landmarks_.push_back(lm);
        scores.push_back(d[14]);
      }
    }
  } else if (cfg_.detector == "dnn" && !res10_.empty()) {
    cv::Mat blob = cv::dnn::blobFromImage(
      small, 1.0, cv::Size(kRes10Size, kRes10Size), cv::Scalar(104.0, 177.0, 123.0), false,
      false);
    res10_.setInput(blob);
    const cv::Mat out = res10_.forward();
    // Sortie 1x1xNx7 : [_, _, confiance, x1, y1, x2, y2] en coordonnees NORMALISEES.
    const cv::Mat det(out.size[2], out.size[3], CV_32F, const_cast<float *>(out.ptr<float>()));
    for (int i = 0; i < det.rows; ++i) {
      const float conf = det.at<float>(i, 2);
      if (conf < static_cast<float>(cfg_.det_conf)) { continue; }
      const int x1 = static_cast<int>(std::lround(det.at<float>(i, 3) * fw));
      const int y1 = static_cast<int>(std::lround(det.at<float>(i, 4) * fh));
      const int x2 = static_cast<int>(std::lround(det.at<float>(i, 5) * fw));
      const int y2 = static_cast<int>(std::lround(det.at<float>(i, 6) * fh));
      det_boxes_.push_back(cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)));
      det_landmarks_.push_back(Landmarks{});   // res10 ne rend PAS de points de repere
      scores.push_back(conf);
    }
  } else if (cfg_.detector == "haar" && !haar_.empty()) {
    cv::Mat gray;
    cv::cvtColor(small, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);
    std::vector<cv::Rect> found;
    haar_.detectMultiScale(gray, found, 1.2, 5, 0, cv::Size(30, 30));
    for (const cv::Rect & r : found) {
      det_boxes_.push_back(
        cv::Rect(
          static_cast<int>(std::lround(r.x * scale)), static_cast<int>(std::lround(r.y * scale)),
          static_cast<int>(std::lround(r.width * scale)),
          static_cast<int>(std::lround(r.height * scale))));
      det_landmarks_.push_back(Landmarks{});
      // Haar ne donne AUCUNE confiance : on annonce 1.0 plutot qu'une valeur inventee
      // intermediaire, et le commentaire est la pour que score_min ne soit pas interprete
      // comme comparable entre detecteurs.
      scores.push_back(1.0f);
    }
  }

  landmarks = det_landmarks_;
  return det_boxes_;
}

Landmarks FaceDetection::landmarksFor(const cv::Rect & box) const
{
  // Appariement par EGALITE ENTIERE de x et y, pas par recouvrement. Les boites viennent du
  // meme calcul que det_rows_ donc elles sont identiques au bit pres ; une tolerance
  // flottante, elle, pourrait apparier le visage du VOISIN dans un groupe serre, et donnerait
  // un alignement credible sur la mauvaise personne.
  for (size_t i = 0; i < det_boxes_.size(); ++i) {
    if (det_boxes_[i].x == box.x && det_boxes_[i].y == box.y) {
      return det_landmarks_[i];
    }
  }
  return Landmarks{};
}

// Un CYCLE de detection, et la mise a jour de tout ce qui se compte PAR CYCLE. La porte
// redetect_ms fait que ces cycles sont maintenant plus rares que les trames : tout ce qui
// se comptait par trame doit donc passer ici, sinon max_det_misses et det_fps mesureraient
// la cadence video et non celle de la detection.
void FaceDetection::runDetect(const cv::Mat & frame, double now_s)
{
  std::vector<Landmarks> lms;
  std::vector<float> scores;
  detect(frame, lms, scores);
  det_scores_ = scores;
  tickFps(now_s);
  last_redetect_s_ = now_s;
  last_n_faces_ = static_cast<int>(det_boxes_.size());
}

double FaceDetection::scoreOf(size_t i) const
{
  // Haar ne rend aucune confiance : detect() annonce alors 1.0, et c'est cette valeur-la
  // qu'on relaie ici plutot qu'une valeur intermediaire inventee.
  return (i < det_scores_.size()) ? static_cast<double>(det_scores_[i]) : 1.0;
}

int FaceDetection::pickLargest() const
{
  // Verrou LIBRE : la plus GRANDE boite, donc le visage le plus proche. Critere du
  // prototype, et le bon pour une camera de suivi : le sujet qui s'adresse au robot est
  // celui qui s'en approche.
  int best = -1;
  int best_area = -1;
  for (size_t i = 0; i < det_boxes_.size(); ++i) {
    if (scoreOf(i) < cfg_.score_min) { continue; }
    if (det_boxes_[i].area() > best_area) {
      best_area = det_boxes_[i].area();
      best = static_cast<int>(i);
    }
  }
  return best;
}

int FaceDetection::pickNearest(const cv::Rect & ref, double & best_iou) const
{
  // Verrou TENU : la detection qui recouvre le plus la position courante, jamais la plus
  // grande ni la plus confiante -- ces deux criteres feraient sauter le verrou sur un
  // passant qui entre dans le cadre. DIVERGENCE ASSUMEE avec le prototype, qui reprend la
  // plus grande meme sous verrou.
  best_iou = -1.0;
  int best = -1;
  for (size_t i = 0; i < det_boxes_.size(); ++i) {
    if (scoreOf(i) < cfg_.score_min) { continue; }
    const double v = iou(det_boxes_[i], ref);
    if (v > best_iou) {
      best_iou = v;
      best = static_cast<int>(i);
    }
  }
  return best;
}

TrackStateOut & FaceDetection::publish(TrackStateOut & st, double now_s, int fw, int fh) const
{
  st.locked = locked_;
  st.lost_reason = lost_reason_;
  st.lock_id = lock_id_;
  if (locked_) {
    st.has_main = true;
    st.main = cur_ & cv::Rect(0, 0, fw, fh);
    st.main_landmarks = cur_landmarks_;
    st.score = cur_score_;
    st.lock_age_s = now_s - lock_t0_;
  } else {
    st.src = "off";
  }
  return st;
}

// ---------------------------------------------------------------------------
// Machine a etats du verrou. LE COEUR DE L'ALGORITHME.
//
// ORDRE DES ETAGES, et c'est tout l'objet du lot H5 : le TRACKER passe AVANT le detecteur,
// et le detecteur ne repasse qu'a la periode redetect_ms (ou immediatement si le tracker a
// lache). Avant, le detecteur tournait a CHAQUE trame et le tracker etait reinitialise
// juste apres -- soit les deux etages les plus chers du chemin chaud payes ensemble a
// chaque trame, pour un detecteur dont la sortie etait ecrasee dans la foulee.
// Mesure au banc T15b (640x480, YuNet a det_width 320, MIL) : YuNet 41,9 ms et MIL
// init+update 218,2 ms, contre MIL update SEUL 167,8 ms. La porte retire donc ~92 ms de la
// trame courante quatre fois sur cinq a 400 ms de periode.
// Le detecteur reste le MAITRE : il fait foi des qu'il parle, le tracker n'est qu'un
// interpolateur entre deux de ses passages -- jamais une source de verite.

TrackStateOut FaceDetection::trackStep(const cv::Mat & frame, double now_s)
{
  TrackStateOut st;
  if (frame.empty()) { return st; }
  const int fw = frame.cols;
  const int fh = frame.rows;

  // --- 1. VERROU LIBRE : le detecteur est le PORTIER --------------------------------
  // On ne verrouille QUE sur un visage detecte. Un tracker initialise sur autre chose
  // suivrait cette autre chose avec une confiance parfaite, et la camera irait la regarder.
  if (!locked_) {
    runDetect(frame, now_s);
    st.n_faces = last_n_faces_;
    miss_streak_ = 0;
    const int best = pickLargest();
    if (best >= 0 && initTracker(frame, det_boxes_[best])) {
      locked_ = true;
      lost_reason_.clear();
      cur_ = det_boxes_[best];
      cur_score_ = scoreOf(static_cast<size_t>(best));
      cur_landmarks_ = det_landmarks_[best];
      last_confirm_s_ = now_s;
      ++lock_id_;          // NOUVEL EPISODE : seule transition libre -> verrouille. C'est
      lock_t0_ = now_s;    // ce numero qui vide l'EMA de facerecog_node, donc il ne doit
      st.src = "detect";   // changer QU'ICI -- un redetect garde le meme episode.
    }
    return publish(st, now_s, fw, fh);
  }

  // --- 2. VERROU TENU : le TRACKER D'ABORD ------------------------------------------
  bool track_ok = false;
  cv::Rect tracked;
  std::string lost_reason;
  if (tracker_) {
    cv::Rect upd;
    bool ok = false;
    try {
      ok = tracker_->update(frame, upd);
    } catch (const cv::Exception &) {
      ok = false;
    }
    // Ordre des tests = ordre de SEVERITE, et on retient le PREMIER motif declencheur :
    // c'est lui qui part dans lost_reason et qui rend le decrochage lisible au HUD.
    if (!ok) {
      lost_reason = "update_ko";
    } else if (degenerate(upd)) {
      lost_reason = "degenerate";
    } else if (implausible(upd, fw, fh)) {
      lost_reason = "implausible";
    } else {
      track_ok = true;
      tracked = upd;
    }
  }
  const bool lost = !lost_reason.empty();

  // --- 3. LA PORTE DE RE-DETECTION --------------------------------------------------
  // Immediate si le tracker a lache (il faut rattraper le verrou tout de suite), sinon
  // periodique. Mode "none" : aucun tracker, donc aucune interpolation possible et la
  // detection reprend a chaque trame -- ce mode garde son comportement historique.
  const bool do_redetect = lost || !tracker_ ||
    (now_s - last_redetect_s_) * 1000.0 >= cfg_.redetect_ms;
  int best = -1;
  // Hors cycle de detection, le nombre de visages est celui du DERNIER cycle : c'est une
  // mesure par cycle, pas par trame. Le HUD affiche ainsi la derniere valeur MESUREE et non
  // zero, qui se lirait comme "personne devant la camera".
  st.n_faces = last_n_faces_;
  if (do_redetect) {
    runDetect(frame, now_s);
    st.n_faces = last_n_faces_;
    double best_iou = -1.0;
    best = pickNearest(track_ok ? tracked : cur_, best_iou);
    // Echecs detecteur CONSECUTIFS comptes en CYCLES de re-detection, comme au prototype :
    // max_det_misses devient ainsi une duree (~max_det_misses x redetect_ms) independante
    // de la cadence video, au lieu de dependre du nombre de trames traitees.
    miss_streak_ = (best >= 0) ? 0 : miss_streak_ + 1;
    if (best >= 0) {
      st.src = (best_iou < cfg_.iou_reanchor) ? "recenter" : "reanchor";
    }
  }

  // --- 4. LE DETECTEUR FAIT FOI : on se recale dessus -------------------------------
  if (best >= 0) {
    const cv::Rect box = det_boxes_[best];
    // Le tracker est REINITIALISE sur la verite terrain a chaque recalage : c'est ce qui
    // l'empeche de cumuler sa derive entre deux passages du detecteur.
    if (initTracker(frame, box)) {
      cur_ = box;
      cur_score_ = scoreOf(static_cast<size_t>(best));
      cur_landmarks_ = det_landmarks_[best];
      last_confirm_s_ = now_s;
      // Le verrou etait perdu et la detection le RATTRAPE : meme episode, meme personne,
      // donc lock_id_ ne bouge pas -- seule la source le dit.
      if (lost) { st.src = "redetect"; }
      return publish(st, now_s, fw, fh);
    }
  }

  // --- 5. Perdu, et aucune detection pour rattraper -> on LACHE ----------------------
  if (lost) {
    releaseLock(lost_reason);
    return publish(st, now_s, fw, fh);
  }

  // --- 6. Detecteur muet mais tracker qui tient : decision de maintien --------------
  // Ces deux gardes ne sont evaluees QUE sur un cycle de re-detection : hors cycle le
  // detecteur ne s'est pas tu, il n'a pas ete consulte, et compter ce silence relacherait
  // le verrou pour une raison inexistante.
  if (do_redetect) {
    // SECURITE ANTI-DERIVE, PRIORITAIRE sur tout le reste : un tracker accroche a un mur
    // suit le mur avec une confiance parfaite. Le detecteur muet trop longtemps est le seul
    // signal disponible que la cible est partie.
    if (miss_streak_ >= cfg_.max_det_misses) {
      releaseLock("det_miss");
      return publish(st, now_s, fw, fh);
    }
    // HOLD borne par hold_ms, assoupli pour un verrou qui etait FRANC : un bon verrou
    // merite plus de patience qu'un verrou limite (suivi de profil).
    const double hold_ms = (cur_score_ >= cfg_.hold_score_min) ? cfg_.hold_ms * 2.0
                                                               : cfg_.hold_ms;
    if ((now_s - last_confirm_s_) * 1000.0 > hold_ms) {
      releaseLock("hold_timeout");
      return publish(st, now_s, fw, fh);
    }
  }

  if (!track_ok) {
    // Ni detection ni interpolation (mode "none") : on tient sur la DERNIERE position
    // connue, bornee par les deux gardes ci-dessus. cur_ reste tel quel.
    st.src = "track";
    return publish(st, now_s, fw, fh);
  }
  cur_ = tracked;
  cur_landmarks_.clear();   // boite INTERPOLEE : aucun point de repere valide
  st.src = "track";
  return publish(st, now_s, fw, fh);
}

}  // namespace bamboo_videotracking
