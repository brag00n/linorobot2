// face_detection.hpp - portage de tools/robot_controlv3/.../FaceDetection.py (598 l.).
//
// Deux responsabilites, volontairement dans le meme objet parce qu'elles partagent un
// etat que separer rendrait incoherent :
//   1. DETECTER des visages (trois detecteurs interchangeables a chaud) ;
//   2. tenir un VERROU sur une cible a travers les trames, en arbitrant entre le
//      detecteur et un tracker de region (cv::TrackerMIL).
//
// Pourquoi un tracker en plus du detecteur : le detecteur est sans memoire -- il rend des
// boites, pas des identites. Suivre "le meme visage" demande de l'etat, et le tracker
// fournit une boite meme sur les trames ou la detection decroche (profil, contre-jour).
// C'est ce qui rend la commande servo lisse au lieu de sauter d'un visage a l'autre.
//
// ---------------------------------------------------------------------------
// ARBITRAGE, ET C'EST LE COEUR DE L'ALGORITHME : LE DETECTEUR GAGNE TOUJOURS.
// Le tracker est un INTERPOLATEUR entre deux detections, jamais une source de verite.
// Des qu'une detection plausible existe on se recale dessus ; en son absence prolongee on
// LACHE le verrou au lieu de suivre une boite qui derive. Un tracker qui a verrouille un
// mur suit le mur avec une confiance parfaite -- la derive silencieuse est le mode de
// panne dominant de ce genre de chaine, d'ou la liberation prioritaire sur det_miss.
//
// CADENCE : sous verrou le tracker passe a CHAQUE trame et le detecteur seulement toutes les
// `redetect_ms` (400 ms par defaut, le defaut du prototype). C'est ce qui rend la chaine
// abordable : les deux etages les plus chers ne sont plus payes ensemble a chaque trame.
// Consequence a garder en tete en lisant la suite : tout ce qui compte des "detections
// manquees" compte des CYCLES, pas des trames.
//
// ---------------------------------------------------------------------------
// cv::TrackerVit EST ABSENT de l'OpenCV 4.5.4 de Humble/Jammy -- verifie DANS le
// conteneur, pas suppose (absent des en-tetes C++ ET du binding Python ; il arrive en
// 4.7). Le mode "vit" reste donc DECLARE et repond "indisponible" : un repli silencieux
// sur MIL laisserait croire qu'on mesure ViT au banc V6.3. Le prototype fait de meme --
// son trackReady("vit") teste hasattr(cv2, "TrackerVit_create").
//
// PIEGE D'API attrape ici : cv::TrackerMIL existe en DEUX versions dans cette
// installation -- la moderne de libopencv_video (opencv2/video/tracking.hpp:750) et la
// cv::legacy::TrackerMIL du contrib. C'est la MODERNE qu'on utilise, et elle seule est
// garantie presente sans contrib.

#ifndef BAMBOO_VIDEOTRACKING__FACE_DETECTION_HPP_
#define BAMBOO_VIDEOTRACKING__FACE_DETECTION_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/objdetect/face.hpp>
#include <opencv2/video/tracking.hpp>

namespace bamboo_videotracking
{

/// Les 5 points de repere de YuNet (oeil D, oeil G, nez, bouche D, bouche G), dans
/// l'ORDRE et l'ECHELLE qu'attend cv::FaceRecognizerSF::alignCrop. Ne pas les reordonner :
/// SFace aligne sur cet ordre exact et un echange silencieux donnerait des empreintes
/// stables mais FAUSSES -- la pire des pannes, parce qu'elle ne ressemble pas a une panne.
using Landmarks = std::vector<cv::Point2f>;

/// Reglages exposes en parametres ROS et modifiables a chaud (lot V3.6). Les valeurs sont
/// celles du prototype -- de ses defauts ARGPARSE (`RobotMain.py`), pas de ceux de ses
/// constructeurs de classe, que RobotMain ecrase tous. Les noms aussi sont les siens, pour
/// qu'un essai se transpose du banc au ROS sans retraduction. Les changer sans mesure au
/// banc V6.3 n'est pas un reglage, c'est une derive.
struct DetectConfig
{
  std::string detector{"yunet"};   ///< haar | dnn | yunet
  std::string track_mode{"mil"};   ///< none | mil | vit (vit -> indisponible)
  int det_width{320};              ///< largeur de travail ; la detection ne voit QUE ca
  double det_conf{0.6};            ///< seuil de confiance du detecteur

  // --- machine a etats de verrou ---
  // PORTE DE RE-DETECTION (nom et defaut du prototype, `--redetect-ms`). Sous verrou, le
  // detecteur ne repasse qu'a cette periode ; entre deux passages c'est le tracker seul qui
  // fournit la boite. A 0 le detecteur repasse a chaque trame, ce qui est l'ancien
  // comportement de ce noeud -- et son cout : ~92 ms de plus par trame (banc T15b).
  double redetect_ms{400.0};       ///< periode de re-detection pendant le verrou
  double iou_reanchor{0.2};        ///< en dessous : le detecteur a trouve AILLEURS -> recentrage
  double score_min{0.3};           ///< score tracker sous lequel on lache
  double hold_ms{5000.0};          ///< duree de survie sans detection
  double hold_score_min{0.6};      ///< au-dela de ce score, hold_ms est assoupli
  int max_det_misses{8};           ///< CYCLES de re-detection muets avant liberation
                                   ///< ANTI-DERIVE (donc une duree, ~ x redetect_ms)
  // --- vraisemblance de la boite ---
  // max_grow est le nom du prototype (`--track-max-grow`) et compare des AIRES, comme lui.
  double max_grow{4.5};            ///< boite > 4,5x l'aire de reference -> invraisemblable
  double area_ratio_min{0.25};     ///< boite < 1/4 -> invraisemblable (ajout ROS, sans
                                   ///< equivalent au prototype)
  double max_area_frac{0.5};       ///< garde ABSOLUE : boite couvrant plus de la moitie de
                                   ///< l'image -> invraisemblable quoi qu'en dise le rapport
  double inter_frac_min{0.2};      ///< fraction de la boite devant rester dans l'image
};

/// Etat rendu a chaque cycle. Les noms de `src` et de `lost_reason` sont ceux du prototype
/// et forment le vocabulaire commun du HUD, de TrackState et des journaux : les renommer
/// couperait la comparaison avec les essais deja faits.
struct TrackStateOut
{
  bool locked{false};
  /// off | detect | track | reanchor | recenter | redetect
  std::string src{"off"};
  /// "" | update_ko | degenerate | implausible | score | det_miss | hold_timeout | switch
  std::string lost_reason;
  double score{0.0};
  bool has_main{false};
  cv::Rect main;                ///< boite retenue, en pixels de la trame PLEINE
  /// VIDE si la boite vient du tracker : une boite interpolee n'a PAS de points de repere,
  /// et en fabriquer par mise a l'echelle de la boite donnerait a SFace un alignement
  /// credible et faux. facerecog_node doit donc redetecter, ou s'abstenir.
  Landmarks main_landmarks;
  int n_faces{0};
  /// Poignee d'EPISODE : incrementee a chaque nouveau verrou. C'est elle, et non un
  /// booleen, qui permet aux etages avals de savoir qu'ils changent de personne --
  /// facerecog_node reamorce son hysteresis sur ce changement.
  uint32_t lock_id{0};
  double lock_age_s{0.0};
};

class FaceDetection
{
public:
  FaceDetection() = default;

  /// Charge les modeles. `models_dir` porte les .onnx / .caffemodel du depot firmware
  /// (montes en bind-mount). Renvoie false ET remplit `err` si le detecteur demande est
  /// inutilisable : on ne demarre pas en mode degrade silencieux.
  bool build(const std::string & models_dir, const DetectConfig & cfg, std::string & err);

  /// Change de detecteur ou de tracker a chaud. Reconstruit ce qui doit l'etre et LACHE le
  /// verrou (motif `switch`) : garder un verrou etabli par un autre detecteur melangerait
  /// deux referentiels de score, donc rendrait `score_min` ininterpretable.
  bool reconfigure(const DetectConfig & cfg, std::string & err);

  const DetectConfig & config() const { return cfg_; }

  /// Un cycle complet sur une trame PLEINE resolution.
  /// `now_s` est l'horloge du noeud (monotone). La mise a l'echelle vers det_width est faite
  /// ICI et les boites rendues sont TOUJOURS en pixels de la trame pleine : aucun appelant
  /// n'a a connaitre det_width, ce qui evite le facteur d'echelle oublie a un etage.
  TrackStateOut trackStep(const cv::Mat & frame, double now_s);

  /// Detection seule, sans toucher au verrou. Utilisee par facerecog_node pour obtenir des
  /// points de repere FRAIS sur la boite verrouillee (alignCrop de SFace en a besoin) et par
  /// les services de reconnaissance sur fichier.
  std::vector<cv::Rect> detect(
    const cv::Mat & frame, std::vector<Landmarks> & landmarks, std::vector<float> & scores);

  /// Points de repere associes a une boite deja connue, ou vide.
  /// APPARIEMENT PAR EGALITE ENTIERE de x et y, comme le prototype, et c'est volontaire :
  /// les boites viennent du meme calcul donc elles sont identiques au bit pres, tandis
  /// qu'une tolerance flottante pourrait apparier le mauvais visage dans un groupe.
  Landmarks landmarksFor(const cv::Rect & box) const;

  /// Cadence de detection mesuree (fps), comptee sur une fenetre glissante d'une seconde.
  double detFps() const { return det_fps_; }

private:
  // --- geometrie, transposition directe des helpers du prototype ---
  static double iou(const cv::Rect & a, const cv::Rect & b);
  /// Boite vide ou absurde. Premier test, parce que les suivants divisent par son aire.
  static bool degenerate(const cv::Rect & r);
  /// Fraction de `r` qui reste dans une image w x h. Une boite qui sort de l'image est le
  /// signe classique d'un tracker qui derive vers un bord.
  static double interFrac(const cv::Rect & r, int w, int h);
  bool implausible(const cv::Rect & r, int w, int h) const;

  bool initTracker(const cv::Mat & frame, const cv::Rect & box);
  void releaseLock(const std::string & reason);
  void tickFps(double now_s);

  /// Un CYCLE de detection : remplit det_boxes_ / det_landmarks_ / det_scores_ et met a jour
  /// tout ce qui se compte PAR CYCLE (cadence, horodatage de la porte, nombre de visages).
  /// Depuis la porte redetect_ms, les cycles sont plus rares que les trames : ce qui se
  /// compte ici ne doit PAS se compter dans trackStep, sous peine de mesurer la cadence
  /// video au lieu de celle de la detection.
  void runDetect(const cv::Mat & frame, double now_s);
  /// Confiance de la detection i, ou 1.0 pour un detecteur qui n'en rend pas (Haar).
  double scoreOf(size_t i) const;
  /// Index de la plus GRANDE detection admissible, ou -1. Critere de PRISE de verrou.
  int pickLargest() const;
  /// Index de la detection qui recouvre le plus `ref`, ou -1, et l'IoU obtenu. Critere de
  /// MAINTIEN de verrou : suivre la meme cible, pas la plus grosse.
  int pickNearest(const cv::Rect & ref, double & best_iou) const;
  /// Recopie l'etat interne dans `st`. Un seul endroit ou l'etat sort, parce que la machine
  /// a etats a six sorties et qu'un champ oublie dans l'une d'elles serait invisible.
  TrackStateOut & publish(TrackStateOut & st, double now_s, int fw, int fh) const;

  DetectConfig cfg_;
  std::string models_dir_;

  // Les trois detecteurs. Un seul est actif, mais on garde les objets construits : la
  // reconfiguration a chaud doit etre instantanee (la manette la declenche) et recharger un
  // .onnx sur RPi4 se compte en centaines de ms.
  cv::Ptr<cv::FaceDetectorYN> yunet_;
  cv::dnn::Net res10_;
  cv::CascadeClassifier haar_;

  cv::Ptr<cv::Tracker> tracker_;

  /// Lignes BRUTES de la derniere detection YuNet : 15 colonnes
  /// [x, y, w, h, 5x(px,py), score], a l'echelle de la trame PLEINE. On les conserve telles
  /// quelles parce que alignCrop veut cette forme ; les recomposer plus tard depuis des
  /// boites arrondies perdrait la precision sous-pixel des points de repere.
  cv::Mat det_rows_;
  std::vector<cv::Rect> det_boxes_;
  std::vector<Landmarks> det_landmarks_;
  std::vector<float> det_scores_;
  /// Nombre de visages du DERNIER cycle de detection. Conserve parce que la porte
  /// redetect_ms fait des trames sans detection : publier 0 sur celles-la se lirait comme
  /// "personne devant la camera" alors que le verrou tient.
  int last_n_faces_{0};

  bool locked_{false};
  cv::Rect cur_;
  double cur_score_{0.0};
  Landmarks cur_landmarks_;
  double ref_area_{0.0};       ///< aire a l'instant du verrou, reference du test de vraisemblance
  /// Cycles de re-detection CONSECUTIFS sans visage (et non trames : c'est ce qui rend
  /// max_det_misses independant de la cadence video).
  int miss_streak_{0};
  double last_redetect_s_{0.0};   ///< t du dernier passage du detecteur (porte redetect_ms)
  double last_confirm_s_{0.0};    ///< t du dernier recalage sur une DETECTION (borne hold_ms)
  uint32_t lock_id_{0};
  double lock_t0_{0.0};
  std::string lost_reason_;

  double fps_win_t0_{0.0};
  int fps_count_{0};
  double det_fps_{0.0};
};

}  // namespace bamboo_videotracking

#endif  // BAMBOO_VIDEOTRACKING__FACE_DETECTION_HPP_
