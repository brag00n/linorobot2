// bamboo_videotracking / facerecog_node -- reconnaissance de visage (lot V3.2).
//
// PORTAGE de tools/robot_controlv3/nodes/FaceRecogNode.py + interaction/FaceRecognizer.py.
//
// CE QUE FAIT CE NOEUD, ET SURTOUT CE QU'IL NE FAIT PAS :
//   Il ne decode AUCUNE image et n'ouvre AUCUNE camera. Il lit la MEME cv::Mat que
//   tracking_node vient de publier sur le FrameBus, en `const` et par pointeur : c'est
//   tout l'interet d'etre un composable dans le meme processus. Un second imdecode
//   couterait ~8 ms de CPU RPi4 par trame pour produire une copie bit a bit identique.
//
//   Ce n'est PAS de la classification, c'est de la COMPARAISON D'EMPREINTES. SFace rend
//   un vecteur de 128 dimensions, compare en cosinus a une galerie de visages enroles.
//   Un score de 0,4 veut dire "proche de cette empreinte", pas "certain a 40 %".
//
// POURQUOI UNE HYSTERESIS A DEUX SEUILS ET UNE EMA, ET PAS UN SIMPLE SEUIL :
//   un cosinus instantane oscille autour du seuil des qu'un visage bouge ou que la
//   lumiere change. Avec un seuil unique le badge clignote entre deux identites, ce qui
//   est pire qu'une erreur franche parce que l'operateur ne sait plus ce qu'il lit. On
//   lisse donc (EMA 0,4), on ENGAGE une identite a `cos_on` et on ne la LACHE qu'en
//   dessous de `cos_off` -- exactement la semantique du prototype.
//
// L'EPISODE est la notion centrale : tant que le verrou de suivi porte le meme
// `lock_id`, c'est la meme personne devant la camera, donc la meme EMA et le meme lot
// d'acquisition. Un changement de `lock_id` (et c'est le SEUL signal disponible : deux
// episodes separes par une trame perdue sont sinon indiscernables) remet tout a zero.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/face.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "bamboo_interfaces/msg/mode_cmd.hpp"
#include "bamboo_interfaces/msg/recognition_result.hpp"
#include "bamboo_interfaces/msg/track_state.hpp"
#include "bamboo_videotracking/frame_bus.hpp"
#include "bamboo_videotracking/gallery.hpp"

namespace bamboo_videotracking
{
namespace
{
constexpr int kAlignSize = 112;        // impose par SFace, pas un reglage
constexpr int kEmbedDim = 128;         // idem : face_recognition_sface_2021dec.onnx
const char * kSFaceFile = "face_recognition_sface_2021dec.onnx";
const char * kYuNetFile = "face_detection_yunet_2023mar.onnx";

std::string join(const std::string & dir, const std::string & file)
{
  if (dir.empty()) {return file;}
  return (dir.back() == '/' || dir.back() == '\\') ? dir + file : dir + "/" + file;
}

/// mkdir -p minimal : le lot d'acquisition cree <faces_dir>/unknown/<id_lot>/.
bool mkdirs(const std::string & path)
{
  for (size_t i = 1; i <= path.size(); ++i) {
    if (i == path.size() || path[i] == '/') {
      const std::string part = path.substr(0, i);
      if (part.empty()) {continue;}
      struct stat st;
      if (::stat(part.c_str(), &st) == 0) {continue;}
      if (::mkdir(part.c_str(), 0775) != 0) {return false;}
    }
  }
  return true;
}
}  // namespace

class FaceRecogNode : public rclcpp::Node
{
public:
  explicit FaceRecogNode(const rclcpp::NodeOptions & opts)
  : rclcpp::Node("facerecog_node", opts)
  {
    models_dir_ = declare_parameter<std::string>("models_dir", "");
    faces_dir_ = declare_parameter<std::string>("faces_dir", "");
    // Les deux seuils du prototype. cos_on 0,363 est la valeur livree avec le modele
    // SFace ; cos_off 0,30 est PLUS BAS a dessein -- c'est ce creux qui empeche le badge
    // de clignoter (cf. l'en-tete).
    cos_on_ = declare_parameter<double>("cos_on", 0.363);
    cos_off_ = declare_parameter<double>("cos_off", 0.30);
    ema_a_ = declare_parameter<double>("ema_alpha", 0.4);
    stable_s_ = declare_parameter<double>("stable_s", 2.0);
    // Acquisition : on ne sature ni la carte SD ni le CPU. Le prototype enregistre au
    // fil de l'eau ; on borne explicitement, parce qu'ici personne ne regarde un HUD.
    acq_period_s_ = declare_parameter<double>("acquire_period_s", 0.4);
    acq_max_ = static_cast<int>(declare_parameter<int>("acquire_max_per_episode", 40));

    std::string err;
    if (!build(err)) {
      // ECHEC FATAL ET NOMME : un noeud de reconnaissance qui demarre sans modele
      // publierait "unknown" indefiniment, ce qui ressemble a "personne n'est reconnu"
      // et non a "le modele manque".
      throw std::runtime_error(err);
    }

    // La galerie peut etre vide (robot neuf) : ce n'est PAS une erreur, on le dit et on
    // demarre quand meme, en repondant "unknown".
    std::string gerr;
    if (!gal_.reload(faces_dir_, gerr)) {
      RCLCPP_ERROR(get_logger(), "galerie illisible : %s", gerr.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "galerie : %zu personne(s), %zu empreinte(s)%s%s",
        gal_.persons(), gal_.embeddings(), gerr.empty() ? "" : " -- ecartes :",
        gerr.c_str());
    }

    pub_ = create_publisher<bamboo_interfaces::msg::RecognitionResult>(
      "/videotracking/recognition", rclcpp::QoS(1).reliable());

    // TrackState est la CADENCE de ce noeud : il n'a pas d'horloge propre. Une trame sans
    // verrou ne demande aucun calcul, donc aucun timer n'a de sens ici.
    track_sub_ = create_subscription<bamboo_interfaces::msg::TrackState>(
      "/videotracking/track_state", rclcpp::QoS(1).reliable(),
      [this](bamboo_interfaces::msg::TrackState::ConstSharedPtr m) {onTrack(m);});

    // `transient_local` (latche) est INCOMPATIBLE avec l'intra-process : rclcpp refuse en
    // construction "intraprocess communication allowed only with volatile durability". On
    // desactive donc l'intra-process sur CETTE SEULE entite -- elle est hors chemin chaud (une
    // commande de mode par appui de touche), alors que les IMAGES, elles, doivent rester en
    // zero-copy : c'est tout l'interet du container.
    rclcpp::SubscriptionOptions mode_opts;
    mode_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    mode_sub_ = create_subscription<bamboo_interfaces::msg::ModeCmd>(
      "/videotracking/mode_cmd", rclcpp::QoS(1).reliable().transient_local(),
      [this](bamboo_interfaces::msg::ModeCmd::ConstSharedPtr m) {onMode(m);}, mode_opts);

    // Identifiant de session : prefixe des lots d'acquisition, pour qu'une seance de
    // jeudi ne se melange pas a celle de mercredi dans unknown/.
    char buf[32];
    std::snprintf(
      buf, sizeof(buf), "%ld",
      static_cast<long>(now().nanoseconds() / 1000000000LL));
    session_ = buf;
    RCLCPP_INFO(get_logger(), "facerecog_node pret (session %s)", session_.c_str());
  }

private:
  bool build(std::string & err)
  {
    const std::string sface = join(models_dir_, kSFaceFile);
    const std::string yunet = join(models_dir_, kYuNetFile);
    try {
      rec_ = cv::FaceRecognizerSF::create(sface, "");
    } catch (const cv::Exception & e) {
      err = "SFace introuvable ou illisible (" + sface + ") : " + e.what();
      return false;
    }
    try {
      // Detecteur PROPRE a ce noeud, et c'est voulu : il travaille en PLEINE RESOLUTION
      // pour obtenir des points de repere precis, alors que tracking_node detecte a
      // det_width 320 pour tenir la cadence. Deux usages, deux reglages.
      det_ = cv::FaceDetectorYN::create(yunet, "", cv::Size(320, 320), 0.6f, 0.3f, 5000);
    } catch (const cv::Exception & e) {
      err = "YuNet introuvable ou illisible (" + yunet + ") : " + e.what();
      return false;
    }
    if (rec_.empty() || det_.empty()) {err = "creation des modeles echouee"; return false;}
    return true;
  }

  void onMode(const bamboo_interfaces::msg::ModeCmd::ConstSharedPtr & msg)
  {
    // Garde anti-rejeu : le topic est `transient_local`, donc un noeud qui redemarre
    // recoit la derniere commande et la rejouerait sans ce test.
    if (msg->seq != 0 && msg->seq == last_mode_seq_) {return;}
    last_mode_seq_ = msg->seq;

    if (msg->target == "recognition" || msg->target == "acquisition") {
      // Les deux modes du prototype sont EXCLUSIFS : acquisition implique recognition
      // (on predit ET on enregistre), off coupe les deux.
      if (msg->value == "off") {mode_ = "off";} else if (msg->target == "acquisition") {
        mode_ = "acquisition";
      } else {mode_ = "recognition";}
      RCLCPP_INFO(get_logger(), "mode reconnaissance -> %s", mode_.c_str());
      return;
    }
    if (msg->target == "training" && msg->value == "done") {
      // L'apprentissage vient de reecrire identified/ : sans ce rechargement, le noeud
      // continuerait a comparer a l'ANCIENNE galerie et la personne tout juste enrolee
      // resterait "unknown" -- symptome classiquement pris pour un echec d'enrolement.
      std::string gerr;
      gal_.reload(faces_dir_, gerr);
      RCLCPP_INFO(
        get_logger(), "galerie rechargee : %zu personne(s), %zu empreinte(s)",
        gal_.persons(), gal_.embeddings());
      return;
    }
  }

  void onTrack(const bamboo_interfaces::msg::TrackState::ConstSharedPtr & ts)
  {
    const bool has_main = ts->bbox_w > 0 && ts->bbox_h > 0;

    if (!rec_ || mode_ == "off" || gal_.persons() == 0) {
      // PUBLICATION UNIQUE d'un etat `idle`, puis silence. Sans cela le dernier resultat
      // reste latche cote abonne et l'affichage semble "ne pas se desactiver".
      if (!idle_sent_) {
        flushEpisode();
        bamboo_interfaces::msg::RecognitionResult out;
        out.header.stamp = ts->header.stamp;
        out.status = "idle";
        out.id_pred = -1;
        out.name = "unknown";
        out.mode = mode_;
        pub_->publish(out);
        idle_sent_ = true;
      }
      return;
    }
    idle_sent_ = false;

    // PORTE D'ENTREE, identique au prototype : on ne reconnait que sur un verrou STABLE.
    // Sans le delai `stable_s`, la reconnaissance travaille sur les premieres trames d'un
    // verrou, ou la boite est encore mal calee, et engage une identite fausse -- qu'il
    // faut ensuite defaire, ce que l'hysteresis rend justement lent.
    if (!ts->locked || !has_main || ts->lock_age_s < stable_s_) {return;}

    const Frame f = frameBus().latest();
    if (!f.valid()) {return;}
    // THROTTLE PAR SEQUENCE DE TRAME : TrackState peut arriver plus vite que la trame ne
    // change (republication), et recalculer une empreinte sur la meme image couterait
    // ~20 ms pour un resultat identique.
    if (f.seq == last_frame_seq_) {return;}
    last_frame_seq_ = f.seq;

    const cv::Mat & img = *f.mat;
    cv::Rect box(ts->bbox_x, ts->bbox_y, ts->bbox_w, ts->bbox_h);

    // On redetecte en PLEINE RESOLUTION pour obtenir les 5 points de repere : la boite de
    // TrackState peut venir du tracker ou d'une interpolation, qui n'en fournissent pas,
    // et alignCrop sans points de repere ne REDRESSE PAS un visage penche -- l'empreinte
    // est alors comparee a une galerie redressee, et le cosinus chute sans raison visible.
    cv::Mat row;
    const bool aligned = landmarkRow(img, box, row);

    cv::Mat crop = alignCrop(img, box, row, aligned);
    if (crop.empty()) {return;}

    cv::Mat emb;
    try {
      rec_->feature(crop, emb);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(get_logger(), "SFace feature a echoue : %s", e.what());
      return;
    }
    if (emb.empty() || emb.total() != static_cast<size_t>(kEmbedDim)) {
      RCLCPP_WARN(get_logger(), "empreinte de taille %zu inattendue", emb.total());
      return;
    }
    cv::Mat emb32;
    emb.convertTo(emb32, CV_32F);
    emb32 = emb32.reshape(1, 1);

    int id_cand = -1;
    std::string name_cand = "unknown";
    double cos = -1.0;
    gal_.nearest(emb32.ptr<float>(0), kEmbedDim, id_cand, name_cand, cos);

    if (ts->lock_id != ep_lock_ || !ep_open_) {
      // NOUVEL EPISODE. L'EMA est AMORCEE a la mesure courante et non a zero : partir de
      // zero ferait grimper le lisse pendant plusieurs trames avant de pouvoir engager
      // une identite, ce qui retarderait la reconnaissance sans rien stabiliser.
      flushEpisode();
      ep_open_ = true;
      ep_lock_ = ts->lock_id;
      ep_ema_ = cos;
      ep_id_ = -1;
      ep_name_ = "unknown";
      ep_n_ = 0;
      ep_known_ = 0;
      ep_flips_ = 0;
      ep_saved_ = 0;
      ep_last_save_s_ = -1.0e9;
      char buf[96];
      std::snprintf(
        buf, sizeof(buf), "%s-%u", session_.c_str(), static_cast<unsigned>(ts->lock_id));
      ep_lot_ = buf;
    } else {
      ep_ema_ = ema_a_ * cos + (1.0 - ema_a_) * ep_ema_;
    }
    ++ep_n_;

    // HYSTERESIS A DEUX SEUILS, quatre branches, dans l'ordre du prototype.
    if (ep_id_ < 0) {
      if (ep_ema_ >= cos_on_ && id_cand >= 0) {      // 1. engagement
        ep_id_ = id_cand;
        ep_name_ = name_cand;
      }
    } else if (id_cand == ep_id_) {
      if (ep_ema_ < cos_off_) {                       // 2. relachement
        ep_id_ = -1;
        ep_name_ = "unknown";
        ++ep_flips_;
      }
    } else if (ep_ema_ >= cos_on_ && id_cand >= 0) {  // 3. bascule d'identite
      ep_id_ = id_cand;
      ep_name_ = name_cand;
      ++ep_flips_;
    } else if (ep_ema_ < cos_off_) {                  // 4. retour a l'inconnu
      ep_id_ = -1;
      ep_name_ = "unknown";
      ++ep_flips_;
    }
    if (ep_id_ >= 0) {++ep_known_;}

    if (mode_ == "acquisition") {acquire(crop, ts->header.stamp);}

    bamboo_interfaces::msg::RecognitionResult out;
    // L'horodatage est celui de la TRAME, repris tel quel : c'est le contrat de latence
    // bout en bout du lot V6, et le reecrire ici masquerait le retard accumule.
    out.header.stamp = ts->header.stamp;
    out.status = (ep_id_ >= 0) ? "known" : "unknown";
    out.id_pred = ep_id_;
    out.name = ep_name_;
    out.score = static_cast<float>(ep_ema_);
    out.raw_score = static_cast<float>(cos);
    out.stability = (ep_n_ > 0) ?
      static_cast<float>(static_cast<double>(ep_known_) / static_cast<double>(ep_n_)) : 0.0f;
    out.id_lot = ep_lot_;
    out.lock_id = ep_lock_;
    out.mode = mode_;
    pub_->publish(out);
  }

  /// Detecte en pleine resolution et rend la ligne 1x15 attendue par alignCrop.
  bool landmarkRow(const cv::Mat & img, const cv::Rect & box, cv::Mat & row)
  {
    cv::Mat faces;
    try {
      det_->setInputSize(cv::Size(img.cols, img.rows));
      det_->detect(img, faces);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(get_logger(), "YuNet plein cadre a echoue : %s", e.what());
      return false;
    }
    if (faces.empty() || faces.cols < 15) {return false;}
    // On retient la detection qui RECOUVRE LE MIEUX la boite suivie, pas la plus grande :
    // avec deux visages dans le champ, la plus grande serait souvent la mauvaise.
    int best = -1;
    double best_inter = 0.0;
    for (int i = 0; i < faces.rows; ++i) {
      const float * r = faces.ptr<float>(i);
      const cv::Rect c(
        static_cast<int>(r[0]), static_cast<int>(r[1]),
        static_cast<int>(r[2]), static_cast<int>(r[3]));
      const double inter = (box & c).area();
      if (inter > best_inter) {best_inter = inter; best = i;}
    }
    if (best < 0 || best_inter <= 0.0) {return false;}
    row = faces.row(best).clone();
    return true;
  }

  /// alignCrop quand on a les points de repere, sinon recadrage simple -- jamais rien.
  cv::Mat alignCrop(
    const cv::Mat & img, const cv::Rect & box, const cv::Mat & row, bool have_row)
  {
    if (have_row) {
      try {
        cv::Mat out;
        rec_->alignCrop(img, row, out);
        if (!out.empty()) {return out;}
      } catch (const cv::Exception & e) {
        // REPLI SILENCIEUX ASSUME (le prototype fait de meme) : un alignCrop qui echoue
        // sur une trame ne doit pas interrompre la reconnaissance, le recadrage brut
        // donne un cosinus plus faible mais exploitable.
        RCLCPP_DEBUG(get_logger(), "alignCrop : %s", e.what());
      }
    }
    // Recadrage borne a l'image : une boite predite peut sortir du cadre.
    cv::Rect b = box & cv::Rect(0, 0, img.cols, img.rows);
    if (b.width < 8 || b.height < 8) {return cv::Mat();}
    cv::Mat out;
    cv::resize(img(b), out, cv::Size(kAlignSize, kAlignSize), 0, 0, cv::INTER_AREA);
    return out;
  }

  /// Mode acquisition : enregistre les vignettes alignees dans unknown/<id_lot>/.
  void acquire(const cv::Mat & crop, const builtin_interfaces::msg::Time & stamp)
  {
    if (ep_saved_ >= acq_max_) {return;}
    const double t = static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
    if (t - ep_last_save_s_ < acq_period_s_) {return;}
    ep_last_save_s_ = t;

    const std::string dir = join(join(faces_dir_, "unknown"), ep_lot_);
    if (!mkdirs(dir)) {
      RCLCPP_ERROR(get_logger(), "creation de %s impossible", dir.c_str());
      ep_saved_ = acq_max_;   // on n'inonde pas le journal a chaque trame
      return;
    }
    char nm[64];
    std::snprintf(nm, sizeof(nm), "%04d.jpg", ep_saved_);
    const std::string path = join(dir, nm);
    if (cv::imwrite(path, crop)) {
      ++ep_saved_;
    } else {
      RCLCPP_WARN(get_logger(), "ecriture de %s echouee", path.c_str());
      ep_saved_ = acq_max_;
    }
  }

  void flushEpisode()
  {
    if (!ep_open_) {return;}
    // Trace de fin d'episode : c'est la seule facon de savoir apres coup si une identite a
    // tenu ou si elle a bascule dix fois (ep_flips_), donc de distinguer "mal enrole" de
    // "mal eclaire".
    RCLCPP_INFO(
      get_logger(), "episode %u clos : %s, n=%d known=%d flips=%d saved=%d ema=%.3f",
      static_cast<unsigned>(ep_lock_), ep_name_.c_str(), ep_n_, ep_known_, ep_flips_,
      ep_saved_, ep_ema_);
    ep_open_ = false;
  }

  std::string models_dir_;
  std::string faces_dir_;
  double cos_on_{0.363};
  double cos_off_{0.30};
  double ema_a_{0.4};
  double stable_s_{2.0};
  double acq_period_s_{0.4};
  int acq_max_{40};

  cv::Ptr<cv::FaceRecognizerSF> rec_;
  cv::Ptr<cv::FaceDetectorYN> det_;
  Gallery gal_;

  std::string mode_{"off"};
  std::string session_;
  bool idle_sent_{false};
  uint32_t last_mode_seq_{0};
  uint64_t last_frame_seq_{0};

  // Etat d'EPISODE (remis a zero sur changement de lock_id).
  bool ep_open_{false};
  uint32_t ep_lock_{0};
  std::string ep_lot_;
  double ep_ema_{-1.0};
  int ep_id_{-1};
  std::string ep_name_{"unknown"};
  int ep_n_{0};
  int ep_known_{0};
  int ep_flips_{0};
  int ep_saved_{0};
  double ep_last_save_s_{-1.0e9};

  rclcpp::Subscription<bamboo_interfaces::msg::TrackState>::SharedPtr track_sub_;
  rclcpp::Subscription<bamboo_interfaces::msg::ModeCmd>::SharedPtr mode_sub_;
  rclcpp::Publisher<bamboo_interfaces::msg::RecognitionResult>::SharedPtr pub_;
};

}  // namespace bamboo_videotracking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bamboo_videotracking::FaceRecogNode)
