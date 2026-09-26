// bamboo_videotracking / overlay_node -- incrustation et mesure (lots V3.3, V6.1, V6.2).
//
// ⚠️ LE PIEGE DE PORTAGE DE TOUT CE CHANTIER EST ICI.
//   Le prototype dessine son HUD DANS la ndarray qu'il vient de publier : la trame du bus
//   n'y est pas immuable, et l'etage d'incrustation ecrit dessus. En intra-process C++ la
//   cv::Mat est partagee PAR POINTEUR et `const` -- dessiner dessus corromprait la trame
//   que tracking_node et facerecog_node lisent encore. Ce noeud COPIE donc la Mat avant
//   de tracer, et le `const` du FrameBus fait de l'erreur inverse une erreur de
//   compilation plutot qu'un bug d'affichage intermittent.
//
// C'est aussi LE SEUL ETAGE QUI RE-ENCODE (imencode JPEG), donc le seul qui coute du CPU
// sans rien calculer. Il est optionnel par construction : quand ce groupe est arrete, le
// stream_mux de bamboo_video sert le flux brut et la meme URL continue de fonctionner.
//
// POURQUOI LES STATISTIQUES SONT PUBLIEES ICI ET PAS DANS UN NOEUD DEDIE :
//   la latence bout en bout est `now - header.stamp` de la trame ANNOTEE -- elle n'est
//   mesurable qu'au dernier etage. Et comme ce noeud voit passer les trames brutes
//   (FrameBus), les TrackState et les RecognitionResult, il peut compter les quatre
//   cadences sans qu'aucun autre noeud ait a publier de compteur. Un stats_node separe
//   aurait exige trois topics de service supplementaires pour le meme resultat.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "bamboo_interfaces/msg/mode_cmd.hpp"
#include "bamboo_interfaces/msg/recognition_result.hpp"
#include "bamboo_interfaces/msg/track_state.hpp"
#include "bamboo_interfaces/msg/tracking_stats.hpp"
#include "bamboo_interfaces/srv/register_overlay.hpp"
#include "bamboo_videotracking/frame_bus.hpp"

namespace bamboo_videotracking
{
namespace
{
// Palette lisible sur une image quelconque : le vert pur est le seul ton qui ressorte
// aussi bien sur un mur clair que sur un fond sombre, et le rouge est reserve a l'etat
// "perdu" pour qu'un coup d'oeil suffise.
const cv::Scalar kLocked(0, 255, 0);
const cv::Scalar kLost(0, 0, 255);
const cv::Scalar kPredicted(0, 200, 255);
const cv::Scalar kInk(255, 255, 255);
const cv::Scalar kShade(0, 0, 0);

/// Texte avec ombre portee : sans elle, le blanc disparait sur un fond clair.
void putShadowed(cv::Mat & im, const std::string & s, cv::Point at, double sc, int th = 1)
{
  cv::putText(im, s, at + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, sc, kShade, th + 1,
    cv::LINE_AA);
  cv::putText(im, s, at, cv::FONT_HERSHEY_SIMPLEX, sc, kInk, th, cv::LINE_AA);
}

/// Pastille d'etat d'un mode : remplie quand actif, cerclee quand inactif.
int modePill(cv::Mat & im, int x, int y, const std::string & label, bool on)
{
  const int pad = 6;
  int base = 0;
  const cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &base);
  const cv::Rect r(x, y, ts.width + 2 * pad, ts.height + 2 * pad);
  const cv::Scalar col = on ? kLocked : cv::Scalar(90, 90, 90);
  cv::rectangle(im, r, col, on ? cv::FILLED : 1, cv::LINE_AA);
  cv::putText(im, label, cv::Point(x + pad, y + pad + ts.height - 1),
    cv::FONT_HERSHEY_SIMPLEX, 0.4, on ? kShade : kInk, 1, cv::LINE_AA);
  // On renvoie le X SUIVANT, pas la largeur : l'appelant chaine les pastilles.
  return x + r.width + 5;
}

/// Cadence sur une fenetre glissante : un simple compteur remis a zero a chaque
/// publication de stats suffit, et coute un entier au lieu d'un historique.
struct RateCounter
{
  unsigned n{0};
  void tick() {++n;}
  float take(double win_s)
  {
    const float v = (win_s > 0.0) ? static_cast<float>(n / win_s) : 0.0f;
    n = 0;
    return v;
  }
};
}  // namespace

class OverlayNode : public rclcpp::Node
{
public:
  explicit OverlayNode(const rclcpp::NodeOptions & opts)
  : rclcpp::Node("overlay_node", opts)
  {
    // Le topic enrichi est un PARAMETRE parce que c'est exactement la chaine qu'on declare
    // au stream_mux : le mux souscrit ce que le client annonce, il ne devine rien.
    enriched_topic_ = declare_parameter<std::string>(
      "enriched_topic", "/videotracking/enriched/compressed");
    jpeg_quality_ = static_cast<int>(declare_parameter<int>("jpeg_quality", 80));
    // 0 = on annote chaque trame. Au-dela, on saute des trames : c'est le premier levier
    // a tirer si le RPi4 sature, avant de degrader la detection.
    min_period_s_ = declare_parameter<double>("min_period_s", 0.0);
    stats_period_s_ = declare_parameter<double>("stats_period_s", 1.0);
    draw_stats_ = declare_parameter<bool>("draw_stats", true);
    register_with_mux_ = declare_parameter<bool>("register_with_mux", true);
    mux_service_ = declare_parameter<std::string>(
      "mux_service", "/video/register_overlay");
    overlay_timeout_s_ = declare_parameter<double>("overlay_timeout_s", 2.0);

    if (jpeg_quality_ < 10 || jpeg_quality_ > 100) {
      throw std::runtime_error("jpeg_quality hors de [10, 100]");
    }

    pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      enriched_topic_, rclcpp::SensorDataQoS());
    stats_pub_ = create_publisher<bamboo_interfaces::msg::TrackingStats>(
      "/videotracking/stats", rclcpp::QoS(1).reliable());

    // CADENCE DU NOEUD : TrackState. C'est le seul signal qui arrive une fois par cycle de
    // tracking et qui porte deja l'horodatage de la trame -- s'abonner en plus au flux
    // brut ne ferait que dupliquer le reveil.
    track_sub_ = create_subscription<bamboo_interfaces::msg::TrackState>(
      "/videotracking/track_state", rclcpp::QoS(1).reliable(),
      [this](bamboo_interfaces::msg::TrackState::ConstSharedPtr m) {onTrack(m);});
    recog_sub_ = create_subscription<bamboo_interfaces::msg::RecognitionResult>(
      "/videotracking/recognition", rclcpp::QoS(1).reliable(),
      [this](bamboo_interfaces::msg::RecognitionResult::ConstSharedPtr m) {
        rec_ = m;
        recog_rate_.tick();
      });
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

    stats_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(stats_period_s_ * 1000.0)),
      [this]() {publishStats();});

    if (register_with_mux_) {
      mux_cli_ = create_client<bamboo_interfaces::srv::RegisterOverlay>(mux_service_);
      // ON RETENTE, on n'echoue pas : bamboo_video peut demarrer apres nous, et le groupe
      // tracking doit rester utilisable sans lui (banc V6.3, robot eteint).
      reg_timer_ = create_wall_timer(
        std::chrono::seconds(2), [this]() {tryRegister();});
    }
    last_stats_ = now_steady_.now();
    RCLCPP_INFO(get_logger(), "overlay_node -> %s", enriched_topic_.c_str());
  }

  ~OverlayNode() override
  {
    // Desenregistrement EXPLICITE : le mux sait aussi retomber sur le brut par perte de
    // liveliness ou par timeout, mais le dire franchement evite `overlay_timeout_s`
    // secondes d'image figee a l'ecran pendant un simple redemarrage du groupe.
    if (mux_cli_ && registered_ && mux_cli_->service_is_ready()) {
      auto req = std::make_shared<bamboo_interfaces::srv::RegisterOverlay::Request>();
      req->node_name = std::string(get_name());
      req->topic = enriched_topic_;
      req->deregister = true;
      mux_cli_->async_send_request(req);
    }
  }

private:
  void onMode(bamboo_interfaces::msg::ModeCmd::ConstSharedPtr m)
  {
    // On ne REJOUE pas une commande deja vue : meme garde anti-rejeu que tracking_node,
    // le topic etant transient_local un abonne tardif recoit la derniere commande.
    if (m->seq != 0 && m->seq == last_mode_seq_) {return;}
    last_mode_seq_ = m->seq;
    // Les noms de cible sont ceux de ModeCmd, pas des synonymes : "tracker"/"predict"
    // et non "track_mode"/"predict_mode". Une valeur VIDE est une bascule ou un cycle
    // (un appui de bouton) -- ici on ne fait que refleter l'etat, donc une bascule se
    // contente d'inverser l'affichage, et un cycle laisse la pastille inchangee tant
    // que le noeud concerne n'a pas republie sa valeur.
    const bool on = (m->value == "on" || m->value == "true" || m->value == "1");
    if (m->target == "tracking") {
      tracking_on_ = m->value.empty() ? !tracking_on_ : on;
    } else if (m->target == "recognition") {
      if (m->value.empty()) {
        recog_mode_ = (recog_mode_ == "off") ? "recognition" : "off";
      } else {
        recog_mode_ = on ? "recognition" : m->value;
      }
    } else if (m->target == "acquisition") {
      if (m->value.empty()) {
        recog_mode_ = (recog_mode_ == "acquisition") ? "recognition" : "acquisition";
      } else {
        recog_mode_ = on ? "acquisition" : m->value;
      }
    } else if (m->target == "tracker") {
      if (!m->value.empty()) {track_mode_ = m->value;}
    } else if (m->target == "predict") {
      if (!m->value.empty()) {predict_mode_ = m->value;}
    } else if (m->target == "detector") {
      if (!m->value.empty()) {detector_ = m->value;}
    }
  }

  void onTrack(bamboo_interfaces::msg::TrackState::ConstSharedPtr ts)
  {
    track_ = ts;
    if (ts->locked) {
      ++lock_cycles_;
      lock_source_ = ts->src;
    } else if (!ts->lost_reason.empty()) {
      unlock_reason_ = ts->lost_reason;
    }
    ++track_cycles_;

    const Frame f = frameBus().latest();
    if (!f.valid()) {return;}
    if (f.seq == last_seq_) {return;}

    // COMPTAGE DES TRAMES SAUTEES : le bus est une case de profondeur 1, donc une trame
    // perdue ici est une trame que l'incrustation n'a jamais vue. C'est la mesure la plus
    // honnete de la saturation du RPi4 -- plus parlante qu'un fps moyen.
    if (last_seq_ != 0 && f.seq > last_seq_ + 1) {
      dropped_ += static_cast<unsigned>(f.seq - last_seq_ - 1);
    }
    last_seq_ = f.seq;

    const rclcpp::Time t_now = now_steady_.now();
    if (min_period_s_ > 0.0 && last_draw_.nanoseconds() != 0) {
      if ((t_now - last_draw_).seconds() < min_period_s_) {return;}
    }
    last_draw_ = t_now;

    draw(f, *ts);
  }

  void draw(const Frame & f, const bamboo_interfaces::msg::TrackState & ts)
  {
    // LA COPIE. Le prototype dessinait dans la ndarray qu'il venait de publier ; ici la
    // Mat du bus est partagee `const` par pointeur avec tracking_node et facerecog_node.
    // Dessiner dessus corromprait l'image que les autres etages sont en train de lire.
    cv::Mat im = f.mat->clone();

    if (ts.bbox_w > 0 && ts.bbox_h > 0) {
      const cv::Scalar col = (ts.src == "predicted") ? kPredicted
        : (ts.locked ? kLocked : kLost);
      const cv::Rect box(ts.bbox_x, ts.bbox_y, ts.bbox_w, ts.bbox_h);
      cv::rectangle(im, box & cv::Rect(0, 0, im.cols, im.rows), col, 2);
      std::ostringstream tag;
      tag << ts.src;
      if (ts.score > 0.0f) {tag << " " << std::fixed << std::setprecision(2) << ts.score;}
      putShadowed(im, tag.str(), cv::Point(box.x, std::max(12, box.y - 6)), 0.45);
      if (rec_ && rec_->status == "known") {
        std::ostringstream id;
        id << rec_->name << " " << std::fixed << std::setprecision(2) << rec_->score;
        putShadowed(im, id.str(),
          cv::Point(box.x, std::min(im.rows - 4, box.y + box.height + 16)), 0.5);
      }
    } else if (!ts.lost_reason.empty()) {
      // On AFFICHE la cause de perte : sans elle, l'operateur voit "plus de cadre" et ne
      // peut pas distinguer un visage sorti du champ d'un tracker qui a decroche.
      putShadowed(im, "perdu: " + ts.lost_reason, cv::Point(8, im.rows - 10), 0.45);
    }

    // Pastilles d'etat : c'est la reponse a l'exigence "boutons activation des modes" --
    // le flux externe est en lecture seule, donc on montre l'ETAT, pas un bouton cliquable.
    int x = 8;
    x = modePill(im, x, 8, "TRACK", tracking_on_);
    x = modePill(im, x, 8, "RECO", recog_mode_ == "recognition" || recog_mode_ == "acquisition");
    x = modePill(im, x, 8, "ACQ", recog_mode_ == "acquisition");
    x = modePill(im, x, 8, track_mode_.empty() ? "-" : track_mode_, ts.locked);
    modePill(im, x, 8, predict_mode_.empty() ? "off" : predict_mode_,
      predict_mode_ != "off" && !predict_mode_.empty());

    const double lat_ms = latencyMs(f.stamp_ns);
    if (lat_ms > lat_max_ms_) {lat_max_ms_ = lat_ms;}
    lat_sum_ms_ += lat_ms;
    ++lat_n_;

    if (draw_stats_) {
      std::ostringstream s;
      s << std::fixed << std::setprecision(1)
        << "ovl " << last_overlay_fps_ << " fps  det " << last_detect_fps_
        << "  lat " << std::setprecision(0) << lat_ms << " ms";
      putShadowed(im, s.str(), cv::Point(8, im.rows - 28), 0.45);
    }

    std::vector<unsigned char> buf;
    const std::vector<int> par{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", im, buf, par)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "imencode a echoue");
      return;
    }

    sensor_msgs::msg::CompressedImage out;
    // HORODATAGE DE CAPTURE, jamais `now()` : c'est ce qui rend la latence bout en bout
    // mesurable en aval. Le reecrire ici remettrait la mesure a zero.
    out.header.stamp = rclcpp::Time(f.stamp_ns);
    out.header.frame_id = "camera";
    out.format = "jpeg";
    out.data.assign(buf.begin(), buf.end());
    pub_->publish(out);
    overlay_rate_.tick();
  }

  double latencyMs(int64_t stamp_ns) const
  {
    // La trame est horodatee par tracking_node avec l'horloge du noeud ; on compare donc
    // avec la MEME horloge, pas avec l'horloge monotone de l'ordonnancement interne.
    const int64_t now_ns = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds();
    return static_cast<double>(now_ns - stamp_ns) / 1.0e6;
  }

  void publishStats()
  {
    const rclcpp::Time t = now_steady_.now();
    double win = (last_stats_.nanoseconds() == 0) ? stats_period_s_
      : (t - last_stats_).seconds();
    if (win <= 0.0) {win = stats_period_s_;}
    last_stats_ = t;

    last_overlay_fps_ = overlay_rate_.take(win);
    last_detect_fps_ = static_cast<float>(track_cycles_) / static_cast<float>(win);

    bamboo_interfaces::msg::TrackingStats m;
    m.header.stamp = get_clock()->now();
    m.header.frame_id = "camera";
    // capture_fps = cadence du bus, donc du decodage : c'est ce que tracking_node a
    // reellement absorbe, trames sautees comprises.
    m.capture_fps = last_detect_fps_;
    m.detect_fps = last_detect_fps_;
    m.recog_fps = recog_rate_.take(win);
    m.overlay_fps = last_overlay_fps_;
    m.latency_ms = (lat_n_ > 0) ? static_cast<float>(lat_sum_ms_ / lat_n_) : 0.0f;
    m.latency_ms_max = static_cast<float>(lat_max_ms_);
    m.loss_rate = (track_cycles_ > 0)
      ? 1.0f - static_cast<float>(lock_cycles_) / static_cast<float>(track_cycles_)
      : 0.0f;
    m.lock_source = lock_source_;
    m.unlock_reason = unlock_reason_;
    // cpu_percent reste a 0 : le mesurer honnetement demande /proc/stat du CONTENEUR,
    // qui voit les 4 coeurs de l'hote -- un chiffre faux serait pire que pas de chiffre.
    m.cpu_percent = 0.0f;
    m.dropped_frames = dropped_;
    stats_pub_->publish(m);

    lat_sum_ms_ = 0.0;
    lat_n_ = 0;
    lat_max_ms_ = 0.0;
    lock_cycles_ = 0;
    track_cycles_ = 0;
  }

  void tryRegister()
  {
    if (registered_ || !mux_cli_) {return;}
    if (!mux_cli_->service_is_ready()) {
      // PAS un avertissement a chaque essai : bamboo_video peut legitimement ne pas
      // tourner (banc V6.3). On le dit une fois, en INFO.
      if (!reg_warned_) {
        RCLCPP_INFO(get_logger(), "%s absent : on continue sans mux, retente",
          mux_service_.c_str());
        reg_warned_ = true;
      }
      return;
    }
    if (reg_pending_) {return;}
    auto req = std::make_shared<bamboo_interfaces::srv::RegisterOverlay::Request>();
    req->node_name = std::string(get_name());
    req->topic = enriched_topic_;
    req->timeout_s = static_cast<float>(overlay_timeout_s_);
    req->deregister = false;
    reg_pending_ = true;
    mux_cli_->async_send_request(
      req,
      [this](rclcpp::Client<bamboo_interfaces::srv::RegisterOverlay>::SharedFuture fut) {
        reg_pending_ = false;
        const auto r = fut.get();
        if (r->accepted) {
          registered_ = true;
          reg_timer_->cancel();
          RCLCPP_INFO(get_logger(), "enregistre au mux video");
        } else {
          // REFUS MOTIVE, pas un ecrasement silencieux : un second client deja enregistre
          // est une erreur de deploiement, et le motif est le seul moyen de le voir.
          RCLCPP_WARN(get_logger(), "mux a refuse l'enregistrement : %s",
            r->reason.c_str());
        }
      });
  }

  // --- parametres ---
  std::string enriched_topic_;
  std::string mux_service_;
  int jpeg_quality_{80};
  double min_period_s_{0.0};
  double stats_period_s_{1.0};
  double overlay_timeout_s_{2.0};
  bool draw_stats_{true};
  bool register_with_mux_{true};

  // --- ROS ---
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::Publisher<bamboo_interfaces::msg::TrackingStats>::SharedPtr stats_pub_;
  rclcpp::Subscription<bamboo_interfaces::msg::TrackState>::SharedPtr track_sub_;
  rclcpp::Subscription<bamboo_interfaces::msg::RecognitionResult>::SharedPtr recog_sub_;
  rclcpp::Subscription<bamboo_interfaces::msg::ModeCmd>::SharedPtr mode_sub_;
  rclcpp::Client<bamboo_interfaces::srv::RegisterOverlay>::SharedPtr mux_cli_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr reg_timer_;

  // --- etat affiche ---
  bamboo_interfaces::msg::TrackState::ConstSharedPtr track_;
  bamboo_interfaces::msg::RecognitionResult::ConstSharedPtr rec_;
  std::string recog_mode_{"off"};
  std::string track_mode_{"mil"};
  std::string predict_mode_{"off"};
  std::string detector_{"yunet"};
  bool tracking_on_{false};
  uint32_t last_mode_seq_{0};

  // --- mesure ---
  // Horloge MONOTONE pour les fenetres de comptage : un saut de /clock ou de NTP ferait
  // sinon apparaitre un fps absurde, et c'est ce chiffre qu'on publie.
  rclcpp::Clock now_steady_{RCL_STEADY_TIME};
  rclcpp::Time last_draw_;
  rclcpp::Time last_stats_;
  RateCounter overlay_rate_;
  RateCounter recog_rate_;
  uint64_t last_seq_{0};
  unsigned dropped_{0};
  unsigned lock_cycles_{0};
  unsigned track_cycles_{0};
  double lat_sum_ms_{0.0};
  unsigned lat_n_{0};
  double lat_max_ms_{0.0};
  float last_overlay_fps_{0.0f};
  float last_detect_fps_{0.0f};
  std::string lock_source_{"none"};
  std::string unlock_reason_{""};

  // --- enregistrement au mux ---
  bool registered_{false};
  bool reg_pending_{false};
  bool reg_warned_{false};
};

}  // namespace bamboo_videotracking

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bamboo_videotracking::OverlayNode)
