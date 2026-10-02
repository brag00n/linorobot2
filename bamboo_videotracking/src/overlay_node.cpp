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
#include <std_msgs/msg/bool.hpp>

#include "bamboo_interfaces/msg/mode_cmd.hpp"
#include "bamboo_interfaces/msg/recognition_result.hpp"
#include "bamboo_interfaces/msg/track_state.hpp"
#include "bamboo_interfaces/msg/servo_cmd.hpp"
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
int modePill(
  cv::Mat & im, int x, int y, const std::string & label, bool on, double u = 1.0)
{
  // `u` comme partout ailleurs dans ce HUD : sans lui, les pastilles resteraient de
  // taille 480p sur un flux 720p, seules a ne pas suivre.
  const double sc = 0.4 * u;
  const int pad = std::max(2, static_cast<int>(std::lround(6 * u)));
  int base = 0;
  const cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base);
  const cv::Rect r(x, y, ts.width + 2 * pad, ts.height + 2 * pad);
  const cv::Scalar col = on ? kLocked : cv::Scalar(90, 90, 90);
  cv::rectangle(im, r, col, on ? cv::FILLED : 1, cv::LINE_AA);
  cv::putText(im, label, cv::Point(x + pad, y + pad + ts.height - 1),
    cv::FONT_HERSHEY_SIMPLEX, sc, on ? kShade : kInk, 1, cv::LINE_AA);
  // On renvoie le X SUIVANT, pas la largeur : l'appelant chaine les pastilles.
  return x + r.width + std::max(2, static_cast<int>(std::lround(5 * u)));
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
// --- palette et grammaire visuelle DU PROTOTYPE ---------------------------------------
// Valeurs reprises a l'IDENTIQUE de robot_controlv3/RobotMain.py:100-116, y compris les
// gris. L'exigence du lot est de passer d'un essai proto a un essai ROS sans reapprendre a
// lire l'ecran : un gris de libelle different suffirait a faire douter qu'on regarde la
// meme grandeur, et c'est exactement ce qu'on veut eviter en debogage.
const cv::Scalar kBorder(95, 95, 95);      // cadre pointille de carte
const cv::Scalar kLabel(150, 150, 150);    // libelles
const cv::Scalar kVal(235, 235, 235);      // valeurs
const cv::Scalar kOn(0, 220, 0);           // etat actif
const cv::Scalar kOff(120, 120, 120);      // etat inactif
const cv::Scalar kZoneIn(0, 200, 0);       // zone morte atteinte (aucune correction)
const cv::Scalar kZoneOut(0, 165, 255);    // zone morte non atteinte
const cv::Scalar kMarker(0, 0, 255);       // point de la cible (mesure)
const cv::Scalar kLockId(147, 20, 255);    // identite d'episode (DeepPink en BGR)
const cv::Scalar kOrange(0, 140, 255);     // prediction
const cv::Scalar kCross(80, 80, 80);       // croix de centrage
const cv::Scalar kBad(0, 0, 255);          // absent / en faute (`_C_BAD` du proto)

/// Pastille MANETTE : portage de la pastille d'etat du bouton MANETTE du prototype
/// (RobotMain.py, `_draw_help_matrix`), TROIS etats et non deux :
///   +1 vert  = manette connectee et lue ;
///    0 rouge = noeud manette en marche, manette ABSENTE ;
///   -1 gris  = noeud manette ARRETE (aucun etat recu) -- le `None` du proto.
/// Le gris n'est pas un raffinement : sans lui, "noeud arrete" et "manette eteinte"
/// s'afficheraient pareil, et on chercherait la manette quand c'est le conteneur qui manque.
int gamepadPill(cv::Mat & im, int x, int y, int state, double u = 1.0)
{
  // Memes proportions que modePill, pour que la seconde ligne s'aligne sur la premiere.
  const double sc = 0.4 * u;
  const int pad = std::max(2, static_cast<int>(std::lround(6 * u)));
  const int dot_r = std::max(2, static_cast<int>(std::lround(4 * u)));
  const int dot_gap = std::max(2, static_cast<int>(std::lround(8 * u)));
  const std::string label = "MANETTE";
  int base = 0;
  const cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, sc, 1, &base);
  const cv::Rect r(x, y, ts.width + dot_gap + 2 * dot_r + 2 * pad, ts.height + 2 * pad);
  cv::rectangle(im, r, cv::Scalar(90, 90, 90), 1, cv::LINE_AA);
  cv::putText(im, label, cv::Point(x + pad, y + pad + ts.height - 1),
    cv::FONT_HERSHEY_SIMPLEX, sc, kInk, 1, cv::LINE_AA);
  const cv::Scalar col = state < 0 ? kOff : (state > 0 ? kOn : kBad);
  cv::circle(im, cv::Point(x + pad + ts.width + dot_gap + dot_r, y + r.height / 2), dot_r,
    col, cv::FILLED, cv::LINE_AA);
  return x + r.width + std::max(2, static_cast<int>(std::lround(5 * u)));
}

// Duree de vie d'episode au-dela de laquelle le verrou est juge STABLE : l'age vire au
// vert dans la carte, comme `_LOCK_STABLE_S` du proto.
const double kLockStableS = 2.0;

/// Mise a l'echelle d'une distance du proto : `u` = hauteur de trame / 480.
inline int iu(double v, double u) {return static_cast<int>(std::lround(v * u));}

/// Texte SANS ombre, couleur libre : l'equivalent de `_put` du proto.
void put(
  cv::Mat & im, const std::string & s, int x, int y, const cv::Scalar & col,
  double sc = 0.45, int th = 1)
{
  cv::putText(im, s, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, sc, col, th, cv::LINE_AA);
}

/// Nombre formate a `nd` decimales, sans passer par printf (types flottants varies).
std::string fmt(double v, int nd)
{
  std::ostringstream o;
  o << std::fixed << std::setprecision(nd) << v;
  return o.str();
}

void dashedLine(
  cv::Mat & im, cv::Point p1, cv::Point p2, const cv::Scalar & col, int dash = 7,
  int gap = 5)
{
  const double dx = p2.x - p1.x;
  const double dy = p2.y - p1.y;
  const int dist = static_cast<int>(std::lround(std::sqrt(dx * dx + dy * dy)));
  if (dist == 0) {return;}
  for (int i = 0; i < dist; i += dash + gap) {
    const double a = i / static_cast<double>(dist);
    const double b = std::min(i + dash, dist) / static_cast<double>(dist);
    cv::line(
      im,
      cv::Point(
        static_cast<int>(p1.x + dx * a), static_cast<int>(p1.y + dy * a)),
      cv::Point(
        static_cast<int>(p1.x + dx * b), static_cast<int>(p1.y + dy * b)),
      col, 1, cv::LINE_AA);
  }
}

void dashedRect(cv::Mat & im, cv::Rect r, const cv::Scalar & col, double u)
{
  const int dash = std::max(2, iu(7, u));
  const int gap = std::max(2, iu(5, u));
  dashedLine(im, r.tl(), cv::Point(r.x + r.width, r.y), col, dash, gap);
  dashedLine(im, cv::Point(r.x + r.width, r.y), r.br(), col, dash, gap);
  dashedLine(im, r.br(), cv::Point(r.x, r.y + r.height), col, dash, gap);
  dashedLine(im, cv::Point(r.x, r.y + r.height), r.tl(), col, dash, gap);
}

/// Fond de carte assombri (proto `_card_bg`, alpha 0.55). Le proto melange la ROI avec un
/// rectangle noir ; melanger avec du noir revient a MULTIPLIER par (1 - alpha), donc on
/// evite l'allocation d'une Mat de zeros a chaque trame pour le meme resultat exact.
void cardBg(cv::Mat & im, cv::Rect r, double alpha = 0.55)
{
  const cv::Rect roi = r & cv::Rect(0, 0, im.cols, im.rows);
  if (roi.width <= 0 || roi.height <= 0) {return;}
  // `Mat& operator*=(Mat&, double)` est une fonction LIBRE : elle ne se lie pas a un
  // temporaire, d ou l en-tete nomme. Il partage les pixels de `im`, donc l ecriture
  // porte bien sur l image.
  cv::Mat band = im(roi);
  band *= (1.0 - alpha);
}

/// Un champ de ligne de carte : decalage en x DEPUIS le bord de la carte, texte, couleur.
struct Field
{
  int dx;
  std::string text;
  cv::Scalar col;
};

/// Une ligne de carte a colonnes FIXES. Chaque champ est dessine independamment, donc une
/// valeur qui change de longueur ne decale JAMAIS ses voisines -- c'est l'exigence de
/// format stable du proto (`_row`) : un HUD dont les colonnes sautent est illisible en
/// mouvement, et c'est precisement en mouvement qu'on le lit.
void cardRow(cv::Mat & im, int x, int y, double u, const std::vector<Field> & fields)
{
  for (const auto & f : fields) {
    put(im, f.text, x + iu(f.dx, u), y, f.col, 0.45 * u);
  }
}

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
    // Periode du BATTEMENT d'enregistrement au mux (cf. tryRegister). 2 s = le flux
    // annote revient moins de 2 s apres un redemarrage du groupe video, sans intervention.
    reg_period_s_ = declare_parameter<double>("reg_period_s", 2.0);

    // --- HUD : chaque trace s'eteint sans recompiler, et porte le nom du proto ---------
    draw_reticle_ = declare_parameter<bool>("draw_reticle", true);
    draw_marker_ = declare_parameter<bool>("draw_marker", true);
    draw_prediction_ = declare_parameter<bool>("draw_prediction", true);
    draw_detections_ = declare_parameter<bool>("draw_detections", true);
    draw_card_ = declare_parameter<bool>("draw_card", true);
    // ZONE MORTE ET ASPECT : les MEMES que servocam_node, et c'est le launch qui les pousse
    // depuis la MEME source (le YAML du robot). Ce noeud ne peut pas les lire chez l'autre
    // -- un parametre appartient a un noeud -- donc la coherence est une exigence de
    // cablage, verifiee a chaque trame par drawReticle() qui JOURNALISE une divergence.
    // Deux valeurs differentes donneraient un reticule qui ne dit pas la zone morte
    // reellement appliquee : le pire des cas, un HUD qui ment.
    deadzone_ = declare_parameter<double>("deadzone", 0.25);
    aspect_ = declare_parameter<double>("aspect", 0.75);
    servo_cmd_topic_ = declare_parameter<std::string>("servo_cmd_topic", "/servo/cmd");
    // Etat de la manette publie par bamboo_teleop (std_msgs/Bool a 2 Hz) : MEME nom que
    // `gamepad_state_topic` de bamboo_control/config/joy_bamboo.yaml.
    gamepad_state_topic_ = declare_parameter<std::string>(
      "gamepad_state_topic", "/gamepad/connected");

    // FIABILITE DU FLUX ENRICHI. La compatibilite QoS de DDS n'est PAS symetrique : un
    // abonne `reliable` REFUSE un publieur `best_effort` (et le journal le dit alors :
    // "requesting incompatible QoS. No messages will be sent to it"), tandis qu'un publieur
    // `reliable` satisfait AUSSI les abonnes `best_effort`. Or les clients de ce topic sont
    // de deux familles : stream_mux_node souscrit en best_effort, mais rosbridge,
    // foxglove_bridge et web_video_server souscrivent en reliable par defaut. Publier en
    // reliable est donc le seul reglage qui les serve TOUS avec un seul nom de topic -- et
    // c'est deja ce que fait le mux un saut plus loin (stream_mux_node.py:66).
    // Profondeur 1 pour la meme raison qu'en H1 : une trame d'affichage perimee ne vaut
    // rien, la mettre en file ne fait que retarder la suivante.
    // Le parametre existe parce qu'un writer reliable dont l'historique n'est pas acquitte
    // peut BLOQUER publish() jusqu'a max_blocking_time (100 ms par defaut) : negligeable a
    // la cadence de detection, mais on veut pouvoir revenir a best_effort SANS recompiler
    // -- une reconstruction coute une vingtaine de minutes sur ce RPi.
    enriched_reliable_ = declare_parameter<bool>("enriched_reliable", true);

    if (jpeg_quality_ < 10 || jpeg_quality_ > 100) {
      throw std::runtime_error("jpeg_quality hors de [10, 100]");
    }

    auto enr_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (enriched_reliable_) {enr_qos.reliable();} else {enr_qos.best_effort();}
    pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(enriched_topic_, enr_qos);
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

    // CONSIGNES SERVO : c'est la derniere fusion qui manquait a ce noeud pour etre le
    // point de fusion COMPLET du groupe (~9 Hz, message minuscule, cout nul). Sans elle,
    // impossible de montrer a l'ecran que la loi se TAIT quand la cible est dans la zone
    // morte -- or c'est la contre-epreuve qui prouve que le reticule dit la verite.
    gamepad_sub_ = create_subscription<std_msgs::msg::Bool>(
      gamepad_state_topic_, rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        gamepad_ok_ = m->data;
        last_gamepad_ = now_steady_.now();
        have_gamepad_ = true;
      });
    servo_sub_ = create_subscription<bamboo_interfaces::msg::ServoCmd>(
      servo_cmd_topic_, rclcpp::QoS(10).reliable(),
      [this](bamboo_interfaces::msg::ServoCmd::ConstSharedPtr m) {
        // `relative` n'est pas gere ici a dessein : servocam_node n'emet que de l'ABSOLU
        // (emit(), angle entier). Un increment viendrait d'un autre producteur, et
        // l'accumuler ici afficherait un angle que personne n'a commande.
        if (m->relative) {return;}
        if (m->axis == "pan") {
          pan_deg_ = m->angle_deg;
        } else if (m->axis == "tilt") {
          tilt_deg_ = m->angle_deg;
        } else {
          return;
        }
        servo_seen_ = true;
        last_servo_ = now_steady_.now();
      });

    stats_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(stats_period_s_ * 1000.0)),
      [this]() {publishStats();});

    if (register_with_mux_) {
      mux_cli_ = create_client<bamboo_interfaces::srv::RegisterOverlay>(mux_service_);
      // ON RETENTE SANS FIN, on n'echoue pas et on n'arrete jamais : bamboo_video peut
      // demarrer apres nous (le groupe tracking doit rester utilisable sans lui, banc V6.3)
      // ET redemarrer sous nous. Ce minuteur n'est donc pas un essai, c'est un BATTEMENT.
      reg_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(reg_period_s_ * 1000.0)),
        [this]() {tryRegister();});
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
    } else if (m->target == "deadzone") {
      // DELTA signe, memes bornes que servocam_node : le reticule doit montrer la zone
      // morte REELLEMENT appliquee, sinon le HUD mentirait des le premier appui.
      if (!m->value.empty()) {
        try {
          deadzone_ = std::min(0.45, std::max(0.03, deadzone_ + std::stod(m->value)));
        } catch (const std::exception &) {
          RCLCPP_WARN(get_logger(), "deadzone refuse : \"%s\" n'est pas un nombre",
            m->value.c_str());
        }
      }
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

  // --- aides de trace, portage 1:1 du prototype -------------------------------------
  // Toutes prennent `u` = hauteur/480 et multiplient CHAQUE dx, dy, rayon et echelle de
  // police par lui. C'est ainsi que l'exigence "tenir compte de la resolution" est tenue
  // par CONSTRUCTION : passer le flux en 1280x720 met le HUD a l'echelle sans qu'aucune
  // constante de ce fichier ne change. Les marqueurs, eux, sont deja relatifs a la trame
  // (deadzone*fh/2, pred_nx*fw/2) : ils n'ont besoin de `u` que pour leurs libelles.

  /// Croix de centrage + SURFACE centrale visee (proto `_overlay_reticle`).
  void drawReticle(cv::Mat & im, const bamboo_interfaces::msg::TrackState & ts, double u)
  {
    const int fw = im.cols;
    const int fh = im.rows;
    cv::line(im, cv::Point(fw / 2, 0), cv::Point(fw / 2, fh), kCross, 1);
    cv::line(im, cv::Point(0, fh / 2), cv::Point(fw, fh / 2), kCross, 1);

    // Boite CARREE a l'ecran, demi-cote inscrit dans la HAUTEUR : nx et ny etant
    // normalises chacun par sa demi-dimension, le seuil horizontal est resserre par
    // `aspect` (= h/w) pour que la boite dessinee et la zone morte reellement appliquee
    // coincident. On utilise le PARAMETRE `aspect`, pas le rapport mesure de la trame :
    // c'est le parametre que servocam_node applique, donc c'est lui la verite de la loi.
    // S'ils divergent, on le DIT plutot que de dessiner une zone morte fausse.
    const double measured = (fw > 0) ? static_cast<double>(fh) / fw : 1.0;
    if (std::fabs(measured - aspect_) > 0.05) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "parametre aspect=%.3f contre %.3f mesure sur la trame (%dx%d) : la zone morte "
        "dessinee suit le PARAMETRE, comme la loi pan -- corriger le YAML du robot",
        aspect_, measured, fw, fh);
    }
    const int half = static_cast<int>(deadzone_ * fh / 2.0);
    const bool has = (ts.bbox_w > 0 && ts.bbox_h > 0);
    const bool in_zone = has &&
      std::fabs(static_cast<double>(ts.nx)) <= deadzone_ * aspect_ &&
      std::fabs(static_cast<double>(ts.ny)) <= deadzone_;
    cv::rectangle(
      im, cv::Point(fw / 2 - half, fh / 2 - half), cv::Point(fw / 2 + half, fh / 2 + half),
      in_zone ? kZoneIn : kZoneOut, std::max(1, static_cast<int>(std::lround(u))));
  }

  /// Point de la cible + identite d'episode + nx/ny/aire (proto `_overlay_main_marker`).
  void drawMainMarker(cv::Mat & im, const bamboo_interfaces::msg::TrackState & ts, double u)
  {
    if (ts.bbox_w <= 0 || ts.bbox_h <= 0) {return;}
    const int cx = ts.bbox_x + ts.bbox_w / 2;
    const int cy = ts.bbox_y + ts.bbox_h / 2;
    cv::circle(im, cv::Point(cx, cy), std::max(2, iu(4, u)), kMarker, cv::FILLED);

    if (ts.locked) {
      std::ostringstream id;
      id << "id #" << ts.lock_id << "  " << std::fixed << std::setprecision(1)
         << ts.lock_age_s << "s";
      put(im, id.str(), ts.bbox_x, std::max(0, ts.bbox_y - iu(28, u)), kLockId, 0.6 * u, 2);
    }
    std::ostringstream p;
    p << std::fixed << std::setprecision(2) << std::showpos
      << "nx=" << ts.nx << " ny=" << ts.ny << std::noshowpos
      << " aire=" << std::setprecision(1) << ts.area_pct << "%";
    put(im, p.str(), ts.bbox_x, std::max(0, ts.bbox_y - iu(8, u)), kLocked, 0.6 * u, 2);
  }

  /// Vecteur de prediction Kalman (proto `_overlay_prediction`).
  void drawPrediction(cv::Mat & im, const bamboo_interfaces::msg::TrackState & ts, double u)
  {
    // Le proto se garde sur `pred_nx is None` ; en ROS le champ existe toujours, donc
    // c'est `pred_phase` qui dit si la prediction est armee. Dessiner un vecteur en mode
    // off ferait croire a une anticipation alors que la consigne suit la mesure brute.
    // "home" est traite comme "off" : le prototype renvoie None pour le point predit quand
    // la roue libre est finie, donc il ne dessine rien. Sans ce garde, nos champs pred_*
    // restes a zero feraient pointer une fleche sur le CENTRE de l image, qu on lirait comme
    // une prediction alors qu il n y a plus de cible du tout.
    if (ts.pred_phase.empty() || ts.pred_phase == "off" || ts.pred_phase == "home") {return;}
    const int fw = im.cols;
    const int fh = im.rows;
    int px = static_cast<int>(std::lround(fw / 2.0 + ts.pred_nx * fw / 2.0));
    int py = static_cast<int>(std::lround(fh / 2.0 + ts.pred_ny * fh / 2.0));
    px = std::max(6, std::min(fw - 6, px));
    py = std::max(6, std::min(fh - 6, py));
    // En `coast` le visage est perdu : la fleche part alors du CENTRE, comme dans le proto.
    const cv::Point src = (ts.bbox_w > 0 && ts.bbox_h > 0)
      ? cv::Point(ts.bbox_x + ts.bbox_w / 2, ts.bbox_y + ts.bbox_h / 2)
      : cv::Point(fw / 2, fh / 2);
    cv::arrowedLine(im, src, cv::Point(px, py), kOrange, 2, cv::LINE_AA, 0, 0.3);
    cv::circle(im, cv::Point(px, py), std::max(3, iu(6, u)), kOrange, 2);
    put(im, "pred", px + iu(8, u), py - iu(8, u), kOrange, 0.5 * u);
  }

  /// Carte CAM, meme grammaire que `_draw_cam_card` du proto (colonnes FIXES).
  void drawCard(cv::Mat & im, const bamboo_interfaces::msg::TrackState & ts, double u)
  {
    // 300x137 a l'echelle 1 : en-tete 38 + SIX lignes au pas de 19 (= 114) + 4 de
    // jambage. Le plan annoncait 118, qui ne tient que cinq lignes -- on garde les six
    // du proto plutot que d'en amputer une, soit 13,4 % de l'image en 640x480.
    const int x = iu(8, u);
    const int y = iu(8, u);
    cardBg(im, cv::Rect(x, y, iu(300, u), iu(137, u)));
    dashedRect(im, cv::Rect(x, y, iu(300, u), iu(137, u)), kBorder, u);

    const int hy = y + iu(18, u);
    cv::circle(
      im, cv::Point(x + iu(13, u), hy - iu(5, u)), std::max(2, iu(5, u)),
      tracking_on_ ? kOn : kOff, cv::FILLED);
    put(im, "CAM", x + iu(25, u), hy, kVal, 0.5 * u);
    std::ostringstream port;
    port << im.cols << "x" << im.rows;
    put(im, port.str(), x + iu(92, u), hy, kLabel, 0.45 * u);

    const int yc = y + iu(38, u);
    const int step = iu(19, u);
    const bool lk = ts.locked;

    cardRow(im, x, yc, u, {
      {10, "suivi", kLabel}, {78, tracking_on_ ? "ON" : "off", tracking_on_ ? kOn : kOff},
      {150, "det", kLabel}, {200, detector_.empty() ? "?" : detector_, kVal}});
    cardRow(im, x, yc + step, u, {
      {10, "trk", kLabel}, {78, track_mode_.empty() ? "none" : track_mode_, kVal},
      {150, "lock", kLabel}, {200, lk ? ("ON(" + ts.src + ")") : "off", lk ? kOn : kOff}});
    cardRow(im, x, yc + 2 * step, u, {
      {10, "sc", kLabel}, {78, fmt(ts.score, 2), kVal},
      // `det hit/miss` du proto demande la boite BRUTE du detecteur, qui n'est pas dans
      // TrackState (lot H4, reporte). On affiche "--" plutot que de remplir la colonne
      // avec autre chose : une valeur absente doit se VOIR absente.
      {150, "det", kLabel}, {200, "--", kOff}});
    cardRow(im, x, yc + 3 * step, u, {
      // P1 = cadence d'AFFICHAGE, P2 = cadence de DETECTION : memes deux chiffres que le
      // proto, ou P1 vaut `self.disp_fps` et P2 `met.det_fps`.
      {10, "P1", kLabel}, {48, fmt(last_overlay_fps_, 0), kVal},
      {110, "P2", kLabel}, {148, fmt(last_detect_fps_, 0), kVal},
      {210, "vis", kLabel}, {258, std::to_string(ts.n_faces), kVal}});
    const std::string pm = predict_mode_.empty() ? "off" : predict_mode_;
    if (pm != "off") {
      cardRow(im, x, yc + 4 * step, u, {
        {10, "pred", kLabel}, {78, pm, kVal},
        {150, "ph", kLabel}, {200, ts.pred_phase.empty() ? "--" : ts.pred_phase, kVal}});
    } else {
      cardRow(im, x, yc + 4 * step, u, {{10, "pred", kLabel}, {78, "off", kOff}});
    }
    if (lk) {
      cardRow(im, x, yc + 5 * step, u, {
        {10, "id", kLabel}, {48, "#" + std::to_string(ts.lock_id), kVal},
        {110, "age", kLabel},
        {150, fmt(ts.lock_age_s, 1) + "s", ts.lock_age_s >= kLockStableS ? kOn : kVal}});
    } else {
      cardRow(im, x, yc + 5 * step, u, {{10, "id", kLabel}, {48, "--", kOff}});
    }
  }

  void draw(const Frame & f, const bamboo_interfaces::msg::TrackState & ts)
  {
    // LA COPIE. Le prototype dessinait dans la ndarray qu'il venait de publier ; ici la
    // Mat du bus est partagee `const` par pointeur avec tracking_node et facerecog_node.
    // Dessiner dessus corromprait l'image que les autres etages sont en train de lire.
    cv::Mat im = f.mat->clone();
    // FACTEUR D'ECHELLE UNIQUE de tout le HUD. 480 est la hauteur de reference du proto.
    const double u = (im.rows > 0) ? im.rows / 480.0 : 1.0;

    if (draw_reticle_) {drawReticle(im, ts, u);}

    if (draw_detections_ && ts.bbox_w > 0 && ts.bbox_h > 0) {
      // UNE SEULE boite : la cible fusionnee. La liste PAR VISAGE et la boite brute du
      // detecteur du proto demandent des champs absents de TrackState (lot H4).
      const cv::Scalar col = (ts.src == "predicted") ? kPredicted
        : (ts.locked ? kLocked : kLost);
      const cv::Rect box(ts.bbox_x, ts.bbox_y, ts.bbox_w, ts.bbox_h);
      cv::rectangle(im, box & cv::Rect(0, 0, im.cols, im.rows), col, 2);
      if (!draw_card_) {
        // src et score vivent dans la carte ; sans carte on les remet sur la boite, mais
        // JAMAIS les deux : ce libelle se superpose au nx/ny du marqueur principal.
        std::ostringstream tag;
        tag << ts.src;
        if (ts.score > 0.0f) {tag << " " << std::fixed << std::setprecision(2) << ts.score;}
        putShadowed(im, tag.str(), cv::Point(box.x, std::max(12, box.y - 6)), 0.45 * u);
      }
      if (rec_ && rec_->status == "known") {
        std::ostringstream id;
        id << rec_->name << " " << std::fixed << std::setprecision(2) << rec_->score;
        putShadowed(im, id.str(),
          cv::Point(box.x, std::min(im.rows - 4, box.y + box.height + iu(16, u))), 0.5 * u);
      }
    } else if (!ts.lost_reason.empty()) {
      // On AFFICHE la cause de perte : sans elle, l'operateur voit "plus de cadre" et ne
      // peut pas distinguer un visage sorti du champ d'un tracker qui a decroche.
      putShadowed(
        im, "perdu: " + ts.lost_reason, cv::Point(8, im.rows - iu(10, u)), 0.45 * u);
    }

    if (draw_prediction_) {drawPrediction(im, ts, u);}
    if (draw_marker_) {drawMainMarker(im, ts, u);}

    // Pastilles d'etat : c'est la reponse a l'exigence "boutons activation des modes" --
    // le flux externe est en lecture seule, donc on montre l'ETAT, pas un bouton cliquable.
    // En HAUT A DROITE, la carte occupant desormais le haut a gauche.
    const int pill_y = iu(8, u);
    const int pill_x0 = std::max(iu(8, u), im.cols - iu(330, u));
    int x = pill_x0;
    x = modePill(im, x, pill_y, "TRACK", tracking_on_, u);
    x = modePill(
      im, x, pill_y, "RECO", recog_mode_ == "recognition" || recog_mode_ == "acquisition", u);
    x = modePill(im, x, pill_y, "ACQ", recog_mode_ == "acquisition", u);
    x = modePill(im, x, pill_y, track_mode_.empty() ? "-" : track_mode_, ts.locked, u);
    modePill(im, x, pill_y, predict_mode_.empty() ? "off" : predict_mode_,
      predict_mode_ != "off" && !predict_mode_.empty(), u);

    // MANETTE sur une SECONDE ligne : la premiere s'arrete au bord de la carte (x = 308 a
    // 640 px de large), il n'y a pas la place d'une sixieme pastille sans la chevaucher.
    // Gris si l'etat se tait depuis 4 periodes : bamboo_teleop publie a 2 Hz, donc 2 s de
    // silence ne peut pas etre un simple retard.
    int gp_state = -1;
    if (have_gamepad_ && (now_steady_.now() - last_gamepad_).seconds() <= 2.0) {
      gp_state = gamepad_ok_ ? 1 : 0;
    }
    gamepadPill(im, pill_x0, pill_y + iu(26, u), gp_state, u);

    const double lat_ms = latencyMs(f.stamp_ns);
    if (lat_ms > lat_max_ms_) {lat_max_ms_ = lat_ms;}
    lat_sum_ms_ += lat_ms;
    ++lat_n_;

    if (draw_card_) {drawCard(im, ts, u);}

    if (draw_stats_) {
      // LATENCE et DESYNCHRO, en gros et en bas a gauche : ce sont les deux chiffres qui
      // auraient montre d'emblee le temps mort de 2,5 s de T10. P1/P2 sont dans la carte.
      // La desynchro est un AVEU : draw() dessine sur frameBus().latest(), pas sur la
      // trame qui a produit ce TrackState -- le cadre peut donc porter sur une trame plus
      // recente que la mesure. Un HUD qui avoue son decalage vaut mieux qu'un HUD qui le
      // cache. Apres H1 elle doit valoir une periode de trame, pas deux secondes.
      const double desync_ms = (static_cast<double>(f.stamp_ns) -
        static_cast<double>(rclcpp::Time(ts.header.stamp).nanoseconds())) / 1.0e6;
      std::ostringstream s;
      s << std::fixed << std::setprecision(0)
        << "lat " << lat_ms << " ms   desync " << desync_ms << " ms";
      putShadowed(im, s.str(), cv::Point(iu(8, u), im.rows - iu(28, u)), 0.5 * u);

      // Consignes servo VUES PASSER, en bas a droite. Vert quand elles se TAISENT : c'est
      // la contre-epreuve de la zone morte (T14), la boite doit verdir au meme instant.
      if (servo_seen_) {
        const double age_s = (now_steady_.now() - last_servo_).seconds();
        std::ostringstream sv;
        sv << std::fixed << std::setprecision(0) << "pan " << pan_deg_
           << "  tilt " << tilt_deg_ << "  cmd " << std::setprecision(1) << age_s << "s";
        put(im, sv.str(), im.cols - iu(250, u), im.rows - iu(10, u),
          (age_s > 0.5) ? kOn : kZoneOut, 0.5 * u);
      } else {
        put(im, "servo: aucune consigne", im.cols - iu(250, u), im.rows - iu(10, u),
          kOff, 0.5 * u);
      }
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

  /// Le battement a perdu le mux : le DIRE une fois, puis laisser tryRegister() retenter.
  void noteLost(const char * why)
  {
    if (!registered_) {return;}
    registered_ = false;
    RCLCPP_WARN(
      get_logger(),
      "enregistrement au mux PERDU (%s) : le flux annote n'est plus servi, on retente", why);
  }

  void tryRegister()
  {
    // BATTEMENT, et non un essai unique : on ne sort PAS sur `registered_`. Le mux peut
    // redemarrer sans nous -- il perd alors son registre, tandis que nous nous croyons
    // toujours enregistres, et le flux annote cesse d'etre servi EN SILENCE. Mesure : le
    // mux avait redemarre quatre fois, l'enrichi avait 0 souscripteur, et l'overlay
    // continuait a calculer une image que personne ne consommait. Le service est idempotent
    // pour le meme client sur le meme topic (stream_mux_node.py:193), donc reaffirmer
    // l'enregistrement a chaque battement est gratuit et AUTORITAIRE -- compter les
    // souscripteurs du topic ne le serait pas, web_video_server pouvant s'y abonner seul
    // quand on regarde l'URL de debogage.
    if (!mux_cli_) {return;}
    if (!mux_cli_->service_is_ready()) {
      // PAS un avertissement a chaque essai : bamboo_video peut legitimement ne pas
      // tourner (banc V6.3). On le dit une fois, en INFO.
      if (!reg_warned_) {
        RCLCPP_INFO(get_logger(), "%s absent : on continue sans mux, retente",
          mux_service_.c_str());
        reg_warned_ = true;
      }
      noteLost("service absent");
      return;
    }
    reg_warned_ = false;  // reapparition : un prochain depart sera de nouveau annonce
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
          // Journalise la TRANSITION, pas le battement : sinon une ligne toutes les
          // `reg_period_s` noierait le journal.
          if (!registered_) {
            RCLCPP_INFO(get_logger(), "enregistre au mux video (%s)", r->reason.c_str());
          }
          registered_ = true;
          last_refusal_.clear();
        } else {
          // REFUS MOTIVE, pas un ecrasement silencieux : un second client deja enregistre
          // est une erreur de deploiement, et le motif est le seul moyen de le voir. On ne
          // le repete que s'il CHANGE.
          if (r->reason != last_refusal_) {
            RCLCPP_WARN(get_logger(), "mux a refuse l'enregistrement : %s",
              r->reason.c_str());
            last_refusal_ = r->reason;
          }
          noteLost("refus du mux");
        }
      });
  }

  // --- parametres ---
  std::string enriched_topic_;
  // true = publieur reliable (sert le mux ET les clients generiques), false = best_effort.
  bool enriched_reliable_{true};
  std::string mux_service_;
  int jpeg_quality_{80};
  double min_period_s_{0.0};
  double stats_period_s_{1.0};
  double overlay_timeout_s_{2.0};
  double reg_period_s_{2.0};
  bool draw_stats_{true};
  bool draw_reticle_{true};
  bool draw_marker_{true};
  bool draw_prediction_{true};
  bool draw_detections_{true};
  bool draw_card_{true};
  // Zone morte et aspect : les memes que servocam_node, pousses par le launch depuis
  // la MEME source. Cf. la declaration pour la raison -- un reticule qui ne dirait pas
  // la zone morte reellement appliquee serait un HUD qui MENT.
  double deadzone_{0.25};
  double aspect_{0.75};
  std::string servo_cmd_topic_;
  bool register_with_mux_{true};

  // --- ROS ---
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::Publisher<bamboo_interfaces::msg::TrackingStats>::SharedPtr stats_pub_;
  rclcpp::Subscription<bamboo_interfaces::msg::TrackState>::SharedPtr track_sub_;
  rclcpp::Subscription<bamboo_interfaces::msg::RecognitionResult>::SharedPtr recog_sub_;
  rclcpp::Subscription<bamboo_interfaces::msg::ModeCmd>::SharedPtr mode_sub_;
  rclcpp::Subscription<bamboo_interfaces::msg::ServoCmd>::SharedPtr servo_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gamepad_sub_;
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
  // Etat MANETTE : `have_gamepad_` distingue "jamais rien recu" d'un etat perime.
  bool gamepad_ok_{false};
  bool have_gamepad_{false};
  std::string gamepad_state_topic_;
  // Derniere consigne servo VUE PASSER -- jamais l angle ATTEINT : aucune carte du
  // projet ne sait relire la position d un servo PWM (cf. ServoCmd.msg).
  float pan_deg_{0.0f};
  float tilt_deg_{0.0f};
  bool servo_seen_{false};
  rclcpp::Time last_servo_;
  uint32_t last_mode_seq_{0};

  // --- mesure ---
  // Horloge MONOTONE pour les fenetres de comptage : un saut de /clock ou de NTP ferait
  // sinon apparaitre un fps absurde, et c'est ce chiffre qu'on publie.
  rclcpp::Clock now_steady_{RCL_STEADY_TIME};
  rclcpp::Time last_gamepad_{0, 0, RCL_STEADY_TIME};
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
  std::string last_refusal_;  // dernier motif de refus journalise (evite la repetition)
};

}  // namespace bamboo_videotracking

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bamboo_videotracking::OverlayNode)
