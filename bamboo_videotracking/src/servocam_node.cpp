// bamboo_videotracking / servocam_node -- loi de commande pan/tilt (lot V3.4).
//
// CE NOEUD NE TOUCHE AUCUNE IMAGE ET N'OUVRE AUCUN PORT. Il consomme des TrackState
// normalises (nx, ny dans [-1, +1]) et publie des ServoCmd en adressant les axes PAR
// LEUR NOM ("pan", "tilt"). Il ignore deliberement quelle carte les execute : c'est la
// table `servo_axes` du fichier canonique qui resout nom -> (controleur, voie, bornes).
//
// La couture de decouplage n'est PAS inventee ici : le prototype a deja un
// _ServoCmdProxy (TrackingNode.py:28-43) qui separe la loi de commande du pilote. On le
// promeut en topic ROS, on ne le concoit pas.
//
// IL TOURNE SANS EXECUTEUR, et c'est voulu : sans carte branchee il publie dans le vide
// sans erreur. C'est precisement ce qui rend la loi de commande rejouable au banc V6.3,
// robot eteint, et comparable d'une execution a l'autre.
//
// DEUX ETAGES, A NE PAS CONFONDRE (les deux viennent de RobotServoMotor.py) :
//   1. _trackAxis : de l'erreur de cadrage vers une CIBLE d'angle -- zone morte avec
//      hysteresis, soustraction de la zone morte pour eviter le saut au bord, pas borne.
//   2. slew       : de la cible vers l'angle EMIS -- limitation de vitesse et
//      d'acceleration avec freinage anticipe. C'est cet etage qui rend le mouvement
//      regardable ; sans lui la camera saccade a chaque detection.
// Et une regle qui compte autant que les deux : ON N'EMET QUE SUR CHANGEMENT ENTIER
// d'angle. Le SG90 ne distingue pas 88,3 de 88,4 degres, mais chaque trame emise occupe
// l'UART partage avec la telemetrie de la carte.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "bamboo_interfaces/msg/mode_cmd.hpp"
#include "bamboo_interfaces/msg/servo_cmd.hpp"
#include "bamboo_interfaces/msg/track_state.hpp"
#include "bamboo_interfaces/srv/nudge.hpp"

namespace bamboo_videotracking
{
namespace
{
/// Un axe : sa loi, ses bornes, son etat de rampe. Deux instances, aucune duplication.
struct Axis
{
  std::string name;
  double gain{18.0};       // deg par unite d'erreur normalisee
  double dead{0.08};       // zone morte en erreur normalisee
  double dead_hyst{0.05};  // relachement : on ressort de la zone morte plus tard
  double max_step{6.0};    // deg par cycle, avant limitation de vitesse
  double min_deg{8.0};
  double max_deg{172.0};
  double home{88.0};
  bool invert{false};

  double target{88.0};     // cible de la loi de commande
  double angle{88.0};      // angle courant de la rampe (ombre d'hote)
  double vel{0.0};         // deg/s
  int last_sent{-1};        // dernier ENTIER emis ; -1 = rien emis encore
  bool inside{true};       // etat de l'hysteresis de zone morte
};
}  // namespace

class ServoCamNode : public rclcpp::Node
{
public:
  explicit ServoCamNode(const rclcpp::NodeOptions & opts)
  : rclcpp::Node("servocam_node", opts)
  {
    pan_.name = "pan";
    tilt_.name = "tilt";

    // Valeurs du prototype (RobotServoMotor.py), A UNE CORRECTION PRES : le prototype
    // borne le pan a 17-178, mais la fiche carte etablit que 172 deg est le DERNIER
    // angle sain du SG90 -- au-dela il force en butee en continu (~700 mA, pignons
    // plastique). Le clamp vit donc ici, cote hote, sans reflash.
    pan_.gain = declare_parameter<double>("pan_gain", 18.0);
    pan_.min_deg = declare_parameter<double>("pan_min_deg", 8.0);
    pan_.max_deg = declare_parameter<double>("pan_max_deg", 172.0);
    pan_.home = declare_parameter<double>("pan_home_deg", 88.0);
    pan_.invert = declare_parameter<bool>("pan_invert", false);

    tilt_.gain = declare_parameter<double>("tilt_gain", 10.0);
    tilt_.min_deg = declare_parameter<double>("tilt_min_deg", 20.0);
    tilt_.max_deg = declare_parameter<double>("tilt_max_deg", 70.0);
    tilt_.home = declare_parameter<double>("tilt_home_deg", 42.0);
    tilt_.invert = declare_parameter<bool>("tilt_invert", false);

    const double dz = declare_parameter<double>("deadzone", 0.08);
    const double dh = declare_parameter<double>("deadzone_hyst", 0.05);
    const double ms = declare_parameter<double>("max_step_deg", 6.0);
    pan_.dead = tilt_.dead = dz;
    pan_.dead_hyst = tilt_.dead_hyst = dh;
    pan_.max_step = tilt_.max_step = ms;

    max_vel_ = declare_parameter<double>("max_vel_deg_s", 120.0);
    max_accel_ = declare_parameter<double>("max_accel_deg_s2", 400.0);
    smooth_ = declare_parameter<bool>("smooth", true);
    // Le cadrage n'est pas carre : une erreur de 0,1 en x et en y ne represente pas le
    // meme ecart angulaire. La zone morte du pan est donc mise a l'echelle de l'aspect.
    aspect_ = declare_parameter<double>("aspect", 640.0 / 480.0);
    rate_hz_ = declare_parameter<double>("rate_hz", 30.0);
    // Sans TrackState depuis ce delai on RELACHE la rampe (vitesse a zero) au lieu de
    // continuer vers une cible perimee : un tracker qui decroche ne doit pas faire
    // partir la camera en buee vers la derniere position connue.
    idle_timeout_s_ = declare_parameter<double>("idle_timeout_s", 0.5);

    pan_.target = pan_.angle = clampAxis(pan_, pan_.home);
    tilt_.target = tilt_.angle = clampAxis(tilt_, tilt_.home);

    pub_ = create_publisher<bamboo_interfaces::msg::ServoCmd>(
      "/servo/cmd", rclcpp::QoS(10).reliable());
    state_pub_ = create_publisher<bamboo_interfaces::msg::ServoCmd>(
      "/servo/state", rclcpp::QoS(1).reliable());

    track_sub_ = create_subscription<bamboo_interfaces::msg::TrackState>(
      "/videotracking/track_state", rclcpp::QoS(1).reliable(),
      [this](bamboo_interfaces::msg::TrackState::ConstSharedPtr m) {onTrack(m);});
    mode_sub_ = create_subscription<bamboo_interfaces::msg::ModeCmd>(
      "/videotracking/mode_cmd", rclcpp::QoS(1).reliable().transient_local(),
      [this](bamboo_interfaces::msg::ModeCmd::ConstSharedPtr m) {onMode(m);});

    nudge_srv_ = create_service<bamboo_interfaces::srv::Nudge>(
      "/servo/nudge",
      [this](bamboo_interfaces::srv::Nudge::Request::SharedPtr req,
      bamboo_interfaces::srv::Nudge::Response::SharedPtr res) {onNudge(req, res);});

    // LA RAMPE A SA PROPRE HORLOGE, independante de la cadence de detection : c'est ce
    // qui donne un mouvement regulier meme quand la detection tombe a 6 fps.
    const int ms_period = static_cast<int>(1000.0 / std::max(1.0, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(ms_period), [this]() {onTick();});
    last_track_ = now_steady_.now();
    last_tick_ = now_steady_.now();
    RCLCPP_INFO(get_logger(),
      "servocam_node : pan [%.0f, %.0f] repos %.0f, tilt [%.0f, %.0f] repos %.0f",
      pan_.min_deg, pan_.max_deg, pan_.home, tilt_.min_deg, tilt_.max_deg, tilt_.home);
  }

private:
  static double clampAxis(const Axis & a, double v)
  {
    return std::min(a.max_deg, std::max(a.min_deg, v));
  }

  void onMode(bamboo_interfaces::msg::ModeCmd::ConstSharedPtr m)
  {
    if (m->seq != 0 && m->seq == last_mode_seq_) {return;}
    last_mode_seq_ = m->seq;
    if (m->target == "tracking") {
      const bool on = (m->value == "on" || m->value == "true" || m->value == "1");
      enabled_ = m->value.empty() ? !enabled_ : on;
      if (!enabled_) {
        // ON NE RECENTRE PAS en desarmant : la camera reste ou elle est. Recentrer
        // ferait bouger le robot a l'instant ou l'operateur coupe le suivi, ce qui est
        // exactement le contraire de ce qu'il demande.
        pan_.target = pan_.angle;
        tilt_.target = tilt_.angle;
      }
    } else if (m->target == "recenter") {
      pan_.target = clampAxis(pan_, pan_.home);
      tilt_.target = clampAxis(tilt_, tilt_.home);
    }
  }

  void onNudge(bamboo_interfaces::srv::Nudge::Request::SharedPtr req,
    bamboo_interfaces::srv::Nudge::Response::SharedPtr res)
  {
    if (req->axis.size() != req->delta.size()) {
      res->success = false;
      res->reason = "axis et delta de longueurs differentes";
      return;
    }
    // ATOMIQUE : on valide TOUS les axes avant d'en bouger un seul, sinon un nom
    // errone en seconde position laisserait le premier axe deja deplace.
    for (const std::string & n : req->axis) {
      if (n != "pan" && n != "tilt") {
        res->success = false;
        res->reason = "axe inconnu de servocam_node : " + n;
        return;
      }
    }
    for (size_t i = 0; i < req->axis.size(); ++i) {
      Axis & a = (req->axis[i] == "pan") ? pan_ : tilt_;
      a.target = clampAxis(a, a.target + req->delta[i]);
      res->angle_deg.push_back(a.target);
    }
    res->success = true;
    res->reason = "";
  }

  void onTrack(bamboo_interfaces::msg::TrackState::ConstSharedPtr ts)
  {
    last_track_ = now_steady_.now();
    if (!enabled_ || !ts->locked) {return;}
    if (ts->bbox_w <= 0 || ts->bbox_h <= 0) {return;}

    // On suit la PREDICTION quand elle est disponible : c'est tout l'interet du mode
    // anticip, qui compense le retard de la chaine de detection. Sinon la mesure.
    const bool use_pred = (ts->pred_phase == "anticip" || ts->pred_phase == "coast");
    const double nx = use_pred ? ts->pred_nx : ts->nx;
    const double ny = use_pred ? ts->pred_ny : ts->ny;

    // SIGNE : nx > 0 = visage a DROITE de l'image. Augmenter l'angle de pan tourne la
    // camera d'un cote qui depend du montage -> `pan_invert` existe pour ca, et c'est la
    // seule chose a changer si la camera part du mauvais cote au premier essai (T6).
    trackAxis(pan_, nx, pan_.dead * aspect_);
    trackAxis(tilt_, ny, tilt_.dead);
  }

  /// Etage 1 : erreur normalisee -> cible d'angle. Transposition de _trackAxis.
  void trackAxis(Axis & a, double err, double dz)
  {
    const double mag = std::fabs(err);
    // HYSTERESIS DE ZONE MORTE : on entre a `dz` mais on ne ressort qu'a `dz + hyst`.
    // Sans elle, un visage pile a la frontiere fait osciller la camera indefiniment.
    if (a.inside) {
      if (mag < dz + a.dead_hyst) {return;}
      a.inside = false;
    } else if (mag < dz) {
      a.inside = true;
      return;
    }
    // SOUSTRACTION DE LA ZONE MORTE : sans elle, le pas saute de 0 a gain*dz des qu'on
    // franchit la frontiere -- un a-coup visible a chaque reprise de suivi.
    const double sgn = (err >= 0.0) ? 1.0 : -1.0;
    const double eff = err - dz * sgn;
    double step = eff * a.gain;
    step = std::min(a.max_step, std::max(-a.max_step, step));
    if (a.invert) {step = -step;}
    a.target = clampAxis(a, a.target + step);
  }

  void onTick()
  {
    const rclcpp::Time t = now_steady_.now();
    double dt = (t - last_tick_).seconds();
    last_tick_ = t;
    // dt BORNE : au premier tick, apres une pause de l'ordonnanceur ou un reveil de
    // conteneur, un dt de plusieurs secondes ferait franchir toute la course en un pas.
    dt = std::min(0.2, std::max(1.0e-3, dt));

    if ((t - last_track_).seconds() > idle_timeout_s_) {
      // Plus de TrackState : on gele la cible et on laisse la rampe s'arreter proprement
      // par sa propre deceleration, plutot que de couper la vitesse net.
      pan_.target = pan_.angle;
      tilt_.target = tilt_.angle;
    }

    slew(pan_, dt);
    slew(tilt_, dt);
  }

  /// Etage 2 : cible -> angle emis, avec limitation de vitesse et d'acceleration.
  void slew(Axis & a, double dt)
  {
    const double d = a.target - a.angle;

    if (!smooth_) {
      a.angle = clampAxis(a, a.target);
      a.vel = 0.0;
      emit(a);
      return;
    }

    // FREINAGE ANTICIPE : la vitesse maximale admissible a la distance |d| est celle
    // qu'on peut encore annuler avec max_accel. Sans ce terme la rampe depasse la cible
    // puis revient -- un depassement bien visible sur une camera.
    const double v_brake = std::sqrt(2.0 * max_accel_ * std::fabs(d));
    const double v_want = std::min(max_vel_, v_brake) * ((d >= 0.0) ? 1.0 : -1.0);

    const double dv_max = max_accel_ * dt;
    a.vel += std::min(dv_max, std::max(-dv_max, v_want - a.vel));

    // AMORTISSEMENT en demi-vie de 0,3 s, repris du prototype : il tue le residu de
    // vitesse quand la cible ne bouge plus, sans dependre de la cadence du timer.
    const double decay = std::pow(0.5, dt / 0.3);
    if (std::fabs(d) < 0.05) {a.vel *= decay;}

    double next = a.angle + a.vel * dt;
    // On ne DEPASSE pas la cible : si le pas la franchit, on s'y arrete.
    if ((d >= 0.0 && next > a.target) || (d < 0.0 && next < a.target)) {
      next = a.target;
      a.vel = 0.0;
    }
    a.angle = clampAxis(a, next);
    emit(a);
  }

  /// N'emet QUE sur changement d'angle ENTIER : le SG90 ne distingue pas 88,3 de 88,4,
  /// et chaque trame occupe l'UART partage avec la telemetrie de la carte.
  void emit(Axis & a)
  {
    const int deg = static_cast<int>(std::lround(a.angle));
    if (deg == a.last_sent) {return;}
    a.last_sent = deg;

    bamboo_interfaces::msg::ServoCmd m;
    m.header.stamp = get_clock()->now();
    m.axis = a.name;
    m.angle_deg = static_cast<float>(deg);
    m.relative = false;
    // id = 0 : L'EXECUTEUR RESOUT par `servo_axes`. Remplir ce champ ici recreerait le
    // couplage a une carte que tout ce chantier retire.
    m.id = 0;
    pub_->publish(m);
    // /servo/state est une OMBRE D'HOTE : c'est ce qu'on a emis, jamais ce que le servo
    // a atteint (aucune carte du projet ne sait relire l'angle d'un servo PWM).
    state_pub_->publish(m);
  }

  Axis pan_;
  Axis tilt_;
  double max_vel_{120.0};
  double max_accel_{400.0};
  double aspect_{640.0 / 480.0};
  double rate_hz_{30.0};
  double idle_timeout_s_{0.5};
  bool smooth_{true};
  // DESARME au demarrage : un groupe tracking qui demarre en bougeant la camera tout
  // seul n'est pas acceptable (meme raison qui le tient hors du profil `boot`).
  bool enabled_{false};
  uint32_t last_mode_seq_{0};

  rclcpp::Publisher<bamboo_interfaces::msg::ServoCmd>::SharedPtr pub_;
  rclcpp::Publisher<bamboo_interfaces::msg::ServoCmd>::SharedPtr state_pub_;
  rclcpp::Subscription<bamboo_interfaces::msg::TrackState>::SharedPtr track_sub_;
  rclcpp::Subscription<bamboo_interfaces::msg::ModeCmd>::SharedPtr mode_sub_;
  rclcpp::Service<bamboo_interfaces::srv::Nudge>::SharedPtr nudge_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Horloge MONOTONE : le dt de la rampe ne doit pas bondir sur un saut de /clock ou de
  // NTP -- une camera qui traverse sa course d'un coup n'est pas un bug d'affichage.
  rclcpp::Clock now_steady_{RCL_STEADY_TIME};
  // Type d'horloge donne a la DECLARATION : un rclcpp::Time par defaut est en
  // RCL_SYSTEM_TIME, et le soustraire d'un temps steady leve une exception.
  rclcpp::Time last_tick_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_track_{0, 0, RCL_STEADY_TIME};
};

}  // namespace bamboo_videotracking

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bamboo_videotracking::ServoCamNode)
