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
  double max_step{6.0};    // deg par SECONDE de cible (voir trackAxis), pas par cycle
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

    // Valeurs du prototype, et de la BONNE COUCHE : `RobotServoMotor.__init__` porte des
    // defauts de classe (panGain 18, tiltGain 10, deadzone 0,08, maxStep 6) que
    // `RobotMain.py` ECRASE tous par ses defauts argparse (10, 6, 0,25, 4). Le prototype
    // etant TOUJOURS lance par RobotMain, ce sont ces derniers qui sont calibres -- avoir lu
    // la couche constructeur est ce qui a produit une camera en butee au premier essai ROS.
    // Les NOMS sont eux aussi ceux du prototype (`--invert-pan`, `--max-step`...) pour qu'un
    // essai se transpose du banc au ROS sans retraduction.
    // UNE SEULE CORRECTION assumee : le prototype borne le pan a 17-178, mais la fiche carte
    // etablit que 172 deg est le DERNIER angle sain du SG90 -- au-dela il force en butee en
    // continu (~700 mA, pignons plastique). Le clamp vit donc ici, cote hote, sans reflash,
    // et les bornes arrivent de toute facon du fichier canonique (cf. le launch).
    pan_.gain = declare_parameter<double>("pan_gain", 10.0);
    pan_.min_deg = declare_parameter<double>("pan_min", 8.0);
    pan_.max_deg = declare_parameter<double>("pan_max", 172.0);
    pan_.home = declare_parameter<double>("pan_home", 88.0);
    pan_.invert = declare_parameter<bool>("invert_pan", false);

    tilt_.gain = declare_parameter<double>("tilt_gain", 6.0);
    tilt_.min_deg = declare_parameter<double>("tilt_min", 20.0);
    tilt_.max_deg = declare_parameter<double>("tilt_max", 70.0);
    tilt_.home = declare_parameter<double>("tilt_home", 42.0);
    // true, comme `--invert-tilt` du prototype : ny > 0 designe un visage BAS dans l'image,
    // et c'est en DIMINUANT l'angle du S2 que la camera descend. Sans cette inversion la
    // boucle est a contre-reaction POSITIVE et le tilt part en butee (mesure a l'essai T7).
    tilt_.invert = declare_parameter<bool>("invert_tilt", true);

    const double dz = declare_parameter<double>("deadzone", 0.25);
    const double dh = declare_parameter<double>("dead_hyst", 0.05);
    // max_step : MEME NOM et MEME VALEUR que le prototype (`--max-step`), mais lu ici en
    // budget par SECONDE et non par cycle -- voir trackAxis, ou ce choix est justifie.
    const double ms = declare_parameter<double>("max_step", 4.0);
    pan_.dead = tilt_.dead = dz;
    pan_.dead_hyst = tilt_.dead_hyst = dh;
    pan_.max_step = tilt_.max_step = ms;

    max_vel_ = declare_parameter<double>("max_vel", 120.0);
    max_accel_ = declare_parameter<double>("max_accel", 400.0);
    smooth_ = declare_parameter<bool>("smooth", true);
    // aspect = HAUTEUR / LARGEUR, et le sens compte : nx est normalise par w/2 et ny par
    // h/2, donc une zone morte CARREE A L'ECRAN demande un seuil de pan multiplie par h/w
    // (0,5625 en 16:9). Le rapport inverse, lui, ELARGIT la zone morte du pan au lieu de la
    // resserrer -- c'est ce qui laissait le pan muet sur un visage a nx = -0,13. La valeur
    // reelle est derivee de camera_width/camera_height par le launch ; ce defaut compile
    // vaut 1.0, le meme repli neutre que ServoNode.py du prototype.
    aspect_ = declare_parameter<double>("aspect", 1.0);
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
      // L'etat ARME etait invisible : rien ne distinguait "desarme" de "aucune cible", et
      // les deux se traduisent par une camera immobile. On le journalise donc.
      RCLCPP_INFO(get_logger(), "suivi %s (pan %.1f, tilt %.1f)",
        enabled_ ? "ARME" : "desarme", pan_.angle, tilt_.angle);
      if (!enabled_) {
        // ON NE RECENTRE PAS en desarmant : la camera reste ou elle est. Recentrer
        // ferait bouger le robot a l'instant ou l'operateur coupe le suivi, ce qui est
        // exactement le contraire de ce qu'il demande.
        pan_.target = pan_.angle;
        tilt_.target = tilt_.angle;
      }
    } else if (m->target == "recenter") {
      returnHome();
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

  /// Vise le repos en laissant la rampe y glisser -- transposition de
  /// RobotServoMotor.returnHome (RobotServoMotor.py:142-158). Deux proprietes portent tout :
  /// on NE SAUTE PAS au centre (on pose la cible et slew() l atteint, meme profil de vitesse
  /// que le suivi, VITESSE COURANTE INTACTE pour ne pas casser l elan en cours) ; et c est
  /// IDEMPOTENT, donc appelable a chaque trame sans a-coup. Le prototype remet aussi son
  /// `_settled` a faux ; nous n avons pas d equivalent a rouvrir, `emit()` ne se bloquant que
  /// sur un angle entier inchange, ce que la rampe fait varier d elle-meme.
  void returnHome()
  {
    pan_.target = clampAxis(pan_, pan_.home);
    tilt_.target = clampAxis(tilt_, tilt_.home);
  }

  void onTrack(bamboo_interfaces::msg::TrackState::ConstSharedPtr ts)
  {
    // CADENCE REELLE de la mesure, calculee AVANT d ecraser last_track_. C est le pas de
    // temps de l etage 1, et il n a rien a voir avec rate_hz_ qui cadence l etage 2 : le
    // TrackState arrive a la cadence de DETECTION, qui suit la charge du RPi (4,4 Hz
    // mesures a T17, contre 15 a 30 Hz sur le prototype). Borne a idle_timeout_s_ par le
    // haut : au-dela, onTick gele la cible de toute facon, donc un pas plus grand ne
    // decrirait aucun mouvement reel.
    const rclcpp::Time t = now_steady_.now();
    dt_track_ = std::min(idle_timeout_s_, std::max(1.0e-3, (t - last_track_).seconds()));
    last_track_ = t;
    if (!enabled_) {return;}
    // CIBLE PERDUE POUR DE BON. tracking_node publie un TrackState a chaque trame, verrouille
    // ou non, donc la phase terminale du predicteur nous parvient bel et bien -- c est notre
    // propre sortie sur !locked qui l interceptait avant, et le retour au centre n existait
    // donc pas. Le prototype fait ce depart ici meme (RobotWebCamMotorized.py:200-204) :
    // cible presente -> track(), phase "home" -> returnHome(), et RIEN sinon.
    if (ts->pred_phase == "home") {
      returnHome();
      return;
    }
    // ROUE LIBRE : le verrou est DEJA retombe (c'est justement ce qui la declenche), donc
    // exiger `locked` la tuait -- un trou qui ne se voyait pas tant que predictLost ne
    // coastait qu en mode "coast", que personne n utilise. La bbox est vide pendant la roue
    // libre, d'ou un chemin qui ne la consulte pas : la cible est le point extrapole.
    const bool coasting = (ts->pred_phase == "coast");
    if (!coasting) {
      if (!ts->locked) {return;}
      if (ts->bbox_w <= 0 || ts->bbox_h <= 0) {return;}
    }

    // On suit la PREDICTION quand elle est disponible : c'est tout l'interet du mode
    // anticip, qui compense le retard de la chaine de detection. Sinon la mesure.
    const bool use_pred = (ts->pred_phase == "anticip" || coasting);
    const double nx = use_pred ? ts->pred_nx : ts->nx;
    const double ny = use_pred ? ts->pred_ny : ts->ny;

    // SIGNE : nx > 0 = visage a DROITE de l'image. Augmenter l'angle de pan tourne la
    // camera d'un cote qui depend du montage -> `invert_pan` existe pour ca, et c'est la
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
    // PLAFOND DU PAS, exprime en deg par SECONDE et non par cycle. DIVERGENCE ASSUMEE de
    // forme avec le prototype, qui borne a `max_step` par appel tout court -- et qui a
    // raison de le faire chez lui, ou l appel vient a 15-30 Hz. Ici la detection tourne a
    // 4,4 Hz (T17) : le meme 4 deg par appel plafonnait l avance de la cible a 17,6 deg/s,
    // soit SEPT fois moins que la rampe (max_vel 120) n en demandait. La camera etait donc
    // lente non par un gain trop faible mais par un plafond compte dans la mauvaise unite,
    // et les 30 pour cent de gigue de la cadence se lisaient directement en saccade.
    // On garde donc le NOMBRE du prototype et sa signification A 30 Hz (4 x 30 = 120 deg/s,
    // exactement max_vel) et on le ramene au temps reellement ecoule.
    const double cap = a.max_step * rate_hz_ * dt_track_;
    step = std::min(cap, std::max(-cap, step));
    if (a.invert) {step = -step;}
    // REBASE SUR L ANGLE COURANT (`before + step` du prototype, before = angle emis) et
    // NON sur la cible precedente : en cumulant sur la cible, celle-ci prend de l avance
    // sur la rampe des que l etage 2 ne suit pas, et la camera court vers une cible
    // perimee -- un depassement que le freinage anticipe ne peut pas rattraper puisque la
    // cible elle-meme est fausse.
    a.target = clampAxis(a, a.angle + step);
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

    // ARRET FRANC, a l identique du prototype : arrive a moins de 0,05 deg de la cible avec
    // moins de 1 deg/s, on annule la vitesse au lieu de la laisser tendre vers zero.
    // Ce qui vivait ici avant etait un amortissement en demi-vie de 0,3 s applique a la
    // VITESSE, annonce comme repris du prototype : il ne l etait pas. Le `0.5 ** (dt/0.3)`
    // du prototype decroit `_stepPeak`, une metrique de HUD a maintien de crete, et pas la
    // vitesse de la rampe. Applique a la vitesse, il faisait tramer la fin de course.
    if (std::fabs(d) < 0.05 && std::fabs(a.vel) < 1.0) {a.vel = 0.0;}

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
  // Ecrase des le constructeur par le parametre `aspect` (defaut 1.0, repli neutre) : cet
  // initialiseur ne sert qu'a ne jamais lire un membre non initialise.
  double aspect_{1.0};
  double rate_hz_{30.0};
  double idle_timeout_s_{0.5};
  // Pas de temps de l ETAGE 1, mesure entre deux TrackState (et non rate_hz_, qui cadence
  // l etage 2). Defaut = la periode nominale de 30 Hz, donc comportement du prototype tant
  // qu aucune mesure n est arrivee.
  double dt_track_{1.0 / 30.0};
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
