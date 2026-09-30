// tracking_node.cpp - V3.1 : PREMIER etage du chemin chaud, et SEUL decodeur JPEG du groupe.
//
// Portage de RobotWebCamMotorized (prototype robot_controlv3) en composable rclcpp.
//
// SA RESPONSABILITE LA PLUS IMPORTANTE N'EST PAS LE SUIVI, C'EST LE DECODAGE UNIQUE.
// Il est le seul noeud a appeler cv::imdecode, et il depose la cv::Mat obtenue dans le
// FrameBus intra-processus. facerecog_node et overlay_node y lisent LA MEME matrice par
// pointeur : un second imdecode couterait ~8 ms de CPU par trame sur RPi4 pour produire une
// copie bit-a-bit identique. C'est tout le gain du C++ dans ce chantier ; le reste du
// portage aurait pu rester en Python.
//
// COORDONNEES NORMALISEES, identiques au prototype (RobotWebCamMotorized.py:341-349) :
//   nx = (cx - fw/2) / (fw/2)   -> +1 a droite du cadre, -1 a gauche
//   ny = (cy - fh/2) / (fh/2)   -> +1 en BAS du cadre (sens image, pas sens maths)
// Elles sont normalisees et pas en pixels pour que la loi de commande pan/tilt de
// servocam_node soit independante de la resolution de capture : changer 640x480 pour
// 1280x720 ne doit pas demander de retoucher un gain.

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>        // cv::flip (miroir de trame, cf. parametre `flip`)
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "bamboo_interfaces/msg/mode_cmd.hpp"
#include "bamboo_interfaces/msg/track_state.hpp"
#include "bamboo_videotracking/face_detection.hpp"
#include "bamboo_videotracking/frame_bus.hpp"
#include "bamboo_videotracking/kalman_predictor.hpp"

namespace bamboo_videotracking
{

class TrackingNode : public rclcpp::Node
{
public:
  explicit TrackingNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tracking_node", options)
  {
    // --- parametres -----------------------------------------------------------------
    // Valeurs du prototype. Le chemin des modeles est un fait d'HOTE (bind-mount du depot
    // firmware), donc un parametre ; les noms de fichiers sont des constantes de
    // face_detection.cpp, parce que ce sont des noms de modeles et pas des reglages.
    models_dir_ = declare_parameter<std::string>("models_dir", "");
    const std::string in_topic =
      declare_parameter<std::string>("input_topic", "/video/raw/compressed");
    predict_mode_ = declare_parameter<std::string>("predict_mode", "anticip");
    predict_horizon_s_ = declare_parameter<double>("predict_horizon_s", 0.12);
    coast_max_s_ = declare_parameter<double>("coast_max_s", 0.7);
    // predict_min_speed : NOM ET VALEUR du prototype (--predict-min-speed, defaut 0,4), en
    // unites normalisees par seconde. Sous ce seuil, une perte de cible ne declenche aucune
    // roue libre et passe directement en phase "home".
    predict_min_speed_ = declare_parameter<double>("predict_min_speed", 0.4);
    // MIROIR DE TRAME, equivalent de `--flip h` du prototype, applique la ou le prototype
    // l'applique : a la CAPTURE (RobotSensorWebCam.read), donc en amont de tout. Memes
    // valeurs que lui : none | v | h | 180. Le prototype ne compense PAS le miroir dans sa
    // loi de commande -- le mot `flip` n'apparait nulle part dans device/motion/ -- et c'est
    // pour cela que le triplet (flip, invert_pan, invert_tilt) se regle ENSEMBLE.
    flip_ = declare_parameter<std::string>("flip", "h");
    if (flip_ != "none" && flip_ != "v" && flip_ != "h" && flip_ != "180") {
      // Bruyant, pas de repli muet : un miroir ignore en silence met le pan a l'envers, et
      // une camera qui fuit son sujet ne ressemble pas a une faute de configuration.
      RCLCPP_FATAL(get_logger(), "flip '%s' inconnu (none|v|h|180)", flip_.c_str());
      throw std::runtime_error("flip invalide : " + flip_);
    }

    DetectConfig cfg;
    cfg.detector = declare_parameter<std::string>("detector", cfg.detector);
    cfg.track_mode = declare_parameter<std::string>("track_mode", cfg.track_mode);
    cfg.det_width = declare_parameter<int>("det_width", cfg.det_width);
    cfg.det_conf = declare_parameter<double>("det_conf", cfg.det_conf);
    cfg.redetect_ms = declare_parameter<double>("redetect_ms", cfg.redetect_ms);
    cfg.iou_reanchor = declare_parameter<double>("iou_reanchor", cfg.iou_reanchor);
    cfg.score_min = declare_parameter<double>("score_min", cfg.score_min);
    cfg.hold_ms = declare_parameter<double>("hold_ms", cfg.hold_ms);
    cfg.hold_score_min = declare_parameter<double>("hold_score_min", cfg.hold_score_min);
    cfg.max_det_misses = declare_parameter<int>("max_det_misses", cfg.max_det_misses);
    cfg.max_grow = declare_parameter<double>("max_grow", cfg.max_grow);
    cfg.max_area_frac = declare_parameter<double>("max_area_frac", cfg.max_area_frac);

    std::string err;
    if (!det_.build(models_dir_, cfg, err)) {
      // ECHEC BRUYANT, et volontairement pas un mode degrade : un noeud de suivi qui
      // demarre sans detecteur publierait des TrackState eternellement deverrouilles, ce
      // qui ressemble exactement a "il n'y a personne devant la camera".
      RCLCPP_FATAL(get_logger(), "detecteur indisponible : %s", err.c_str());
      throw std::runtime_error(err);
    }
    RCLCPP_INFO(
      get_logger(), "detecteur=%s tracker=%s det_width=%d modeles=%s", cfg.detector.c_str(),
      cfg.track_mode.c_str(), cfg.det_width, models_dir_.c_str());

    // --- interfaces ROS -------------------------------------------------------------
    // Le flux d'entree est BEST-EFFORT, et c'est un choix : une trame perdue vaut mieux
    // qu'une trame en retard. Un suivi qui rattrape une file d'attente suit le passe.
    //
    // PROFONDEUR 1, et NON `SensorDataQoS()` qui en garde 5 : sur un flux a 30 Hz consomme a
    // 3 Hz, une file de 5 est en permanence PLEINE, donc chaque trame traitee a cinq trames
    // de retard -- la file GARANTIT le retard, elle ne l'absorbe pas. Mesure : l'age de
    // l'en-tete valait 9 ms en sortie de gscam2 et 2461 ms en mediane ici, avec 188 trames
    // perdues. Un asservissement dont la mesure a 2,5 s de retard oscille par construction :
    // il corrige une position que le visage a deja quittee. C'etait la cause du pompage
    // pan/tilt observe a T10, alors que la loi de commande et ses gains etaient justes.
    // Avec une profondeur de 1, l'intergiciel ECRASE la trame en attente : on traite
    // toujours la plus recente, et le retard retombe a une periode de detection.
    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    // GROUPE DE RAPPELS DEDIE A L'IMAGE, et ce n'est pas une optimisation : la detection
    // occupe 130 a 230 ms par trame pour une cadence de 6 Hz, donc elle SATURE son groupe.
    // Le groupe par defaut etant MUTUELLEMENT EXCLUSIF, tout ce qui y vivait aussi --
    // l'abonnement aux modes ET les services de parametres que rclcpp y installe -- n'a
    // JAMAIS obtenu de creneau : le noeud publiait son TrackState en restant SOURD a toute
    // commande, sans le moindre message d'erreur. Mesure a l'essai T22 de bambooSTM32YB :
    // `ros2 param get` sans reponse, et un `predict` volontairement invalide qui ne
    // journalisait meme pas son refus. Le conteneur est multi-thread (component_container_mt),
    // donc deux groupes distincts tournent en parallele et le defaut redevient libre.
    img_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions img_opts;
    img_opts.callback_group = img_cbg_;
    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      in_topic, img_qos,
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {onImage(msg);}, img_opts);

    // TrackState en RELIABLE profondeur 1 : c'est un message de COMMANDE (servocam_node en
    // derive des consignes servo) et il doit etre lisible a travers rosbridge, que
    // sensor_data rend invisible -- piege deja paye au chantier 1.
    pub_ = create_publisher<bamboo_interfaces::msg::TrackState>(
      "/videotracking/track_state", rclcpp::QoS(1).reliable());

    // `transient_local` (latche) est INCOMPATIBLE avec l'intra-process : rclcpp refuse en
    // construction "intraprocess communication allowed only with volatile durability". On
    // desactive donc l'intra-process sur CETTE SEULE entite -- elle est hors chemin chaud (une
    // commande de mode par appui de touche), alors que les IMAGES, elles, doivent rester en
    // zero-copy : c'est tout l'interet du container.
    rclcpp::SubscriptionOptions mode_opts;
    mode_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    mode_sub_ = create_subscription<bamboo_interfaces::msg::ModeCmd>(
      "/videotracking/mode_cmd", rclcpp::QoS(1).reliable().transient_local(),
      [this](bamboo_interfaces::msg::ModeCmd::ConstSharedPtr msg) {onMode(msg);}, mode_opts);

    params_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & ps) {return onParams(ps);});
  }

  double detFps() const {return det_.detFps();}

private:
  // --- reception d'une trame -------------------------------------------------------
  void onImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
  {
    // DECODAGE UNIQUE DU GROUPE. cv::IMREAD_COLOR force 3 canaux BGR : les etages
    // suivants (SFace, incrustation) supposent tous du BGR 8 bits, et une webcam qui
    // rendrait du gris en JPEG ferait autrement echouer un reshape tres loin d'ici.
    cv::Mat bgr = cv::imdecode(cv::Mat(1, static_cast<int>(msg->data.size()), CV_8UC1,
        const_cast<unsigned char *>(msg->data.data())), cv::IMREAD_COLOR);
    if (bgr.empty()) {
      ++decode_fail_;
      // Journal ETRANGLE : un flux casse produirait 30 lignes par seconde, ce qui
      // noierait exactement le message qu'on veut lire.
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "imdecode a echoue (%u trames perdues)",
        decode_fail_);
      return;
    }

    // MIROIR AVANT TOUT LE RESTE : detection, reconnaissance, incrustation et Foxglove
    // partagent ainsi la MEME image, celle que l'operateur voit. Le flux brut publie par
    // bamboo_video, lui, n'est pas touche. cv::flip : 0 = haut/bas, 1 = gauche/droite,
    // -1 = les deux -- memes codes que RobotSensorWebCam.applyFlip du prototype.
    if (flip_ == "v") {
      cv::flip(bgr, bgr, 0);
    } else if (flip_ == "h") {
      cv::flip(bgr, bgr, 1);
    } else if (flip_ == "180") {
      cv::flip(bgr, bgr, -1);
    }

    // HORODATAGE DE CAPTURE, repris de l'amont et JAMAIS remplace par l'heure courante :
    // c'est lui qui rend mesurable la latence bout en bout du lot V6, et une reecriture
    // ici remettrait le compteur a zero a chaque etage, donc afficherait une latence
    // flatteuse et fausse.
    const rclcpp::Time stamp(msg->header.stamp);
    const int64_t stamp_ns = stamp.nanoseconds();

    Frame f;
    f.mat = std::make_shared<const cv::Mat>(std::move(bgr));
    f.seq = ++seq_;
    f.stamp_ns = stamp_ns;
    // PUBLICATION AVANT CALCUL, deliberement : facerecog_node et overlay_node peuvent
    // ainsi travailler sur cette trame PENDANT que le suivi tourne, au lieu d'attendre
    // notre fin de cycle. Le TrackState qu'ils liront est celui de la trame precedente,
    // decalage d'une trame assume et documente -- a 30 fps il vaut 33 ms, la ou une
    // serialisation stricte des trois etages couterait la somme de leurs temps.
    frameBus().publish(f);

    // Base de temps MONOTONE pour la machine a etats (hold_ms, lock_age, dt de Kalman).
    // Surtout PAS l'horloge ROS : un /clock ou un saut de NTP ferait expirer tous les
    // verrous d'un coup.
    const double now_s = static_cast<double>(now_steady_.now().nanoseconds()) * 1e-9;

    // Verrouille pour TOUTE la suite : trackStep, la lecture de config() et le filtre de
    // Kalman forment un seul etat coherent, et le relacher entre deux laisserait une
    // reconfiguration s'inserer au milieu d'une trame.
    std::lock_guard<std::mutex> lk(state_mu_);
    TrackStateOut st = det_.trackStep(*f.mat, now_s);
    publishState(st, *f.mat, stamp, now_s);
  }

  // --- normalisation + prediction + publication -------------------------------------
  void publishState(
    const TrackStateOut & st, const cv::Mat & frame, const rclcpp::Time & stamp, double now_s)
  {
    bamboo_interfaces::msg::TrackState m;
    // STAMP DE LA TRAME, pas de maintenant : voir onImage. C'est le contrat de mesure.
    m.header.stamp = stamp;
    m.header.frame_id = "camera";
    // `mode` porte le TRACKER demande (none | mil | vit), pas le mode de prediction :
    // celui-la vit dans pred_phase. Deux champs, deux notions -- les confondre ferait
    // croire a un lecteur que couper la prediction a coupe le suivi.
    m.mode = det_.config().track_mode;
    m.locked = st.locked;
    m.src = st.src;
    m.score = static_cast<float>(st.score);
    m.n_faces = static_cast<int32_t>(st.n_faces);
    m.lock_id = static_cast<uint32_t>(st.lock_id);
    m.lock_age_s = static_cast<float>(st.lock_age_s);
    m.lost_reason = st.lost_reason;

    const double fw = static_cast<double>(frame.cols);
    const double fh = static_cast<double>(frame.rows);

    if (st.has_main) {
      const double cx = st.main.x + st.main.width * 0.5;
      const double cy = st.main.y + st.main.height * 0.5;
      // Normalisation IDENTIQUE au prototype (RobotWebCamMotorized.py:341-349). Le demi-cadre
      // au denominateur, donc +-1 aux bords ; un changement de resolution ne touche donc
      // aucun gain de servocam_node.
      const double nx = (cx - fw * 0.5) / (fw * 0.5);
      const double ny = (cy - fh * 0.5) / (fh * 0.5);
      m.nx = static_cast<float>(nx);
      m.ny = static_cast<float>(ny);
      m.area_pct = static_cast<float>(100.0 * st.main.area() / (fw * fh));
      m.bbox_x = st.main.x;
      m.bbox_y = st.main.y;
      m.bbox_w = st.main.width;
      m.bbox_h = st.main.height;
      predictLocked(nx, ny, now_s, m);
    } else {
      // CIBLE ABSENTE : on ne publie PAS de nx/ny a zero comme si le visage etait
      // pile au centre -- zero est une position, pas une absence. Les champs restent a
      // leur defaut et c'est `locked` qui fait foi ; servocam_node n'agit que verrou pris.
      predictLost(now_s, m);
    }
    pub_->publish(m);
  }

  // Machine a etats de prediction, transposee de RobotWebCamMotorized._predictStep.
  //   off     : aucune extrapolation, on renvoie la mesure telle quelle
  //   anticip : le filtre est corrige par la mesure, et on AVANCE de predict_horizon_s
  //   coast   : cible perdue -> on continue sur la vitesse estimee, borne dans le temps
  void predictLocked(double nx, double ny, double now_s, bamboo_interfaces::msg::TrackState & m)
  {
    last_seen_s_ = now_s;
    if (predict_mode_ == "off") {
      kal_.resetUninit();
      m.pred_phase = "off";
      m.pred_nx = static_cast<float>(nx);
      m.pred_ny = static_cast<float>(ny);
      return;
    }
    double fnx = nx;
    double fny = ny;
    kal_.update(nx, ny, dtSince(now_s), fnx, fny);
    // peek() est ANALYTIQUE (position + vitesse * horizon) et ne touche pas l'etat du
    // filtre, contrairement a un predict() supplementaire qui ferait avancer le filtre
    // deux fois par trame et doublerait sa vitesse apparente.
    double pnx = fnx;
    double pny = fny;
    if (predict_horizon_s_ > 0.0) {
      kal_.peek(predict_horizon_s_, pnx, pny);
    }
    m.pred_phase = "anticip";
    m.pred_nx = static_cast<float>(pnx);
    m.pred_ny = static_cast<float>(pny);
  }

  // Transposition LIGNE A LIGNE de la fin de _predictStep (RobotWebCamMotorized.py:255-272).
  // Trois conditions doivent TOUTES tenir pour meriter une roue libre : un filtre initialise,
  // un visage qui bougeait a la perte, et une perte encore recente. Sinon -> phase "home",
  // qui est le NOM DU PROTOTYPE et le contrat avec servocam_node : la camera rentre au repos.
  void predictLost(double now_s, bamboo_interfaces::msg::TrackState & m)
  {
    // Le SEUL court-circuit du prototype. En particulier il coaste AUSSI en "anticip" :
    // ce qui vivait ici avant exigeait predict_mode == "coast", donc dans notre defaut
    // (anticip) il n y avait ni roue libre NI phase terminale -- rien a quoi accrocher un
    // retour au centre. "coast" ne nomme pas la roue libre, il nomme ce que le servo suit
    // pendant le verrouillage (mesure brute la, point anticipe en "anticip").
    if (predict_mode_ == "off") {
      m.pred_phase = "off";
      return;
    }
    // BORNE DE ROUE LIBRE, et c'est la protection essentielle du mode : sans elle le
    // filtre extrapolerait indefiniment une vitesse mesuree avant la perte, et la
    // camera partirait en butee en suivant un fantome.
    const bool within = (now_s - last_seen_s_) <= coast_max_s_;
    // SEUIL DE VITESSE, nom et valeur du prototype. Un visage immobile a l instant de la
    // perte n a pas d elan a extrapoler : le coaster ferait deriver la cible sur du bruit
    // de filtre. Sous le seuil, on rentre tout de suite.
    const bool moving = kal_.speed() >= predict_min_speed_;
    if (kal_.initialised() && moving && within) {
      double pnx = 0.0;
      double pny = 0.0;
      kal_.coast(dtSince(now_s), pnx, pny);
      m.pred_phase = "coast";
      m.pred_nx = static_cast<float>(pnx);
      m.pred_ny = static_cast<float>(pny);
      return;
    }
    // Plus de cible : on le dit, et servocam_node en deduit le retour au repos. Le filtre
    // est remis a zero pour que la prochaine acquisition reparte d une page blanche.
    kal_.resetUninit();
    m.pred_phase = "home";
  }

  double dtSince(double now_s)
  {
    const double dt = (last_step_s_ > 0.0) ? (now_s - last_step_s_) : 0.0;
    last_step_s_ = now_s;
    return dt;   // le bornage [1e-3, 0.2] est fait par KalmanPredictor::setDt
  }

  // --- commandes de mode (manette, MCP) ----------------------------------------------
  void onMode(const bamboo_interfaces::msg::ModeCmd::ConstSharedPtr & msg)
  {
    // GARDE ANTI-REJEU sur seq, semantique EXACTE du prototype : sa case de profondeur 1
    // retient la derniere commande, donc un consommateur qui ne filtrerait pas la
    // rejouerait a chaque cycle. Ici le topic est `transient_local`, ce qui a le meme
    // effet a la reconnexion : un noeud qui redemarre recoit la derniere commande et la
    // rejouerait sans cette garde.
    if (msg->seq != 0 && msg->seq == last_mode_seq_) {return;}
    last_mode_seq_ = msg->seq;

    // DEUX NOMS POUR CHAQUE CIBLE, et ce n'est pas de la complaisance : le CONTRAT est
    // `bamboo_interfaces/msg/ModeCmd.msg`, qui documente `predict` et `tracker` -- ce que
    // `overlay_node` (:204,:206) consomme deja pour ses pastilles. Ce noeud, lui, n'ecoutait
    // historiquement que `predict_mode` / `track_mode`. Une commande CONFORME au contrat
    // allumait donc la pastille du HUD sans rien changer au comportement du suivi -- defaut
    // silencieux, et le pire des deux mondes. On accepte les deux, les noms du contrat
    // d'abord ; les anciens restent valides pour ne casser aucun script existant.
    std::lock_guard<std::mutex> lk(state_mu_);
    if (msg->target == "predict" || msg->target == "predict_mode") {
      if (msg->value != "off" && msg->value != "anticip" && msg->value != "coast") {
        RCLCPP_WARN(get_logger(), "predict_mode inconnu : '%s'", msg->value.c_str());
        return;
      }
      predict_mode_ = msg->value;
      kal_.resetUninit();   // un changement de mode invalide l'etat accumule
      RCLCPP_INFO(get_logger(), "predict_mode -> %s", predict_mode_.c_str());
      return;
    }
    if (msg->target == "detector" || msg->target == "tracker" || msg->target == "track_mode") {
      DetectConfig cfg = det_.config();
      if (msg->target == "detector") {cfg.detector = msg->value;} else {
        cfg.track_mode = msg->value;
      }
      std::string err;
      if (!det_.reconfigure(cfg, err)) {
        // La configuration PRECEDENTE reste en vigueur (reconfigure() la restaure) : le
        // suivi continue avec l'ancien detecteur au lieu de s'arreter sur une faute de
        // frappe venue de la manette.
        RCLCPP_ERROR(get_logger(), "%s refuse : %s", msg->target.c_str(), err.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "%s -> %s", msg->target.c_str(), msg->value.c_str());
      return;
    }
    // Les autres cibles (tracking, recognition, acquisition, train) appartiennent a
    // d'autres noeuds du groupe : on ne les traite pas et on ne s'en plaint pas.
  }

  // --- reconfiguration a chaud par parametres ----------------------------------------
  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter> & ps)
  {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    // ⚠️ EN HUMBLE CE RAPPEL EST APPELE *AVANT* APPLICATION (add_pre/post_set_parameters
    // n'existent qu'a partir d'Iron) : l'effet de bord doit donc se faire ICI, et la
    // valeur lue dans `ps` est la seule qui vaille -- get_parameter() rendrait l'ANCIENNE.
    std::lock_guard<std::mutex> lk(state_mu_);
    DetectConfig cfg = det_.config();
    bool touch_det = false;

    for (const auto & p : ps) {
      const std::string & n = p.get_name();
      if (n == "predict_mode") {
        const std::string v = p.as_string();
        if (v != "off" && v != "anticip" && v != "coast") {
          res.successful = false;
          res.reason = "predict_mode doit valoir off, anticip ou coast";
          return res;
        }
        predict_mode_ = v;
        kal_.resetUninit();
      } else if (n == "predict_horizon_s") {
        if (p.as_double() < 0.0) {
          res.successful = false;
          res.reason = "predict_horizon_s doit etre >= 0";
          return res;
        }
        predict_horizon_s_ = p.as_double();
      } else if (n == "coast_max_s") {
        coast_max_s_ = p.as_double();
      } else if (n == "predict_min_speed") {
        if (p.as_double() < 0.0) {
          res.successful = false;
          res.reason = "predict_min_speed doit etre >= 0";
          return res;
        }
        predict_min_speed_ = p.as_double();
      } else if (n == "detector") {
        cfg.detector = p.as_string(); touch_det = true;
      } else if (n == "track_mode") {
        cfg.track_mode = p.as_string(); touch_det = true;
      } else if (n == "det_width") {
        cfg.det_width = static_cast<int>(p.as_int()); touch_det = true;
      } else if (n == "det_conf") {
        cfg.det_conf = p.as_double(); touch_det = true;
      } else if (n == "redetect_ms") {
        cfg.redetect_ms = p.as_double(); touch_det = true;
      } else if (n == "iou_reanchor") {
        cfg.iou_reanchor = p.as_double(); touch_det = true;
      } else if (n == "score_min") {
        cfg.score_min = p.as_double(); touch_det = true;
      } else if (n == "hold_ms") {
        cfg.hold_ms = p.as_double(); touch_det = true;
      } else if (n == "hold_score_min") {
        cfg.hold_score_min = p.as_double(); touch_det = true;
      } else if (n == "max_det_misses") {
        cfg.max_det_misses = static_cast<int>(p.as_int()); touch_det = true;
      } else if (n == "max_grow") {
        cfg.max_grow = p.as_double(); touch_det = true;
      } else if (n == "max_area_frac") {
        cfg.max_area_frac = p.as_double(); touch_det = true;
      } else if (n == "flip") {
        // REJET NOMME plutot qu'un repli muet : un miroir inconnu accepte en silence
        // mettrait le pan a l'envers, et la camera fuirait le sujet sans un mot.
        const std::string v = p.as_string();
        if (v != "none" && v != "v" && v != "h" && v != "180") {
          res.successful = false;
          res.reason = "flip doit valoir none, v, h ou 180";
          return res;
        }
        flip_ = v;
      }
      // models_dir est volontairement absent : recharger les modeles sous une autre
      // racine en marche est une operation de deploiement, pas un reglage.
    }

    if (touch_det) {
      if (cfg.det_width < 64) {
        res.successful = false;
        res.reason = "det_width < 64 : le detecteur ne verrait plus un visage de pres";
        return res;
      }
      std::string err;
      if (!det_.reconfigure(cfg, err)) {
        // REJET NOMME, pas un echec silencieux : c'est ce message que MCP affiche.
        res.successful = false;
        res.reason = err;
        return res;
      }
    }
    return res;
  }

  // --- etat -------------------------------------------------------------------------
  std::string models_dir_;
  std::string flip_{"h"};
  std::string predict_mode_;
  double predict_horizon_s_{0.12};
  double coast_max_s_{0.7};
  double predict_min_speed_{0.4};

  FaceDetection det_;
  KalmanPredictor kal_;

  // Horloge STEADY, pas l'horloge du noeud : la machine a etats raisonne sur des DUREES
  // (hold_ms, lock_age, dt) et un saut de /clock ou de NTP ferait expirer tous les
  // verrous d'un coup.
  rclcpp::Clock now_steady_{RCL_STEADY_TIME};
  double last_step_s_{0.0};
  double last_seen_s_{0.0};
  uint64_t seq_{0};
  uint32_t last_mode_seq_{0};
  unsigned decode_fail_{0};

  // VERROU DE L'ETAT DE SUIVI : `det_` (detecteur + tracker OpenCV) et `kal_` sont
  // touches par TROIS rappels -- onImage dans son groupe dedie, onMode et onParams dans le
  // groupe par defaut. Tant que tout vivait dans UN groupe mutuellement exclusif, la
  // serialisation etait ACCIDENTELLE et ce verrou inutile ; separer les groupes a rendu la
  // concurrence reelle, et l'essai T22 de bambooSTM32YB l'a payee d'un SIGSEGV immediat
  // (reconfigure() pendant un trackStep()). Le verrou fait donc partie du meme correctif,
  // pas d'une precaution ajoutee apres coup.
  std::mutex state_mu_;
  rclcpp::CallbackGroup::SharedPtr img_cbg_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Subscription<bamboo_interfaces::msg::ModeCmd>::SharedPtr mode_sub_;
  rclcpp::Publisher<bamboo_interfaces::msg::TrackState>::SharedPtr pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_;
};

}  // namespace bamboo_videotracking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bamboo_videotracking::TrackingNode)
