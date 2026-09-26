// kalman_predictor.hpp - portage de tools/robot_controlv3/.../KalmanPredictor.py (92 l.).
//
// Modele a vitesse constante sur la position NORMALISEE du visage :
//   etat  = [nx, ny, vnx, vny]   mesure = [nx, ny]
// Travailler en normalise et non en pixels est deliberi et se paie en simplicite :
// l'etat reste comparable d'une resolution de detection a l'autre, et la loi de
// commande pan/tilt consomme exactement la meme grandeur -- aucune conversion entre
// le predicteur et le servo, donc aucun facteur d'echelle a se tromper.
//
// cv::KalmanFilter est la MEME classe que celle utilisee par le prototype : le Python
// n'en etait qu'un binding. Le portage est donc une transposition, pas une reecriture,
// et les constantes ci-dessous sont celles du prototype, a l'identique.

#ifndef BAMBOO_VIDEOTRACKING__KALMAN_PREDICTOR_HPP_
#define BAMBOO_VIDEOTRACKING__KALMAN_PREDICTOR_HPP_

#include <opencv2/video/tracking.hpp>

namespace bamboo_videotracking
{

class KalmanPredictor
{
public:
  KalmanPredictor();

  /// Remet le filtre a l'etat NON INITIALISE : la prochaine mesure reamorce l'etat au
  /// lieu d'etre fusionnee avec un historique perime. Appele a chaque nouvel episode de
  /// verrou -- sans cela, un visage qui apparait a l'autre bout de l'image serait tire
  /// vers la position de l'ancien pendant plusieurs trames.
  void resetUninit();

  bool initialised() const { return init_; }

  /// Fusionne une mesure. `dt` en secondes, BORNE a [1e-3, 0.2] : en dessous la matrice
  /// de transition degenere vers l'identite (division par un dt quasi nul dans le gain),
  /// au-dessus une trame perdue ferait extrapoler la vitesse sur un horizon absurde.
  /// Le bornage vient du prototype et il est la vraie protection contre les hoquets de
  /// cadence du RPi -- un `dt` de 3 s apres une pause GC enverrait la prediction hors
  /// image.
  /// Renvoie l'etat corrige (nx, ny).
  void update(double nx, double ny, double dt, double & out_nx, double & out_ny);

  /// Avance le modele SANS mesure (mode `coast` : la cible est perdue, on continue sur
  /// l'elan). Le statePost est recopie depuis le statePre pour que plusieurs coast
  /// consecutifs s'enchainent -- sans cette recopie, cv::KalmanFilter repartirait a
  /// chaque fois du dernier etat CORRIGE et la prediction n'avancerait pas.
  void coast(double dt, double & out_nx, double & out_ny);

  /// Extrapole `horizon_s` en avant SANS toucher a l'etat (mode `anticip` : on vise ou
  /// la cible sera, la mesure courante restant la verite). Lecture pure.
  void peek(double horizon_s, double & out_nx, double & out_ny) const;

  /// Norme de l'innovation du dernier update (|mesure - prediction|), en unites
  /// normalisees. Sert d'indicateur de confiance : une innovation qui explose signale
  /// un saut de cible, pas un mouvement.
  double lastErr() const { return last_err_; }

private:
  void setDt(double dt);

  mutable cv::KalmanFilter kf_;
  bool init_{false};
  double last_err_{0.0};
};

}  // namespace bamboo_videotracking

#endif  // BAMBOO_VIDEOTRACKING__KALMAN_PREDICTOR_HPP_
