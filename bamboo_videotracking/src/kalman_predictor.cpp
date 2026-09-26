// Implementation du predicteur de Kalman - transposition de KalmanPredictor.py.

#include "bamboo_videotracking/kalman_predictor.hpp"

#include <algorithm>
#include <cmath>

namespace bamboo_videotracking
{

namespace
{
// Constantes du prototype, reprises telles quelles : elles ont ete reglees a l'usage
// sur ce robot et les rejouer differemment invaliderait la comparaison du banc V6.3.
//   PROC  : bruit de processus -- petit, le visage bouge de facon lisse
//   MEAS  : bruit de mesure -- plus grand, la boite de detection tremble
//   POST0 : incertitude initiale, volontairement large pour que la premiere mesure
//           domine l'amorcage
constexpr double kProcNoise = 1e-2;
constexpr double kMeasNoise = 5e-2;
constexpr double kPost0 = 1.0;
constexpr double kDtMin = 1e-3;
constexpr double kDtMax = 0.2;
}  // namespace

KalmanPredictor::KalmanPredictor()
: kf_(4, 2, 0)
{
  // Mesure = les deux premieres composantes de l'etat (on observe la position, jamais
  // la vitesse : c'est precisement ce que le filtre doit estimer).
  kf_.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
  kf_.measurementMatrix.at<float>(0, 0) = 1.0f;
  kf_.measurementMatrix.at<float>(1, 1) = 1.0f;

  cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(kProcNoise));
  cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(kMeasNoise));
  cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(kPost0));

  // transitionMatrix est reconstruite a chaque pas par setDt() : le dt n'est PAS
  // constant ici (la cadence de detection varie avec la charge du RPi4), donc le figer
  // a la construction fausserait la vitesse estimee des que les fps bougent.
  cv::setIdentity(kf_.transitionMatrix);
  resetUninit();
}

void KalmanPredictor::resetUninit()
{
  init_ = false;
  last_err_ = 0.0;
  kf_.statePost = cv::Mat::zeros(4, 1, CV_32F);
  kf_.statePre = cv::Mat::zeros(4, 1, CV_32F);
  cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(kPost0));
}

void KalmanPredictor::setDt(double dt)
{
  const double d = std::clamp(dt, kDtMin, kDtMax);
  // [1 0 d 0 ; 0 1 0 d ; 0 0 1 0 ; 0 0 0 1] -- vitesse constante.
  cv::setIdentity(kf_.transitionMatrix);
  kf_.transitionMatrix.at<float>(0, 2) = static_cast<float>(d);
  kf_.transitionMatrix.at<float>(1, 3) = static_cast<float>(d);
}

void KalmanPredictor::update(double nx, double ny, double dt, double & out_nx, double & out_ny)
{
  if (!init_) {
    // AMORCAGE : on POSE l'etat sur la mesure, vitesse nulle, au lieu de le corriger.
    // Corriger depuis un etat nul tirerait la premiere estimation vers le centre de
    // l'image pendant plusieurs trames -- exactement le defaut que resetUninit() evite.
    kf_.statePost.at<float>(0) = static_cast<float>(nx);
    kf_.statePost.at<float>(1) = static_cast<float>(ny);
    kf_.statePost.at<float>(2) = 0.0f;
    kf_.statePost.at<float>(3) = 0.0f;
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(kPost0));
    init_ = true;
    last_err_ = 0.0;
    out_nx = nx;
    out_ny = ny;
    return;
  }

  setDt(dt);
  const cv::Mat pred = kf_.predict();
  // Innovation MESUREE AVANT correction : apres, l'ecart est absorbe et l'information
  // est perdue. C'est l'ordre du prototype (predict -> lastErr -> correct).
  const double ex = nx - pred.at<float>(0);
  const double ey = ny - pred.at<float>(1);
  last_err_ = std::sqrt(ex * ex + ey * ey);

  cv::Mat meas(2, 1, CV_32F);
  meas.at<float>(0) = static_cast<float>(nx);
  meas.at<float>(1) = static_cast<float>(ny);
  const cv::Mat st = kf_.correct(meas);
  out_nx = st.at<float>(0);
  out_ny = st.at<float>(1);
}

void KalmanPredictor::coast(double dt, double & out_nx, double & out_ny)
{
  if (!init_) {
    out_nx = 0.0;
    out_ny = 0.0;
    return;
  }
  setDt(dt);
  const cv::Mat st = kf_.predict();
  // SANS cette recopie, le prochain predict() repartirait du dernier etat CORRIGE :
  // deux coast consecutifs rendraient deux fois la meme position et la cible cesserait
  // d'avancer alors qu'on est precisement dans le mode qui doit la faire avancer.
  kf_.statePost = st.clone();
  out_nx = st.at<float>(0);
  out_ny = st.at<float>(1);
}

void KalmanPredictor::peek(double horizon_s, double & out_nx, double & out_ny) const
{
  if (!init_) {
    out_nx = 0.0;
    out_ny = 0.0;
    return;
  }
  // Extrapolation ANALYTIQUE, pas un predict() : appeler le filtre modifierait statePre
  // et errorCovPre, donc polluerait le prochain update. Le modele etant a vitesse
  // constante, x + v*h est exactement ce que predict() calculerait.
  const double h = std::clamp(horizon_s, 0.0, kDtMax);
  out_nx = kf_.statePost.at<float>(0) + kf_.statePost.at<float>(2) * h;
  out_ny = kf_.statePost.at<float>(1) + kf_.statePost.at<float>(3) * h;
}

}  // namespace bamboo_videotracking
