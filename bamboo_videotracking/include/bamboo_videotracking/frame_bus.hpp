// frame_bus.hpp - partage de la trame decodee ENTRE composables du meme processus.
//
// POURQUOI CE FICHIER EXISTE, et ce n'est pas un detour esthetique : un topic ROS ne
// peut pas porter une cv::Mat. Le plan exige que facerecog_node lise LA MEME Mat que
// tracking_node, en const, sans recopie ni re-decodage -- et la verification #3 du
// plan demande que `ros2 topic list` ne montre AUCUN sensor_msgs/Image entre les trois
// noeuds. Passer par un topic d'image brute violerait les deux : 921 Ko par trame en
// rgb8, et la memoire partagee DDS est coupee par l'entrypoint (shm_off.xml), donc le
// saut couterait plein tarif meme entre deux noeuds du meme conteneur.
//
// La solution est un canal LATERAL, hors ROS : une case protegee par mutex dans le
// processus. C'est legitime ici et seulement ici parce que les quatre composables sont
// charges dans UN SEUL component_container_mt -- s'ils etaient eclates en processus,
// ce fichier serait un mensonge et il faudrait revenir au topic. D'ou l'invariant a ne
// pas perdre de vue : le FrameBus est une optimisation de COMPOSITION, pas une API.
//
// PAR CONSTRUCTION la trame publiee est immuable : on ne partage qu'un
// shared_ptr<const cv::Mat>. C'est la reponse directe au piege de portage signale dans
// le plan -- le prototype dessine son HUD dans la ndarray qu'il vient de publier
// (`Core` modifie en place la trame de /camera/image). Ici le compilateur l'interdit :
// overlay_node ne PEUT PAS dessiner sur la trame du bus, il doit faire sa copie.
//
// Le consommateur ne bloque jamais le producteur : il prend le dernier instantane
// disponible et repart. Une trame sautee vaut mieux qu'un etage qui retarde la
// detection -- c'est le meme choix que la case de profondeur 1 du prototype (roslite
// Port), pas une degradation.

#ifndef BAMBOO_VIDEOTRACKING__FRAME_BUS_HPP_
#define BAMBOO_VIDEOTRACKING__FRAME_BUS_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>

namespace bamboo_videotracking
{

/// Instantane immuable d'une trame decodee, tel qu'il circule entre les etages.
struct Frame
{
  /// Trame BGR decodee UNE SEULE FOIS par tracking_node. const : personne ne dessine
  /// dessus (cf. le piege de portage en tete de fichier).
  std::shared_ptr<const cv::Mat> mat;
  /// Numero de sequence monotone, source unique de l'etranglement des etages avals :
  /// facerecog_node ne recalcule que sur un seq NOUVEAU (semantique du prototype,
  /// FaceRecogNode.py -- `if res.seq == self._seq_done: return`).
  uint64_t seq{0};
  /// Horodatage de CAPTURE (celui du CompressedImage entrant), PAS l'instant de
  /// publication. C'est ce qui rend la latence bout en bout du lot V6 mesurable :
  /// `now - header.stamp` n'a de sens que si le stamp traverse la chaine intact.
  int64_t stamp_ns{0};

  bool valid() const { return mat != nullptr && !mat->empty(); }
};

/// Case de profondeur 1, protegee par mutex : le producteur ecrase, les consommateurs
/// lisent le dernier etat. Aucune file, donc aucune latence qui s'accumule en silence.
class FrameBus
{
public:
  /// Publie une trame. Appele par tracking_node juste apres l'imdecode, AVANT la
  /// detection : les etages avals peuvent ainsi travailler pendant que YuNet tourne.
  void publish(const Frame & f)
  {
    std::lock_guard<std::mutex> lk(mu_);
    latest_ = f;
  }

  /// Dernier instantane, ou une Frame invalide si rien n'a encore ete publie.
  Frame latest() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return latest_;
  }

  /// Instantane SEULEMENT s'il est plus recent que `since` ; sinon une Frame invalide.
  /// C'est la primitive d'etranglement des consommateurs : elle evite qu'ils
  /// retravaillent la meme trame quand ils tournent plus vite que la detection.
  Frame latestAfter(uint64_t since) const
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (latest_.seq <= since) {
      return Frame{};
    }
    return latest_;
  }

private:
  mutable std::mutex mu_;
  Frame latest_;
};

/// Bus unique du processus.
///
/// UNE VARIABLE DE PORTEE PROCESSUS, ASSUMEE, et voici pourquoi elle n'est pas
/// evitable proprement : les composables sont instancies par le container, qui ne
/// fournit aucun moyen de passer un objet partage a leurs constructeurs -- leur seul
/// argument est un rclcpp::NodeOptions. L'alternative serait un service ou un
/// parametre pour echanger un pointeur, c'est-a-dire la meme chose en moins lisible.
/// Le singleton est donc la consequence du modele de composition, pas un raccourci.
///
/// Corollaire a ne pas oublier : DEUX containers dans le meme processus partageraient
/// ce bus. On n'en lance qu'un (videotracking.launch.py), et si cela changeait il
/// faudrait clefer le bus par espace de noms.
inline FrameBus & frameBus()
{
  static FrameBus bus;
  return bus;
}

}  // namespace bamboo_videotracking

#endif  // BAMBOO_VIDEOTRACKING__FRAME_BUS_HPP_
