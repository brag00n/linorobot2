// gallery.hpp - galerie d'empreintes SFace, lue DIRECTEMENT dans les .npz du trainer.
//
// POURQUOI UN LECTEUR NPZ EN C++, et pas un format d'echange dedie : le producteur de la
// galerie est face_train_node.py (numpy, np.savez), et il reste en Python parce que
// l'enrolement n'est pas temps reel. Faire ecrire au trainer un second fichier "pour le
// C++" creerait DEUX verites pour la meme galerie, qui divergeraient au premier plantage
// entre les deux ecritures. On lit donc le .npz tel qu'il est.
//
// Le format est atteignable sans dependance : un .npz est un ZIP dont les membres sont des
// .npy, et np.savez ecrit SANS COMPRESSION (methode STORED). Un .npz compresse
// (np.savez_compressed) est donc REFUSE AVEC UN MESSAGE EXPLICITE plutot que lu de
// travers -- il faudrait zlib, que l'image n'installe pas pour ca.
//
// Ce que ce fichier n'est PAS : un moteur de reconnaissance. Il ne calcule aucune
// empreinte (c'est cv::FaceRecognizerSF qui le fait) ; il stocke des references et rend
// la plus proche, sans seuil, comme FaceRecognizer.nearest() du prototype -- l'hysteresis
// appartient a l'appelant.
#ifndef BAMBOO_VIDEOTRACKING__GALLERY_HPP_
#define BAMBOO_VIDEOTRACKING__GALLERY_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace bamboo_videotracking
{

// Une personne de la galerie : plusieurs empreintes de reference (poses variees).
struct GalleryPerson
{
  int id_pred{-1};
  std::string name{"unknown"};
  // N x D, D valant 128 pour SFace. Stockees ligne par ligne : la comparaison est un
  // produit scalaire par ligne, jamais une operation matricielle -> rien a gagner a
  // aplatir, et un vecteur de vecteurs reste lisible au debogage.
  std::vector<std::vector<float>> embeddings;
};

class Gallery
{
public:
  // (Re)charge <faces_dir>/identified/<id>-<nom>/gallery.npz, exactement l'arborescence
  // que le trainer produit. Une entree illisible est IGNOREE avec un motif accumule dans
  // `err`, sans faire echouer les autres : une seule personne corrompue ne doit pas
  // aveugler la reconnaissance de tout le monde.
  bool reload(const std::string & faces_dir, std::string & err);

  // Identite la PLUS PROCHE, sans seuil (cf. FaceRecognizer.nearest). Rend false si la
  // galerie est vide ou la dimension ne correspond pas ; dans ce cas cos vaut -1.
  bool nearest(const float * emb, std::size_t dim, int & id_pred, std::string & name,
    double & cos) const;

  std::size_t persons() const {return people_.size();}
  std::size_t embeddings() const;
  const std::vector<GalleryPerson> & people() const {return people_;}

private:
  std::vector<GalleryPerson> people_;
};

// --- briques exposees, parce qu'elles sont la partie fragile et donc la partie a tester --
// `<id_pred>-<nom>` -> (id, nom). Tolerant comme parseIdentifiedName du prototype : un
// dossier sans prefixe numerique est refuse (false), pas silencieusement numerote.
bool parseIdentifiedName(const std::string & dirname, int & id_pred, std::string & name);

// Extrait le tableau `embeddings` (N x D, float32) d'un .npz non compresse.
bool loadNpzEmbeddings(
  const std::string & path, std::vector<std::vector<float>> & out, std::string & err);

// Cosinus de deux empreintes. Rend -1.0 si l'une des normes est nulle, comme le
// prototype : une empreinte nulle n'est PAS a distance maximale, elle est invalide.
double cosineSim(const float * a, const float * b, std::size_t n);

}  // namespace bamboo_videotracking

#endif  // BAMBOO_VIDEOTRACKING__GALLERY_HPP_
