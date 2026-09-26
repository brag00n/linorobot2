// gallery.cpp - lecture du .npz du trainer et recherche de l'identite la plus proche.
#include "bamboo_videotracking/gallery.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

#include <dirent.h>
#include <sys/stat.h>

namespace bamboo_videotracking
{
namespace
{

// --- lecture d'entiers petit-boutiens ----------------------------------------------
// ZIP et NPY sont TOUS DEUX petit-boutiens par specification, independamment de la
// machine : on lit donc octet par octet au lieu de memcpy-er un uint32, ce qui serait
// faux sur un hote gros-boutien. Le RPi4 est petit-boutien, mais un bug qui ne se voit
// que sur une autre machine est un bug qu'on n'a pas trouve, pas un bug absent.
uint16_t rd16(const unsigned char * p) {return static_cast<uint16_t>(p[0] | (p[1] << 8));}

uint32_t rd32(const unsigned char * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

bool readFile(const std::string & path, std::vector<unsigned char> & buf, std::string & err)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {err = "ouverture impossible : " + path; return false;}
  f.seekg(0, std::ios::end);
  const std::streamoff sz = f.tellg();
  if (sz <= 0) {err = "fichier vide : " + path; return false;}
  f.seekg(0, std::ios::beg);
  buf.resize(static_cast<size_t>(sz));
  f.read(reinterpret_cast<char *>(buf.data()), sz);
  if (!f) {err = "lecture incomplete : " + path; return false;}
  return true;
}

// --- ZIP : localiser un membre par son nom ----------------------------------------
// On passe par le REPERTOIRE CENTRAL et non par les en-tetes locaux : zipfile peut
// ecrire les tailles dans un "data descriptor" APRES les donnees, laissant l'en-tete
// local a zero. Le repertoire central, lui, est toujours renseigne.
bool findZipMember(
  const std::vector<unsigned char> & z, const std::string & want,
  size_t & data_off, size_t & data_len, std::string & err)
{
  if (z.size() < 22) {err = "trop court pour un zip"; return false;}
  // EOCD (PK\5\6) cherche depuis la fin : son commentaire final fait au plus 64 Kio.
  const size_t scan = std::min<size_t>(z.size(), 65557);
  size_t eocd = 0;
  bool found = false;
  for (size_t i = 0; i + 22 <= scan; ++i) {
    const size_t p = z.size() - 22 - i;
    if (z[p] == 'P' && z[p + 1] == 'K' && z[p + 2] == 5 && z[p + 3] == 6) {
      eocd = p; found = true; break;
    }
  }
  if (!found) {err = "fin de repertoire central (EOCD) introuvable"; return false;}

  const uint16_t nent = rd16(&z[eocd + 10]);
  size_t cd = rd32(&z[eocd + 16]);
  for (uint16_t i = 0; i < nent; ++i) {
    if (cd + 46 > z.size() || std::memcmp(&z[cd], "PK\x01\x02", 4) != 0) {
      err = "entree de repertoire central invalide"; return false;
    }
    const uint16_t method = rd16(&z[cd + 10]);
    const uint32_t usize = rd32(&z[cd + 24]);
    const uint16_t nlen = rd16(&z[cd + 28]);
    const uint16_t elen = rd16(&z[cd + 30]);
    const uint16_t clen = rd16(&z[cd + 32]);
    const size_t lho = rd32(&z[cd + 42]);
    const std::string name(reinterpret_cast<const char *>(&z[cd + 46]), nlen);
    if (name == want) {
      if (method != 0) {
        // REFUS EXPLICITE : decompresser demanderait zlib. np.savez ecrit en STORED ;
        // seul np.savez_compressed produit ce cas, et le trainer ne l'utilise pas.
        err = "membre '" + want + "' compresse (methode " + std::to_string(method) +
          ") : regenerer la galerie avec np.savez, pas np.savez_compressed";
        return false;
      }
      if (lho + 30 > z.size() || std::memcmp(&z[lho], "PK\x03\x04", 4) != 0) {
        err = "en-tete local invalide"; return false;
      }
      // Les longueurs de nom/extra de l'en-tete LOCAL peuvent differer de celles du
      // repertoire central (champs extra distincts) : c'est celles-la qui donnent le
      // debut des donnees.
      const uint16_t lnlen = rd16(&z[lho + 26]);
      const uint16_t lelen = rd16(&z[lho + 28]);
      data_off = lho + 30 + lnlen + lelen;
      data_len = usize;
      if (data_off + data_len > z.size()) {err = "donnees hors fichier"; return false;}
      return true;
    }
    cd += 46 + nlen + elen + clen;
  }
  err = "membre '" + want + "' absent du npz";
  return false;
}

// --- NPY : en-tete + donnees --------------------------------------------------------
// L'en-tete est un litteral de dict Python, p.ex.
//   {'descr': '<f4', 'fortran_order': False, 'shape': (7, 128), }
// On le lit par recherche de sous-chaines et non par un vrai analyseur : le producteur
// est numpy, son ecriture est canonique, et un analyseur complet ici serait du code non
// teste pour un format qu'on ne verra jamais varier.
bool parseNpyF32(
  const unsigned char * p, size_t len, std::vector<std::vector<float>> & out,
  std::string & err)
{
  static const char kMagic[] = "\x93NUMPY";
  if (len < 12 || std::memcmp(p, kMagic, 6) != 0) {err = "signature npy absente"; return false;}
  const unsigned major = p[6];
  size_t hlen = 0, hoff = 0;
  if (major == 1) {
    hlen = rd16(p + 8); hoff = 10;
  } else if (major == 2 || major == 3) {
    hlen = rd32(p + 8); hoff = 12;
  } else {
    err = "version npy " + std::to_string(major) + " inconnue"; return false;
  }
  if (hoff + hlen > len) {err = "en-tete npy tronque"; return false;}
  const std::string hdr(reinterpret_cast<const char *>(p + hoff), hlen);

  if (hdr.find("'<f4'") == std::string::npos && hdr.find("\"<f4\"") == std::string::npos) {
    // Refus nomme : une galerie en float64 se lirait en produisant du bruit, ce qui
    // ressemblerait a "la reconnaissance marche mal" et non a "le fichier est mauvais".
    err = "dtype attendu '<f4' (float32), en-tete : " + hdr;
    return false;
  }
  if (hdr.find("'fortran_order': False") == std::string::npos) {
    err = "fortran_order True non gere (galerie ecrite en ordre colonne)"; return false;
  }
  const size_t sp = hdr.find("'shape':");
  const size_t op = (sp == std::string::npos) ? std::string::npos : hdr.find('(', sp);
  const size_t cp = (op == std::string::npos) ? std::string::npos : hdr.find(')', op);
  if (cp == std::string::npos) {err = "champ shape illisible : " + hdr; return false;}
  std::string dims = hdr.substr(op + 1, cp - op - 1);
  std::replace(dims.begin(), dims.end(), ',', ' ');
  std::istringstream is(dims);
  std::vector<size_t> shape;
  size_t v = 0;
  while (is >> v) {shape.push_back(v);}

  size_t rows = 0, cols = 0;
  if (shape.size() == 2) {
    rows = shape[0]; cols = shape[1];
  } else if (shape.size() == 1) {
    // Une seule empreinte enregistree en 1D : saveGallery du prototype la remet en 2D,
    // mais une galerie ecrite a la main peut arriver ainsi. On l'accepte comme 1 x D.
    rows = 1; cols = shape[0];
  } else {
    err = "forme de rang " + std::to_string(shape.size()) + " inattendue"; return false;
  }
  if (rows == 0 || cols == 0) {err = "galerie vide (forme nulle)"; return false;}

  const size_t need = rows * cols * sizeof(float);
  if (hoff + hlen + need > len) {err = "donnees npy tronquees"; return false;}
  const unsigned char * d = p + hoff + hlen;

  out.assign(rows, std::vector<float>(cols, 0.0f));
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      // Recomposition octet par octet puis reinterpretation : meme raison qu'en haut,
      // le format est petit-boutien par specification.
      const uint32_t bits = rd32(d + (r * cols + c) * 4);
      float f = 0.0f;
      std::memcpy(&f, &bits, sizeof(f));
      out[r][c] = f;
    }
  }
  return true;
}

bool isDir(const std::string & p)
{
  struct stat st {};
  return stat(p.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool isFile(const std::string & p)
{
  struct stat st {};
  return stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

}  // namespace

// --- API publique -------------------------------------------------------------------
double cosineSim(const float * a, const float * b, std::size_t n)
{
  double dot = 0.0, na = 0.0, nb = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    dot += static_cast<double>(a[i]) * static_cast<double>(b[i]);
    na += static_cast<double>(a[i]) * static_cast<double>(a[i]);
    nb += static_cast<double>(b[i]) * static_cast<double>(b[i]);
  }
  if (na == 0.0 || nb == 0.0) {return -1.0;}   // empreinte invalide, pas "tres loin"
  return dot / (std::sqrt(na) * std::sqrt(nb));
}

bool parseIdentifiedName(const std::string & dirname, int & id_pred, std::string & name)
{
  // On accepte les deux separateurs de chemin : le trainer tourne sur le RPi, mais le
  // banc V6.3 rejoue des dossiers copies depuis Windows.
  size_t slash = dirname.find_last_of("/\\");
  std::string base = (slash == std::string::npos) ? dirname : dirname.substr(slash + 1);
  const size_t dash = base.find('-');
  if (dash == std::string::npos || dash == 0) {return false;}
  const std::string head = base.substr(0, dash);
  for (char c : head) {
    if (c < '0' || c > '9') {return false;}
  }
  id_pred = std::atoi(head.c_str());
  name = base.substr(dash + 1);
  if (name.empty()) {name = "unknown";}
  return true;
}

bool loadNpzEmbeddings(
  const std::string & path, std::vector<std::vector<float>> & out, std::string & err)
{
  std::vector<unsigned char> z;
  if (!readFile(path, z, err)) {return false;}
  size_t off = 0, len = 0;
  // np.savez(path, embeddings=...) nomme le membre "embeddings.npy".
  if (!findZipMember(z, "embeddings.npy", off, len, err)) {return false;}
  return parseNpyF32(&z[off], len, out, err);
}

bool Gallery::reload(const std::string & faces_dir, std::string & err)
{
  people_.clear();
  err.clear();
  if (faces_dir.empty()) {err = "faces_dir vide"; return false;}
  const std::string root = faces_dir + "/identified";
  if (!isDir(root)) {
    // PAS une erreur : un robot neuf n'a encore identifie personne. La reconnaissance
    // doit demarrer et dire "unknown", pas refuser de demarrer.
    err = "aucun dossier identified/ sous " + faces_dir + " (galerie vide)";
    return true;
  }
  DIR * d = opendir(root.c_str());
  if (d == nullptr) {err = "lecture impossible de " + root; return false;}

  std::vector<std::string> names;
  for (struct dirent * e = readdir(d); e != nullptr; e = readdir(d)) {
    const std::string n = e->d_name;
    if (n == "." || n == "..") {continue;}
    names.push_back(n);
  }
  closedir(d);
  std::sort(names.begin(), names.end());   // ordre stable = journaux comparables

  std::ostringstream skipped;
  for (const std::string & n : names) {
    const std::string dir = root + "/" + n;
    const std::string npz = dir + "/gallery.npz";
    if (!isDir(dir) || !isFile(npz)) {continue;}
    int id = -1;
    std::string person;
    if (!parseIdentifiedName(n, id, person)) {
      skipped << " [" << n << ": nom sans prefixe <id>-]";
      continue;
    }
    GalleryPerson p;
    p.id_pred = id;
    p.name = person;
    std::string e2;
    if (!loadNpzEmbeddings(npz, p.embeddings, e2)) {
      // UNE personne illisible n'aveugle pas les autres : on note et on continue.
      skipped << " [" << n << ": " << e2 << "]";
      continue;
    }
    people_.push_back(std::move(p));
  }
  err = skipped.str();
  return true;
}

std::size_t Gallery::embeddings() const
{
  std::size_t n = 0;
  for (const GalleryPerson & p : people_) {n += p.embeddings.size();}
  return n;
}

bool Gallery::nearest(
  const float * emb, std::size_t dim, int & id_pred, std::string & name, double & cos) const
{
  id_pred = -1;
  name = "unknown";
  cos = -1.0;
  bool any = false;
  for (const GalleryPerson & p : people_) {
    for (const std::vector<float> & ref : p.embeddings) {
      // DIMENSION VERIFIEE ligne par ligne : une galerie enrolee avec un autre modele
      // SFace se lirait sinon en comparant 128 valeurs contre 512, ce qui donnerait un
      // cosinus plausible mais denue de sens.
      if (ref.size() != dim) {continue;}
      const double c = cosineSim(emb, ref.data(), dim);
      any = true;
      // MEILLEUR cosinus par personne, puis meilleur entre personnes : c'est la
      // semantique de FaceRecognizer.nearest(), robuste aux variations de pose du
      // sous-lot d'enrolement.
      if (c > cos) {cos = c; id_pred = p.id_pred; name = p.name;}
    }
  }
  return any;
}

}  // namespace bamboo_videotracking
