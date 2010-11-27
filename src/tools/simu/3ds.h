#ifndef _3DS_H_
#define _3DS_H_

//! @file 3ds.h
//! @brief Contient les classes de chargement de fichiers *.3ds
//! @author Jean-baptiste Trédez

#include <cstdio>
#include <fcntl.h>
#include "modele3D.h"

//! @struct bloc3DS
//! @brief description d'un bloc d'un fichier 3DS
struct bloc3DS{
	unsigned short int id; //!< id du bloc
	unsigned int taille; //!< taille totale du bloc (entête incluse)
	unsigned int octetsLus; //!< nombre d'octets déjà lus
};

//! @class modele3DS
//! @brief Representation d'un objet 3DS
//! @todo TODO constructeur avec chargement + tester double chargement (voir si on a bien fusion)
class modele3DS : public modele3D
{
public:
    modele3DS(); //!< Constructeur
	~modele3DS(); //!< Destructeur
    bool chargement(const char *nomFichier); //!< Chargement d'un fichier

private:
	void lireBlocMain3DS(bloc3DS* bloc); //!< lecture du bloc principal d'un fichier 3ds
	void lireBlocEdit3DS(bloc3DS* bloc); //!< lecture d'un bloc de type edit d'un fichier 3ds
	void lireBlocEditObjet3DS(bloc3DS* bloc); //!< lecture d'un bloc de type editObject d'un fichier 3ds
	void lireBlocObjetTrimesh(objet3D* objet, bloc3DS* bloc); //!< lecture d'un bloc de type object trimesh d'un fichier 3ds
	int lireNom(char* buffer); //!< lecture d'une chaine de caractères dans un fichier 3ds
	void lireEnteteBloc(bloc3DS* bloc); //!< lecture de l'entête d'un bloc
	void lireIndicesVecteurs(objet3D* objet, bloc3DS* bloc); //!< lecture d'un bloc d'indice de vecteurs d'un fichier 3ds
	void lireVecteurs(objet3D* objet, bloc3DS* bloc); //!< lecture d'un bloc de vecteurs d'un fichier 3ds
	void lireBlocMateriaux(bloc3DS* bloc); //!< lecture d'un bloc materiaux d'un fichier 3ds
	void lireBlocCouleur(unsigned char* couleur,bloc3DS* bloc); //!< lecture d'un bloc couleur d'un fichier 3ds
	void lireBlocTexture(materiau* mat,bloc3DS* bloc); //!< lecture d'un bloc texture d'un fichier 3ds
	void lireMatObj(objet3D *objet, bloc3DS* bloc); //!< lecture de l'id du materiau de l'objet dans le bloc
	void calculerNormales(); //!< calcule les normales de tout les vecteurs
	FILE* fichier; //!< fichier en cours de lecture
};

#endif
