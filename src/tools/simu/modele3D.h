#ifndef _MODELE3D_H_
#define _MODELE3D_H_

//! @file modele3D.h
//! @brief Contient les classes modele3D, objet3D et les structures materiaux et tFace pour traiter les objets 3D
//! @author Jean-Baptiste Trédez 

#include "vecteur.h"
#include <vector>
#include <GL/gl.h>
#include <memory.h>

//! @struct tFace
//! @brief représente une face
struct tFace{
	int indicesVecteurs[3];			//!< indices des vecteurs utilisés pour le triangle
	int indicesCoordText[3];		//!< indices des coordonnées utilisées pour le triangle // à charger............... dans le 3ds
};

//! @struct materiau
//! @brief représente un matériau
struct materiau{
	char  nom[255];						//!< nom du matériau
	char  nomFichierTexture[255];		//!< nom du fichier à charger pour la texture
	unsigned char couleurDiffuse[3];	//!< couleur diffuse
	unsigned char couleurAmbient[3];	//!< couleur ambiante
	unsigned char couleurSpecular[3];	//!< couleur spéculaire
	unsigned int textureId;				//!< indice de la texture
};

//! @class objet3D
//! @brief représentation d'un objet 3D
class objet3D{
public:
	objet3D();	// Constructeur
	inline objet3D(const objet3D &obj);	//!< Constructeur de copie
	~objet3D();	//!< Destructeur
	void translation(float dx,float dy,float dz);	//!< translate l'objet de (dx,dy,dz)
	void rotation(float alpha,float x,float y,float z);	//!< rotation d'angle alpha autour du vecteur (x,y,z)
	char nom[255];				//!< nom de l'objet
	bool texture;				//!< indique si l'objet est texturé ou non
	int  nbVect;				//!< nombre de vecteurs de l'objet
	int  nbFaces;				//!< nombre de faces de l'objet
	int  nbCoordText;			//!< nombre de coordonnées de texture
	int  matId;					//!< indice du matériau utilisé
	vecteur3f* vect;			//!< vecteurs de l'objet
	vecteur3f* normales;		//!< normales (pour la lumière) de l'objet
	vecteur2f* coordText;		//!< coordonnées de la texture
	tFace* faces;				//!< faces de l'objet (triangles) 
	float matrice[16];			//!< matrice de translation et rotation de l'objet
};

//! @class modele3D
//! @brief représentation d'un modele 3D (contient plusieurs objets 3D)
class modele3D{
public:
	modele3D();								//!< Constructeur
	~modele3D();							//!< Destructeur @todo attention, destructeur non fini, il faut libérer la mémoire
	void afficher();						//!< affiche tout le modèle
	void afficher(int obj);					//!< affiche l'objet i du modèle
	void compiler();						//!< compile les listes d'affichage de chaque objet
	int trouveObjet(const char* nom);		//!< trouve l'id d'un objet en fonction de son nom. Retourne si on ne le trouve pas
	int nbObj;								//!< nombre d'objets du modele
	int nbMat;								//!< nombre de materiaux
	std::vector<materiau> materiaux;		//!< liste des materiaux
	std::vector<objet3D> objets;			//!< liste des objets
	int listeAffichage;						//!< début des indices des listes d'affichage opengl
};

//! @param obj objet à copier
inline objet3D::objet3D(const objet3D &obj)
{
	memcpy(this,&obj,sizeof(objet3D));
}

#endif

