#include <cmath>
#include "3ds.h"
#include "log.h"

//! @file 3ds.cxx
//! @brief Contient la classe modele3DS
//! @author TREDEZ Jean-Baptiste

// bloc principal
#define MAIN3DS				0x4D4D	//!< id du bloc principal

// blocs pouvant être dans le bloc principal
#define EDIT3DS				0x3D3D	//!< id du bloc éditeur 3D
#define KEYF3DS				0xB000	//!< id du bloc de description des animations
#define VERSION3DS			0X0002	//!< id du bloc version du fichier 3ds

// blocs pouvant être dans le bloc edit3DS
#define EDIT_MATERIAL		0xAFFF	//!< id du bloc des matériaux
#define EDIT_CONFIG1		0x0100	//!< id du bloc de config 1
#define EDIT_CONFIG2		0x3E3D	//!< id du bloc de config 2
#define EDIT_MESH_VERSION	0x3D3E	//!< id du bloc de version de vecteurs
#define EDIT_VIEW1			0x7001	//!< id du bloc de description de vues
#define EDIT_BACKGR			0x1200	//!< id du bloc couleur de fond
#define EDIT_AMBIENT		0x2100	//!< id du bloc couleur ambiante
#define EDIT_OBJECT			0x4000	//!< id du bloc objet

// blocs pouvant être dans le bloc edit_view1
#define EDIT_VIEW_P1		0x7012	//!< id du bloc de description de la vue 1
#define EDIT_VIEW_P2		0x7011	//!< id du bloc de description de la vue 2
#define EDIT_VIEW_P3		0x7020	//!< id du bloc de description de la vue 3

// blocs pouvant être dans le bloc edit_Object
#define OBJ_TRIMESH			0x4100	//!< id du bloc de triangles
#define OBJ_LIGHT			0x4600	//!< id du bloc de lumières
#define OBJ_CAMERA			0x4700	//!< id du bloc de position/orientation de la camera

// blocs pouvant être dans le bloc obj_trimesh
#define TRI_VERTEXL			0x4110	//!< id du bloc contenant les vecteurs
#define TRI_VERTEXOPTION	0x4111 	//!< id du bloc contenant les options des vecteurs
#define TRI_FACEL1			0x4120	//!< id du bloc de description des faces (id des vecteurs à utiliser pour former les triangles)
#define TRI_LOCAL			0x4160	//!< id du bloc de coordonnées locales
#define TRI_MAT_GROUP		0x4130	//!< id du bloc de la liste des matériaux à appliquer aux faces
#define TRI_TEX_VERTS		0x4140	//!< id du bloc des coordonnées des textures
/*
#define TRI_SMOOTH			0x4150
#define TRI_VISIBLE			0x4165
*/

// blocs pouvant être dans le bloc edit_material
#define	MAT_NAME			0xA000	//!< id du bloc du nom du matériau
#define MAT_AMBIENT			0xA010	//!< id du bloc de la couleur ambiante du matériau
#define MAT_DIFFUSE			0xA020	//!< id du bloc de la couleur diffuse du matériau
#define MAT_SPECULAR		0xA030	//!< id du bloc de la couleur spéculaire du matériau
#define MAT_TRANSP			0xA050	//!< id du bloc du pourcentage de transparence
#define MAT_TEXMAP			0xA200	//!< id du bloc de l'id de texture du matériau

// blocs pouvat être dans le bloc mat_texmap (textures)
#define MAT_MAPNAME			0xA300	//!< id du bloc contenant le nom du fichier à charger (contenant la texture)

// blocs de couleur
#define COLOR_24			0x0011	//!< id du bloc de couleur 24 bit

modele3DS::modele3DS()
{

}

modele3DS::~modele3DS()
{

}

//! @param bloc bloc à traiter
void modele3DS::lireBlocMain3DS(bloc3DS* bloc)
{
	bloc3DS sousBloc;
	u_int32_t version = -1;
	// on regarde les sous blocs et on les traitent en fonction de ce que c'est.
	// si on ne sait pas le lire (pas prévu), on saute le bloc.
	while(bloc->octetsLus < bloc->taille){
		lireEnteteBloc(&sousBloc);
		switch(sousBloc.id){
			case VERSION3DS:
				sousBloc.octetsLus += fread(&version,1,4,fichier);
				meslog(_3ds_,"version3DS: %#.4X",version);
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);	// s'il reste quelque chose, on passe
				break;
			case EDIT3DS:
				lireBlocEdit3DS(&sousBloc);
				break;
			case KEYF3DS:
				// je crois que c'est les animations, on saute.
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);
				meslog(_3ds_,"on a sauté le bloc: %#.4X de taille %u",sousBloc.id,sousBloc.taille);
				sousBloc.octetsLus = sousBloc.taille;
				break;
			default:
				// ce n'est pas pris en compte ou on ne sait pas ce que c'est, on saute le bloc.
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);
				meslog(_3ds_,"on a sauté le bloc: %#.4X de taille %u",sousBloc.id,sousBloc.taille);
				sousBloc.octetsLus = sousBloc.taille;
				break;
		}
		bloc->octetsLus += sousBloc.octetsLus;
	}
}

//! @param bloc bloc à traiter
void modele3DS::lireBlocEdit3DS(bloc3DS* bloc)
{
	bloc3DS sousBloc;
	u_int32_t version = -1;

	// on regarde les sous blocs et on les traitent en fonction de ce que c'est.
	// si on ne sait pas le lire (pas prévu), on saute le bloc.
	while(bloc->octetsLus < bloc->taille){
		lireEnteteBloc(&sousBloc);
		switch(sousBloc.id){
			case EDIT_MESH_VERSION:
				sousBloc.octetsLus += fread(&version,1,4,fichier);
				meslog(_3ds_,"version vecteurs: %#.4X",version);
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);	// s'il reste quelque chose, on passe
				break;
			case EDIT_OBJECT:
				lireBlocEditObjet3DS(&sousBloc);
				break;
			case EDIT_MATERIAL:
				lireBlocMateriaux(&sousBloc);
				break;
			default:
				// ce n'est pas pris en compte ou on ne sait pas ce que c'est, on saute le bloc.
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);
				meslog(_3ds_,"on a sauté le bloc: %#.4X de taille %u",sousBloc.id,sousBloc.taille);
				sousBloc.octetsLus = sousBloc.taille;
				break;
		}
		bloc->octetsLus += sousBloc.octetsLus;
	}
}

//! @param bloc bloc à traiter
void modele3DS::lireBlocEditObjet3DS(bloc3DS* bloc)
{
	bloc3DS sousBloc;
	objet3D objet;

	nbObj++;
	objets.push_back(objet);

	// le bloc commence par le nom de l'objet.
	bloc->octetsLus += lireNom(objets[nbObj - 1].nom);
	meslog(_3ds_,"lecture de l'objet %s",objets[nbObj - 1].nom);

	// on regarde les sous blocs et on les traitent en fonction de ce que c'est.
	// si on ne sait pas le lire (pas prévu), on saute le bloc.
	while(bloc->octetsLus < bloc->taille){
		lireEnteteBloc(&sousBloc);
		switch(sousBloc.id){
			case OBJ_TRIMESH:
				lireBlocObjetTrimesh(&(objets[nbObj - 1]),&sousBloc);
				break;
			default:
				// ce n'est pas pris en compte ou on ne sait pas ce que c'est, on saute le bloc.
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);
				meslog(_3ds_,"on a sauté le bloc: %#.4X de taille %u",sousBloc.id,sousBloc.taille);
				sousBloc.octetsLus = sousBloc.taille;
				break;
		}
		bloc->octetsLus += sousBloc.octetsLus;
	}
}

//! @param objet objet à remplir
//! @param bloc bloc à traiter
void modele3DS::lireBlocObjetTrimesh(objet3D* objet,bloc3DS* bloc)
{
	bloc3DS sousBloc;
	// on regarde les sous blocs et on les traitent en fonction de ce que c'est.
	// si on ne sait pas le lire (pas prévu), on saute le bloc.
	while(bloc->octetsLus < bloc->taille){
		lireEnteteBloc(&sousBloc);
		switch(sousBloc.id){
			case TRI_VERTEXL:
				lireVecteurs(objet,&sousBloc);
 				break;
			case TRI_FACEL1:
				lireIndicesVecteurs(objet,&sousBloc);
				break;
			case TRI_MAT_GROUP:
				lireMatObj(objet,&sousBloc);
				break;
			case TRI_TEX_VERTS:
				sousBloc.octetsLus += fread(&objet->nbCoordText,1,2,fichier);
				objet->coordText = new vecteur2f [objet->nbCoordText];
				sousBloc.octetsLus += fread(objet->coordText,1,sousBloc.taille - sousBloc.octetsLus, fichier);
				break;
			case TRI_LOCAL:
				// on charge la matrice de transformation 4x4 de l'objet.
				// La dernière colonne est (0,0,0,1), seul les 3 premières colonnes sont dans le fichier
				sousBloc.octetsLus += fread(objet->matrice,1,3*4,fichier);
				sousBloc.octetsLus += fread(objet->matrice+4,1,3*4,fichier);
				sousBloc.octetsLus += fread(objet->matrice+8,1,3*4,fichier);
				sousBloc.octetsLus += fread(objet->matrice+12,1,3*4,fichier);
				objet->matrice[3] = 0.0f;
				objet->matrice[7] = 0.0f;
				objet->matrice[11] = 0.0f;
				objet->matrice[15] = 1.0f;
				break;
			default:
				// ce n'est pas pris en compte ou on ne sait pas ce que c'est, on saute le bloc.
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);
				meslog(_3ds_,"on a sauté le bloc: %#.4X de taille %u",sousBloc.id,sousBloc.taille);
				sousBloc.octetsLus = sousBloc.taille;
				break;
		}
		bloc->octetsLus += sousBloc.octetsLus;
	}
}

//! @param bloc bloc à traiter
void modele3DS::lireBlocMateriaux(bloc3DS* bloc)
{
	bloc3DS sousBloc;
	materiau mat;

	nbMat++;
	memset(&(mat), 0x00, sizeof(materiau));
	materiaux.push_back(mat);
	mat.textureId = -1;

	// on regarde les sous blocs et on les traitent en fonction de ce que c'est.
	// si on ne sait pas le lire (pas prévu), on saute le bloc.
	while(bloc->octetsLus < bloc->taille){
		lireEnteteBloc(&sousBloc);
		switch(sousBloc.id){
			case MAT_NAME:
				// lecture du nom
            	sousBloc.octetsLus += fread(materiaux[nbMat - 1].nom,1,sousBloc.taille - sousBloc.octetsLus,fichier);
				break;
			case MAT_DIFFUSE:
				lireBlocCouleur(materiaux[nbMat - 1].couleurDiffuse,&sousBloc);
				break;
			case MAT_AMBIENT:
				lireBlocCouleur(materiaux[nbMat - 1].couleurAmbient,&sousBloc);
				break;
			case MAT_SPECULAR:
				lireBlocCouleur(materiaux[nbMat - 1].couleurSpecular,&sousBloc);
				break;
			case MAT_TEXMAP:
				lireBlocTexture(&(materiaux[nbMat - 1]),&sousBloc);
				break;
			case MAT_TRANSP:
				meslog(_erreur_,"transparence: a implementer");
				break;
			default:
				// ce n'est pas pris en compte ou on ne sait pas ce que c'est, on saute le bloc.
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);
				meslog(_3ds_,"on a sauté le bloc: %#.4X de taille %u",sousBloc.id,sousBloc.taille);
				sousBloc.octetsLus = sousBloc.taille;
				break;
		}
		bloc->octetsLus += sousBloc.octetsLus;
	}
}

//! @param mat matériau à remplir
//! @param bloc bloc à traiter
void modele3DS::lireBlocTexture(materiau* mat,bloc3DS* bloc)
{
	bloc3DS sousBloc;
	// on regarde les sous blocs et on les traitent en fonction de ce que c'est.
	// si on ne sait pas le lire (pas prévu), on saute le bloc.
	while(bloc->octetsLus < bloc->taille){
		lireEnteteBloc(&sousBloc);
		switch(sousBloc.id){
			case MAT_MAPNAME:
				sousBloc.octetsLus += fread(mat->nomFichierTexture,1,sousBloc.taille - sousBloc.octetsLus,fichier);
				break;
			default:
				// ce n'est pas pris en compte ou on ne sait pas ce que c'est, on saute le bloc.
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);
				meslog(_3ds_,"on a sauté le bloc: %#.4X de taille %u",sousBloc.id,sousBloc.taille);
				sousBloc.octetsLus = sousBloc.taille;
				break;
		}
		bloc->octetsLus += sousBloc.octetsLus;
	}
}

//! @param couleur couleur à remplir
//! @param bloc bloc à traiter
void modele3DS::lireBlocCouleur(unsigned char* couleur,bloc3DS* bloc)
{
	bloc3DS sousBloc;

	// on regarde les sous blocs et on les traitent en fonction de ce que c'est.
	// si on ne sait pas le lire (pas prévu), on saute le bloc.
	while(bloc->octetsLus < bloc->taille){
		lireEnteteBloc(&sousBloc);
		switch(sousBloc.id){
			case COLOR_24:
				// RGB 24 bit: 3 octets
				sousBloc.octetsLus += fread(couleur,1,3, fichier);
				break;
			default:
				// ce n'est pas pris en compte ou on ne sait pas ce que c'est, on saute le bloc.
				fseek(fichier,sousBloc.taille - sousBloc.octetsLus,SEEK_CUR);
				meslog(_3ds_,"on a sauté le bloc: %#.4X de taille %u",sousBloc.id,sousBloc.taille);
				sousBloc.octetsLus = sousBloc.taille;
				break;
		}
		bloc->octetsLus += sousBloc.octetsLus;
	}
}

//! @param objet objet à remplir
//! @param bloc bloc à traiter
void modele3DS::lireIndicesVecteurs(objet3D* objet, bloc3DS* bloc)
{
    u_int16_t ind[4];

	// les deux premiers octets: le nombre de faces
	bloc->octetsLus += fread(&objet->nbFaces, 1, 2, fichier);
	objet->faces = new tFace [objet->nbFaces];
//	memset(objet->faces,0x00,sizeof(tFace) * objet->nbFaces);

	// lecture des faces, il y a des paquets de 4 entiers de 2 octets, je ne sais pas trop à quoi sert le dernier donc on le prend pas...
	for(int i = 0; i < objet->nbFaces; i++){
		bloc->octetsLus += fread(ind,1,sizeof(ind),fichier);
		objet->faces[i].indicesVecteurs[0] = ind[0];
		objet->faces[i].indicesVecteurs[1] = ind[1];
		objet->faces[i].indicesVecteurs[2] = ind[2];
	}
}

//! @param objet objet à remplir
//! @param bloc bloc à traiter
void modele3DS::lireVecteurs(objet3D* objet, bloc3DS* bloc)
{
	// les deux premiers octets: le nombre de vecteurs
	bloc->octetsLus += fread(&(objet->nbVect), 1, 2, fichier);
	objet->vect = new vecteur3f [objet->nbVect];
	memset(objet->vect,0x00,sizeof(vecteur3f) * objet->nbVect);

	// lecture du reste du bloc, les vecteurs sont enregistrés par paquet de 3 float (de 4 octets).
	bloc->octetsLus += fread(objet->vect, 1, bloc->taille - bloc->octetsLus, fichier);
}

//! @param bloc: pointeur vers le bloc à remplir
//!
//! format:
//! 2 octets: id
//! 4 octets: taille totale du chunk (ces 6 octets compris)
void modele3DS::lireEnteteBloc(bloc3DS *bloc)
{
	bloc->octetsLus = fread(&bloc->id, 1, 2, fichier);
    bloc->octetsLus += fread(&bloc->taille, 1, 4, fichier);
	meslog(_3ds_,"id: %#.4X\ttaille: %u",bloc->id,bloc->taille);
}

//! @param buffer contient à la fin le nom que l'on a lu dans le fichier (mémoire allouée à l'avance d'au moins 255 octets !!!)
//! @return taille de la chaine de caractère
//!
//! on lit une chaine de caractère.
//! elles se terminent par 0x00
int modele3DS::lireNom(char* buffer)
{
	int ind = 0;
	size_t trash; // Pour eviter un warning
	do{
		trash = fread(buffer + ind, 1, 1, fichier);
	}while (*(buffer + ind++) != 0 && ind < 255);

	return ind;
}

//! @param nomFichier nom du fichier à lire
bool modele3DS::chargement(const char *nomFichier)
{
	bloc3DS bloc;
	fichier = fopen(nomFichier, "rb");

	if(!fichier){
		logerror("impossible de charger le fichier %s:fopen()",nomFichier);		return false;
	}

	// lecture de l'entête
	lireEnteteBloc(&bloc);

	if(bloc.id != MAIN3DS){	// ce n'est pas un fichier corrspondant au format 3ds.
		meslog(_erreur_,"l'entête de fichier %s ne correspond pas au format 3ds: %#.4X",nomFichier,bloc.id);
		return false;
	}

	lireBlocMain3DS(&bloc);

	calculerNormales();

	fclose(fichier);
    return true;
}

//! @param objet objet à remplir
//! @param bloc bloc à traiter
void modele3DS::lireMatObj(objet3D* objet, bloc3DS* bloc)
{
	char nomMat[255] = {0};

	bloc->octetsLus += lireNom(nomMat);

	// on recherche l'indice de l'endroit ou on a enregistré le matériau.
	// si c'est une texture, on l'indique dans le champ de l'objet
	objet->matId = -1;
	objet->texture = false;

	for(int i = 0; i < nbMat; i++){
		if(strcmp(nomMat, materiaux[i].nom) == 0){
			objet->matId = i;
			if(strlen(materiaux[i].nomFichierTexture) > 0) objet->texture = true;
			break;
		} 
	}
	// on passe le reste du bloc vu que c'est inutile pour nous
	fseek(fichier,bloc->taille - bloc->octetsLus,SEEK_CUR);
	bloc->octetsLus = bloc->taille;
}           

//!
//! on calcule les normales des sommets pour la lumière
void modele3DS::calculerNormales()
{
	vecteur3f normale;
	vecteur3f point[3];

	for(int i = 0; i < nbObj; i++){
		objet3D* obj = &(objets[i]);

		vecteur3f* normalesFace	= new vecteur3f[obj->nbFaces];	// normales non normées (pour la moyenne)
		obj->normales			= new vecteur3f[obj->nbVect];

		// calcul des normales de toutes les faces
		for(int j=0; j < obj->nbFaces; j++){
			point[0] = obj->vect[obj->faces[j].indicesVecteurs[0]];
			point[1] = obj->vect[obj->faces[j].indicesVecteurs[1]];
			point[2] = obj->vect[obj->faces[j].indicesVecteurs[2]];

			normalesFace[j] = prodVect(point[0] - point[2], point[2] - point[1]);	// on ne norme pas pour la moyenne ensuite
		}

		// on fait la moyenne des normales des faces k dont j est un des sommets
		for(int j = 0; j < obj->nbVect; j++){
			vecteur3f sommeNormales(0,0,0);
			int nbFacesAutour = 0;

			for(int k = 0; k < obj->nbFaces; k++){
				if (obj->faces[k].indicesVecteurs[0] == j || obj->faces[k].indicesVecteurs[1] == j || obj->faces[k].indicesVecteurs[2] == j){
					// le sommet j appartient à la face k, on ajoute la normale (non normée) de la face k
					sommeNormales += normalesFace[k];
					nbFacesAutour++;
				}
			}      
			// calcul de la moyenne
			obj->normales[j] = sommeNormales / nbFacesAutour;
			obj->normales[j].normer();
		}
		delete [] normalesFace;
	}
}

