#include "modele3D.h"

//! @file modele3D.cxx
//! @brief Contient les classes modele3D, objet3D et les structures materiaux et tFace pour traiter les objets 3D
//! @author Jean-Baptiste Trédez 

modele3D::modele3D()
{
	nbObj = 0;
	nbMat = 0;
	listeAffichage = -1;
}

modele3D::~modele3D()
{
	if(listeAffichage >= 0)
		glDeleteLists(listeAffichage,nbObj);
}

void modele3D::afficher()
{
	glPushMatrix();
	glMultMatrixf(matrice);
	if(listeAffichage >= 0)
	{
		for(int i=0;i<nbObj;i++)
		{
			glPushMatrix();
			glMultMatrixf(objets[i].matrice);
			glCallList(listeAffichage+i);
			glPopMatrix();
		}
	}
	glPopMatrix();
}

void modele3D::afficher(int i)
{
	glPushMatrix();
	glMultMatrixf(matrice);
	if(listeAffichage >= 0)
	{
		if(i >= 0 && i < nbObj)
		{
			glPushMatrix();
			glMultMatrixf(objets[i].matrice);
			glCallList(listeAffichage+i);
			glPopMatrix();
		}
	}
	glPopMatrix();
}

void modele3D::compiler()
{
	// si ce n'est pas le premier appel, si le nombre d'objet change, il faut libérer puis allouer de nouveau la mémoire
	if(listeAffichage >= 0)
		glDeleteLists(listeAffichage, nbObj);

	listeAffichage = glGenLists(nbObj);

	for(int j=0;j<nbObj;j++){
		glNewList(listeAffichage+j,GL_COMPILE);
		glPushMatrix();
			if(objets[j].matId >= 0){
				materiau mat = materiaux[objets[j].matId];
				float diff[4] = {mat.couleurDiffuse[0]/255.0f,mat.couleurDiffuse[1]/255.0f,mat.couleurDiffuse[2]/255.0f,1};
				float amb[4] = {mat.couleurAmbient[0]/255.0f,mat.couleurAmbient[1]/255.0f,mat.couleurAmbient[2]/255.0f,1};
				float specular[4] = {mat.couleurSpecular[0]/255.0f,mat.couleurSpecular[1]/255.0f,mat.couleurSpecular[2]/255.0f,1};
				glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular);
				glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50);
				glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diff);
				glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,amb);
			}
			else{
				float diff[4] = {0.5,0.5,0.5,1};
				float amb[4] = {0.5,0.5,0.5,1};
				float specular[4] = {0,0,0,1};
				glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,specular);
				glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,diff);
				glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,amb);
			}
			glBegin(GL_TRIANGLES);
				for(int i=0;i<objets[j].nbFaces;i++){
					tFace facei = objets[j].faces[i];
					glNormal3fv((GLfloat*)&objets[j].normales[facei.indicesVecteurs[0]]);
					glVertex3fv((GLfloat*)&objets[j].vect[facei.indicesVecteurs[0]]);
					glNormal3fv((GLfloat*)&objets[j].normales[facei.indicesVecteurs[1]]);
					glVertex3fv((GLfloat*)&objets[j].vect[facei.indicesVecteurs[1]]);
					glNormal3fv((GLfloat*)&objets[j].normales[facei.indicesVecteurs[2]]);
					glVertex3fv((GLfloat*)&objets[j].vect[facei.indicesVecteurs[2]]);
				}
			glEnd();
		glPopMatrix();
		glEndList();
	}
}

int modele3D::trouveObjet(const char* nom)
{
	for(int i=0;i<nbObj;i++){
		if(strcmp(objets[i].nom,nom) == 0) return i;
	}
	return -1;
}

objet3D::objet3D()
{
	nom[0] = 0;
	texture = false;
	nbVect = 0;
	nbFaces = 0;
	nbCoordText = 0;
	matId = -1;
	vect = NULL;
	normales = NULL;
	coordText = NULL;
	faces = NULL;
}

objet3D::~objet3D()
{
	// attention, ne pas mettre les delete ici sinon il y a un pb avec vector qui détruit l'objet lors de l'accès par l'operateur [ ]....
}

