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
	if(listeAffichage >= 0){
		for(int i=0;i<nbObj;i++){
			glPushMatrix();
			glMultMatrixf(objets[i].matrice);
			glCallList(listeAffichage+i);
			glPopMatrix();
		}
	}
}

void modele3D::afficher(int i)
{
	if(listeAffichage >= 0){
		if(i >= 0 && i < nbObj){
			glPushMatrix();
			glMultMatrixf(objets[i].matrice);
			glCallList(listeAffichage+i);
			glPopMatrix();
		}
	}
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
	for(int i=0;i<15;i++) matrice[i] = 0;
	matrice[0] = 1.0f;
	matrice[5] = 1.0f;
	matrice[10] = 1.0f;
	matrice[15] = 1.0f;
}

objet3D::~objet3D()
{
	// attention, ne pas mettre les delete ici sinon il y a un pb avec vector qui détruit l'objet lors de l'accès par l'operateur [ ]....
}

void objet3D::translation(float dx,float dy,float dz)
{
// matrice homogène:
// on multiplie par une matrice de translation:
// ( m0 m4 m8  m12 )   ( 1  0  0  dx )
// ( m1 m5 m9  m13 ) * ( 0  1  0  dy )
// ( m2 m6 m10 m14 )   ( 0  0  1  dz )
// ( m3 m7 m11 m15 )   ( 0  0  0  1  )

	matrice[12] += matrice[0] * dx + matrice[4] * dy + matrice[8] * dz;
	matrice[13] += matrice[1] * dx + matrice[5] * dy + matrice[9] * dz;
	matrice[14] += matrice[2] * dx + matrice[6] * dy + matrice[10] * dz;
	matrice[15] += matrice[3] * dx + matrice[7] * dy + matrice[11] * dz;
}

void objet3D::rotation(float alpha,float x,float y,float z)
{
// matrice homogène:
// on multiplie par une matrice de rotation:
// ( m0 m4 m8  m12 )   ( x²(1-c)+c   xy(1-c)-zs  xz(1-c)+ys  0 )
// ( m1 m5 m9  m13 ) * ( xy(1-c)+zs  y²(1-c)+c   yz(1-c)-xs  0 )
// ( m2 m6 m10 m14 )   ( xz(1-c)-ys  yz(1-c)-xs  z²(1-c)+c   0 )
// ( m3 m7 m11 m15 )   ( 0           0           0           1 )

	// produit mat fait "à l'arrache"
	float c = cos(alpha);
	float s = sin(alpha);
	float unMc = 1-c;
	float xyUnMc = x*y*unMc;
	float xzUnMc = x*z*unMc;
	float yzUnMc = y*z*unMc;
	float xs = x*s;
	float ys = y*s;
	float zs = z*s;

	float rot[9] = { x*x*unMc + c, xyUnMc + zs, xzUnMc - ys, xyUnMc - zs, y*y*unMc + c, yzUnMc - xs, xzUnMc + ys, yzUnMc - xs, z*z*unMc + c};

	float m0  = matrice[0] * rot[0] + matrice[4] * rot[1] + matrice[8]  * rot[2];
	float m1  = matrice[1] * rot[0] + matrice[5] * rot[1] + matrice[9]  * rot[2];
	float m2  = matrice[2] * rot[0] + matrice[6] * rot[1] + matrice[10] * rot[2];
	float m4  = matrice[0] * rot[3] + matrice[4] * rot[4] + matrice[8]  * rot[5];
	float m5  = matrice[1] * rot[3] + matrice[5] * rot[4] + matrice[9]  * rot[5];
	float m6  = matrice[2] * rot[3] + matrice[6] * rot[4] + matrice[10] * rot[5];
	float m8  = matrice[0] * rot[6] + matrice[4] * rot[7] + matrice[8]  * rot[8];
	float m9  = matrice[1] * rot[6] + matrice[5] * rot[7] + matrice[9]  * rot[8];
	float m10 = matrice[2] * rot[6] + matrice[6] * rot[7] + matrice[10] * rot[8];

	matrice[0]  = m0;
	matrice[1]  = m1;
	matrice[2]  = m2;
	matrice[4]  = m4;
	matrice[5]  = m5;
	matrice[6]  = m6;
	matrice[8]  = m8;
	matrice[9]  = m9;
	matrice[10] = m10;
}

