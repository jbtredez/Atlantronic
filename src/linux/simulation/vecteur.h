#ifndef _VECTEUR_H_
#define _VECTEUR_H_

#include <cmath>
#define EPSILON			0.0001f	//!< comparaison entre float à EPSILON près (ou 3 EPSILON près pour les vecteurs)

//! @file vecteur.h
//! @brief Contient les classes vecteur, vecteur2f et vecteur3f
//! @author TREDEZ Jean-Baptiste

//! @class vecteur
//! @brief Representation de la position du robot ( vecteur 2D + orientation )
//! Il contient l'abscisse, l'ordonnee et l'angle d'orientation
class vecteur{
public:
	inline vecteur();	//!< Constructeur par defaut.
	inline vecteur(float X,float Y,float Alpha);	//!< Constructeur
	inline vecteur(const vecteur &v);	//!< Constructeur de copie
	inline ~vecteur();	//!< Destructeur
	inline vecteur operator+ (const vecteur &v) const;	//!< Operateur +
	inline vecteur operator- (const vecteur &v) const;	//!< Operateur -
	inline void operator += (const vecteur &v);	//!< Operateur +=
	inline void operator -= (const vecteur &v);	//!< Operateur-=
	inline bool operator == (const vecteur &v);	//!< Operateur de comparaison ==

	inline float norme() const; //!< calcule la norme du vecteur
	inline float norme2() const; //!< calcule la norme au carré du vecteur (évite de prendre la racine et de remettre le tout au carré)
	inline void bornerAngle();	//!< replace l'angle entre -PI et PI

	// fonctions complexes
	float distanceSegment(vecteur a,vecteur b);	//!< calcule la distance du point au segment [a,b] (et non à la droite !!)
	void odoEvolution(float deltaAvance,float deltaRot);	//!< Permet de calculer l'odométrie (ATTENTION: fonction déportée sur le PIC, ne sert plus que pour le debug ou pour la simulation) 

	float x;	//!< Abscisse
	float y;	//!< Ordonnee
	float alpha;	//!< Angle d'orientation
};

inline float determinant(const vecteur &v1, const vecteur &v2);	//!< calcule le déterminant de deux vecteurs
inline float scalaire(const vecteur &v1, const vecteur &v2);	//!< calcule le produit scalaire de deux vecteurs
inline float trouverRotation(float debut, float fin);	//!< détermine l'angle optimal de rotation à effectuer pour passer de l'orientation debut à l'orientation fin
inline float bornerAngle(float alpha);	//!< replace l'angle entre -PI et PI
vecteur baseTableVersBaseRobot(const vecteur bTable, const vecteur positionRobot); //!< changement de repere de la base de la table vers la base robot
vecteur baseRobotVersBaseTable(const vecteur bRobot, const vecteur positionRobot); //!< changement de repere de la base du robot vers la base table

//! @class vecteur3f
//! @brief Representation d'un vecteur 3D ("3 float")
class vecteur3f{
public:
	inline vecteur3f();	//!< Constructeur par defaut
	inline vecteur3f(float a,float b,float c);	//!< Constructeur
	inline vecteur3f(const vecteur3f &v);	//!< Constructeur de copie
	inline ~vecteur3f();	//!< Destructeur
	inline vecteur3f operator- (const vecteur3f &v); //!< Operateur -
	inline vecteur3f operator+ (const vecteur3f &v); //!< Operateur +
	inline vecteur3f operator/ (float a); //!< Operateur /
	inline void operator+= (const vecteur3f &v); //!< Operateur +=
	inline void operator-= (const vecteur3f &v); //!< Operateur -=

	inline void normer();	//!< norme le vecteur
	inline float norme() const; //!< calcule la norme du vecteur
	inline float norme2() const; //!< calcule la norme au carré du vecteur (évite de prendre la racine et de remettre le tout au carré)

	float x;	//!< coordonnée x
	float y;	//!< coordonnée y
	float z;	//!< coordonnée z
};

inline vecteur3f operator* (float a, const vecteur3f &v); //!< Operateur *
inline vecteur3f prodVect(const vecteur3f &v1,const vecteur3f &v2);	//!< produit vectoriel
inline float prodScalaire(const vecteur3f &v1,const vecteur3f &v2);	//!< produit scalaire

//! @class vecteur2f
//! @brief Representation d'un vecteur 2D ("2 float")
class vecteur2f{
public:
	inline vecteur2f(); //!< Constructeur par defaut
	inline vecteur2f(float a,float b); //!< Constructeur
	inline vecteur2f(const vecteur2f &v); //!< Constructeur de copie
	inline vecteur2f operator- (const vecteur2f &v); //!< Operateur -
	inline vecteur2f operator+ (const vecteur2f &v); //!< Operateur +
	inline vecteur2f operator/ (float a); //!< Operateur /
	inline void operator+= (const vecteur2f &v); //!< Operateur +=
	inline void operator-= (const vecteur2f &v); //!< Operateur -=

	inline void normer(); //!< norme le vecteur
	inline float norme() const; //!< calcule la norme du vecteur
	inline float norme2() const; //!< calcule la norme au carré du vecteur (évite de prendre la racine et de remettre le tout au carré)

	float x;	//!< Abscisse
	float y;	//!< Ordonnee
};

vecteur2f baseTableVersBaseRobot(const vecteur2f bTable, const vecteur positionRobot); //!< changement de repere de la base de la table vers la base robot
vecteur2f baseRobotVersBaseTable(const vecteur2f bRobot, const vecteur positionRobot); //!< changement de repere de la base du robot vers la base table
inline vecteur2f homographie(const vecteur2f &v, const float* matriceHomographie); //!< calcule l'homographie
inline vecteur2f operator* (float a, const vecteur2f &v); //!< Operateur *

//-----------------------------------------------------------------------------
// fonctions "inline", "fin" de l'entête
//-----------------------------------------------------------------------------

inline vecteur::vecteur()
{
	
}

//! @param X coordonnée x
//! @param Y coordonnée y
//! @param Alpha angle d'orientation alpha
inline vecteur::vecteur(float X,float Y,float Alpha)
	: x(X), y(Y), alpha(Alpha)
{

}

//! @param v Vecteur a recopier
inline vecteur::vecteur(const vecteur &v)
	: x(v.x), y(v.y), alpha(v.alpha)
{

}

inline vecteur::~vecteur()
{

}

//! @param v vecteur a additionner
inline vecteur vecteur::operator+ (const vecteur &v) const
{
	return vecteur(x+v.x,y+v.y,alpha+v.alpha);
}

//! @param v vecteur a soustraire
inline vecteur vecteur::operator- (const vecteur &v) const
{
	return vecteur(x-v.x,y-v.y,alpha-v.alpha);
}

//! @param v vecteur a additionner
inline void vecteur::operator += (const vecteur &v)
{
	x += v.x;
	y += v.y;
	alpha += alpha;
}

//! @param v vecteur a soustraire
inline void vecteur::operator -= (const vecteur &v)
{
	x -= v.x;
	y -= v.y;
	alpha -= alpha;
}

//! @param v vecteur a comparer
inline bool vecteur::operator == (const vecteur &v)
{
	return ( fabsf(v.x - x) + fabsf (v.y - y) + fabsf(v.alpha - alpha) < EPSILON); 
}

inline float vecteur::norme2() const
{
	return x*x + y*y;
}

inline float vecteur::norme() const
{
	return sqrt(x*x + y*y);
}

inline void vecteur::bornerAngle()
{
	alpha = ::bornerAngle(alpha);
}

inline vecteur2f::vecteur2f()
{

}

//! @param a coordonnée x
//! @param b coordonnée y
inline vecteur2f::vecteur2f(float a,float b)
	: x(a), y(b)
{

}

//! @param v Vecteur a recopier
inline vecteur2f::vecteur2f(const vecteur2f &v)
	: x(v.x), y(v.y)
{

}

//! @param v Vecteur a soustraire
inline vecteur2f vecteur2f::operator- (const vecteur2f &v)
{
	return vecteur2f(x-v.x,y-v.y);
}

//! @param v Vecteur a ajouter
inline vecteur2f vecteur2f::operator+ (const vecteur2f &v)
{
	return vecteur2f(x+v.x,y+v.y);
}

//! @param a division par a
inline vecteur2f vecteur2f::operator/ (float a)
{
	return vecteur2f(x/a,y/a);
}

//! @param v Vecteur a ajouter
inline void vecteur2f::operator+= (const vecteur2f &v)
{
	x += v.x;
	y += v.y;
}

//! @param v Vecteur a soustraire
inline void vecteur2f::operator-= (const vecteur2f &v)
{
	x -= v.x;
	y -= v.y;
}

inline void vecteur2f::normer()
{
	float n = sqrt(x*x+y*y);
	x /= n;
	y /= n;
}

//! @return norme du vecteur
inline float vecteur2f::norme() const
{
	return sqrt(x*x + y*y);
}

//! @return norme au carré du vecteur
inline float vecteur2f::norme2() const
{
	return x*x + y*y;
}

inline vecteur2f operator* (float a, const vecteur2f &v)
{
	return vecteur2f(a*v.x,a*v.y);
}

//! @param v vecteur dans les anciennes coordonnées
//! @param matriceHomographie matrice d'homographie
//! @return vecteur dans les nouvelles coordonnées
inline vecteur2f homographie(const vecteur2f &v, const float* matriceHomographie)
{
	float x = matriceHomographie[0]*v.x + matriceHomographie[1]*v.y + matriceHomographie[2];
	float y = matriceHomographie[3]*v.x + matriceHomographie[4]*v.y + matriceHomographie[5];
	float w = matriceHomographie[6]*v.x + matriceHomographie[7]*v.y + matriceHomographie[8];
	x /= w;
	y /= w;
	return vecteur2f(x,y);
}

inline vecteur3f::vecteur3f()
{
	
}

//! @param a coordonnée x
//! @param b coordonnée y
//! @param c coordonnée z
inline vecteur3f::vecteur3f(float a,float b,float c)
	: x(a), y(b), z(c)
{

}

//! @param v vecteur à copier
inline vecteur3f::vecteur3f(const vecteur3f &v)
	: x(v.x), y(v.y), z(v.z)
{

}

inline vecteur3f::~vecteur3f()
{

}

//! @param v vecteur a soustraire
inline vecteur3f vecteur3f::operator- (const vecteur3f &v)
{
	return vecteur3f(x-v.x,y-v.y,z-v.z);
}

//! @param v vecteur a additionner
inline vecteur3f vecteur3f::operator+ (const vecteur3f &v)
{
	return vecteur3f(x+v.x,y+v.y,z+v.z);
}

inline vecteur3f vecteur3f::operator/ (float a)
{
	return vecteur3f(x/a,y/a,z/a);
}

inline vecteur3f operator* (float a, const vecteur3f &v)
{
	return vecteur3f(a*v.x,a*v.y,a*v.z);
}

//! @param v vecteur a additionner
inline void vecteur3f::operator+= (const vecteur3f &v)
{
	x += v.x;
	y += v.y;
	z += v.z;
}

//! @param v vecteur a soustraire
inline void vecteur3f::operator-= (const vecteur3f &v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
}

inline void vecteur3f::normer()
{
	float n = sqrt(x*x+y*y+z*z);
	x /= n;
	y /= n;
	z /= n;
}

//! @return norme du vecteur
inline float vecteur3f::norme() const
{
	return sqrt(x*x + y*y + z*z);
}

//! @return norme au carré du vecteur
inline float vecteur3f::norme2() const
{
	return x*x + y*y + z*z;
}

//! @param v1 premier vecteur
//! @param v2 second vecteur
//! @return determinant de v1 et v2
inline float determinant(const vecteur &v1, const vecteur &v2)
{
	return v1.x*v2.y - v1.y*v2.x;
}

//! @param v1 premier vecteur
//! @param v2 second vecteur
//! @return produit scalaire de v1 et v2
inline float scalaire(const vecteur &v1, const vecteur &v2)
{
	return v1.x*v2.x + v1.y*v2.y;
}

//! @param alpha angle à borner entre -Pi et Pi
//! @return valeur de l'angle dans [-Pi;Pi]
inline float bornerAngle(float alpha)
{
	alpha = fmod(alpha, 2*M_PI); // Retour dans [-2*PI;2*PI]
	
	// Retour dans [-PI;PI] si neccessaire
	if (alpha > M_PI){
		alpha -= 2*M_PI;
	}
	else if (alpha < -M_PI){
		alpha += 2*M_PI;
	}
	
	return alpha;
}

//! @param debut orientation de depart
//! @param fin orientation finale
//! @return Valeur de la rotation
inline float trouverRotation(float debut, float fin)
{
	return bornerAngle(fin-debut);
}

//! @param v1 premier vecteur
//! @param v2 second vecteur
//! @return produit vectoriel de v1 et v2
inline vecteur3f prodVect(const vecteur3f &v1,const vecteur3f &v2)
{
	return vecteur3f(v2.y*v1.z - v2.z*v1.y, v1.x*v2.z - v1.z*v2.x, v2.x*v1.y - v2.y*v1.x);
}

//! @param v1 premier vecteur
//! @param v2 second vecteur
//! @return produit scalaire de v1 et v2
inline float prodScalaire(const vecteur3f &v1,const vecteur3f &v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

#endif
