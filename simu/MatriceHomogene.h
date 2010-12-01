#ifndef _MATRICE_HOMOGENE_H_
#define _MATRICE_HOMOGENE_H_

//! @file MatriceHomogene.h
//! @brief Matrice homogène 4x4
//! @author Jean-baptiste Trédez

class MatriceHomogene
{
public:
	MatriceHomogene();
	void translation(float dx,float dy,float dz);       //!< translate l'objet de (dx,dy,dz)
	void rotation(float alpha,float x,float y,float z); //!< rotation d'angle alpha autour du vecteur (x,y,z)
	float matrice[16];                                  //!< matrice de translation et rotation de l'objet
};

#endif
