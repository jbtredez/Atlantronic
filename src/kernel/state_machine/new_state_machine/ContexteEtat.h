//
//Classe de contexte pour le pattern Etat
//Cette classe contient la liste des etats possibles dans une map<string,Cetat>
//Cette classe contient l etat courant
#ifndef CONTEXT_ETAT_H
#define CONTEXT_ETAT_H

#include "Etat.h"

class ContextEtat
{
	protected:

		//Etat courant
		Etat * 		mp_EtatCourant;
		Etat *		mp_EtatPrecedent;


		////////////////////////////////////////
		//Recherche un etat dans la liste des Etats
		//Param : le nom de l'etat
		//retourne: CEtat * l'etat recherche est retourne
		//Etat * findEtat(int etatName);

		////////////////////////////////////////
	public:
		///////////////
		///Constructeur
		//ContextEtat();

		////////////////////////////////////////
		//Recoup�re l'etat courant
		//Param : Etat *  retourne l'etat courant
		//retourne: true si l'op�ration est r�ussite
		Etat * setCurrentState(Etat * EtatCourant){ mp_EtatCourant = EtatCourant; return mp_EtatCourant;};

		////////////////////////////////////////
		//Recoup�re l'etat courant
		//Param : Etat *  retourne l'etat courant
		//retourne: true si l'op�ration est r�ussite
		Etat * getCurrentState(){ return mp_EtatCourant;};

		////////////////////////////////////////
		//Recoup�re l'etat courant
		//Param : Etat *  retourne l'etat courant
		//retourne: true si l'op�ration est r�ussite
		Etat * getLastState(){ return mp_EtatPrecedent;};

		int execute();
};


#endif
