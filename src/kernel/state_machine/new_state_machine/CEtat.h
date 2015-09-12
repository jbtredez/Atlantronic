//
//Classe mere d'un etat
//Cette classe contient le nom de l'etat


#ifndef CETAT
#define CETAT

class Etat
{

	protected:
		//Liste des etats li�e au contexte
		const char * m_name;
	public:
		///////////////
		///Constructeur
		Etat(){m_name = "";};

		///////////////
		///Constructeur
		Etat(const char * name){m_name = name;};
		///////////////
		///Destructeur
		~Etat(){};

		////////////////////////////////////////
		//r�cup�re le nom de l'etat
		//Param :
		//retourne: le nom de l'etat
		const char * getNameEtat(){return m_name;};

		////////////////////////////////////////
		//Ajoute un Etat dans la liste des Etats
		//Param :
		//retourne: le nom de l'etat
		void setNameEtat(char* name){m_name = name ;};
		
		////////////////////////////////////////
		//m�thode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: R�ussite de l'action		
		virtual bool run() = 0;

		////////////////////////////////////////
		//m�thode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: R�ussite de l'action		
		virtual bool entry() = 0;		
		////////////////////////////////////////
		//m�thode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: R�ussite de l'action		
		virtual bool out() = 0;
		

		////////////////////////////////////////
		//m�thode recupere l'etat suivant
		//Param :
		//retourne: Nom de l'etat suivant		
		virtual Etat * getProchainEtat() = 0;
		
};


#endif
