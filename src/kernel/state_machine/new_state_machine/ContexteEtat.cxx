#include "kernel/log.h"

#include "ContexteEtat.h"
#include "Etat.h"

/*
ContextEtat::ContextEtat()
{
	mp_EtatCourant = 0;
	mp_EtatPrecedent = 0;
}

*/
int ContextEtat::execute()
{
	Etat * p_Etatchange = 0;
	if( mp_EtatCourant != mp_EtatPrecedent)
	{
		mp_EtatCourant->entry();
	}

	mp_EtatCourant->run();


	p_Etatchange = mp_EtatCourant->getProchainEtat();


	//Si on arrive dans un etat inconnu
	if( 0 == p_Etatchange)
	{
		log_format(LOG_ERROR, "invalid state machine transition from  %s",mp_EtatCourant->getNameEtat());
		return -1;
	}
	else
	{
		mp_EtatPrecedent = mp_EtatCourant;
		mp_EtatCourant = p_Etatchange;
	}

	//Si changement d'etat
	if( mp_EtatCourant != mp_EtatPrecedent)
	{
		log_format(LOG_INFO, "Changement d'etat %s vers l'etat %s", mp_EtatPrecedent->getNameEtat(), mp_EtatCourant->getNameEtat());
		mp_EtatPrecedent->out();
	}

	return 0;
}
