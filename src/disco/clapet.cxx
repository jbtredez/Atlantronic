

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/wing.h"
#include "clapet.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"
clapet::clapet(VectPlan firstcheckpoint):action(firstcheckpoint)
{

	 
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int clapet::do_action()
{

	int bresult = 0;
	int essaie =0;
	int second_x_position = 0;

    	VectPlan nextToClap ;


	if(m_try < 0 )
	{
		return 0;
	}

	nextToClap = m_firstcheckpoint;


   	//On se déplace sur le vecteur y vers l'origine de la taille du clap (un décalage postif si le x est négatif)
	if(m_firstcheckpoint.x > 0)
	{
		second_x_position = m_firstcheckpoint.x - 185;
		nextToClap.theta = -3.14f;
	}
	else
	{
		second_x_position = m_firstcheckpoint.x + 185;
		
	}


	//Mise en place de la position
        trajectory_goto(nextToClap, WAY_FORWARD, AVOIDANCE_STOP);

   	//On ouvre nos ailes, pas besoin de réflechir de quel coté on est(homologation).
 
	//Si on arrive pas à joindre le clapet on abandonne
	if(trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) != 0)
	{
		return -1; 
	 }
	//On ouvre l'aile pas besoin de réfléchir
   	wing_set_position(WING_OPEN, WING_OPEN);


	nextToClap.x = second_x_position;

	//On essaie de se déplacer 3 fois afin d'abandonner
	do
	{
    		trajectory_goto(nextToClap, WAY_FORWARD, AVOIDANCE_STOP);
		if(trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) == 0)
		{
			bresult = 0;
		}
		else
		{

			bresult = -1;
		}
	
		essaie++;
	}while(essaie <= 3 && !bresult); 

	//On ferme l'aile pas besoin de réfléchir
	wing_set_position(WING_PARK, WING_PARK);

	return bresult;
}
	
	

