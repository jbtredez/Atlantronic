#include "strat_simple.h"
#include "kernel/log.h"
#include "kernel/motion/trajectory.h"


#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"



////////////////////////////////////////////////
/// function    : Initialise()
/// descrition  : intialise the checkpoint color
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
void stratcomplexedef::Initialise(int stratcolor)
{
	m_stratcolor = stratcolor;
	for(int i = 0 ; i < m_size_actionlist ; i++)
	{
		if( m_list_action[i] != 0 )
		{
			m_list_action[i]->Initialise(stratcolor);
		}
	}

}


////////////////////////////////////////////////
/// function    : run()
/// descrition  : execute the strategie
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int stratcomplexedef::run()
{
		//light 1 drop

	firstcheckpoint.x = 1285 + LIGHT_APPROX_DIST;
	firstcheckpoint.y = 0;
	light light1(firstcheckpoint,robot);

	int result =0;
	if(m_size_actionlist == 0)
	{
		return -1;
	}
	///Strategie complexe  sans recherche de meilleur action
	///Action 0  spotlight //done
	///Action 1  Sortir de lzone de départ //done
	///Action 2 clapet left  //done 
	///Action 3  Carpette //done
	///Action 4  spotlight 2 //done
	///Action 5  Gobelet 1 //done
	///Action 6  Gobelet 2 //done
	///Action 7  Gobelet 3	//done
	///Action 8 clapet right
	///Action 9 clapet other side

	// On récupere le spotlight 1
	m_list_action[0]->do_action();

	//On sort de la zone de départ
	m_list_action[1]->do_action();

	//spotlight 1
	/// On recherche le pied le plus proche
	m_list_action[0]->do_action();
	
	///Position of dropping
	drop(firstcheckpoint,robot);
	drop.do_action();

	movebackward(firstcheckpoint,robot);
	movebackward.do_action();


	m_list_action[0]->add_action(&light1);

	//GOBELET prise startzone	
	m_list_action[5]->do_action();
	
	//Gobelet drop startzone
	m_list_action[5]->do_action();

	//spotlight 1
	//Prise de la lumière
	m_list_action[0]->do_action();
	robot->setnumberelement(2);

	
	//spotlight 1
	//Pied 2
	m_list_action[0]->do_action();

	//spotlight 1
	//Prise pied 3
	m_list_action[0]->do_action();

	///Position of dropping 2
	////position du dernier pied
	drop(firstcheckpoint,robot);
	drop.do_action();

	m_list_action[0]->add_action(&light1);

///Fall gobelet?
	//GOBELET prise	mid
	m_list_action[6]->do_action();
	
	///Position of dropping 3
	//////// zone jaune upper
	drop(firstcheckpoint,robot);
	drop.do_action();
////////////

	//GOBELET prise	left
	m_list_action[7]->do_action();


	///Position of dropping 4
	//// zone yellow lower
	drop(firstcheckpoint,robot);
	drop.do_action();
////////
	//spotlight 1
	//Prise de la lumière
	m_list_action[0]->do_action();
	robot->setnumberelement(4);
	
	//spotlight 1
	//Prise pied 4
	m_list_action[0]->do_action();

	//Clapet left
	m_list_action[2]->do_action();

	//drop du spotlight protect zone (middle)s	
	m_list_action[0]->do_action();


	///on recule
	movebackward(firstcheckpoint,robot);
	movebackward.do_action();

	//spotlight 2
	//Prise de la lumière
	m_list_action[4]->do_action();




	//carpette
	m_list_action[3]->do_action();

	//spotlight 2
	//Prise pied 1
	m_list_action[4]->do_action();


	//spotlight 2
	//Prise pied 2 
	m_list_action[4]->do_action();

	/// TODO DROP STEP


	//spotlight 2
	//Prise pied 3 
	m_list_action[4]->do_action();




	//spotlight 2
	//Drop unr starzone 
	m_list_action[4]->do_action();


	//GOBELET prise	left
	m_list_action[7]->do_action();
	//drop gobelt green lower
	m_list_action[7]->do_action();

	m_list_action[9]->do_action();
	return 0;

}

////////////////////////////////////////////////
/// function    : find_action_not_done()
/// descrition  : find the nearest a type action wich are not done
/// param       :int : type of the action
/// retrun      : -1 if fail or Number of the action in the list  if sucess
////////////////////////////////////////////////
Action * stratcomplexedef::find_action_not_done(int type, VectPlan position)
{
	
	float distance = 99999;
	VectPlan Vectdistance;
	float result = 0;
	Action * p_actiontodo = 0;
	for( int i = 0 ; i< NB_MAX_COMPO_ACTION ; i++)
	{

		
		if( (m_list_action[i]->m_actiontype == type) 
			&& m_list_action[i]->m_try > -1)
		{
			Vectdistance = (position - (m_list_action[i]->get_firstcheckpoint()));
			distance = Vectdistance.norm();
			if(result < distance )
			{
				distance = result;
				p_actiontodo = m_list_action[i];
			}
 
			

		}

	}
	return p_actiontodo;
}
