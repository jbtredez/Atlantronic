#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "control/control.h"
#include <stdio.h>
#include <stdlib.h>
#include "control/control.h"
#include "event.h"

#include "avoidance/macro_fonction.h"

//int test = 0;

unsigned char deplaceRobot(int x, int y, int *prbx, int *prby)
{
//     int autreX, autreY;
    
/*    if(test>=1) 
    {*/ 
      control_goto(x, y);
      vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
      //TODO j suis ou la???
      *prbx=0;*prby=0;
      return 0;
/*    }
    caseToVecteur(21, &autreX, &autreY);
    control_goto(autreX, autreY);
    vTaskWaitEvent(EVENT_CONTROL_READY);
    
//    update_table(*prbCase, OBSTACLE); // => y a quoi en face??? res retour d un capteur??? 
//	   			    // =>mettre autre chose que obstacle
//backwardRobot(350, &x, &y); //=> on recule ???

    update_table(28, ROBOT_ADV);
    *prbx = autreX;
    *prby = autreY;
    
    test++;
    return 1;*/
 
}


//si on ne sait pas quoi faire on remonte x et y pour changer d orde global
unsigned char goTo(int departCase, int destCase, int *prbCase)
{
	int numCase = departCase;
// 	int accidentx=0, accidenty=0;
	int x, y;
	int prbx=0, prby=0;
	unsigned char res = 0;
	int cmp=0;
	int dc, dl, ac, al;
	int cout1;
	
	int pathc,  pathl;

	while( (numCase != destCase) && (cmp < 5) )
	{	

		//a simplifier par une fonction plus sexy via case au lieu des coordonnées des cases
		dc = (numCase-1) % COLONNE;
		dl = (numCase-1) / COLONNE;
		ac = (destCase-1) % COLONNE;
		al = (destCase-1) / COLONNE;
		
		cout1 = coutNonRecursif(dc,dl,ac,al,&pathc,&pathl);
		//printf("depart en (%d,%d) pour (%d,%d), cout=%d\n", dc,dl,pathc,pathl, cout1);
		if( cout1 >= INFINI )
		{
			//printf("aucun bulletin aujourd hui\n");
			*prbCase = numCase;
			return 1;
		
		}
		else 
		{
		     //la voie est libre
		     caseToVecteur(pathc + 1 + (pathl * COLONNE), &x, &y);
		    
		}

		//GO GO! on bouge
		res=deplaceRobot(x, y, &prbx, &prby);
		if(res != 0) //voir le retour de deplaceRobot
		{
		   //on a eu un prb
		   vecteurToCase(prbx, prby, prbCase); 
		
		   if(check_path(*prbCase, destCase) == 1) //pas de muret donc obstacle mouvant
		   { 
			   
			   numCase = *prbCase; //vecteurToCase(x, y, &numCase); 
			   
		   }
		   //TODO
		   //else trouver une solution pour trouver le seul chemin :(
		   
		  
		}
		else 
		{
		   vecteurToCase(x, y, &numCase);
		  //numCase = destCase;
		}

		//update_table(numCase, ROBOT_ATL); //initule debug only

		cmp++;
		
	}
	if(cmp == 5) return 2; //boucle infini
	return 0;

}

#if 0
int main()
{
	int startCase;
	int numCase, destCase, prbCase;
	//int ligne=0, colonne=6;

	//init table + robots	
	init_table();

	//get robot color
	startCase = CASE_BLEU;
	if(startCase == CASE_ROUGE)
	{
		update_table(CASE_BLEU,ROBOT_ADV);
		//update_table(CASE_ROUGE,ROBOT_ATL); inutile debug only
	}
	else 
	{
                update_table(CASE_ROUGE,ROBOT_ADV);
                //update_table(CASE_BLEU,ROBOT_ATL); inutile only
	}

	
/* ou l inverse ^^
---------------------------------
| A |   |   |   |   |   |   | R |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
*/
	//faire sortir le robot de sa case de départ
        //straightOn( +350 en x)

	//a virer	
	numCase = 7;

	update_table(numCase,ROBOT_ATL); //inutile debug only
	afficher_table();
/*
---------------------------------
| A |   |   |   |   |   | R |   |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
*/

	//on va descendre de deux cases
	destCase = numCase + (2*COLONNE);
	if( goTo(numCase, destCase, &prbCase) != 0 )
	{
	   //prb, on doit changer la strategie
	   //notre pos est dans prbCase
	}

/*
---------------------------------
| A |   |   |   |   |   |   |   |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   | R | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
| P |   |   |   |   |   |   | P |
---------------------------------
*/
	afficher_table();
	//turnRobot()
	//getObject();	
	//backwardRobot()
	//turnRobot()
	//goTo
	//...  il faudrait faire une liste de chose prédéfini pour la premiere partie du match

	return 0;
}
#endif
