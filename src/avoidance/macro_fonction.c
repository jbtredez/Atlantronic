#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "avoidance/macro_fonction.h"

unsigned char etap_path;

/*Le déplacement est impossible entre :
1 et 9
2 et 9
1 et 10
8 et 15
7 et 16
8 et 16
41 et 42
41 et 34
48 et 47
48 et 39
33 et 42
40 et 47
43 et 44
43 et 36
46 et 45
46 et 37*/
// => VÉRIFIE QU IL N Y A PAS DE PETIT MUR
unsigned char check_path(int case1, int case2)
{
  unsigned char result = 1;

  if( (case1==1) && ((case2==9) || (case2==10))) //1 et 9 ou 1 et 10
    result = 0;

  if( (case1==2) && (case2==9)) //2 et 9
    result = 0;

  if( (case1==8) && ((case2==15) || (case2==16)) ) //8 et 15 ou 8 et 16
    result = 0;

  if( (case1==7) && (case2==16) ) //7 et 16
    result = 0;

  if( (case1==41) && ((case2==42) || (case2==34)) ) //41 et 42 OU 41 et 34
    result = 0;

  if( (case1==48) && ((case2==47) || (case2==39)) ) //48 et 47 OU 48 et 39
    result = 0;

  if( (case1==33) && (case2==42) ) //33 et 42
    result = 0;

  if( (case1==40) && (case2==47) ) //40 et 47
    result = 0;

  if( (case1==43) && ((case2==44) || (case2==36)) )//43 et 44 OU 43 et 36
    result = 0;

  if( (case1==46) && ((case2==45) || (case2==37)) )//46 et 45 OU 46 et 37
    result = 0;

  return result;
}



unsigned char check_path_with_coordinate(int c1, int l1, int c2, int l2)
{
  unsigned char result = 1;

  if( (c1==0)&&(l1==0) && (((c2==0)&&(l2==1)) || ((c2==1)&&(l2==1)))) //1 et 9 ou 1 et 10
    result = 0;

  if( (c1==1)&&(l1==0) && (c2==0)&&(l2==1) ) //2 et 9
    result = 0;
    
  if( (c1==7)&&(l1==0) && (((c2==6)&&(l2==1)) || ((c2==7)&&(l2==1)))) //8 et 15 ou 8 et 16
    result = 0;

  if( (c1==6)&&(l1==0) && (c2==7)&&(l2==1) ) //7 et 16
    result = 0;

  if( (c1==0)&&(l1==5) && (((c2==1)&&(l2==5)) || ((c2==1)&&(l2==4)))) //41 et 42 OU 41 et 34
    result = 0;
  
  if( (c1==7)&&(l1==5) && (((c2==6)&&(l2==5)) || ((c2==6)&&(l2==4)))) //48 et 47 OU 48 et 39
    result = 0;
  
  if( (c1==0)&&(l1==4) && (c2==1)&&(l2==5) ) //33 et 42
    result = 0;
  
  if( (c1==7)&&(l1==4) && (c2==6)&&(l2==5) ) //40 et 47
    result = 0;
  
  if( (c1==2)&&(l1==5) && (((c2==2)&&(l2==5)) || ((c2==3)&&(l2==5))))//43 et 44 OU 43 et 36
    result = 0;
  
  if( (c1==5)&&(l1==5) && (((c2==4)&&(l2==5)) || ((c2==4)&&(l2==4))))//46 et 45 OU 46 et 37
    result = 0;
  
  return result;
}


//renvoi 0 si un obstacl est trouvé sur la route
//renvoie le nombre de case à traverser pour y aller
int Bresenham(int c1, int l1, int c2, int l2, int *oc, int *ol)
{     
  
      int dc, dl, p, endc, endl;
      int colonne, ligne;
      int incl = 1;
      unsigned int compteur = 0;
      
      if( (c1 >= COLONNE) || (c2 >= COLONNE) || (l1 >= LIGNE) || (l2 >= LIGNE) )
      {
// 	printf("<*************** OUT OF TABLE *******************>\n");
	return INFINI;
      }
      if( (c1 == c2) && (l1 == l2) ) return 1; //patch
	
      dc = abs(c1 - c2);
      dl = abs(l1 - l2);
      p = 2 * dl - dc;
      if(c1 > c2)
      {
            colonne = c2;
            ligne = l2;
            endc = c1;
	    endl = l1;
	    if(l1 < l2) incl = -1;
      }
      else
      {
            colonne = c1;
            ligne = l1;	
            endc = c2;
	    endl = l2;
	    if(l1 > l2) incl = -1;
      }
      
      
      //printf("c1=%d l1=%d c2=%d l2=%d incl=%d\n",c1, l1, c2, l2, incl);
      
      if((table[colonne+(ligne*COLONNE)]& OBSTACLE) == OBSTACLE)
      {
	*oc = colonne;
	*ol = ligne;
	return 0;
      }
//       table[colonne+(ligne*COLONNE)]='X';
//       printf("ligne=%d colonne=%d\n",ligne,colonne);
      while(colonne < endc)
      {
            colonne = colonne + 1;
            if(p < 0)
            {
                  p = p + 2 * dl;
            }
            else
            {
             	  //on tente la diago 
		  if( ((table[colonne+(ligne*COLONNE)] & OBSTACLE) != OBSTACLE) && ((table[(colonne-1)+((ligne+incl)*COLONNE)] & OBSTACLE) != OBSTACLE) ) 
		  {
		        ligne = ligne + incl;
		  }
		  else
		  //la diago n est pas accessible, quel obstacle est à coté?
		  {
		    if( ((table[colonne+(ligne*COLONNE)] & OBSTACLE) == OBSTACLE) && ((table[(colonne-1)+((ligne+incl)*COLONNE)] & OBSTACLE) == OBSTACLE) ) 
		    {
			//cul de sac
			*oc = colonne - 1;
			*ol = ligne;
			//printf("Bresenham CUL DE SAC\n");
			//return 0;
			return INFINI;
		    }
		    else 
		    {
		      //printf("detour\n");
		      if ((table[(colonne-1)+((ligne+incl)*COLONNE)]& OBSTACLE) != OBSTACLE) 
		      {
			colonne += -1;
			ligne += incl;
// 			table[colonne+(ligne*COLONNE)]='X';
			compteur++;
			colonne += 1;
			
			//v2
/*			*oc = colonne - 1;
			*ol = ligne + incl;
			//printf("Bresenham obstacle\n");
			return 0;*/
		      }
		      else
		      {
// 			    table[colonne+(ligne*COLONNE)]='X';
			    compteur++;
			    ligne += incl;
			    
			    //v2
/*			    printf("Bresenham obstacle\n");
			    *ol = ligne + incl;			
			    *oc = colonne;
			    return 0;*/
		      } 
		    }
		  }
		  
                  p = p + 2 * (dl - dc);
            }
            
// 	    printf("ligne=%d colonne=%d dans le while \n",ligne,colonne);
	    if((table[colonne+(ligne*COLONNE)]& OBSTACLE) == OBSTACLE) 
	    {
	      *oc = colonne;
	      *ol = ligne;
	      //printf("Bresenham obstacle\n");
	      return 0;
	    }
	    compteur++;
// 	    table[colonne+(ligne*COLONNE)]='X';
      }
      
//       printf("ligne=%d colonne=%d\n",ligne,colonne);
      while (ligne != endl) 
      {
	ligne = ligne + incl;
	if( (ligne>=LIGNE)|| (ligne < 0 ))
	{
	  //printf("--------------- LIGNE MIN or MAX REACHED --------------------\n");
	  return INFINI;
	}
// 	printf("ligne=%d colonne=%d\n",ligne,colonne);
	if((table[colonne+(ligne*COLONNE)]& OBSTACLE) == OBSTACLE) 
	{
	    *oc = colonne;
	    *ol = ligne;
	    //printf("Bresenham obstacle\n");
	    return 0;
	}

// 	table[colonne+(ligne*COLONNE)]='X';
	compteur++;
      }
      //printf("compteur=%d\n", compteur);
      return compteur;
}





//renvoie 0 si pas de point, 1 si un point de déviation trouvé => priorité à droite, 2 si deux points de contournement
unsigned int BisonFute(int dc, int dl, int ac, int al, int oc, int ol, int *tribordc, int *tribordl, int *babordc, int *babordl)
{
  unsigned int ret=0;
  /*
   Soit D notre point de départ définit par Xd et Yd.
   Soit A notre point d'arrivée définit par Xa et Yd.
   Soit O le point de l'obstacle définit par Xo et Yo.
   On suppose que ses cases sont différentes l'une de l'autre et l'obstacle distant d'au moins de deux cases des deux autres.
   si Xa = Xd alors contourner par Xo - 1 ou Xo + 1 et Yo (cas n°2)
   sinon si Ya = Yd alors contourner par Xo et Yo -1 ou Yo + 1 (cas n°3)
   sinon si  Ya – Yd > 0 alors contourner par Xo – 1 et Yo – 1 ou Xo + 1 et Yo + 1 (cas n°4)
   sinon contourner par Xo + 1 et Yo – 1 ou Xo - 1 et Yo + 1 (cas n°1)
   On relance l'algo de Bresenham entre le point de déviation et le point d'arrivée.
  */
   //printf("BF dc=%d, dl=%d ac=%d, al=%d, oc=%d, ol=%d\n", dc, dl, ac, al, oc, ol);
   
  
   if((dc == ac) || ((al == ol+1)&&(oc == ac)) || ((al == ol-1)&&(oc == ac))
		    || ((dl == ol+1)&&(oc == dc)) || ((dl == ol+1)&&(oc == dc)))
   {
      if( ((table[(oc-1)+(ol*COLONNE)]& OBSTACLE) != OBSTACLE) && ( oc > 0 ) ) 
	{
	    *tribordc = oc - 1;
	    *tribordl = ol;
	    ret=1;
	}
	
	if( ((table[(oc+1)+(ol*COLONNE)]& OBSTACLE) != OBSTACLE) && ( oc <(COLONNE-1)) )
	{
	  if(ret) 
	  { 
	      *babordc = oc + 1;
	      *babordl = ol;
	      ret=2;
	  }
	  else 
	  {
	      *tribordc = oc + 1;
	      *tribordl = ol;
	      ret=1;
	  }
	}


   }
   else
   {
     if( (dl == al) || ((ac == oc+1)&&(ol == al)) || ((ac == oc-1)&&(ol == al))
		    || ((dc == oc+1)&&(ol == dl)) || ((dc == oc+1)&&(ol == dl)))
     {
	if( ((table[oc+((ol-1)*COLONNE)]& OBSTACLE) != OBSTACLE) && (ol>0) ) 
	{
	    //Xo et Yo -1 
	    *tribordc = oc;
	    *tribordl = ol-1;
	    ret=1;
	}
	
	if( ((table[oc+((ol+1)*COLONNE)]& OBSTACLE) != OBSTACLE) && (ol <(LIGNE-1)) )
	{
	  //Xo et Yo + 1
	  if(ret) 
	  { 
	      *babordc = oc;
	      *babordl = ol+1;
	      ret=2;
	  }
	  else 
	  {
	      *tribordc = oc;
	      *tribordl = ol+1;
	      ret=1;
	  }
	}
      	
     }
     else
     {
       
      if( (((dl - al) > 0) && ((dc - ac) > 0)) ||  (((dl - al) < 0) && ((dc - ac) < 0)) )
      {
      
	if (((table[(oc+1)+((ol-1)*COLONNE)]& OBSTACLE) != OBSTACLE)&&(oc<(COLONNE-1))&&(ol>0) ) 
	{
	    //Xo + 1 et Yo – 1 
	    *tribordc = oc+1;
	    *tribordl = ol-1;
	    ret=1;
	}
	if( ((table[(oc-1)+((ol+1)*COLONNE)]& OBSTACLE) != OBSTACLE)&&(oc>0)&&(ol<(LIGNE-1)) ) 
	{
	  //Xo - 1 et Yo + 1 
	  if(ret) 
	  { 
	      *babordc = oc-1;
	      *babordl = ol+1;
	      ret=2;
	  }
	  else 
	  {
	      *tribordc = oc-1;
	      *tribordl = ol+1;
	      ret=1;
	  }
	}
      
      }
      else
      {
	if(((table[(oc-1)+((ol-1)*COLONNE)]& OBSTACLE) != OBSTACLE)&&(ol>0)&&(oc>0)) 
	{
	    //Xo – 1 et Yo – 1
	    *tribordc = oc-1;
	    *tribordl = ol-1;
	    ret=1;
	}
	
	if(((table[(oc+1)+((ol+1)*COLONNE)]& OBSTACLE) != OBSTACLE)&&(oc<(COLONNE-1))&&(ol<(LIGNE-1))) 
	{
	  //Xo + 1 et Yo + 1
	  if(ret) 
	  { 
	      *babordc = oc+1;
	      *babordl = ol+1;
	      ret=2;
	  }
	  else 
	  {
	      *tribordc = oc+1;
	      *tribordl = ol+1;
	      ret=1;
	  }
	}
      }
     }
   }
   
   //if(ret > 0) table[(*tribordc)+(*tribordl*COLONNE)]='B'; 
   //if(ret == 2) table[*babordc+(*babordl*COLONNE)]='B'; 
   //printf("BF => ret=%d tribordc=%d tribordl=%d babordc=%d babordl=%d\n",ret,*tribordc, *tribordl, *babordc,*babordl);
   return ret;
}


// void init_best_path(void)
// {
//   int i;
//   etap_path=0;
//   for(i=0; i<PROFONDEUR;i++) best_path[i]='V'; //vide par défaut
// }

// int min_path(int A, int B)
// {
//  if( A < B ) 
//  {
//    best_path[i]=='F';
//    return A;
//  }
//  else return B;
// }

int cout(int dc, int dl, int ac, int al, int *pathc, int *pathl)
{
  int obsctacle_colonne, obstacle_ligne;
  int res;
  int tribordc, tribordl, babordc, babordl;
  int cout1, cout2;
  
  res = Bresenham(dc,dl,ac,al,&obsctacle_colonne,&obstacle_ligne);
  
  if( res > 0) 
  {
    *pathc = ac; 
    *pathl = al;
    return res; //besoin de differencier avec INFINI?
  }
//   for (i=pow(2,profondeur-1)-1;i<pow(2,profondeur);i++) printf("i=%d\n",i);
  //printf("call BisonFute avec dc=%d dl=%d ac=%d al=%d obsctacle_colonne=%d obstacle_ligne=%d\n", dc, dl, ac, al, obsctacle_colonne, obstacle_ligne); 
  res = BisonFute(dc, dl, ac, al, obsctacle_colonne, obstacle_ligne, &tribordc, &tribordl, &babordc, &babordl); 
  if(res==0) 
  {
    //printf("pas de bulletin\n");
    return INFINI; 
  }
  else
  {
    cout1 = cout( tribordc, tribordl, ac, al,pathc, pathl) + cout( dc, dl, tribordc, tribordl,pathc, pathl) ;
    if(res==1)
    {
      cout2 = INFINI;
    }
    else cout2 =  cout( babordc, babordl, ac, al, pathc, pathl) + cout( dc, dl, babordc, babordl, pathc, pathl) ;
  }
  if(cout1 < cout2)
  {
    if(!etap_path)
    {
      *pathc = tribordc; 
      *pathl = tribordl;
      etap_path=1;
      //printf("cout1 => pathc=%d pathl=%d\n", *pathc, *pathl);
    }
    //table[(tribordc)+(tribordl*COLONNE)]='F';     
    return cout1;
  }
  else
  {
    //table[babordc+(babordl*COLONNE)]='F'; 
    if(!etap_path)
    {
      *pathc = babordc; 
      *pathl = babordl;
      //printf("cout2 => pathc=%d pathl=%d\n", *pathc, *pathl);
      etap_path=1;
    }
    return cout2; 
  }
}


int coutNonRecursif(int dc, int dl, int ac, int al, int *pathc, int *pathl)
{
  int obsctacle_colonne, obstacle_ligne, dummy_c, dummy_l;
  int res1, res2;
  int tribordc1, tribordl1, babordc1, babordl1;
  int tribordc2, tribordl2, babordc2, babordl2;
  int pathc1, pathl1, pathc2, pathl2;
  int cout = INFINI, coutmp, cout1=INFINI, cout2=INFINI, cout3=INFINI, cout4=INFINI;
  
  cout = Bresenham(dc,dl,ac,al,&obsctacle_colonne,&obstacle_ligne);
  
  if( cout > 0) 
  {
//    printf("ligne directe\n");
    *pathc = ac; 
    *pathl = al;
    return cout; 
  }
  res1 = BisonFute(dc, dl, ac, al, obsctacle_colonne, obstacle_ligne, &tribordc1, &tribordl1, &babordc1, &babordl1); 
  if(res1==0) 
  {
    //printf("pas de bulletin\n");
    return INFINI; 
  }
  else
  {
//     printf("obstacle en (%d,%d) : sol (%d,%d) ou (%d,%d)\n",obsctacle_colonne, obstacle_ligne, tribordc1, tribordl1, babordc1, babordl1); 
    cout = Bresenham(dc, dl, tribordc1, tribordl1, &obsctacle_colonne,&obstacle_ligne);
    if( cout > 0) 
    {
      cout1 = cout;
//      printf("ligne directe pour cout1 : de (%d,%d) => (%d,%d)\n", dc, dl, tribordc1, tribordl1);
      pathc1 = tribordc1; 
      pathl1 = tribordl1;

    }
    else 
    {

      res2 = BisonFute(dc, dl, tribordc1, tribordl1, obsctacle_colonne, obstacle_ligne, &tribordc2, &tribordl2, &babordc2, &babordl2); 
      if(res2==0) 
      {
// 	printf("pas de bulletin\n");
	cout1 = INFINI; 
      }
      else
      {
// 	printf("obstacle en (%d,%d) : sol1 (%d,%d) ou (%d,%d)\n",obsctacle_colonne, obstacle_ligne, tribordc2, tribordl2, babordc2, babordl2); 
	coutmp = Bresenham(dc, dl, tribordc2, tribordl2, &dummy_c, &dummy_l);
	if( coutmp > 0) 
	{
	    cout = Bresenham(tribordc2, tribordl2, tribordc1, tribordl1, &dummy_c, &dummy_l);
	    if( cout > 0 )
	    {  
	      pathc1 = tribordc2; 
	      pathl1 = tribordl2; 
	      cout1 = cout + coutmp;
	    }
	    else cout1 = INFINI;
	}
	else cout1 = INFINI; //besoin de differencier avec INFINI?
	    
	if( res2 == 2)
	{
	  coutmp = Bresenham(dc, dl, babordc2, babordl2, &dummy_c, &dummy_l);
	  if (coutmp > 0) 
	  {
	    cout = Bresenham(babordc2, babordl2, tribordc1, tribordl1, &dummy_c, &dummy_l);
	    if( (cout > 0 ) && ((coutmp + cout) < cout1) )
	    {
	      cout1 = cout + coutmp;
	      pathc1 = babordc2; 
	      pathl1 = babordl2; 
	    }
	  }         
	}
      } 
    }
    if(res1 == 2)
    {
      coutmp = Bresenham(dc, dl, babordc1, babordl1, &obsctacle_colonne,&obstacle_ligne);
      if( coutmp > 0) 
      {
// 	printf("ligne directe pour cout2 : (%d,%d) => (%d,%d)\n", dc, dl, babordc1, babordl1);
	cout2 = coutmp;
	pathc2 = babordc1; 
	pathl2 = babordl1;

      }
      else 
      {
	  res2 = BisonFute(dc, dl, babordc1, babordl1, obsctacle_colonne, obstacle_ligne, &tribordc2, &tribordl2, &babordc2, &babordl2); 
	  if(res2==0) 
	  {
	    //printf("pas de bulletin\n");
	    cout2 = INFINI; 
	  }
	  else
	  {
// 	    printf("obstacle en (%d,%d) : sol2 (%d,%d) ou (%d,%d)\n",obsctacle_colonne, obstacle_ligne, tribordc2, tribordl2, babordc2, babordl2); 
	    coutmp = Bresenham(dc, dl, tribordc2, tribordl2, &dummy_c, &dummy_l);
	    if( coutmp > 0) 
	    {
		cout = Bresenham(tribordc2, tribordl2, babordc1, babordl1, &dummy_c, &dummy_l);
		if( cout > 0 )
		{  
		  pathc2 = tribordc2; 
		  pathl2 = tribordl2; 
		  cout2 = cout + coutmp;
		}
		else cout2 = INFINI;
	    } 
	  }
	  if( res2 == 2)
	  {
	    coutmp = Bresenham(dc, dl, babordc2, babordl2, &dummy_c, &dummy_l);
	    if (coutmp > 0) 
	    {
	      cout = Bresenham(babordc2, babordl2, babordc1, babordl1, &dummy_c, &dummy_l);
	      if( (cout > 0 ) && ((coutmp + cout) < cout2) )
	      {
		cout2 = cout + coutmp;
		pathc2 = babordc2; 
		pathl2 = babordl2; 
	      }
	    }         
	  }
	      		      
      }  
    } 
   
    //seconde motie de l arbre
    cout = Bresenham(ac, al, tribordc1, tribordl1, &obsctacle_colonne,&obstacle_ligne);
    if( cout > 0) 
    {
//      printf("ligne directe pour cout3 : (%d,%d) => (%d,%d)\n", ac, al,tribordc1, tribordl1);
      cout3 = cout;
/*      if(dc>ac) //attention on calcule à l envers puis qu on part de l arrivé pour aller au départ
      {
	    pathc1 = tribordc1; 
	    pathl1 = tribordl1; 
      }
*/
    }
    else 
    {

	  res2 = BisonFute(ac, al, tribordc1, tribordl1, obsctacle_colonne, obstacle_ligne, &tribordc2, &tribordl2, &babordc2, &babordl2); 
	  if(res2==0) 
	  {
	    //printf("pas de bulletin\n");
	    cout3 = INFINI; 
	  }
	  else
	  {
// 	   printf("obstacle en (%d,%d) : sol3 (%d,%d) ou (%d,%d)\n",obsctacle_colonne, obstacle_ligne, tribordc2, tribordl2, babordc2, babordl2); 
	    cout3 = Bresenham(ac, al, tribordc2, tribordl2, &dummy_c, &dummy_l);
	    if( cout3 > 0) 
	    {
	        cout = Bresenham(tribordc2, tribordl2, tribordc1, tribordl1, &dummy_c, &dummy_l);
		if( cout > 0 )
		{  
		  cout3 += cout;
/*		  if(dc>ac) //attention on calcule à l envers puis qu on part de l arrivé pour aller au départ
		  {
		    pathc1 = tribordc2; 
		    pathl1 = tribordl2; 
		  }
*/
		}
		else cout1 = INFINI;
	    }
	    else cout3 = INFINI; //besoin de differencier avec INFINI?
		
	    if( res2 == 2)
	    {
	      coutmp = Bresenham(ac, al, babordc2, babordl2, &dummy_c, &dummy_l);
	      if (coutmp > 0) 
	      {
		cout = Bresenham(babordc2, babordl2, tribordc1, tribordl1, &dummy_c, &dummy_l);
		if( (cout > 0 ) && ((coutmp + cout) < cout3) )
		{
		  cout3 = cout + coutmp;
/*		  if(dc>ac) //attention on calcule à l envers puis qu on part de l arrivé pour aller au départ
		  {
		    pathc1 = babordc2; 
		    pathl1 = babordl2; 
		  }
*/
		}
	      }         
	    }
	  } 
    }
    if(res1 == 2)
    {
      coutmp = Bresenham(ac, al, babordc1, babordl1, &obsctacle_colonne,&obstacle_ligne);
      if( coutmp > 0) 
      {
// 	printf("ligne directe pour cout4 : (%d,%d) => (%d,%d)\n", ac, al, babordc1, babordl1);
	cout4 = coutmp;
/*	if(dc>ac) //attention on calcule à l envers puis qu on part de l arrivé pour aller au départ
	{
	    pathc2 = babordc1; 
	    pathl2 = babordl1; 
	}
*/

      }
      else 
      {
	      res2 = BisonFute(ac, al, babordc1, babordl1, obsctacle_colonne, obstacle_ligne, &tribordc2, &tribordl2, &babordc2, &babordl2); 
	      if(res2==0) 
	      {
		//printf("pas de bulletin\n");
		cout4 = INFINI; 
	      }
	      else
	      {
// 		printf("obstacle en (%d,%d) : sol4 (%d,%d) ou (%d,%d)\n",obsctacle_colonne, obstacle_ligne, tribordc2, tribordl2, babordc2, babordl2); 
		coutmp = Bresenham(ac, al, tribordc2, tribordl2, &dummy_c, &dummy_l);
		if( coutmp > 0) 
		{
		    cout = Bresenham(tribordc2, tribordl2, babordc1, babordl1, &dummy_c, &dummy_l);
		    if( cout > 0 )
		    {  
		      cout4 = cout + coutmp;
/*		      if(dc>ac) //attention on calcule à l envers puis qu on part de l arrivé pour aller au départ
		      {
			pathc2 = tribordc2; 
			pathl2 = tribordl2; 
		      }
*/
		    }
		    else cout4 = INFINI;
		}   
	      }
	      if( res2 == 2)
	      {
		coutmp = Bresenham(ac, al, babordc2, babordl2, &dummy_c, &dummy_l);
		if (coutmp > 0) 
		{
		  cout = Bresenham(babordc2, babordl2, babordc1, babordl1, &dummy_c, &dummy_l);
		  if( (cout > 0 ) && ((coutmp + cout) < cout4) )
		  {
		    cout4 = cout + coutmp;
/*		    if(dc>ac) //attention on calcule à l envers puis qu on part de l arrivé pour aller au départ
		    {
		      pathc2 = babordc2; 
		      pathl2 = babordl2; 
		    }
*/
		  }
		}         
	      }
	      		      
      }  
    }
//     printf("cout1=%d, cout2=%d, cout3=%d, cout4=%d\n",cout1,cout2,cout3,cout4); 
    if( (cout1+cout3) < (cout2+cout4))
    {
// 	printf("solution impair\n");
        *pathc = pathc1; 
	*pathl = pathl1;
	cout = cout1 + cout3;
	return cout; 
    }
    else
    {
// 	printf("solution pair\n");
        *pathc = pathc2; 
	*pathl = pathl2;
	cout = cout2 + cout4;
	return cout; 
    }
  }
}

//table
void init_table(void)
{
  int i;
  for(i=0;i<NB_CASE;i++) table[i]=0;
  table[9-1]=PION;
  table[17-1]=PION;
  table[25-1]=PION;
  table[33-1]=PION;
  table[41-1]=PION;
  table[16-1]=PION;
  table[24-1]=PION;
  table[32-1]=PION;
  table[40-1]=PION;
  table[48-1]=PION;

}



void update_table(int numCase, char objet)
{
	int i;
	if((objet==ROBOT_ADV)||(objet==ROBOT_ATL))
	{ 
		for(i=0;i<NB_CASE;i++) 
		{
			if(table[i]==objet) 
				table[i]=RIEN;
		}
	}
	table[numCase-1]=objet;
}


// outils

unsigned char caseToVecteur(int numeroCase, int *colonne, int *ligne)
{
    unsigned char result = 1;
    int tmpc, tmpl;
    
     if( (numeroCase < 0) || (numeroCase > NB_CASE) )
     {
      result = 0;
      return result;
    }
    numeroCase--;
    tmpc = numeroCase % COLONNE;
    tmpl = numeroCase / COLONNE;
        
    //printf("tmpc=%d tmpl=%d\n", tmpc, tmpl);
    
    if (tmpc < 4) *colonne = (350 * ((tmpc * (-1)) + 3)) + 175;
      else *colonne = (350 * ( (tmpc * (-1)) + 4)) - 175;
      
    if( tmpl < 3)  *ligne =  (350 * (tmpl - (LIGNE - 1) )) - 175;
    *ligne = (350 * (tmpl - 3 )) + 175;
    
    return result;
}

unsigned char vecteurToCase(int colonne, int ligne, int *numeroCase)
{
    unsigned char result = 1;
    int tmpc, resc, tmpl, resl;
        
    if( (colonne < -1500) || (colonne >= 1500) || (ligne < -1050) || (ligne >= 1050) )
    {
      result = 0;
      return result;
    }
    
    tmpc = (colonne / 350);
    resc = (colonne % 350);
    if(resc > 0)   tmpc = -1 * (tmpc - 3);
    else tmpc = (tmpc * -1) + 4;
       
    tmpl = (ligne / 350);
    resl = (ligne % 350);
   
    if(resl > 0) tmpl += 3;
    else tmpl += 2;
       
    *numeroCase = tmpc + 1 + (tmpl * COLONNE);
    
    return result;
}
