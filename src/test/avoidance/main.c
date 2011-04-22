#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "avoidance/macro_fonction.h"

void test_init_table(void)
{
  int i;
  for(i=0;i<NB_CASE;i++) table[i]=0;

}

void afficher_table(void)
{
  int i,j;
  for(i=0;i<LIGNE;i++)
  {
    for(j=0;j<COLONNE;j++) printf("----");
    printf("-");
    printf("\n");
    for(j=0;j<COLONNE;j++) 
    {
      printf("| ");
	if(table[j+(i*COLONNE)]!=0) printf("%c ",table[j+(i*COLONNE)]);
	else printf("  ");
      
    }
    printf("|\n");	
  }
  for(j=0;j<COLONNE;j++) printf("----");
  printf("-");
  printf("\n");
}

#if 0
#define LIGNE 6
#define COLONNE 8
#define NB_CASE LIGNE*COLONNE

#define OBSTACLE 'O'
#define INFINI NB_CASE + LIGNE

char table[NB_CASE];
// #define PROFONDEUR 256
// char best_path[PROFONDEUR];
unsigned char etap_path;

void init_table(void)
{
  int i;
  for(i=0;i<NB_CASE;i++) table[i]=0;
  
/*  for(i=0;i<COLONNE;i++) 
  {
    table[i]='M';
    table[i+(COLONNE*(LIGNE-1))]='M';
  }
  for(i=1;i<(LIGNE-1);i++) 
  {
    table[0+(COLONNE*(i))]='M';
    table[(COLONNE-1)+(COLONNE*(i))]='M';
  }*/  
}

void afficher_table(void)
{
  int i,j;
  for(i=0;i<LIGNE;i++)
  {
    for(j=0;j<COLONNE;j++) printf("----");
    printf("-");
    printf("\n");
    for(j=0;j<COLONNE;j++) 
    {
      printf("| ");
	if(table[j+(i*COLONNE)]!=0) printf("%c ",table[j+(i*COLONNE)]);
	else printf("  ");
      
    }
    printf("|\n");	
  }
  for(j=0;j<COLONNE;j++) printf("----");
  printf("-");
  printf("\n");
}
#endif

/**************************************************************************************/
//renvoi 0 si un obstacl est trouvé sur la route
//renvoie le nombre de case à traverser pour y aller
int Print_Bresenham(int c1, int l1, int c2, int l2, int *oc, int *ol)
{     
  
      int dc, dl, p, endc, endl;
      int colonne, ligne;
      int incl = 1;
      unsigned int compteur = 0;
      
      if( (c1 >= COLONNE) || (c2 >= COLONNE) || (l1 >= LIGNE) || (l2 >= LIGNE) )
      {
	printf("<*************** OUT OF TABLE *******************> $$$\n");
	printf("out : (%d,%d) => (%d,%d)\n",c1, l1,  c2, l2);
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
       table[colonne+(ligne*COLONNE)]='#';
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
			printf("Bresenham CUL DE SAC en (%d,%d)\n", *oc, *ol);
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
 			table[colonne+(ligne*COLONNE)]='#';
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
 			    table[colonne+(ligne*COLONNE)]='#';
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
	      printf("Bresenham obstacle\n");
	      return 0;
	    }
	    compteur++;
 	    table[colonne+(ligne*COLONNE)]='#';
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

 	table[colonne+(ligne*COLONNE)]='#';
	compteur++;
      }
      //printf("compteur=%d\n", compteur);
      return compteur;
}
/*
{     
  
      int dc, dl, p, endc, endl;
      int colonne, ligne;
      int incl = 1;
      unsigned int compteur = 0;
      
      if( (c1 >= COLONNE) || (c2 >= COLONNE) || (l1 >= LIGNE) || (l2 >= LIGNE) )
      {
	printf("<*************** OUT OF TABLE *******************>\n");
	return INFINI;
      }
	
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
      
      
      printf("c1=%d l1=%d c2=%d l2=%d incl=%d\n",c1, l1, c2, l2, incl);
//       printf("colonne=%d, ligne=%d, endc=%d, endl=%d\n", colonne,ligne,endc, endl);
	
      
      if(table[colonne+(ligne*COLONNE)]==OBSTACLE) 
      {
	*oc = colonne;
	*ol = ligne;
	return 0;
      }
       table[colonne+(ligne*COLONNE)]='#';
//        printf("colonne=%d ligne=%d \n", colonne, ligne);
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
		  if( (table[colonne+(ligne*COLONNE)]!=OBSTACLE) && (table[(colonne-1)+((ligne+incl)*COLONNE)]!=OBSTACLE) ) 
		  {
		        ligne = ligne + incl;
		  }
		  else
// 		  //la diago n est pas accessible, quel obstacle est à coté?
// 		  {
// 		    if( (table[colonne+(ligne*COLONNE)]==OBSTACLE) && (table[(colonne-1)+((ligne+incl)*COLONNE)]==OBSTACLE) ) 
// 		    {
// 			//cul de sac
// 			*oc = colonne - 1;
// 			*ol = ligne;
// 			printf("Bresenham CUL DE SAC\n");
// 			return 0;
// 		    }
// 		    else 
// 		    {
// 		      printf("detour\n");
// 		      if (table[(colonne-1)+((ligne+incl)*COLONNE)]==OBSTACLE) 
// 		      {
// 			//colonne += -1;
// 			//ligne += incl;
// // 			table[colonne+(ligne*COLONNE)]='X';
// 			//compteur++;
// 			//colonne += 1;
// 			*oc = colonne - 1;
// 			*ol = ligne + incl;
// 			printf("Bresenham obstacle diag diff\n");
// 			return 0;
// 		      }
// 		      else
// 		      {
// // 			    table[colonne+(ligne*COLONNE)]='X';
// 			    //compteur++;
// 			    //ligne += incl;
// 			    printf("Bresenham obstacle diag diff\n");
// 			    *ol = ligne + incl;			
// 			    *oc = colonne;
// 			    return 0;
// 		      } 
// 		    }
// 		  }
		  ligne += incl;
                  p = p + 2 * (dl - dc);
            }
            
//  	    printf("colonne=%d ligne=%d dans le while\n", colonne, ligne);
	    if(table[colonne+(ligne*COLONNE)]==OBSTACLE) 
	    {
	      *oc = colonne;
	      *ol = ligne;
	      printf("Bresenham obstacle classique\n");
	      return 0;
	    }
	    compteur++;
 	    table[colonne+(ligne*COLONNE)]='#';
      }
      
//       printf("pipo colonne=%d ligne=%d \n", colonne, ligne);
//       printf("while (ligne != endl) => %d vs %d\n", ligne, endl); 
      while (ligne != endl) 
      {
	ligne = ligne + incl;
	if( (ligne>=LIGNE)|| (ligne < 0 ))
	{
	  printf("--------------- LIGNE MIN or MAX REACHED --------------------\n");
	  return INFINI;
	}
//  	printf("plop colonne=%d ligne=%d \n", colonne, ligne);
	if(table[colonne+(ligne*COLONNE)]==OBSTACLE) 
	{
	    *oc = colonne;
	    *ol = ligne;
	    printf("Bresenham obstacle dans col\n");
	    return 0;
	}

 	table[colonne+(ligne*COLONNE)]='#';
	compteur++;
      }
//        printf("compteur=%d\n", compteur);
      return compteur;
}
*/
void test5()
{
  //unsigned int bres;
  //int obsctacle_colonne, obstacle_ligne;
  //int tribordc, tribordl, babordc, babordl;
  int pathc=0, pathl=0;
  int tmpc, tmpl;
  int cmp=0, cout1=0;
//   int res;
  
  int dc=7;
  int dl=5;
  int ac=7;
  int al=3;
  
  int ddc = dc;
  int ddl = dl;
/*  int dc=2;
  int dl=5;
  int ac=7;
  int al=1;
*/
  test_init_table();
  
  table[7+(4*COLONNE)]=OBSTACLE;
  table[6+(3*COLONNE)]=OBSTACLE;
  

  
  
//   init_best_path();
  
 
   while ( ((ac != dc) || (al != dl)) && (cmp<5) )
   {
     printf("Trajet restant : (%d,%d) => (%d,%d)\n", dc,dl, ac,al);
     cout1 = coutNonRecursif(dc,dl,ac,al,&pathc,&pathl);
     if(cout1>=INFINI)
     {
       printf("Infini : (%d,%d) pour (%d,%d), cout=%d\n", dc,dl,pathc,pathl, cout1);
       break;
     }
      printf("depart en (%d,%d) pour (%d,%d), cout=%d\n", dc,dl,pathc,pathl, cout1);
     Print_Bresenham(dc,dl, pathc,pathl, &tmpc, &tmpl);

     dc = pathc;
     dl = pathl;
     cmp++;
   }
  table[ddc+(ddl*COLONNE)]='D';
  table[ac+(al*COLONNE)]='A';
  afficher_table();
}


void test4(void)
{
  int pathc=0, pathl=0;
  int tmpc, tmpl;
  int cmp=0, cout1=0;
  int out=0;
  int total=0;
  int i,j,k,l,m,n,o,p;
//   int res;
  int dc=6;
  int dl=0;
  int ac=2;
  int al=5;

  //test_init_table();

  //table[5+(3*COLONNE)]=OBSTACLE;

for(i=0;i<COLONNE;i++)
{
  for(j=0;j<LIGNE;j++)
  {
    for(k=0;k<COLONNE;k++)
    {
      for(l=0;l<LIGNE;l++)
      {
	
	for(m=0;m<COLONNE;m++)
	{
	   for(n=0;n<LIGNE;n++)
	   {	
	     for(o=0;o<COLONNE;o++)
	     {
     	      for(p=0;p<LIGNE;p++)
              {

        	test_init_table();

	        dc=i;
	        dl=j;
	        ac=k;
	        al=l;
	        cmp=0;
	       	table[m+(n*COLONNE)]=OBSTACLE;
		table[o+(p*COLONNE)]=OBSTACLE;

		total++;

	    	if( ((i==m) && (j==n)) || ((l==n)&&(k==m)) )
			continue;
                if( ((i==o) && (j==p)) || ((l==p)&&(k==o)) )
                        continue;
		while ( ((ac != dc) || (al != dl)) && (cmp<5) )
		{
		     //printf("Trajet restant : (%d,%d) => (%d,%d)\n", dc,dl, ac,al);
		     cout1 = coutNonRecursif(dc,dl,ac,al,&pathc,&pathl);
		     if(cout1 >= INFINI )
		     {
			printf("Infini : (%d,%d) pour (%d,%d), cout=%d\n", i,j,pathc,pathl, cout1);
		 	cmp = 6;
		     }
		     else Print_Bresenham(dc,dl, pathc,pathl, &tmpc, &tmpl);

		     dc = pathc;
		     dl = pathl;
		     cmp++;
        
      
	    	   }
		   if(cmp==5)  printf("Deadlock : (%d,%d) pour (%d,%d), cout=%d\n", i,j,pathc,pathl, cout1);
		   if(cmp>4) 
		   {
			table[i+(j*COLONNE)]='D';
	                table[k+(l*COLONNE)]='A';
			afficher_table();
			/*if(out==500) 
			{
				printf("total cas de blocage %d\n",out);
				exit(1);
			}*/
			out++;
		   }
	      }
	    }
	  }
	}
      }
    }
  }
}

printf("total cas de blocage %d/%d\n",out,total);
}
void test3(void)
{
  //unsigned int bres;
  //int obsctacle_colonne, obstacle_ligne;
  //int tribordc, tribordl, babordc, babordl;
  int pathc=0, pathl=0;
  int tmpc, tmpl;
  int cmp=0, cout1=0;
//   int res;
  
  int dc=6;
  int dl=0;
  int ac=2;
  int al=5;

/*  int dc=2;
  int dl=5;
  int ac=7;
  int al=1;
*/
  test_init_table();
  
  table[5+(3*COLONNE)]=OBSTACLE;

  //table[3+(4*COLONNE)]=OBSTACLE;
  
  table[4+(1*COLONNE)]=OBSTACLE;
   table[5+(4*COLONNE)]=OBSTACLE;
  table[5+(5*COLONNE)]=OBSTACLE;
  table[3+(4*COLONNE)]=OBSTACLE;
//   table[2+(3*COLONNE)]=OBSTACLE;
  //  table[7+(2*COLONNE)]=OBSTACLE;

  
  
//   init_best_path();
  
 
   while ( ((ac != dc) || (al != dl)) && (cmp<5) )
   {
     printf("Trajet restant : (%d,%d) => (%d,%d)\n", dc,dl, ac,al);
     cout1 = coutNonRecursif(dc,dl,ac,al,&pathc,&pathl);
     printf("depart en (%d,%d) pour (%d,%d), cout=%d\n", dc,dl,pathc,pathl, cout1);
     Print_Bresenham(dc,dl, pathc,pathl, &tmpc, &tmpl);
     
     dc = pathc;
     dl = pathl;
     cmp++;
   }
  
  afficher_table();
}





void test2(void)
{
  unsigned int bres;
  int obsctacle_colonne, obstacle_ligne;
  int tribordc, tribordl, babordc, babordl;
  int cout1, cout2;
  
  int dc=1;
  int dl=3;
  int ac=7;
  int al=5;

/*  int dc=7;
  int dl=5;
  int ac=1;
  int al=3;
*/
  
  test_init_table();
  table[4+(4*COLONNE)]=OBSTACLE;
  bres=Bresenham(dc,dl,ac,al,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else 
  {
    printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
    if(!BisonFute(dc, dl, ac, al, obsctacle_colonne, obstacle_ligne, &tribordc, &tribordl, &babordc, &babordl)) 
      printf("aucun bulletin aujourd hui\n");
    
    table[(tribordc)+(tribordl*COLONNE)]='B'; 
    table[babordc+(babordl*COLONNE)]='B'; 
    
    cout1=Bresenham(dc,dl,tribordc,tribordl,&obsctacle_colonne,&obstacle_ligne);
    cout1+=Bresenham(tribordc,tribordl,ac, al, &obsctacle_colonne,&obstacle_ligne);
    printf("chemin 1 cout=%d pour c=%d et l=%d\n",cout1,tribordc,tribordl);
    
    cout2=Bresenham(dc,dl,babordc,babordl,&obsctacle_colonne,&obstacle_ligne);
    cout2+=Bresenham(babordc,babordl,ac, al, &obsctacle_colonne,&obstacle_ligne);
    printf("chemin 2 cout=%d pour c=%d et l=%d\n",cout2,babordc,babordl);
       
  }
  afficher_table();
  
  dc=4;
  dl=1;
  ac=4;
  al=5;
  
  test_init_table();
  table[4+(4*COLONNE)]=OBSTACLE;
  bres=Bresenham(dc,dl,ac,al,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else 
  {
    printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
    if(!BisonFute(dc, dl, ac, al, obsctacle_colonne, obstacle_ligne, &tribordc, &tribordl, &babordc, &babordl)) 
      printf("aucun bulletin aujourd hui\n");
    
    table[(tribordc)+(tribordl*COLONNE)]='B'; 
    table[babordc+(babordl*COLONNE)]='B'; 
    
    cout1=Bresenham(dc,dl,tribordc,tribordl,&obsctacle_colonne,&obstacle_ligne);
    cout1+=Bresenham(tribordc,tribordl,ac, al, &obsctacle_colonne,&obstacle_ligne);
    printf("chemin 1 cout=%d pour c=%d et l=%d\n",cout1,tribordc,tribordl);

    cout2=Bresenham(dc,dl,babordc,babordl,&obsctacle_colonne,&obstacle_ligne);
    cout2+=Bresenham(babordc,babordl,ac, al, &obsctacle_colonne,&obstacle_ligne);
    printf("chemin 2 cout=%d pour c=%d et l=%d\n",cout2,babordc,babordl);
       
  }
  afficher_table();
  
  
  dc=5;
  dl=4;
  ac=1;
  al=4;
  
  test_init_table();
  table[4+(4*COLONNE)]=OBSTACLE;
  bres=Bresenham(dc,dl,ac,al,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else 
  {
    printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
    if(!BisonFute(dc, dl, ac, al, obsctacle_colonne, obstacle_ligne, &tribordc, &tribordl, &babordc, &babordl)) 
      printf("aucun bulletin aujourd hui\n");
    
    table[(tribordc)+(tribordl*COLONNE)]='B'; 
    table[babordc+(babordl*COLONNE)]='B'; 
    
    cout1=Bresenham(dc,dl,tribordc,tribordl,&obsctacle_colonne,&obstacle_ligne);
    cout1+=Bresenham(tribordc,tribordl,ac, al, &obsctacle_colonne,&obstacle_ligne);
    printf("chemin 1 cout=%d pour c=%d et l=%d\n",cout1,tribordc,tribordl);

    cout2=Bresenham(dc,dl,babordc,babordl,&obsctacle_colonne,&obstacle_ligne);
    cout2+=Bresenham(babordc,babordl,ac, al, &obsctacle_colonne,&obstacle_ligne);
    printf("chemin 2 cout=%d pour c=%d et l=%d\n",cout2,babordc,babordl);
       
  }
  afficher_table();
  
  dc=7;
  dl=1;
  ac=2;
  al=5;
  
  test_init_table();
  table[4+(3*COLONNE)]=OBSTACLE;
  bres=Bresenham(dc,dl,ac,al,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else 
  {
    printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
    if(!BisonFute(dc, dl, ac, al, obsctacle_colonne, obstacle_ligne, &tribordc, &tribordl, &babordc, &babordl)) 
      printf("aucun bulletin aujourd hui\n");
    
    table[(tribordc)+(tribordl*COLONNE)]='B'; 
    table[babordc+(babordl*COLONNE)]='B'; 
    
    cout1=Bresenham(dc,dl,tribordc,tribordl,&obsctacle_colonne,&obstacle_ligne);
    cout1+=Bresenham(tribordc,tribordl,ac, al, &obsctacle_colonne,&obstacle_ligne);
    printf("chemin 1 cout=%d pour c=%d et l=%d\n",cout1,tribordc,tribordl);

    cout2=Bresenham(dc,dl,babordc,babordl,&obsctacle_colonne,&obstacle_ligne);
    cout2+=Bresenham(babordc,babordl,ac, al, &obsctacle_colonne,&obstacle_ligne);
    printf("chemin 2 cout=%d pour c=%d et l=%d\n",cout2,babordc,babordl);
       
  }
  afficher_table();
  
}

void test1(void)
{
  unsigned int bres;
  int obsctacle_colonne, obstacle_ligne;
  test_init_table();
  
  bres=Bresenham(1,3,7,5,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
  afficher_table();
  test_init_table();
  bres=Bresenham(7,5,1,3,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
  afficher_table();
  
  test_init_table();
  table[4+(4*COLONNE)]=OBSTACLE;
  bres=Bresenham(1,3,7,5,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
  afficher_table();
  
  test_init_table();
  bres=Bresenham(3,6,6,2,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
  afficher_table();
  
  test_init_table();
  bres=Bresenham(2,5,5,2,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
  afficher_table();
  
  test_init_table();
  bres=Bresenham(5,2,2,5,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
  afficher_table();
 
  test_init_table();
  bres=Bresenham(4,1,4,6,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
  afficher_table();
  
  test_init_table();
  bres=Bresenham(4,6,4,1,&obsctacle_colonne,&obstacle_ligne);
  if(bres) printf("route libre!! cout=%d\n",bres);
  else printf("bouchon sur la case colonne=%d et ligne=%d\n",obsctacle_colonne,obstacle_ligne);
  afficher_table();
}

int main()
{
  test3();	
  return 0;
}
