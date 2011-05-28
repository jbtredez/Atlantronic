#include <stdlib.h>
#include <stdio.h>
#include "avoidance/macro_fonction.h" 

int main()
{
  int x, y, i;
  for(i=1; i<NB_CASE;i++)
  {
    
      caseToVecteur(i, &x, &y);
      printf("case n %d en (%d,%d)\n", i, x,y);
  }
return 0;
}
