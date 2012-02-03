#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

//! @file sin_table_gen.c
//! @brief Algo de generation automatique de la table de sinus en virgule fixe
//! @author Atlantronic

#define TABLE_SIZE      1024
#define BIT               30

int main()
{
	int i;
	int32_t max_e = 0;
	int last = 0;

	printf("int32_t sin_tbl[%d] =\n{", TABLE_SIZE+2);

	for(i = 0; i <= TABLE_SIZE; i++)
	{
		if(i % 8 == 0)
		{
			printf("\n");
		}
		int32_t sx = round(sin(i * M_PI/(2*TABLE_SIZE)) * (1 << BIT));
		printf("%11d,", sx);
		int32_t e = sx - last;
		if(e > max_e)
		{
			max_e = e;
		}
		last = sx;
	}

	printf("%11d,", 0);
	printf("\n};\n// max ecart : %d => %g\n", max_e, max_e / ((float)(1 << BIT)));

	return 0;
}
