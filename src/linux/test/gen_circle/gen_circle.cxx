#include <stdio.h>
#include <math.h>

int main()
{
	int numSeg = 20;
	int rayon = 250;
	float x = 1500;
	float y = -1000;
	for(int i = 0; i < numSeg; i++)
	{
		float theta = i * 2 * M_PI / numSeg;
		printf("{ %f, %f },\n", x + rayon * cos(theta), y + rayon * sin(theta));
	}

	return 0;
}
