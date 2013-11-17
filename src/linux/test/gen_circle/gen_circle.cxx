#include <stdio.h>
#include <math.h>

int main()
{
	int numSeg = 20;
	int rayon = 150;
	float x = 0;
	float y = -50;
	for(int i = 0; i < numSeg; i++)
	{
		float theta = i * 2 * M_PI / numSeg;
		printf("{ %f, %f },\n", x + rayon * cos(theta), y + rayon * sin(theta));
	}

	return 0;
}
