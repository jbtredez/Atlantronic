#include "table.h"
#include "kernel/color.h"

#define ARRAY_SIZE(a)               (sizeof(a)/sizeof(a[0]))

//!< contour de la table
const struct Vect2 tableContour[] =
{
	Vect2( -1500, -460), Vect2( -1500, 1000),
	Vect2( 1500, 1000 ), Vect2( 1500, -460),
	Vect2( 1500,				-460 ),
	Vect2( 1394.6512261113,		-470.3759485823 ),
	Vect2( 1293.3509465229,		-501.1050524439 ),
	Vect2( 1199.9920741694,		-551.0064093566 ),
	Vect2( 1118.1623381593,		-618.1623381593 ),
	Vect2( 1051.0064093566,		-699.9920741694 ),
	Vect2( 1001.1050524439,		-793.3509465229 ),
	Vect2( 970.3759485823,		-894.6512261113 ),
	Vect2( 960,					-1000 ),


	Vect2( -960,				-1000 ),
	Vect2( -970.3759485823,		-894.6512261113 ),
	Vect2( -1001.1050524439,	-793.3509465229 ),
	Vect2( -1051.0064093566,	-699.9920741694 ),
	Vect2( -1118.1623381593,	-618.1623381593 ),
	Vect2( -1199.9920741694,	-551.0064093566 ),
	Vect2( -1293.3509465229,	-501.1050524439 ),
	Vect2( -1394.6512261113,	-470.3759485823 ),
	Vect2( -1500,				-460 ),

};

const struct Vect2 centralMoonBase[] =
{
	Vect2( -200, -1000 ),
	Vect2( -181.08, -915.09 ),
	Vect2( -613.77, -482.4 ),
	Vect2( -517.6, -386.23 ),
	Vect2( -84.91, -818.92 ),
	Vect2( -68, -811.91 ),
	Vect2( -68, -200 ),

	Vect2( 68, -200 ),
	Vect2( 68, -811.91 ),
	Vect2( 84.91, -818.92 ),
	Vect2( 517.6, -386.23 ),
	Vect2( 613.77, -482.4 ),
	Vect2( 181.08, -915.09 ),
	Vect2( 200, -1000 ),
};

const struct Vect2 blueMoonBase[] =
{
	Vect2( -1500, 322 ),
	Vect2( -1392, 322 ),
	Vect2( -1392, -172 ),
	Vect2( -1500, -172 ),
};

const struct Vect2 yellowMoonBase[] =
{
	Vect2( 1500, 322 ),
	Vect2( 1392, 322 ),
	Vect2( 1392, -172 ),
	Vect2( 1500, -172 ),
};

const struct Vect2 blueStartCrater[] =
{
	Vect2( -965, 526.4 ),
	Vect2( -850, 592.79 ),
	Vect2( -735, 526.4 ),
	Vect2( -735, 393.6 ),
	Vect2( -850, 327.21 ),
	Vect2( -965, 393.6 ),
	Vect2( -965, 526.4 ),
};

const struct Vect2 yellowStartCrater[] =
{
	Vect2( 965, 526.4 ),
	Vect2( 850, 592.79 ),
	Vect2( 735, 526.4 ),
	Vect2( 735, 393.6 ),
	Vect2( 850, 327.21 ),
	Vect2( 965, 393.6 ),
	Vect2( 965, 526.4 ),
};

const struct Vect2 blueBaseCrater[] =
	{
	Vect2( -545, -803.6 ),
	Vect2( -430, -737.21 ),
	Vect2( -315, -803.6 ),
	Vect2( -315, -936.4 ),
	Vect2( -430, -1002.79 ),
	Vect2( -545, -936.4 ),
	Vect2( -545, -803.6 ),
};

const struct Vect2 yellowBaseCrater[] =
{
	Vect2( 545, -803.6 ),
	Vect2( 430, -737.21 ),
	Vect2( 315, -803.6 ),
	Vect2( 315, -936.4 ),
	Vect2( 430, -1002.79 ),
	Vect2( 545, -936.4 ),
	Vect2( 545, -803.6 ),
};

const struct Vect2 tableBlueStart[] =
{
	Vect2( -430, 1000),
	Vect2( -430, 640 ),
	Vect2( -1500, 640 ),
	Vect2( -1500, 1000 ),
};

const struct Vect2 tableYellowStart[] =
{
	Vect2( 430, 1000),
	Vect2( 430, 640 ),
	Vect2( 1500, 640 ),
	Vect2( 1500, 1000 ),
};

struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (Vect2*) tableContour, ARRAY_SIZE(tableContour) },
	{ (Vect2*) centralMoonBase, ARRAY_SIZE(centralMoonBase) },
	{ (Vect2*) blueMoonBase, ARRAY_SIZE(blueMoonBase) },
	{ (Vect2*) yellowMoonBase, ARRAY_SIZE(yellowMoonBase) },
	{ (Vect2*) blueStartCrater, ARRAY_SIZE(blueStartCrater) },
	{ (Vect2*) yellowStartCrater, ARRAY_SIZE(yellowStartCrater) },
	{ (Vect2*) blueBaseCrater, ARRAY_SIZE(blueBaseCrater) },
	{ (Vect2*) yellowBaseCrater, ARRAY_SIZE(yellowBaseCrater) },
	{ (Vect2*) tableBlueStart, 0 },
	{ (Vect2*) tableYellowStart, 0 },
};


void setTableColor(int color)
{
	if( color == COLOR_BLUE )
	{
		table_obj[8].size = 0;
		table_obj[9].size = ARRAY_SIZE(tableYellowStart);
	}
	else if( color == COLOR_YELLOW )
	{
		table_obj[8].size = ARRAY_SIZE(tableBlueStart);
		table_obj[9].size = 0;
	}
	else if( color == COLOR_UNKNOWN )
	{
		table_obj[8].size = 0;
		table_obj[9].size = 0;
	}
	else
	{
		table_obj[8].size = ARRAY_SIZE(tableBlueStart);
		table_obj[9].size = ARRAY_SIZE(tableYellowStart);
	}
}
