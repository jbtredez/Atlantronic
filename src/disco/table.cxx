#include "table.h"

#define ARRAY_SIZE(a)               (sizeof(a)/sizeof(a[0]))

//!< contour de la table
const struct Vect2 tableContour[] =
{
	Vect2( -1500, -1000), Vect2( -1500, 1000 ),
	Vect2( -700, 1000 ), Vect2( -700, 800 ), Vect2( -678, 800 ), Vect2( -678, 1000 ),
	Vect2( 678, 1000 ), Vect2( 678, 800 ), Vect2( 700, 800 ), Vect2( 700, 1000 ),
	Vect2( 1500, 1000 ), Vect2( 1500, -1000 ),
	Vect2( 572, -1000 ), Vect2( 572, -978 ), Vect2( 550, -978 ), Vect2( 550, -1000 ),
	Vect2( -550, -1000 ), Vect2( -550, -978 ), Vect2( -572, -978 ), Vect2( -572, -1000 ),
	Vect2( -1500, -1000 )
};

const struct Vect2 tableCenter[] =
{
	Vect2(-600, 250), Vect2(600, 250), Vect2(600, 228), Vect2(24, 228),
	Vect2(24, -350), Vect2(-24, -350),
	Vect2(-24, 228), Vect2(-600, 228), Vect2(-600, 250)
};

const struct Vect2 tableCorner1[] =
{
	Vect2( -1250, -1000 ),
	Vect2( -1262.235872, -922.745749 ),
	Vect2( -1297.745754, -853.053683 ),
	Vect2( -1353.053692, -797.745748 ),
	Vect2( -1422.745760, -762.235868 ),
	Vect2( -1500, -750 ),
};

const struct Vect2 tableCorner2[] =
{
	Vect2( 1250, -1000 ),
	Vect2( 1262.235872, -922.745749 ),
	Vect2( 1297.745754, -853.053683 ),
	Vect2( 1353.053692, -797.745748 ),
	Vect2( 1422.745760, -762.235868 ),
	Vect2( 1500, -750 ),
};

const struct Vect2 tableGreenStart[] =
{
	Vect2( 1500, -100 ),
	Vect2( 1200, -100 ),
	Vect2( 1200, 400 ),
	Vect2( 1500, 400 ),
};

const struct Vect2 tablePurpleStart[] =
{
	Vect2( -1500, -100 ),
	Vect2( -1200, -100 ),
	Vect2( -1200, 400 ),
	Vect2( -1500, 400 ),
};

struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (Vect2*) tableContour, ARRAY_SIZE(tableContour) },
	{ (Vect2*) tableCenter, ARRAY_SIZE(tableCenter) },
	{ (Vect2*) tableCorner1, ARRAY_SIZE(tableCorner1) },
	{ (Vect2*) tableCorner2, ARRAY_SIZE(tableCorner2) },
	{ (Vect2*) tableGreenStart, 0 },
	{ (Vect2*) tablePurpleStart, 0 },
};

void setTableColor(int color)
{
	if( color == 1 )
	{
		table_obj[4].size = 0;
		table_obj[5].size = ARRAY_SIZE(tablePurpleStart);
	}
	else if( color == -1 )
	{
		table_obj[4].size = ARRAY_SIZE(tableGreenStart);
		table_obj[5].size = 0;
	}
	else if( color == 0 )
	{
		table_obj[4].size = 0;
		table_obj[5].size = 0;
	}
	else
	{
		table_obj[4].size = ARRAY_SIZE(tableGreenStart);
		table_obj[5].size = ARRAY_SIZE(tablePurpleStart);
	}
}
