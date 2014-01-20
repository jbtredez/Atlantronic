#include "table.h"

#define ARRAY_SIZE(a)               (sizeof(a)/sizeof(a[0]))

//!< contour de la table
const struct vect2 table_contour[] =
{
	vect2( -1500, -1000), vect2( -1500, 1000 ), vect2( 1500, 1000 ), vect2( 1500, -1000 ), vect2( -1500, -1000 )
};

//!< zone de depot cote rouge
const struct vect2 table_depot_rouge[] =
{
	vect2( -400, 1000 ), vect2( -400, 700 ), vect2( -1100, 700 ), vect2( -1100, 1000 )
};

//!< zone de depot cote jaune
const struct vect2 table_depot_jaune[] =
{
	vect2( 400, 1000 ), vect2( 400, 700 ), vect2( 1100, 700 ), vect2( 1100, 1000 )
};

//!< centre
const struct vect2 table_centre[] =
{
	vect2( 150.000000, -50.000000 ),
	vect2( 142.658477, -3.647450 ),
	vect2( 121.352548, 38.167790 ),
	vect2( 88.167785, 71.352551 ),
	vect2( 46.352544, 92.658479 ),
	vect2( -0.000007, 100.000000 ),
	vect2( -46.352557, 92.658475 ),
	vect2( -88.167781, 71.352554 ),
	vect2( -121.352555, 38.167779 ),
	vect2( -142.658476, -3.647445 ),
	vect2( -150.000000, -50.000013 ),
	vect2( -142.658479, -96.352546 ),
	vect2( -121.352540, -138.167801 ),
	vect2( -88.167760, -171.352570 ),
	vect2( -46.352566, -192.658472 ),
	vect2( 0.000002, -200.000000 ),
	vect2( 46.352569, -192.658471 ),
	vect2( 88.167762, -171.352568 ),
	vect2( 121.352542, -138.167798 ),
	vect2( 142.658480, -96.352542 ),
	vect2( 150.000000, -50.000000 ),
};

//!< depot1
const struct vect2 table_depot1[] =
{
	vect2( -1250.000000, -1000.000000 ),
	vect2( -1262.235872, -922.745749 ),
	vect2( -1297.745754, -853.053683 ),
	vect2( -1353.053692, -797.745748 ),
	vect2( -1422.745760, -762.235868 ),
	vect2( -1500.000011, -750.000000 ),
};

//!< depot2
const struct vect2 table_depot2[] =
{
		vect2( 1499.999989, -750.000000 ),
		vect2( 1422.745739, -762.235875 ),
		vect2( 1353.053699, -797.745743 ),
		vect2( 1297.745741, -853.053701 ),
		vect2( 1262.235874, -922.745742 ),
		vect2( 1250.000000, -1000.000022 ),
};

const struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (vect2*) table_contour, ARRAY_SIZE(table_contour) },
	{ (vect2*) table_depot_rouge, ARRAY_SIZE(table_depot_rouge) },
	{ (vect2*) table_depot_jaune, ARRAY_SIZE(table_depot_jaune) },
	{ (vect2*) table_centre, ARRAY_SIZE(table_centre) },
	{ (vect2*) table_depot1, ARRAY_SIZE(table_depot1) },
	{ (vect2*) table_depot2, ARRAY_SIZE(table_depot2) },
};
