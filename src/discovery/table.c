#include "table.h"

#define ARRAY_SIZE(a)               (sizeof(a)/sizeof(a[0]))

//!< contour de la table
const struct vect2 table_contour[] =
{
	{ -1500, -1000}, { -1500, 1000 }, { 1500, 1000 }, { 1500, -1000 }, { -1500, -1000 }
};

//!< zone de depot cote rouge
const struct vect2 table_depot_rouge[] =
{
	{ -400, 1000 }, { -400, 700 }, { -1100, 700 }, { -1100, 1000 }
};

//!< zone de depot cote jaune
const struct vect2 table_depot_jaune[] =
{
	{ 400, 1000 }, { 400, 700 }, { 1100, 700 }, { 1100, 1000 }
};

//!< centre
const struct vect2 table_centre[] =
{
	{ 150.000000, -50.000000 },
	{ 142.658477, -3.647450 },
	{ 121.352548, 38.167790 },
	{ 88.167785, 71.352551 },
	{ 46.352544, 92.658479 },
	{ -0.000007, 100.000000 },
	{ -46.352557, 92.658475 },
	{ -88.167781, 71.352554 },
	{ -121.352555, 38.167779 },
	{ -142.658476, -3.647445 },
	{ -150.000000, -50.000013 },
	{ -142.658479, -96.352546 },
	{ -121.352540, -138.167801 },
	{ -88.167760, -171.352570 },
	{ -46.352566, -192.658472 },
	{ 0.000002, -200.000000 },
	{ 46.352569, -192.658471 },
	{ 88.167762, -171.352568 },
	{ 121.352542, -138.167798 },
	{ 142.658480, -96.352542 },
	{ 150.000000, -50.000000 },
};

//!< depot1
const struct vect2 table_depot1[] =
{
	{ -1250.000000, -1000.000000 },
	{ -1262.235872, -922.745749 },
	{ -1297.745754, -853.053683 },
	{ -1353.053692, -797.745748 },
	{ -1422.745760, -762.235868 },
	{ -1500.000011, -750.000000 },
};

//!< depot2
const struct vect2 table_depot2[] =
{
		{ 1499.999989, -750.000000 },
		{ 1422.745739, -762.235875 },
		{ 1353.053699, -797.745743 },
		{ 1297.745741, -853.053701 },
		{ 1262.235874, -922.745742 },
		{ 1250.000000, -1000.000022 },
};

const struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (struct vect2*) table_contour, ARRAY_SIZE(table_contour) },
	{ (struct vect2*) table_depot_rouge, ARRAY_SIZE(table_depot_rouge) },
	{ (struct vect2*) table_depot_jaune, ARRAY_SIZE(table_depot_jaune) },
	{ (struct vect2*) table_centre, ARRAY_SIZE(table_centre) },
	{ (struct vect2*) table_depot1, ARRAY_SIZE(table_depot1) },
	{ (struct vect2*) table_depot2, ARRAY_SIZE(table_depot2) },
};
