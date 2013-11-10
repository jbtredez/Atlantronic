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
	{ 150.000000, 0.000000 },
	{ 142.658477, 46.352550 },
	{ 121.352548, 88.167790 },
	{ 88.167785, 121.352551 },
	{ 46.352544, 142.658479 },
	{ -0.000007, 150.000000 },
	{ -46.352557, 142.658475 },
	{ -88.167781, 121.352554 },
	{ -121.352555, 88.167779 },
	{ -142.658476, 46.352555 },
	{ -150.000000, -0.000013 },
	{ -142.658479, -46.352546 },
	{ -121.352540, -88.167801 },
	{ -88.167760, -121.352570 },
	{ -46.352566, -142.658472 },
	{ 0.000002, -150.000000 },
	{ 46.352569, -142.658471 },
	{ 88.167762, -121.352568 },
	{ 121.352542, -88.167798 },
	{ 142.658480, -46.352542 },
	{ 150.000000, 0.000000 },
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
