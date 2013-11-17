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
	{ 250.000000, -50.000000 },
	{ 237.764128, 27.254251 },
	{ 202.254246, 96.946317 },
	{ 146.946308, 152.254252 },
	{ 77.254240, 187.764132 },
	{ -0.000011, 200.000000 },
	{ -77.254261, 187.764125 },
	{ -146.946301, 152.254257 },
	{ -202.254259, 96.946299 },
	{ -237.764126, 27.254258 },
	{ -250.000000, -50.000022 },
	{ -237.764131, -127.254243 },
	{ -202.254233, -196.946334 },
	{ -146.946266, -252.254283 },
	{ -77.254276, -287.764120 },
	{ 0.000003, -300.000000 },
	{ 77.254282, -287.764118 },
	{ 146.946271, -252.254279 },
	{ 202.254237, -196.946329 },
	{ 237.764133, -127.254237 },
	{ 250.000000, -50.000000 },
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
