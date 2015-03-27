#include "table.h"

#define ARRAY_SIZE(a)               (sizeof(a)/sizeof(a[0]))

//!< contour de la table
const struct Vect2 table_contour[] =
{
	Vect2( -1500, -1000), Vect2( -1500, 1000 ), Vect2( 1500, 1000 ), Vect2( 1500, -1000 ), Vect2( -1500, -1000 )
};

const struct Vect2 table_estrade[] =
{
	Vect2( -300, -1000), Vect2( -300, -900 ), Vect2( 300, -900 ), Vect2( 300, -1000 )
};

const struct Vect2 table_bordure_marches[] =
{
	Vect2(-533, 1000), Vect2(-533, 410), Vect2(-511, 410), Vect2(-511, 978), Vect2(-11, 978), Vect2(-11, 410), Vect2(11, 410), Vect2(11, 978), Vect2(511, 978), Vect2(511, 410), Vect2(533, 410), Vect2(533, 1000)
};

const struct Vect2 table_marches_jaunes[] =
{
	Vect2(-511, 410), Vect2(-11, 410), Vect2(-11, 480), Vect2(-511, 480), Vect2(-511, 550), Vect2(-11, 550), Vect2(-11, 620), Vect2(-511, 620)
};

const struct Vect2 table_marches_vertes[] =
{
	Vect2(511, 410), Vect2(11, 410), Vect2(11, 480), Vect2(511, 480), Vect2(511, 550), Vect2(11, 550), Vect2(11, 620), Vect2(511, 620)
};

const struct Vect2 table_depart_vert[] =
{
	Vect2(1500, -222), Vect2(1100, -222), Vect2(1100, -200), Vect2(1430, -200), Vect2(1430, 200), Vect2(1100, 200), Vect2(1100, 222), Vect2(1500, 222)
};

const struct Vect2 table_depart_jaune[] =
{
	Vect2(-1500, -222), Vect2(-1100, -222), Vect2(-1100, -200), Vect2(-1430, -200), Vect2(-1430, 200), Vect2(-1100, 200), Vect2(-1100, 222), Vect2(-1500, 222)
};

const struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (Vect2*) table_contour, ARRAY_SIZE(table_contour) },
	{ (Vect2*) table_estrade, ARRAY_SIZE(table_estrade) },
	{ (Vect2*) table_bordure_marches, ARRAY_SIZE(table_bordure_marches) },
	{ (Vect2*) table_marches_jaunes, ARRAY_SIZE(table_marches_jaunes) },
	{ (Vect2*) table_marches_vertes, ARRAY_SIZE(table_marches_vertes) },
	{ (Vect2*) table_depart_vert, ARRAY_SIZE(table_depart_vert) },
	{ (Vect2*) table_depart_jaune, ARRAY_SIZE(table_depart_jaune) },
};
