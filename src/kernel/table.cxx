#include "table.h"

#define ARRAY_SIZE(a)               (sizeof(a)/sizeof(a[0]))

//!< contour de la table
const struct vect2 table_contour[] =
{
	vect2( -1500, -1000), vect2( -1500, 1000 ), vect2( 1500, 1000 ), vect2( 1500, -1000 ), vect2( -1500, -1000 )
};

const struct vect2 table_bordure_marches[] =
{
	vect2(-533, 1000), vect2(-533, 410), vect2(-511, 410), vect2(-511, 978), vect2(-11, 978), vect2(-11, 410), vect2(11, 410), vect2(11, 978), vect2(511, 978), vect2(511, 410), vect2(533, 410), vect2(533, 1000)
};

const struct vect2 table_marches_jaunes[] =
{
	vect2(-511, 410), vect2(-11, 410), vect2(-11, 480), vect2(-511, 480), vect2(-511, 550), vect2(-11, 550), vect2(-11, 620), vect2(-511, 620)
};

const struct vect2 table_marches_vertes[] =
{
	vect2(511, 410), vect2(11, 410), vect2(11, 480), vect2(511, 480), vect2(511, 550), vect2(11, 550), vect2(11, 620), vect2(511, 620)
};

const struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (vect2*) table_contour, ARRAY_SIZE(table_contour) },
	{ (vect2*) table_bordure_marches, ARRAY_SIZE(table_bordure_marches) },
	{ (vect2*) table_marches_jaunes, ARRAY_SIZE(table_marches_jaunes) },
	{ (vect2*) table_marches_vertes, ARRAY_SIZE(table_marches_vertes) },
};
