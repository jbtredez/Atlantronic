#include "table.h"

//!< contour de la table
const struct fx_vect2 table_contour[5] =
{
	{ -1500 << 16, -1500 << 16}, { -1500 << 16, 1500 << 16 }, { 1500 << 16, 1500 << 16 }, { 1500 << 16, -1500 << 16 }, { -1500 << 16, -1500 << 16 }
};

const struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (struct fx_vect2*) table_contour, 5 },
};