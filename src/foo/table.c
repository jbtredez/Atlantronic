#include "table.h"

//!< contour de la table
const struct fx_vect2 table_contour[5] =
{
	{ -1500 << 16, -1000 << 16}, { -1500 << 16, 1000 << 16 }, { 1500 << 16, 1000 << 16 }, { 1500 << 16, -1000 << 16 }, { -1500 << 16, -1000 << 16 }
};

//!< bordure diagonale zone marron coté bleu
const struct fx_vect2 table_diag_border_blue[2] =
{
	{ -1175 << 16, -1000 << 16 }, { -1053 << 16, -260 << 16 }
};

//!< bordure case depart zone marron coté bleu
const struct fx_vect2 table_start_border_blue[4] =
{
	{ -1500 << 16, 500 << 16 }, { -1000 << 16, 500 << 16 }, { -1000 << 16, 550 << 16 }, { -1500 << 16, 550 << 16 }
};

//!< bordure diagonale zone marron coté rouge
const struct fx_vect2 table_diag_border_red[2] =
{
	{  1175 << 16, -1000 << 16 }, {  1053 << 16, -260 << 16 }
};

//!< bordure case depart zone marron coté rouge
const struct fx_vect2 table_start_border_red[4] =
{
	{ 1500 << 16, 500 << 16 }, { 1000 << 16, 500 << 16 }, { 1000 << 16, 550 << 16 }, { 1500 << 16, 550 << 16 }
};

//!< totem coté bleu
const struct fx_vect2 table_totem_blue[5] =
{
	{ -525 << 16, -125 << 16 }, { -275 << 16, -125 << 16 }, { -275 << 16, 125 << 16 }, { -525 << 16, 125 << 16 }, { -525 << 16, -125 << 16 }
};

//!< totem coté rouge
const struct fx_vect2 table_totem_red[5] =
{
	{ 525 << 16, -125 << 16 }, { 275 << 16, -125 << 16 }, { 275 << 16, 125 << 16 }, { 525 << 16, 125 << 16 }, { 525 << 16, -125 << 16 }
};

//!< palmier (carre englobant)
const struct fx_vect2 table_palm[5] =
{
	{ -75 << 16, -75 << 16 }, { 75 << 16, -75 << 16 }, { 75 << 16, 75 << 16 }, { -75 << 16, 75 << 16 }, { -75 << 16, -75 << 16 }
};

const struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (struct fx_vect2*) table_contour, 5 },
	{ (struct fx_vect2*) table_diag_border_blue, 2 },
	{ (struct fx_vect2*) table_start_border_blue, 4 },
	{ (struct fx_vect2*) table_totem_blue, 5 },
	{ (struct fx_vect2*) table_diag_border_red, 2 },
	{ (struct fx_vect2*) table_start_border_red, 4 },
	{ (struct fx_vect2*) table_totem_red, 5 },
	{ (struct fx_vect2*) table_palm, 5},
};