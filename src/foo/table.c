#include "table.h"

//!< contour de la table
const struct vect2 table_contour[5] =
{
	{ -1500, -1000}, { -1500, 1000 }, { 1500, 1000 }, { 1500, -1000 }, { -1500, -1000 }
};

//!< bordure diagonale zone marron coté bleu
const struct vect2 table_diag_border_blue[2] =
{
	{ -1175, -1000 }, { -1138, -251 }
};

//!< bordure case depart zone marron coté bleu
const struct vect2 table_start_border_blue[4] =
{
	{ -1500, 500 }, { -1000, 500 }, { -1000, 550 }, { -1500, 550 }
};

//!< bordure diagonale zone marron coté rouge
const struct vect2 table_diag_border_red[2] =
{
	{  1175, -1000 }, {  1138, -251 }
};

//!< bordure case depart zone marron coté rouge
const struct vect2 table_start_border_red[4] =
{
	{ 1500, 500 }, { 1000, 500 }, { 1000, 550 }, { 1500, 550 }
};

//!< totem coté bleu
const struct vect2 table_totem_blue[5] =
{
	{ -525, -125 }, { -275, -125 }, { -275, 125 }, { -525, 125 }, { -525, -125 }
};

//!< totem coté rouge
const struct vect2 table_totem_red[5] =
{
	{ 525, -125 }, { 275, -125 }, { 275, 125 }, { 525, 125 }, { 525, -125 }
};

//!< palmier (carre englobant)
const struct vect2 table_palm[5] =
{
	{ -75, -75 }, { 75, -75 }, { 75, 75 }, { -75, 75 }, { -75, -75 }
};

//!< totem coté rouge
const struct vect2 table_map[2] =
{
	{ -300, 990 }, { 300, 990 }
};

const struct polyline table_obj[TABLE_OBJ_SIZE] =
{
	{ (struct vect2*) table_contour, 5 },
	{ (struct vect2*) table_diag_border_blue, 2 },
	{ (struct vect2*) table_start_border_blue, 4 },
	{ (struct vect2*) table_totem_blue, 5 },
	{ (struct vect2*) table_diag_border_red, 2 },
	{ (struct vect2*) table_start_border_red, 4 },
	{ (struct vect2*) table_totem_red, 5 },
	{ (struct vect2*) table_palm, 5},
	{ (struct vect2*) table_map, 2}
};
