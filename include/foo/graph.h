#ifndef GRAPH_H
#define GRAPH_H

//! @file graph.h
//! @brief Graph de points de passage pour l'evitement
//! @author Atlantronic

#include <stdint.h>

#define GRAPH_NUM_PT             20
#define GRAPH_NUM_LINK           26

struct graph_link
{
	uint8_t a;
	uint8_t b;
	uint16_t dist;
};

#ifdef LINUX
extern const struct fx_vect2 graph_pt[GRAPH_NUM_PT];
extern const struct graph_link graph_link[GRAPH_NUM_LINK];
#endif

#endif