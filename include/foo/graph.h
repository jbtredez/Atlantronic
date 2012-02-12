#ifndef GRAPH_H
#define GRAPH_H

//! @file graph.h
//! @brief Graph de points de passage pour l'evitement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

#define GRAPH_NUM_NODE           20
#define GRAPH_NUM_LINK           52

struct graph_link
{
	uint8_t a;
	uint8_t b;
	uint16_t dist;
};

struct graph_node
{
	struct fx_vect2 pos;
	uint8_t link_id;
	uint8_t link_num;
};

struct graph_dijkstra_info
{
	uint16_t dist;
	uint8_t prev_node;
	uint8_t is_best;
};

#ifdef LINUX
extern const struct graph_node graph_node[GRAPH_NUM_NODE];
extern const struct graph_link graph_link[GRAPH_NUM_LINK];
#endif

//! graph_dijkstra_info* info : tableau de taille minimale GRAPH_NUM_NODE
//! valid_links : tableau de taille minimale GRAPH_NUM_LINK
int graph_dijkstra(int a, int b, struct graph_dijkstra_info* info, uint8_t* valid_links);

#endif