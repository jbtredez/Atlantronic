#ifndef GRAPH_H
#define GRAPH_H

//! @file graph.h
//! @brief Graph de points de passage pour l'evitement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect2.h"

#define GRAPH_NUM_NODE           20
#define GRAPH_NUM_LINK           56

struct graph_link
{
	uint8_t a;         //!< noeud de depart
	uint8_t b;         //!< noeud d'arrivé
	uint16_t dist;     //!< distance entre les deux noeuds
	float alpha;       //!< angle de la droite dans le repère absolu (table)
};

struct graph_node
{
	struct Vect2 pos;
	uint8_t link_id;
	uint8_t link_num;
};

struct graph_dijkstra_info
{
	uint16_t dist;
	uint8_t prev_node;
	uint8_t is_best;
};

struct graph_node_dist
{
	uint16_t dist;
	uint16_t id;
};

extern const struct graph_node graph_node[GRAPH_NUM_NODE];
extern const struct graph_link graph_link[GRAPH_NUM_LINK];

//! graph_dijkstra_info* info : tableau de taille minimale GRAPH_NUM_NODE
//! valid_links : tableau de taille minimale GRAPH_NUM_LINK
int graph_dijkstra(int a, int b, struct graph_dijkstra_info* info, uint8_t* valid_links);

//! calcule la distance entre le point et tout les noeuds du graph
//! resultat dans un tableau trié de la plus petite distance à la plus grande
int graph_compute_node_distance(struct Vect2 pos, struct graph_node_dist* node_dist );

#endif
