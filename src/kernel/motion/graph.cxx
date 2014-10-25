#include "kernel/motion/graph.h"
#include <math.h>

//! Le tableau graph_link doit être trié et il doit y avoir un liens dans chaque sens (avec la même distance)
//! Il doit être cohérent avec le tableau de noeud qui indique l'id de début des liens connectés au noeud et la taille
//! Il y a redondance des donées pour permettre un traitement plus simple et plus rapide
//! Les tableaux graph_node et graph_link sont générés automatiquement (cf graph_gen) pour eviter les erreurs

//!< noeuds du graph
const struct graph_node graph_node[GRAPH_NUM_NODE] =
{
	{Vect2(  -600.00f,   700.00f),  0,  2},
	{Vect2(     0.00f,   700.00f),  2,  3},
	{Vect2(   600.00f,   700.00f),  5,  2},
	{Vect2(  -600.00f,   450.00f),  7,  3},
	{Vect2(     0.00f,   450.00f), 10,  3},
	{Vect2(   600.00f,   450.00f), 13,  3},
	{Vect2(  -762.50f,   312.50f), 16,  3},
	{Vect2(   762.50f,   312.50f), 19,  3},
	{Vect2( -1250.00f,   250.00f), 22,  3},
	{Vect2(  1250.00f,   250.00f), 25,  3},
	{Vect2( -1250.00f,     0.00f), 28,  2},
	{Vect2(  -800.00f,     0.00f), 30,  4},
	{Vect2(   800.00f,     0.00f), 34,  4},
	{Vect2(  1250.00f,     0.00f), 38,  2},
	{Vect2(  -860.00f,  -400.00f), 40,  3},
	{Vect2(     0.00f,  -400.00f), 43,  3},
	{Vect2(   860.00f,  -400.00f), 46,  3},
	{Vect2(  -860.00f,  -700.00f), 49,  2},
	{Vect2(     0.00f,  -700.00f), 51,  3},
	{Vect2(   860.00f,  -700.00f), 54,  2},
};

//!< liens du graph.
const struct graph_link graph_link[GRAPH_NUM_LINK] =
{
	{ 0,  1,  600,  0.000000f},
	{ 0,  3,  250, -1.570796f},
	{ 1,  0,  600,  3.141593f},
	{ 1,  2,  600,  0.000000f},
	{ 1,  4,  250, -1.570796f},
	{ 2,  1,  600,  3.141593f},
	{ 2,  5,  250, -1.570796f},
	{ 3,  0,  250,  1.570796f},
	{ 3,  4,  600,  0.000000f},
	{ 3,  6,  213, -2.439336f},
	{ 4,  1,  250,  1.570796f},
	{ 4,  3,  600,  3.141593f},
	{ 4,  5,  600,  0.000000f},
	{ 5,  2,  250,  1.570796f},
	{ 5,  4,  600,  3.141593f},
	{ 5,  7,  213, -0.702257f},
	{ 6,  3,  213,  0.702257f},
	{ 6,  8,  491, -3.014083f},
	{ 6, 11,  315, -1.690225f},
	{ 7,  5,  213,  2.439336f},
	{ 7,  9,  491, -0.127510f},
	{ 7, 12,  315, -1.451367f},
	{ 8,  6,  491,  0.127510f},
	{ 8, 10,  250, -1.570796f},
	{ 8, 11,  515, -0.507098f},
	{ 9,  7,  491,  3.014083f},
	{ 9, 12,  515, -2.634494f},
	{ 9, 13,  250, -1.570796f},
	{10,  8,  250,  1.570796f},
	{10, 11,  450,  0.000000f},
	{11,  6,  315,  1.451367f},
	{11,  8,  515,  2.634494f},
	{11, 10,  450,  3.141593f},
	{11, 14,  404, -1.719686f},
	{12,  7,  315,  1.690225f},
	{12,  9,  515,  0.507098f},
	{12, 13,  450,  0.000000f},
	{12, 16,  404, -1.421906f},
	{13,  9,  250,  1.570796f},
	{13, 12,  450,  3.141593f},
	{14, 11,  404,  1.421906f},
	{14, 15,  860,  0.000000f},
	{14, 17,  300, -1.570796f},
	{15, 14,  860,  3.141593f},
	{15, 16,  860,  0.000000f},
	{15, 18,  300, -1.570796f},
	{16, 12,  404,  1.719686f},
	{16, 15,  860,  3.141593f},
	{16, 19,  300, -1.570796f},
	{17, 14,  300,  1.570796f},
	{17, 18,  860,  0.000000f},
	{18, 15,  300,  1.570796f},
	{18, 17,  860,  3.141593f},
	{18, 19,  860,  0.000000f},
	{19, 16,  300,  1.570796f},
	{19, 18,  860,  3.141593f},
};

int graph_dijkstra(int a, int b, struct graph_dijkstra_info* info, uint8_t* valid_links)
{
	int i;
	int j;

	// init
	for( i=0 ; i < GRAPH_NUM_NODE; i++)
	{
		info[i].dist = 0xFFFF;
		info[i].prev_node = a;
		info[i].is_best = 0;
	}

	// a est a une distance de 0 de lui même et il n'y a pas mieux
	info[a].dist = 0;
	i = a;

	while( i != b)
	{
		info[i].is_best = 1;

		int max = graph_node[i].link_id + graph_node[i].link_num;
		for(j = graph_node[i].link_id; j < max; j++)
		{
			if( valid_links[j])
			{
				int connected_node = graph_link[j].b;
				// calcul de la distance en passant par la
				uint16_t dist = info[i].dist + graph_link[j].dist;
				if( info[connected_node].dist > dist)
				{
					// on a trouvé un chemin plus court pour aller vers "connected_node"
					info[connected_node].dist = dist;
					info[connected_node].prev_node = i;
				}
			}
		}

		uint16_t best_dist = 0xFFFF;
		i = a;
		for(j = 0; j<GRAPH_NUM_NODE; j++)
		{
			if( ! info[j].is_best && info[j].dist < best_dist)
			{
				best_dist = info[j].dist;
				i = j;
			}
		}
		if(i == a)
		{
			// rien de trouvé
			return -1;
		}
	}

	info[i].is_best = 1;

	return 0;
}

int graph_compute_node_distance(struct Vect2 pos, struct graph_node_dist* node_dist )
{
	int i;
	int j;
	float dx;
	float dy;
	uint16_t dist;

	for(i = 0; i< GRAPH_NUM_NODE; i++)
	{
		dx = pos.x - graph_node[i].pos.x;
		dy = pos.y - graph_node[i].pos.y;
		dist = (uint16_t)sqrtf(dx * dx + dy * dy);

		j = i-1;
		while(j >= 0 && node_dist[j].dist > dist)
		{
			node_dist[j+1] = node_dist[j];
			j--;
		}
		node_dist[j+1].dist = dist;
		node_dist[j+1].id = i;
	}

	return 0;
}
