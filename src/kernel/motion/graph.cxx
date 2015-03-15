#include "kernel/motion/graph.h"
#include <math.h>

//! Le tableau graph_link doit être trié et il doit y avoir un liens dans chaque sens (avec la même distance)
//! Il doit être cohérent avec le tableau de noeud qui indique l'id de début des liens connectés au noeud et la taille
//! Il y a redondance des donées pour permettre un traitement plus simple et plus rapide
//! Les tableaux graph_node et graph_link sont générés automatiquement (cf graph_gen) pour eviter les erreurs

//!< noeuds du graph
const struct graph_node graph_node[GRAPH_NUM_NODE] =
{
	{Vect2( -1300.00f,   700.00f),  0,  3},
	{Vect2(  -800.00f,   700.00f),  3,  2},
	{Vect2( -1300.00f,   400.00f),  5,  2},
	{Vect2(  -800.00f,   400.00f),  7,  5},
	{Vect2(  -800.00f,     0.00f), 12,  3},
	{Vect2(  -800.00f,  -400.00f), 15,  5},
	{Vect2( -1300.00f,  -400.00f), 20,  2},
	{Vect2(  -800.00f,  -700.00f), 22,  3},
	{Vect2( -1300.00f,  -700.00f), 25,  3},
	{Vect2(  -400.00f,  -700.00f), 28,  3},
	{Vect2(  -400.00f,  -400.00f), 31,  4},
	{Vect2(  -400.00f,     0.00f), 35,  4},
	{Vect2(  -400.00f,   200.00f), 39,  3},
	{Vect2(     0.00f,   200.00f), 42,  3},
	{Vect2(     0.00f,     0.00f), 45,  4},
	{Vect2(     0.00f,  -400.00f), 49,  4},
	{Vect2(     0.00f,  -600.00f), 53,  3},
	{Vect2(  1300.00f,   700.00f), 56,  3},
	{Vect2(   800.00f,   700.00f), 59,  2},
	{Vect2(  1300.00f,   400.00f), 61,  2},
	{Vect2(   800.00f,   400.00f), 63,  5},
	{Vect2(   800.00f,     0.00f), 68,  3},
	{Vect2(   800.00f,  -400.00f), 71,  5},
	{Vect2(  1300.00f,  -400.00f), 76,  2},
	{Vect2(   800.00f,  -700.00f), 78,  3},
	{Vect2(  1300.00f,  -700.00f), 81,  3},
	{Vect2(   400.00f,  -700.00f), 84,  3},
	{Vect2(   400.00f,  -400.00f), 87,  4},
	{Vect2(   400.00f,     0.00f), 91,  4},
	{Vect2(   400.00f,   200.00f), 95,  3},
};

//!< liens du graph.
const struct graph_link graph_link[GRAPH_NUM_LINK] =
{
	{ 0,  1,  500,  0.000000f},
	{ 0,  2,  300, -1.570796f},
	{ 0,  3,  583, -0.540420f},
	{ 1,  0,  500,  3.141593f},
	{ 1,  3,  300, -1.570796f},
	{ 2,  0,  300,  1.570796f},
	{ 2,  3,  500,  0.000000f},
	{ 3,  0,  583,  2.601173f},
	{ 3,  1,  300,  1.570796f},
	{ 3,  2,  500,  3.141593f},
	{ 3,  4,  400, -1.570796f},
	{ 3, 12,  447, -0.463648f},
	{ 4,  3,  400,  1.570796f},
	{ 4,  5,  400, -1.570796f},
	{ 4, 11,  400,  0.000000f},
	{ 5,  4,  400,  1.570796f},
	{ 5,  6,  500,  3.141593f},
	{ 5,  7,  300, -1.570796f},
	{ 5,  8,  583, -2.601173f},
	{ 5, 10,  400,  0.000000f},
	{ 6,  5,  500,  0.000000f},
	{ 6,  8,  300, -1.570796f},
	{ 7,  5,  300,  1.570796f},
	{ 7,  8,  500,  3.141593f},
	{ 7,  9,  400,  0.000000f},
	{ 8,  5,  583,  0.540420f},
	{ 8,  6,  300,  1.570796f},
	{ 8,  7,  500,  0.000000f},
	{ 9,  7,  400,  3.141593f},
	{ 9, 10,  300,  1.570796f},
	{ 9, 16,  412,  0.244979f},
	{10,  5,  400,  3.141593f},
	{10,  9,  300, -1.570796f},
	{10, 11,  400,  1.570796f},
	{10, 15,  400,  0.000000f},
	{11,  4,  400,  3.141593f},
	{11, 10,  400, -1.570796f},
	{11, 12,  200,  1.570796f},
	{11, 14,  400,  0.000000f},
	{12,  3,  447,  2.677945f},
	{12, 11,  200, -1.570796f},
	{12, 13,  400,  0.000000f},
	{13, 12,  400,  3.141593f},
	{13, 14,  200, -1.570796f},
	{13, 29,  400,  0.000000f},
	{14, 11,  400,  3.141593f},
	{14, 13,  200,  1.570796f},
	{14, 15,  400, -1.570796f},
	{14, 28,  400,  0.000000f},
	{15, 10,  400,  3.141593f},
	{15, 14,  400,  1.570796f},
	{15, 16,  200, -1.570796f},
	{15, 27,  400,  0.000000f},
	{16,  9,  412, -2.896614f},
	{16, 15,  200,  1.570796f},
	{16, 26,  412, -0.244979f},
	{17, 18,  500,  3.141593f},
	{17, 19,  300, -1.570796f},
	{17, 20,  583, -2.601173f},
	{18, 17,  500,  0.000000f},
	{18, 20,  300, -1.570796f},
	{19, 17,  300,  1.570796f},
	{19, 20,  500,  3.141593f},
	{20, 17,  583,  0.540420f},
	{20, 18,  300,  1.570796f},
	{20, 19,  500,  0.000000f},
	{20, 21,  400, -1.570796f},
	{20, 29,  447, -2.677945f},
	{21, 20,  400,  1.570796f},
	{21, 22,  400, -1.570796f},
	{21, 28,  400,  3.141593f},
	{22, 21,  400,  1.570796f},
	{22, 23,  500,  0.000000f},
	{22, 24,  300, -1.570796f},
	{22, 25,  583, -0.540420f},
	{22, 27,  400,  3.141593f},
	{23, 22,  500,  3.141593f},
	{23, 25,  300, -1.570796f},
	{24, 22,  300,  1.570796f},
	{24, 25,  500,  0.000000f},
	{24, 26,  400,  3.141593f},
	{25, 22,  583,  2.601173f},
	{25, 23,  300,  1.570796f},
	{25, 24,  500,  3.141593f},
	{26, 16,  412,  2.896614f},
	{26, 24,  400,  0.000000f},
	{26, 27,  300,  1.570796f},
	{27, 15,  400,  3.141593f},
	{27, 22,  400,  0.000000f},
	{27, 26,  300, -1.570796f},
	{27, 28,  400,  1.570796f},
	{28, 14,  400,  3.141593f},
	{28, 21,  400,  0.000000f},
	{28, 27,  400, -1.570796f},
	{28, 29,  200,  1.570796f},
	{29, 13,  400,  3.141593f},
	{29, 20,  447,  0.463648f},
	{29, 28,  200, -1.570796f},
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
