#include "kernel/motion/graph.h"
#include <math.h>

//! Le tableau graph_link doit être trié et il doit y avoir un liens dans chaque sens (avec la même distance)
//! Il doit être cohérent avec le tableau de noeud qui indique l'id de début des liens connectés au noeud et la taille
//! Il y a redondance des donées pour permettre un traitement plus simple et plus rapide
//! Les tableaux graph_node et graph_link sont générés automatiquement (cf graph_gen) pour eviter les erreurs

//!< noeuds du graph
const struct graph_node graph_node[GRAPH_NUM_NODE] =
{
	{Vect2( -1200.00f,   700.00f),  0,  3},
	{Vect2(  1200.00f,   700.00f),  3,  3},
	{Vect2( -1200.00f,   450.00f),  6,  2},
	{Vect2(  1200.00f,   450.00f),  8,  2},
	{Vect2(  -800.00f,   700.00f), 10,  2},
	{Vect2(   800.00f,   700.00f), 12,  2},
	{Vect2(  -800.00f,   300.00f), 14,  5},
	{Vect2(   800.00f,   300.00f), 19,  5},
	{Vect2(  -800.00f,     0.00f), 24,  3},
	{Vect2(   800.00f,     0.00f), 27,  3},
	{Vect2(  -800.00f,  -400.00f), 30,  5},
	{Vect2(   800.00f,  -400.00f), 35,  5},
	{Vect2( -1200.00f,  -400.00f), 40,  2},
	{Vect2(  1200.00f,  -400.00f), 42,  2},
	{Vect2( -1200.00f,  -700.00f), 44,  3},
	{Vect2(  1200.00f,  -700.00f), 47,  3},
	{Vect2(  -800.00f,  -700.00f), 50,  3},
	{Vect2(   800.00f,  -700.00f), 53,  3},
	{Vect2(  -400.00f,  -700.00f), 56,  2},
	{Vect2(   400.00f,  -700.00f), 58,  2},
	{Vect2(  -400.00f,  -400.00f), 60,  4},
	{Vect2(   400.00f,  -400.00f), 64,  4},
	{Vect2(  -400.00f,     0.00f), 68,  4},
	{Vect2(   400.00f,     0.00f), 72,  4},
	{Vect2(  -400.00f,   200.00f), 76,  3},
	{Vect2(   400.00f,   200.00f), 79,  3},
	{Vect2(     0.00f,   200.00f), 82,  3},
	{Vect2(     0.00f,     0.00f), 85,  4},
	{Vect2(     0.00f,  -400.00f), 89,  3},
	{Vect2(     0.00f,  -600.00f), 255,  0},
};

//!< liens du graph.
const struct graph_link graph_link[GRAPH_NUM_LINK] =
{
	{ 0,  2,  250, -1.570796f},
	{ 0,  4,  400,  0.000000f},
	{ 0,  6,  566, -0.785398f},
	{ 1,  3,  250, -1.570796f},
	{ 1,  5,  400,  3.141593f},
	{ 1,  7,  566, -2.356194f},
	{ 2,  0,  250,  1.570796f},
	{ 2,  6,  427, -0.358771f},
	{ 3,  1,  250,  1.570796f},
	{ 3,  7,  427, -2.782822f},
	{ 4,  0,  400,  3.141593f},
	{ 4,  6,  400, -1.570796f},
	{ 5,  1,  400,  0.000000f},
	{ 5,  7,  400, -1.570796f},
	{ 6,  0,  566,  2.356194f},
	{ 6,  2,  427,  2.782822f},
	{ 6,  4,  400,  1.570796f},
	{ 6,  8,  300, -1.570796f},
	{ 6, 24,  412, -0.244979f},
	{ 7,  1,  566,  0.785398f},
	{ 7,  3,  427,  0.358771f},
	{ 7,  5,  400,  1.570796f},
	{ 7,  9,  300, -1.570796f},
	{ 7, 25,  412, -2.896614f},
	{ 8,  6,  300,  1.570796f},
	{ 8, 10,  400, -1.570796f},
	{ 8, 22,  400,  0.000000f},
	{ 9,  7,  300,  1.570796f},
	{ 9, 11,  400, -1.570796f},
	{ 9, 23,  400,  3.141593f},
	{10,  8,  400,  1.570796f},
	{10, 12,  400,  3.141593f},
	{10, 14,  500, -2.498091f},
	{10, 16,  300, -1.570796f},
	{10, 20,  400,  0.000000f},
	{11,  9,  400,  1.570796f},
	{11, 13,  400,  0.000000f},
	{11, 15,  500, -0.643501f},
	{11, 17,  300, -1.570796f},
	{11, 21,  400,  3.141593f},
	{12, 10,  400,  0.000000f},
	{12, 14,  300, -1.570796f},
	{13, 11,  400,  3.141593f},
	{13, 15,  300, -1.570796f},
	{14, 10,  500,  0.643501f},
	{14, 12,  300,  1.570796f},
	{14, 16,  400,  0.000000f},
	{15, 11,  500,  2.498091f},
	{15, 13,  300,  1.570796f},
	{15, 17,  400,  3.141593f},
	{16, 10,  300,  1.570796f},
	{16, 14,  400,  3.141593f},
	{16, 18,  400,  0.000000f},
	{17, 11,  300,  1.570796f},
	{17, 15,  400,  0.000000f},
	{17, 19,  400,  3.141593f},
	{18, 16,  400,  3.141593f},
	{18, 20,  300,  1.570796f},
	{19, 17,  400,  0.000000f},
	{19, 21,  300,  1.570796f},
	{20, 10,  400,  3.141593f},
	{20, 18,  300, -1.570796f},
	{20, 22,  400,  1.570796f},
	{20, 28,  400,  0.000000f},
	{21, 11,  400,  0.000000f},
	{21, 19,  300, -1.570796f},
	{21, 23,  400,  1.570796f},
	{21, 28,  400,  3.141593f},
	{22,  8,  400,  3.141593f},
	{22, 20,  400, -1.570796f},
	{22, 24,  200,  1.570796f},
	{22, 27,  400,  0.000000f},
	{23,  9,  400,  0.000000f},
	{23, 21,  400, -1.570796f},
	{23, 25,  200,  1.570796f},
	{23, 27,  400,  3.141593f},
	{24,  6,  412,  2.896614f},
	{24, 22,  200, -1.570796f},
	{24, 26,  400,  0.000000f},
	{25,  7,  412,  0.244979f},
	{25, 23,  200, -1.570796f},
	{25, 26,  400,  3.141593f},
	{26, 24,  400,  3.141593f},
	{26, 25,  400,  0.000000f},
	{26, 27,  200, -1.570796f},
	{27, 22,  400,  3.141593f},
	{27, 23,  400,  0.000000f},
	{27, 26,  200,  1.570796f},
	{27, 28,  400, -1.570796f},
	{28, 20,  400,  3.141593f},
	{28, 21,  400,  0.000000f},
	{28, 27,  400,  1.570796f},
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
