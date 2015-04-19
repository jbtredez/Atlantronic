#include "kernel/motion/graph.h"
#include <math.h>

//! Le tableau graph_link doit être trié et il doit y avoir un liens dans chaque sens (avec la même distance)
//! Il doit être cohérent avec le tableau de noeud qui indique l'id de début des liens connectés au noeud et la taille
//! Il y a redondance des donées pour permettre un traitement plus simple et plus rapide
//! Les tableaux graph_node et graph_link sont générés automatiquement (cf graph_gen) pour eviter les erreurs


//!< noeuds du graph


const struct graph_node graph_node[GRAPH_NUM_NODE] =
{
    {Vect2( -1200.00f,   700.00f),  0,  4},
    {Vect2(  1200.00f,   700.00f),  4,  4},
    {Vect2( -1200.00f,   450.00f),  8,  3},
    {Vect2(  1200.00f,   450.00f), 11,  3},
    {Vect2(  -800.00f,   700.00f), 14,  3},
    {Vect2(   800.00f,   700.00f), 17,  3},
    {Vect2(  -800.00f,   200.00f), 20,  5},
    {Vect2(   800.00f,   200.00f), 25,  5},
    {Vect2(  -600.00f,   300.00f), 30,  5},
    {Vect2(   600.00f,   300.00f), 35,  5},
    {Vect2(  -800.00f,     0.00f), 40,  4},
    {Vect2(   800.00f,     0.00f), 44,  4},
    {Vect2(  -800.00f,  -400.00f), 48,  4},
    {Vect2(   800.00f,  -400.00f), 52,  4},
    {Vect2( -1200.00f,  -400.00f), 56,  2},
    {Vect2(  1200.00f,  -400.00f), 58,  2},
    {Vect2( -1200.00f,  -700.00f), 60,  2},
    {Vect2(  1200.00f,  -700.00f), 62,  2},
    {Vect2(  -800.00f,  -700.00f), 64,  3},
    {Vect2(   800.00f,  -700.00f), 67,  3},
    {Vect2(  -400.00f,  -700.00f), 70,  3},
    {Vect2(   400.00f,  -700.00f), 73,  3},
    {Vect2(  -400.00f,  -400.00f), 76,  4},
    {Vect2(   400.00f,  -400.00f), 80,  4},
    {Vect2(  -400.00f,     0.00f), 84,  4},
    {Vect2(   400.00f,     0.00f), 88,  4},
    {Vect2(  -400.00f,   200.00f), 92,  4},
    {Vect2(   400.00f,   200.00f), 96,  4},
    {Vect2(     0.00f,   200.00f), 100,  3},
    {Vect2(     0.00f,     0.00f), 103,  4},
    {Vect2(     0.00f,  -400.00f), 107,  4},
    {Vect2(     0.00f,  -600.00f), 111,  3},
};

//!< liens du graph.
const struct graph_link graph_link[GRAPH_NUM_LINK] =
{
    { 0,  2,  250, -1.570796f},
    { 0,  4,  400,  0.000000f},
    { 0,  6,  640, -0.896055f},
    { 0,  8,  721, -0.588003f},
    { 1,  3,  250, -1.570796f},
    { 1,  5,  400,  3.141593f},
    { 1,  7,  640, -2.245537f},
    { 1,  9,  721, -2.553590f},
    { 2,  0,  250,  1.570796f},
    { 2,  6,  472, -0.558599f},
    { 2,  8,  618, -0.244979f},
    { 3,  1,  250,  1.570796f},
    { 3,  7,  472, -2.582993f},
    { 3,  9,  618, -2.896614f},
    { 4,  0,  400,  3.141593f},
    { 4,  6,  500, -1.570796f},
    { 4,  8,  447, -1.107149f},
    { 5,  1,  400,  0.000000f},
    { 5,  7,  500, -1.570796f},
    { 5,  9,  447, -2.034444f},
    { 6,  0,  640,  2.245537f},
    { 6,  2,  472,  2.582993f},
    { 6,  4,  500,  1.570796f},
    { 6, 10,  200, -1.570796f},
    { 6, 26,  400,  0.000000f},
    { 7,  1,  640,  0.896055f},
    { 7,  3,  472,  0.558599f},
    { 7,  5,  500,  1.570796f},
    { 7, 11,  200, -1.570796f},
    { 7, 27,  400,  3.141593f},
    { 8,  0,  721,  2.553590f},
    { 8,  2,  618,  2.896614f},
    { 8,  4,  447,  2.034444f},
    { 8, 10,  361, -2.158799f},
    { 8, 26,  224, -0.463648f},
    { 9,  1,  721,  0.588003f},
    { 9,  3,  618,  0.244979f},
    { 9,  5,  447,  1.107149f},
    { 9, 11,  361, -0.982794f},
    { 9, 27,  224, -2.677945f},
    {10,  6,  200,  1.570796f},
    {10,  8,  361,  0.982794f},
    {10, 12,  400, -1.570796f},
    {10, 24,  400,  0.000000f},
    {11,  7,  200,  1.570796f},
    {11,  9,  361,  2.158799f},
    {11, 13,  400, -1.570796f},
    {11, 25,  400,  3.141593f},
    {12, 10,  400,  1.570796f},
    {12, 14,  400,  3.141593f},
    {12, 18,  300, -1.570796f},
    {12, 22,  400,  0.000000f},
    {13, 11,  400,  1.570796f},
    {13, 15,  400,  0.000000f},
    {13, 19,  300, -1.570796f},
    {13, 23,  400,  3.141593f},
    {14, 12,  400,  0.000000f},
    {14, 16,  300, -1.570796f},
    {15, 13,  400,  3.141593f},
    {15, 17,  300, -1.570796f},
    {16, 14,  300,  1.570796f},
    {16, 18,  400,  0.000000f},
    {17, 15,  300,  1.570796f},
    {17, 19,  400,  3.141593f},
    {18, 12,  300,  1.570796f},
    {18, 16,  400,  3.141593f},
    {18, 20,  400,  0.000000f},
    {19, 13,  300,  1.570796f},
    {19, 17,  400,  0.000000f},
    {19, 21,  400,  3.141593f},
    {20, 18,  400,  3.141593f},
    {20, 22,  300,  1.570796f},
    {20, 31,  412,  0.244979f},
    {21, 19,  400,  0.000000f},
    {21, 23,  300,  1.570796f},
    {21, 31,  412,  2.896614f},
    {22, 12,  400,  3.141593f},
    {22, 20,  300, -1.570796f},
    {22, 24,  400,  1.570796f},
    {22, 30,  400,  0.000000f},
    {23, 13,  400,  0.000000f},
    {23, 21,  300, -1.570796f},
    {23, 25,  400,  1.570796f},
    {23, 30,  400,  3.141593f},
    {24, 10,  400,  3.141593f},
    {24, 22,  400, -1.570796f},
    {24, 26,  200,  1.570796f},
    {24, 29,  400,  0.000000f},
    {25, 11,  400,  0.000000f},
    {25, 23,  400, -1.570796f},
    {25, 27,  200,  1.570796f},
    {25, 29,  400,  3.141593f},
    {26,  6,  400,  3.141593f},
    {26,  8,  224,  2.677945f},
    {26, 24,  200, -1.570796f},
    {26, 28,  400,  0.000000f},
    {27,  7,  400,  0.000000f},
    {27,  9,  224,  0.463648f},
    {27, 25,  200, -1.570796f},
    {27, 28,  400,  3.141593f},
    {28, 26,  400,  3.141593f},
    {28, 27,  400,  0.000000f},
    {28, 29,  200, -1.570796f},
    {29, 24,  400,  3.141593f},
    {29, 25,  400,  0.000000f},
    {29, 28,  200,  1.570796f},
    {29, 30,  400, -1.570796f},
    {30, 22,  400,  3.141593f},
    {30, 23,  400,  0.000000f},
    {30, 29,  400,  1.570796f},
    {30, 31,  200, -1.570796f},
    {31, 20,  412, -2.896614f},
    {31, 21,  412, -0.244979f},
    {31, 30,  200,  1.570796f},
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
