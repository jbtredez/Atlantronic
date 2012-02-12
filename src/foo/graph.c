#include "foo/graph.h"

//! Le tableau graph_link doit être trié et il doit y avoir un liens dans chaque sens (avec la même distance)
//! Il doit être cohérent avec le tableau de noeud qui indique l'id de début des liens connectés au noeud et la taille
//! Il y a redondance des donées pour permettre un traitement plus simple et plus rapide
// TODO faire fonction de check de cohérence : liens triés, noeud => id de debut et de fin [noeud+1].link_id = [noeud].link_id+[noeud].link_num avec debut à 0 + verif des noeuds avec le bon id du noeud en premier => table de liens, distances  = sqrt (...) +- 1mm ;, liens dans les 2 sens ...

//!< noeuds du graph
const struct graph_node graph_node[GRAPH_NUM_NODE] =
{
	{{  -400   * 65536,  700   * 65536},  0, 2},
	{{               0,  700   * 65536},  2, 3},
	{{   400   * 65536,  700   * 65536},  5, 2},
	{{  -400   * 65536,  400   * 65536},  7, 3},
	{{               0,  400   * 65536}, 10, 3},
	{{   400   * 65536,  400   * 65536}, 13, 3},
	{{ -762.5f * 65536, 312.5f * 65536}, 16, 3},
	{{  762.5f * 65536, 312.5f * 65536}, 19, 3},
	{{ -1250   * 65536,  250   * 65536}, 22, 2},
	{{  1250   * 65536,  250   * 65536}, 24, 2},
	{{ -1250   * 65536,              0}, 26, 2},
	{{  -800   * 65536,              0}, 28, 3},
	{{   800   * 65536,              0}, 31, 3},
	{{ 1250    * 65536,              0}, 34, 2},
	{{ -400    * 65536, -400   * 65536}, 36, 3},
	{{               0, -400   * 65536}, 39, 3},
	{{ 400     * 65536, -400   * 65536}, 42, 3},
	{{ -400    * 65536, -700   * 65536}, 45, 2},
	{{               0, -700   * 65536}, 47, 3},
	{{ 400     * 65536, -700   * 65536}, 50, 2}
};

//!< liens du graph.
const struct graph_link graph_link[GRAPH_NUM_LINK] =
{
	{  0,  1, 400 },
	{  0,  3, 300 },
	{  1,  0, 400 },
	{  1,  2, 400 },
	{  1,  4, 300 },
	{  2,  1, 400 },
	{  2,  5, 300 },
	{  3,  0, 300 },
	{  3,  4, 400 },
	{  3,  6, 373 },
	{  4,  1, 300 },
	{  4,  3, 400 },
	{  4,  5, 400 },
	{  5,  2, 300 },
	{  5,  4, 400 },
	{  5,  7, 373 },
	{  6,  3, 373 },
	{  6,  8, 492 },
	{  6, 11, 315 },
	{  7,  5, 373 },
	{  7,  9, 492 },
	{  7, 12, 315 },
	{  8,  6, 492 },
	{  8, 10, 250 },
	{  9,  7, 492 },
	{  9, 13, 250 },
	{ 10,  8, 250 },
	{ 10, 11, 450 },
	{ 11,  6, 315 },
	{ 11, 10, 450 },
	{ 11, 14, 566 },
	{ 12,  7, 315 },
	{ 12, 13, 450 },
	{ 12, 16, 566 },
	{ 13,  9, 250 },
	{ 13, 12, 450 },
	{ 14, 11, 566 },
	{ 14, 15, 400 },
	{ 14, 17, 300 },
	{ 15, 14, 400 },
	{ 15, 16, 400 },
	{ 15, 18, 300 },
	{ 16, 12, 566 },
	{ 16, 15, 400 },
	{ 16, 19, 300 },
	{ 17, 14, 300 },
	{ 17, 18, 400 },
	{ 18, 15, 300 },
	{ 18, 17, 400 },
	{ 18, 19, 400 },
	{ 19, 16, 300 },
	{ 19, 18, 400 },
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