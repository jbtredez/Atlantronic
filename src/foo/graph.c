#include "foo/graph.h"
#include <math.h>

//! Le tableau graph_link doit être trié et il doit y avoir un liens dans chaque sens (avec la même distance)
//! Il doit être cohérent avec le tableau de noeud qui indique l'id de début des liens connectés au noeud et la taille
//! Il y a redondance des donées pour permettre un traitement plus simple et plus rapide
// TODO faire fonction de check de cohérence : liens triés, noeud => id de debut et de fin [noeud+1].link_id = [noeud].link_id+[noeud].link_num avec debut à 0 + verif des noeuds avec le bon id du noeud en premier => table de liens, distances  = sqrt (...) +- 1mm ;, liens dans les 2 sens ...

//!< noeuds du graph
const struct graph_node graph_node[GRAPH_NUM_NODE] =
{
	{{  -39321600,   45875200},  0,  2},
	{{          0,   45875200},  2,  3},
	{{   39321600,   45875200},  5,  2},
	{{  -39321600,   29491200},  7,  3},
	{{          0,   29491200}, 10,  3},
	{{   39321600,   29491200}, 13,  3},
	{{  -49971200,   20480000}, 16,  3},
	{{   49971200,   20480000}, 19,  3},
	{{  -81920000,   16384000}, 22,  3},
	{{   81920000,   16384000}, 25,  3},
	{{  -81920000,          0}, 28,  2},
	{{  -52428800,          0}, 30,  4},
	{{   52428800,          0}, 34,  4},
	{{   81920000,          0}, 38,  2},
	{{  -56360960,  -26214400}, 40,  3},
	{{          0,  -26214400}, 43,  3},
	{{   56360960,  -26214400}, 46,  3},
	{{  -56360960,  -45875200}, 49,  2},
	{{          0,  -45875200}, 51,  3},
	{{   56360960,  -45875200}, 54,  2},
};

//!< liens du graph.
const struct graph_link graph_link[GRAPH_NUM_LINK] =
{
	{ 0,  1,  600,          0u},
	{ 0,  3,  250, 4278190080u},
	{ 1,  0,  600,   33554432u},
	{ 1,  2,  600,          0u},
	{ 1,  4,  250, 4278190080u},
	{ 2,  1,  600,   33554432u},
	{ 2,  5,  250, 4278190080u},
	{ 3,  0,  250,   16777216u},
	{ 3,  4,  600,          0u},
	{ 3,  6,  213, 4268913433u},
	{ 4,  1,  250,   16777216u},
	{ 4,  3,  600,   33554432u},
	{ 4,  5,  600,          0u},
	{ 5,  2,  250,   16777216u},
	{ 5,  4,  600,   33554432u},
	{ 5,  7,  213, 4287466726u},
	{ 6,  3,  213,    7500569u},
	{ 6,  8,  491, 4262774756u},
	{ 6, 11,  315, 4276914495u},
	{ 7,  5,  213,   26053862u},
	{ 7,  9,  491, 4293605403u},
	{ 7, 12,  315, 4279465664u},
	{ 8,  6,  491,    1361892u},
	{ 8, 10,  250, 4278190080u},
	{ 8, 11,  515, 4289551138u},
	{ 9,  7,  491,   32192539u},
	{ 9, 12,  515, 4266829021u},
	{ 9, 13,  250, 4278190080u},
	{10,  8,  250,   16777216u},
	{10, 11,  450,          0u},
	{11,  6,  315,   15501631u},
	{11,  8,  515,   28138274u},
	{11, 10,  450,   33554432u},
	{11, 14,  404, 4276599828u},
	{12,  7,  315,   18052800u},
	{12,  9,  515,    5416157u},
	{12, 13,  450,          0u},
	{12, 16,  404, 4279780332u},
	{13,  9,  250,   16777216u},
	{13, 12,  450,   33554432u},
	{14, 11,  404,   15186964u},
	{14, 15,  860,          0u},
	{14, 17,  300, 4278190080u},
	{15, 14,  860,   33554432u},
	{15, 16,  860,          0u},
	{15, 18,  300, 4278190080u},
	{16, 12,  404,   18367468u},
	{16, 15,  860,   33554432u},
	{16, 19,  300, 4278190080u},
	{17, 14,  300,   16777216u},
	{17, 18,  860,          0u},
	{18, 15,  300,   16777216u},
	{18, 17,  860,   33554432u},
	{18, 19,  860,          0u},
	{19, 16,  300,   16777216u},
	{19, 18,  860,   33554432u},
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

int graph_compute_node_distance(struct fx_vect2 pos, struct graph_node_dist* node_dist )
{
	int i;
	int j;
	int64_t dx;
	int64_t dy;
	uint16_t dist;

	for(i = 0; i< GRAPH_NUM_NODE; i++)
	{
		dx = pos.x - graph_node[i].pos.x;
		dy = pos.y - graph_node[i].pos.y;
		dist = ((int32_t)sqrtf(dx * dx + dy * dy)) >> 16;

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
