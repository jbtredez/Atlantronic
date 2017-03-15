#include "Graph.h"
#include <math.h>

//! Le tableau graph_link doit être trié et il doit y avoir un liens dans chaque sens (avec la même distance)
//! Il doit être cohérent avec le tableau de noeud qui indique l'id de début des liens connectés au noeud et la taille
//! Il y a redondance des donées pour permettre un traitement plus simple et plus rapide
//! Les tableaux graph_node et graph_link sont générés automatiquement (cf graph_gen) pour eviter les erreurs
//!< noeuds du graph
const struct GraphNode Graph::m_graphNode[GRAPH_NUM_NODE] =
{
		{Vect2(  -350.00f,   690.00f),  0,  3},
		{Vect2(   350.00f,   690.00f),  3,  3},
		{Vect2(  -575.00f,   440.00f),  6,  4},
		{Vect2(   575.00f,   440.00f), 10,  4},
		{Vect2(  -835.00f,   150.00f), 14,  4},
		{Vect2(   835.00f,   150.00f), 18,  4},
		{Vect2( -1190.00f,    55.00f), 22,  4},
		{Vect2(  1190.00f,    55.00f), 26,  4},
		{Vect2( -1195.00f,  -360.00f), 30,  3},
		{Vect2(  1195.00f,  -360.00f), 33,  3},
		{Vect2(  -948.00f,  -480.00f), 36,  3},
		{Vect2(   948.00f,  -480.00f), 39,  3},
		{Vect2(  -830.00f,  -135.00f), 42,  5},
		{Vect2(   830.00f,  -135.00f), 47,  5},
		{Vect2(  -420.00f,   -44.00f), 52,  5},
		{Vect2(   420.00f,   -44.00f), 57,  5},
		{Vect2(  -220.00f,   310.00f), 62,  6},
		{Vect2(   220.00f,   310.00f), 68,  6},
		{Vect2(     0.00f,    50.00f), 255,  0},
		{Vect2(     0.00f,   730.00f), 74,  4},
};

//!< liens du graph.
const struct GraphLink Graph::m_graphLink[GRAPH_NUM_LINK] =
{
		{ 0,  2,  336, -2.303612f},
		{ 0, 16,  402, -1.241172f},
		{ 0, 19,  352,  0.113792f},
		{ 1,  3,  336, -0.837981f},
		{ 1, 17,  402, -1.900421f},
		{ 1, 19,  352,  3.027801f},
		{ 2,  0,  336,  0.837981f},
		{ 2,  4,  389, -2.301703f},
		{ 2, 14,  508, -1.260869f},
		{ 2, 16,  378, -0.351031f},
		{ 3,  1,  336,  2.303612f},
		{ 3,  5,  389, -0.839890f},
		{ 3, 15,  508, -1.880724f},
		{ 3, 17,  378, -2.790562f},
		{ 4,  2,  389,  0.839890f},
		{ 4,  6,  367, -2.880114f},
		{ 4, 12,  285, -1.553254f},
		{ 4, 14,  458, -0.437287f},
		{ 5,  3,  389,  2.301703f},
		{ 5,  7,  367, -0.261479f},
		{ 5, 13,  285, -1.588338f},
		{ 5, 15,  458, -2.704306f},
		{ 6,  4,  367,  0.261479f},
		{ 6,  8,  415, -1.582844f},
		{ 6, 10,  587, -1.146001f},
		{ 6, 12,  407, -0.485622f},
		{ 7,  5,  367,  2.880114f},
		{ 7,  9,  415, -1.558749f},
		{ 7, 11,  587, -1.995592f},
		{ 7, 13,  407, -2.655971f},
		{ 8,  6,  415,  1.558749f},
		{ 8, 10,  275, -0.452247f},
		{ 8, 12,  429,  0.552419f},
		{ 9,  7,  415,  1.582844f},
		{ 9, 11,  275, -2.689345f},
		{ 9, 13,  429,  2.589174f},
		{10,  6,  587,  1.995592f},
		{10,  8,  275,  2.689345f},
		{10, 12,  365,  1.241240f},
		{11,  7,  587,  1.146001f},
		{11,  9,  275,  0.452247f},
		{11, 13,  365,  1.900352f},
		{12,  4,  285,  1.588338f},
		{12,  6,  407,  2.655971f},
		{12,  8,  429, -2.589174f},
		{12, 10,  365, -1.900352f},
		{12, 14,  420,  0.218411f},
		{13,  5,  285,  1.553254f},
		{13,  7,  407,  0.485622f},
		{13,  9,  429, -0.552419f},
		{13, 11,  365, -1.241240f},
		{13, 15,  420,  2.923182f},
		{14,  2,  508,  1.880724f},
		{14,  4,  458,  2.704306f},
		{14, 12,  420, -2.923182f},
		{14, 16,  407,  1.056531f},
		{14, 20,  422,  0.104381f},
		{15,  3,  508,  1.260869f},
		{15,  5,  458,  0.437287f},
		{15, 13,  420, -0.218411f},
		{15, 17,  407,  2.085062f},
		{15, 20,  422,  3.037211f},
		{16,  0,  402,  1.900421f},
		{16,  2,  378,  2.790562f},
		{16, 14,  407, -2.085062f},
		{16, 17,  440,  0.000000f},
		{16, 19,  474,  1.088283f},
		{16, 20,  380, -0.953605f},
		{17,  1,  402,  1.241172f},
		{17,  3,  378,  0.351031f},
		{17, 15,  407, -1.056531f},
		{17, 16,  440,  3.141593f},
		{17, 19,  474,  2.053310f},
		{17, 20,  380, -2.187988f},
		{19,  0,  352, -3.027801f},
		{19,  1,  352, -0.113792f},
		{19, 16,  474, -2.053310f},
		{19, 17,  474, -1.088283f},
		{20, 14,  422, -3.037211f},
		{20, 15,  422, -0.104381f},
		{20, 16,  380,  2.187988f},
		{20, 17,  380,  0.953605f},
};
int Graph::dijkstra(int a, int b)
{
	int i;
	int j;

	// init
	for( i=0 ; i < GRAPH_NUM_NODE; i++)
	{
		m_info[i].dist = 0xFFFF;
		m_info[i].prev_node = a;
		m_info[i].is_best = 0;
	}

	// a est a une distance de 0 de lui même et il n'y a pas mieux
	m_info[a].dist = 0;
	i = a;

	while( i != b)
	{
		m_info[i].is_best = 1;

		int max = m_graphNode[i].link_id + m_graphNode[i].link_num;
		for(j = m_graphNode[i].link_id; j < max; j++)
		{
			if( m_validLinks[j])
			{
				int connected_node = m_graphLink[j].b;
				// calcul de la distance en passant par la
				uint16_t dist = m_info[i].dist + m_graphLink[j].dist;
				if( m_info[connected_node].dist > dist)
				{
					// on a trouvé un chemin plus court pour aller vers "connected_node"
					m_info[connected_node].dist = dist;
					m_info[connected_node].prev_node = i;
				}
			}
		}

		uint16_t best_dist = 0xFFFF;
		i = a;
		for(j = 0; j<GRAPH_NUM_NODE; j++)
		{
			if( ! m_info[j].is_best && m_info[j].dist < best_dist)
			{
				best_dist = m_info[j].dist;
				i = j;
			}
		}
		if(i == a)
		{
			// rien de trouvé
			return -1;
		}
	}

	m_info[i].is_best = 1;

	// on met le chemin dans l'ordre de a vers b dans le tableau m_way
	m_wayCount = 2;
	i = b;
	while(m_info[i].prev_node != a)
	{
		i = m_info[i].prev_node;
		m_wayCount++;
	}

	m_way[0] = a;
	j = m_wayCount - 1;
	m_way[j] = b;
	i = b;
	while(m_info[i].prev_node != a)
	{
		i = m_info[i].prev_node;
		j--;
		m_way[j] = i;
	}

	// affichage debug
	/*for(i=0; i < m_wayCount; i++)
	{
		log_format(LOG_INFO, "chemin - graph : %d : %d", i, m_way[i]);
	}*/

	return 0;
}

int Graph::computeNodeDistance(struct Vect2 pos, struct GraphNodeDist* node_dist )
{
	int i;
	int j;
	float dx;
	float dy;
	uint16_t dist;

	for(i = 0; i< GRAPH_NUM_NODE; i++)
	{
		dx = pos.x - m_graphNode[i].pos.x;
		dy = pos.y - m_graphNode[i].pos.y;
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
