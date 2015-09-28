#include "Graph.h"
#include <math.h>

//! Le tableau graph_link doit être trié et il doit y avoir un liens dans chaque sens (avec la même distance)
//! Il doit être cohérent avec le tableau de noeud qui indique l'id de début des liens connectés au noeud et la taille
//! Il y a redondance des donées pour permettre un traitement plus simple et plus rapide
//! Les tableaux graph_node et graph_link sont générés automatiquement (cf graph_gen) pour eviter les erreurs


//!< noeuds du graph
const struct GraphNode Graph::m_graphNode[GRAPH_NUM_NODE] =
{
	{Vect2( -1200.00f,   700.00f),  0,  3},
	{Vect2(  1200.00f,   700.00f),  3,  3},
	{Vect2( -1200.00f,   500.00f),  6,  2},
	{Vect2(  1200.00f,   500.00f),  8,  2},
	{Vect2(  -750.00f,   700.00f), 10,  2},
	{Vect2(   750.00f,   700.00f), 12,  2},
	{Vect2(  -750.00f,   200.00f), 14,  5},
	{Vect2(   750.00f,   200.00f), 19,  5},
	{Vect2(  -700.00f,     0.00f), 24,  3},
	{Vect2(   700.00f,     0.00f), 27,  3},
	{Vect2(  -800.00f,  -400.00f), 30,  4},
	{Vect2(   800.00f,  -400.00f), 34,  4},
	{Vect2( -1200.00f,  -400.00f), 38,  2},
	{Vect2(  1200.00f,  -400.00f), 40,  2},
	{Vect2( -1200.00f,  -700.00f), 42,  2},
	{Vect2(  1200.00f,  -700.00f), 44,  2},
	{Vect2(  -800.00f,  -700.00f), 46,  3},
	{Vect2(   800.00f,  -700.00f), 49,  3},
	{Vect2(  -400.00f,  -700.00f), 52,  3},
	{Vect2(   400.00f,  -700.00f), 55,  3},
	{Vect2(  -400.00f,  -400.00f), 58,  4},
	{Vect2(   400.00f,  -400.00f), 62,  4},
	{Vect2(  -400.00f,     0.00f), 66,  4},
	{Vect2(   400.00f,     0.00f), 70,  4},
	{Vect2(  -400.00f,   200.00f), 74,  3},
	{Vect2(   400.00f,   200.00f), 77,  3},
	{Vect2(     0.00f,   200.00f), 80,  3},
	{Vect2(     0.00f,     0.00f), 83,  4},
	{Vect2(     0.00f,  -400.00f), 87,  4},
	{Vect2(     0.00f,  -600.00f), 91,  3},
};

//!< liens du graph.
const struct GraphLink Graph::m_graphLink[GRAPH_NUM_LINK] =
{
	{ 0,  2,  200, -1.570796f},
	{ 0,  4,  450,  0.000000f},
	{ 0,  6,  673, -0.837981f},
	{ 1,  3,  200, -1.570796f},
	{ 1,  5,  450,  3.141593f},
	{ 1,  7,  673, -2.303612f},
	{ 2,  0,  200,  1.570796f},
	{ 2,  6,  541, -0.588003f},
	{ 3,  1,  200,  1.570796f},
	{ 3,  7,  541, -2.553590f},
	{ 4,  0,  450,  3.141593f},
	{ 4,  6,  500, -1.570796f},
	{ 5,  1,  450,  0.000000f},
	{ 5,  7,  500, -1.570796f},
	{ 6,  0,  673,  2.303612f},
	{ 6,  2,  541,  2.553590f},
	{ 6,  4,  500,  1.570796f},
	{ 6,  8,  206, -1.325818f},
	{ 6, 24,  350,  0.000000f},
	{ 7,  1,  673,  0.837981f},
	{ 7,  3,  541,  0.588003f},
	{ 7,  5,  500,  1.570796f},
	{ 7,  9,  206, -1.815775f},
	{ 7, 25,  350,  3.141593f},
	{ 8,  6,  206,  1.815775f},
	{ 8, 10,  412, -1.815775f},
	{ 8, 22,  300,  0.000000f},
	{ 9,  7,  206,  1.325818f},
	{ 9, 11,  412, -1.325818f},
	{ 9, 23,  300,  3.141593f},
	{10,  8,  412,  1.325818f},
	{10, 12,  400,  3.141593f},
	{10, 16,  300, -1.570796f},
	{10, 20,  400,  0.000000f},
	{11,  9,  412,  1.815775f},
	{11, 13,  400,  0.000000f},
	{11, 17,  300, -1.570796f},
	{11, 21,  400,  3.141593f},
	{12, 10,  400,  0.000000f},
	{12, 14,  300, -1.570796f},
	{13, 11,  400,  3.141593f},
	{13, 15,  300, -1.570796f},
	{14, 12,  300,  1.570796f},
	{14, 16,  400,  0.000000f},
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
	{18, 29,  412,  0.244979f},
	{19, 17,  400,  0.000000f},
	{19, 21,  300,  1.570796f},
	{19, 29,  412,  2.896614f},
	{20, 10,  400,  3.141593f},
	{20, 18,  300, -1.570796f},
	{20, 22,  400,  1.570796f},
	{20, 28,  400,  0.000000f},
	{21, 11,  400,  0.000000f},
	{21, 19,  300, -1.570796f},
	{21, 23,  400,  1.570796f},
	{21, 28,  400,  3.141593f},
	{22,  8,  300,  3.141593f},
	{22, 20,  400, -1.570796f},
	{22, 24,  200,  1.570796f},
	{22, 27,  400,  0.000000f},
	{23,  9,  300,  0.000000f},
	{23, 21,  400, -1.570796f},
	{23, 25,  200,  1.570796f},
	{23, 27,  400,  3.141593f},
	{24,  6,  350,  3.141593f},
	{24, 22,  200, -1.570796f},
	{24, 26,  400,  0.000000f},
	{25,  7,  350,  0.000000f},
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
	{28, 29,  200, -1.570796f},
	{29, 18,  412, -2.896614f},
	{29, 19,  412, -0.244979f},
	{29, 28,  200,  1.570796f},
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
