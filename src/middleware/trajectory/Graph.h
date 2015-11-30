#ifndef GRAPH_H
#define GRAPH_H

//! @file graph.h
//! @brief Graph de points de passage pour l'evitement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/Vect2.h"

#define GRAPH_NUM_NODE          30
#define GRAPH_NUM_LINK          94

struct GraphLink
{
	uint8_t a;         //!< noeud de depart
	uint8_t b;         //!< noeud d'arrivé
	uint16_t dist;     //!< distance entre les deux noeuds
	float alpha;       //!< angle de la droite dans le repère absolu (table)
};

struct GraphNode
{
	struct Vect2 pos;
	uint8_t link_id;
	uint8_t link_num;
};

struct GrapgDijkstraInfo
{
	uint16_t dist;
	uint8_t prev_node;
	uint8_t is_best;
};

struct GraphNodeDist
{
	uint16_t dist;
	uint16_t id;
};

class Graph
{
	public:
		//! lancement de l'algo dijkstra entre a et b
		uint32_t dijkstra(uint32_t a, uint32_t b);

		//! calcule la distance entre le point et tout les noeuds du graph
		//! resultat dans un tableau trié de la plus petite distance à la plus grande
		uint32_t computeNodeDistance(struct Vect2 pos, struct GraphNodeDist* node_dist);

		inline void setValidLink(uint32_t id, bool valid)
		{
			m_validLinks[id] = valid;
		}

		inline static Vect2 getNode(uint32_t id)
		{
			return m_graphNode[id].pos;
		}

		inline static GraphLink getLink(uint32_t id)
		{
			return m_graphLink[id];
		}

		uint8_t m_way[GRAPH_NUM_NODE];
		int m_wayCount;

	protected:
		struct GrapgDijkstraInfo m_info[GRAPH_NUM_NODE];
		bool m_validLinks[GRAPH_NUM_LINK];
		static const struct GraphNode m_graphNode[GRAPH_NUM_NODE];
		static const struct GraphLink m_graphLink[GRAPH_NUM_LINK];
};

#endif
