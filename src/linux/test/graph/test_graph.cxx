#include <stdio.h>
#include "middleware/trajectory/Graph.h"

int main()
{
	Graph graph;
	int i;

	for(i=0; i< GRAPH_NUM_LINK; i++)
	{
		graph.setValidLink(i, true);
	}

	graph.setValidLink(16, false); // 6 - 3
	graph.setValidLink(9, false);  // 3 - 6
	graph.setValidLink(37, false); // 14 - 15
	graph.setValidLink(39, false);  // 15 - 14

	int res = graph.dijkstra(14, 0);

	printf("dijkstra : res = %d\n", res);

	/*for(i = 0; i<GRAPH_NUM_NODE ; i++)
	{
		printf("%3d : %6d %3d %d\n", i, graph.m_info[i].dist, graph.m_info[i].prev_node, graph.m_info[i].is_best);
	}*/


	Vect2 pos(-350, -600);
	struct GraphNodeDist node_dist[GRAPH_NUM_NODE];
	graph.computeNodeDistance(pos, node_dist);

	printf("graph_compute_node_distance :\n");

	for(i = 0; i<GRAPH_NUM_NODE ; i++)
	{
		printf("%6d %3d\n", node_dist[i].dist, node_dist[i].id);
	}

	return 0;
}
