#include <stdio.h>
#include "foo/graph.h"

int main()
{
	int i;
	struct graph_dijkstra_info info[GRAPH_NUM_NODE];

	graph_dijkstra(0, 19, info);

	for(i = 0; i<GRAPH_NUM_NODE ; i++)
	{
		printf("%3d : %6d %3d %d\n", i, info[i].dist, info[i].prev_node, info[i].is_best);
	}

	return 0;
}