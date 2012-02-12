#include <stdio.h>
#include "foo/graph.h"

int main()
{
	int i;
	struct graph_dijkstra_info info[GRAPH_NUM_NODE];
	uint8_t valid_links[GRAPH_NUM_LINK];

	for(i=0; i< GRAPH_NUM_LINK; i++)
	{
		valid_links[i] = 1;
	}

	valid_links[16] = 0; // 6 - 3
	valid_links[9] = 0;  // 3 - 6
	valid_links[37] = 0; // 14 - 15
	valid_links[39] = 0;  // 15 - 14

	int res = graph_dijkstra(14, 0, info, valid_links);

	printf("res = %d\n", res);

	for(i = 0; i<GRAPH_NUM_NODE ; i++)
	{
		printf("%3d : %6d %3d %d\n", i, info[i].dist, info[i].prev_node, info[i].is_best);
	}

	return 0;
}