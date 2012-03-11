#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kernel/math/vect2.h"
#include "kernel/math/trigo.h"
#include "foo/graph.h"

//!< le but est de générer les données du graph du code du robot
//!< à partir d'une liste de noeud et des liens bidirectionels

const struct fx_vect2 pt[] =
{
	{  -400   * 65536,  700   * 65536},
	{               0,  700   * 65536},
	{   400   * 65536,  700   * 65536},
	{  -400   * 65536,  400   * 65536},
	{               0,  400   * 65536},
	{   400   * 65536,  400   * 65536},
	{ -762.5f * 65536, 312.5f * 65536},
	{  762.5f * 65536, 312.5f * 65536},
	{ -1250   * 65536,  250   * 65536},
	{  1250   * 65536,  250   * 65536},
	{ -1250   * 65536,              0},
	{  -800   * 65536,              0},
	{   800   * 65536,              0},
	{ 1250    * 65536,              0},
	{ -400    * 65536, -400   * 65536},
	{               0, -400   * 65536},
	{ 400     * 65536, -400   * 65536},
	{ -400    * 65536, -700   * 65536},
	{               0, -700   * 65536},
	{ 400     * 65536, -700   * 65536},
};

struct link
{
	uint8_t a;
	uint8_t b;
};

const struct link link[] =
{
	{  0,  1},
	{  0,  3},
	{  1,  2},
	{  1,  4},
	{  2,  5},
	{  3,  4},
	{  3,  6},
	{  4,  5},
	{  5,  7},
	{  6,  8},
	{  6, 11},
	{  7,  9},
	{  7, 12},
	{  8, 10},
	{  9, 13},
	{ 10, 11},
	{ 11, 14},
	{ 12, 13},
	{ 12, 16},
	{ 14, 15},
	{ 14, 17},
	{ 15, 16},
	{ 15, 18},
	{ 16, 19},
	{ 17, 18},
	{ 18, 19},
};

int main()
{
	int num_nodes = sizeof(pt) / sizeof(pt[0]);
	int num_link = sizeof(link) / sizeof(link[0]);
	int i;

	printf("GRAPH_NUM_NODE %d\n", num_nodes);
	printf("GRAPH_NUM_LINK %d\n", 2*num_link);

	struct graph_node* gnode = malloc( sizeof(struct graph_node) * num_nodes);
	struct graph_link* glink = malloc( sizeof(struct graph_link) * 2 * num_link);

	for( i = 0; i < num_nodes; i++)
	{
		gnode[i].pos = pt[i];
		gnode[i].link_id = 255;
		gnode[i].link_num = 0;
	}

	// verification des données
	int max = 0;
	for( i = 0; i < num_link; i++)
	{
		if( link[i].a >= link[i].b)
		{
			printf("erreur - link[%d].a >= link[%d].b (%d > %d)\n", i, i, link[i].a, link[i].b);
			return -1;
		}
		if(link[i].a >= max)
		{
			max = link[i].a;
		}
		else
		{
			printf("erreur - liens non ordonés\n");
			return -1;
		}
	}

	// calcul des liens
	// si on peut passer dans un sens, ça marche aussi dans l'autre
	int j;
	int k;
	for( i = 0, k=0; k < num_link; k++)
	{
		j = i-1;
		while(j >= 0 && (glink[j].a > link[k].a || (glink[j].a == link[k].a && glink[j].b > link[k].b)))
		{
			glink[j+1] = glink[j];
			j--;
		}
		glink[j+1].a = link[k].a;
		glink[j+1].b = link[k].b;
		i++;

		j = i-1;
		while(j >= 0 && (glink[j].a > link[k].b || (glink[j].a == link[k].b && glink[j].b > link[k].a)))
		{
			glink[j+1] = glink[j];
			j--;
		}
		glink[j+1].b = link[k].a;
		glink[j+1].a = link[k].b;
		i++;
	}

	for(i = 0; i < 2*num_link; i++)
	{
		uint8_t a = glink[i].a;
		uint8_t b = glink[i].b;
		uint64_t dx = gnode[b].pos.x - gnode[a].pos.x;
		uint64_t dy = gnode[b].pos.y - gnode[a].pos.y;
		glink[i].dist = (uint16_t) rint((sqrt(dx * dx + dy * dy)/65536.0f));
		glink[i].alpha = fx_atan2(dy, dx);
	}

	for(i = 0; i < 2*num_link; i++)
	{
		if(i < gnode[glink[i].a].link_id)
		{
			gnode[glink[i].a].link_id = i;
		}
		gnode[glink[i].a].link_num++;
	}

	printf("\n//!< noeuds du graph\n");
	printf("const struct graph_node graph_node[GRAPH_NUM_NODE] =\n{\n");
	for( i = 0; i < num_nodes; i++)
	{
		printf("\t{{ %10d, %10d}, %2u, %2u},\n", gnode[i].pos.x, gnode[i].pos.y, gnode[i].link_id, gnode[i].link_num);
	}
	printf("};\n");

	printf("\n//!< liens du graph.\n");
	printf("const struct graph_link graph_link[GRAPH_NUM_LINK] =\n{\n");
	for( i = 0; i < 2*num_link; i++)
	{
		printf("\t{%2u, %2u, %4u, %10uu},\n", glink[i].a, glink[i].b, glink[i].dist, glink[i].alpha);
	}
	printf("};\n");

	if(gnode)
	{
		free(gnode);
	}

	if( glink )
	{
		free(glink);
	}

	return 0;
}
