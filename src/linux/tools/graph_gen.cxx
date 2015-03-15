#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kernel/math/vect2.h"
#include "kernel/math/fx_math.h"
#include "kernel/motion/graph.h"

//!< le but est de générer les données du graph du code du robot
//!< à partir d'une liste de noeud et des liens bidirectionels

const Vect2 pt[] =
{
	Vect2( -1300  ,  700  ),   //  0
	Vect2(  -800  ,  700  ),   //  1
	Vect2( -1300  ,  400  ),   //  2
	Vect2(  -800  ,  400  ),   //  3
	Vect2(  -800  ,    0  ),   //  4
	Vect2(  -800  , -400  ),   //  5
	Vect2( -1300  , -400  ),   //  6
	Vect2(  -800  , -700  ),   //  7
	Vect2( -1300  , -700  ),   //  8
	Vect2(  -400  , -700  ),   //  9
	Vect2(  -400  , -400  ),   // 10
	Vect2(  -400  ,    0  ),   // 11
	Vect2(  -400  ,  200  ),   // 12

	Vect2(     0  ,  200  ),   // 13
	Vect2(     0  ,    0  ),   // 14
	Vect2(     0  , -400  ),   // 15
	Vect2(     0  , -600  ),   // 16

	Vect2(  1300  ,  700  ),   // 17
	Vect2(   800  ,  700  ),   // 18
	Vect2(  1300  ,  400  ),   // 19
	Vect2(   800  ,  400  ),   // 20
	Vect2(   800  ,    0  ),   // 21
	Vect2(   800  , -400  ),   // 22
	Vect2(  1300  , -400  ),   // 23
	Vect2(   800  , -700  ),   // 24
	Vect2(  1300  , -700  ),   // 25
	Vect2(   400  , -700  ),   // 26
	Vect2(   400  , -400  ),   // 27
	Vect2(   400  ,    0  ),   // 28
	Vect2(   400  ,  200  ),   // 29
};

struct link
{
	uint8_t a;
	uint8_t b;
};

const struct link link[] =
{
	{  0,  1 },
	{  0,  2 },
	{  0,  3 },
	{  1,  3 },
	{  2,  3 },
	{  3,  4 },
	{  3, 12 },
	{  4,  5 },
	{  4, 11 },
	{  5,  6 },
	{  5,  7 },
	{  5,  8 },
	{  5, 10 },
	{  6,  8 },
	{  7,  8 },
	{  7,  9 },
	{  9, 10 },
	{  9, 16 },
	{ 10, 11 },
	{ 10, 15 },
	{ 11, 12 },
	{ 11, 14 },
	{ 12, 13 },

	{ 13, 14 },
	{ 13, 29 },
	{ 14, 15 },
	{ 14, 28 },
	{ 15, 16 },
	{ 15, 27 },
	{ 16, 26 },

	{ 17, 18 },
	{ 17, 19 },
	{ 17, 20 },
	{ 18, 20 },
	{ 19, 20 },
	{ 20, 21 },
	{ 20, 29 },
	{ 21, 22 },
	{ 21, 28 },
	{ 22, 23 },
	{ 22, 24 },
	{ 22, 25 },
	{ 22, 27 },
	{ 23, 25 },
	{ 24, 25 },
	{ 24, 26 },
	{ 26, 27 },
	{ 27, 28 },
	{ 28, 29 },
};

int main()
{
	int num_nodes = sizeof(pt) / sizeof(pt[0]);
	int num_link = sizeof(link) / sizeof(link[0]);
	int i;

	printf("GRAPH_NUM_NODE %d\n", num_nodes);
	printf("GRAPH_NUM_LINK %d\n", 2*num_link);

	struct graph_node* gnode = (struct graph_node*)malloc( sizeof(struct graph_node) * num_nodes);
	struct graph_link* glink = (struct graph_link*)malloc( sizeof(struct graph_link) * 2 * num_link);

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
		float dx = gnode[b].pos.x - gnode[a].pos.x;
		float dy = gnode[b].pos.y - gnode[a].pos.y;
		glink[i].dist = (uint16_t) rint(sqrtf(dx * dx + dy * dy));
		glink[i].alpha = atan2(dy, dx);
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
		printf("\t{Vect2( %8.2ff, %8.2ff), %2u, %2u},\n", gnode[i].pos.x, gnode[i].pos.y, gnode[i].link_id, gnode[i].link_num);
	}
	printf("};\n");

	printf("\n//!< liens du graph.\n");
	printf("const struct graph_link graph_link[GRAPH_NUM_LINK] =\n{\n");
	for( i = 0; i < 2*num_link; i++)
	{
		printf("\t{%2u, %2u, %4u, %9.6ff},\n", glink[i].a, glink[i].b, glink[i].dist, glink[i].alpha);
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
