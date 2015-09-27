#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kernel/math/Vect2.h"
#include "middleware/motion/graph.h"

//!< le but est de générer les données du graph du code du robot
//!< à partir d'une liste de noeud et des liens bidirectionels

#define SYM_POINT(x, y)        Vect2( x, y), Vect2( -x, y)
#define SYM_LINK1(a,b)         { a, b }, {a+1, b+1}
#define SYM_LINK2(a,b,c)       { a, b }, { a, c }, {a+1, b+1}, {a+1, c+1}
#define SYM_LINK3(a,b,c,d)     { a, b }, { a, c }, { a, d }, {a+1, b+1}, {a+1, c+1}, {a+1, d+1}
#define SYM_LINK4(a,b,c,d,e)   { a, b }, { a, c }, { a, d }, { a, e }, {a+1, b+1}, {a+1, c+1}, {a+1, d+1}, {a+1, e+1}

const Vect2 pt[] =
{
	SYM_POINT( -1200  ,  700  ),   //  0
	SYM_POINT( -1200  ,  500  ),   //  2
	SYM_POINT(  -750  ,  700  ),   //  4
    SYM_POINT(  -750  ,  200  ),   //  6
    SYM_POINT(  -700  ,    0  ),   //  8
    SYM_POINT(  -800  , -400  ),   //  10
    SYM_POINT( -1200  , -400  ),   //  12
    SYM_POINT( -1200  , -700  ),   //  14
    SYM_POINT(  -800  , -700  ),   //  16
    SYM_POINT(  -400  , -700  ),   //  18
    SYM_POINT(  -400  , -400  ),   //  20
    SYM_POINT(  -400  ,    0  ),   //  22
    SYM_POINT(  -400  ,  200  ),   //  24

    Vect2(     0  ,  200  ),   // 26
    Vect2(     0  ,    0  ),   // 27
    Vect2(     0  , -400  ),   // 28
    Vect2(     0  , -600  ),   // 29
};

struct link
{
	uint8_t a;
	uint8_t b;
};

const struct link link[] =
{
    SYM_LINK3(0,2,4,6),
    SYM_LINK1(2,6),
    SYM_LINK1(4,6),
    SYM_LINK2(6,8,24),
    SYM_LINK2(8,10,22),
    SYM_LINK3(10,12,16,20),
    SYM_LINK1(12,14),
    SYM_LINK1(14,16),
    SYM_LINK1(16,18),

    { 18, 20 },
    { 18, 29 },
    { 19, 21 },
    { 19, 29 },

    { 20, 22 },
    { 20, 28 },
    { 21, 23 },
    { 21, 28 },
    { 22, 24 },
    { 22, 27 },
    { 23, 25 },
    { 23, 27 },
    { 24, 26 },
    { 25, 26 },
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

	struct GraphNode* gnode = (struct GraphNode*)malloc( sizeof(struct GraphNode) * num_nodes);
	struct GraphLink* glink = (struct GraphLink*)malloc( sizeof(struct GraphLink) * 2 * num_link);

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
	printf("const struct GraphNode Graph::graph_node[GRAPH_NUM_NODE] =\n{\n");
	for( i = 0; i < num_nodes; i++)
	{
		printf("\t{Vect2( %8.2ff, %8.2ff), %2u, %2u},\n", gnode[i].pos.x, gnode[i].pos.y, gnode[i].link_id, gnode[i].link_num);
	}
	printf("};\n");

	printf("\n//!< liens du graph.\n");
	printf("const struct GraphLink Graph::graph_link[GRAPH_NUM_LINK] =\n{\n");
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
