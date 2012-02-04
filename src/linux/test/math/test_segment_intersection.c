//! @file test_trapeze.c
//! @brief Test functions of trapeze.c
//! @author Atlantronic

#include "kernel/math/segment_intersection.h"
#include <stdio.h>

int error_count;

void check_segment_intersection(int id, struct fx_vect2 a, struct fx_vect2 b, struct fx_vect2 c, struct fx_vect2 d, int res_expected, struct fx_vect2 h_expected)
{
	struct fx_vect2 h;
	int res = segment_intersection(a, b, c, d, &h);

	if(res == res_expected && (( res == 0 && h.x == h_expected.x && h.y == h_expected.y) || res != 0) )
	{
		printf("TEST %d OK\n", id);
	}
	else
	{
		printf("TEST %d KO <------\n", id);
		printf("expected : res = %d, h.x = %d, h.y = %d (%f %f)\n", res_expected, h_expected.x, h_expected.y, h_expected.x / 65536.0f, h_expected.y / 65536.0f);
		printf("result   : res = %d, h.x = %d, h.y = %d (%f %f)\n", res, h.x, h.y, h.x / 65536.0f, h.y / 65536.0f);
		error_count++;
	}
}

int main()
{
	struct fx_vect2 a;
	struct fx_vect2 b;
	struct fx_vect2 c;
	struct fx_vect2 d;
	struct fx_vect2 h;

	error_count = 0;

	// ab vertical
	// cd horizontal
	a.x = -1 << 16;
	a.y = -5 << 16;
	b.x = a.x;
	b.y = -a.y;
	c.x = -8 << 16;
	c.y =  0;
	d.x = -c.x;
	d.y = c.y;
	h.x = -1 << 16;
	h.y = 0;

	check_segment_intersection(1, a, b, c, d, 0, h);
	check_segment_intersection(2, b, a, c, d, 0, h);
	check_segment_intersection(3, a, b, d, c, 0, h);
	check_segment_intersection(4, b, a, d, c, 0, h);
	check_segment_intersection(5, c, d, a, b, 0, h);

	a.x = (4176 << 16) + 650;
	a.y = -( (754 << 16) + 49858 );
	b.x = (4004 << 16) + 26864;
	b.y = -( (885 << 16) + 52479);
	c.x = (4103 << 16) + 26864;
	c.y =  - ( (817 << 16) + 47236);
	d.x = (4084 << 16) + 10480;
	d.y = - ( (831 << 16) + 1361);
	h.x = (4000 << 16) + 650;
	h.y = -((889 << 16) + 10536);

	check_segment_intersection(6, a, b, c, d, -1, h);

	a.x = (3979 << 16) + 7204;
	a.y = -( (905 << 16) + 7915 );
	b.x = (4004 << 16) + 26864;
	b.y = -( (885 << 16) + 52479);
	c.x = (3905 << 16) + 26864;
	c.y =  - ( (954 << 16) + 34129);
	d.x = (4084 << 16) + 10480;
	d.y = - ( (831 << 16) + 1361);
	h.x = (4000 << 16) + 638;
	h.y = -((889 << 16) + 10545);

	check_segment_intersection(7, a, b, c, d, 0, h);

	a.x = 0;
	a.y = 0;
	b.x = 4096 << 16;
	b.y = 0;
	c.x = 318 << 16;
	c.y = 29 << 16;
	d.x = 353 << 16;
	d.y = -54 << 16;
	h.x = 21639168;
	h.y = 0;

	check_segment_intersection(8, a, b, c, d, 0, h);

	return error_count;
}
