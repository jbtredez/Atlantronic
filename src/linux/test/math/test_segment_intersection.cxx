//! @file test_trapeze.c
//! @brief Test functions of trapeze.c
//! @author Atlantronic

#include "kernel/math/segment_intersection.h"
#include <stdio.h>

int error_count;

void check_segment_intersection(int id, vect2 a, vect2 b, vect2 c, vect2 d, int res_expected, vect2 h_expected)
{
	vect2 h;
	int res = segment_intersection(a, b, c, d, &h);

	if(res == res_expected && (( res == 0 && h.x == h_expected.x && h.y == h_expected.y) || res != 0) )
	{
		printf("TEST %d OK\n", id);
	}
	else
	{
		printf("TEST %d KO <------\n", id);
		printf("expected : res = %d, h.x = %f, h.y = %f\n", res_expected, h_expected.x, h_expected.y);
		printf("result   : res = %d, h.x = %f, h.y = %f\n", res, h.x, h.y);
		error_count++;
	}
}

int main()
{
	vect2 a;
	vect2 b;
	vect2 c;
	vect2 d;
	vect2 h;

	error_count = 0;

	// ab vertical
	// cd horizontal
	a.x = -1;
	a.y = -5;
	b.x = a.x;
	b.y = -a.y;
	c.x = -8;
	c.y =  0;
	d.x = -c.x;
	d.y = c.y;
	h.x = -1;
	h.y = 0;

	check_segment_intersection(1, a, b, c, d, 0, h);
	check_segment_intersection(2, b, a, c, d, 0, h);
	check_segment_intersection(3, a, b, d, c, 0, h);
	check_segment_intersection(4, b, a, d, c, 0, h);
	check_segment_intersection(5, c, d, a, b, 0, h);

	a.x = 4176 + 650/65536.0f;
	a.y = -( 754 + 49858/65536.0f );
	b.x = 4004 + 26864/65536.0f;
	b.y = -( 885 + 52479/65536.0f);
	c.x = 4103 + 26864/65536.0f;
	c.y =  - ( 817 + 47236/65536.0f);
	d.x = 4084 + 10480/65536.0f;
	d.y = - ( 831 + 1361/65536.0f);
	h.x = 4000 + 650/65536.0f;
	h.y = -(889 + 10536/65536.0f);

	check_segment_intersection(6, a, b, c, d, -1, h);

	a.x = 3979 + 7204/65536.0f;
	a.y = -( 905 + 7915/65536.0f );
	b.x = 4004 + 26864/65536.0f;
	b.y = -( 885 + 52479/65536.0f);
	c.x = 3905 + 26864/65536.0f;
	c.y =  - ( 954 + 34129/65536.0f);
	d.x = 4084 + 10480/65536.0f;
	d.y = - ( 831 + 1361/65536.0f);
	h.x = 4000.010254;
	h.y = -889.160461;

	check_segment_intersection(7, a, b, c, d, 0, h);

	a.x = 0;
	a.y = 0;
	b.x = 4096;
	b.y = 0;
	c.x = 318;
	c.y = 29;
	d.x = 353;
	d.y = -54;
	h.x = 330.228912;
	h.y = 0;

	check_segment_intersection(8, a, b, c, d, 0, h);

	return error_count;
}
