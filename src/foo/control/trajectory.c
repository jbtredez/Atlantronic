//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "control/trajectory.h"
#include "kernel/robot_parameters.h"
#include "location/location.h"
#include "detection.h"

#define TRAJECTORY_POS_REACHED_TOLERANCE_X        2.0f
#define TRAJECTORY_POS_REACHED_TOLERANCE_ALPHA   0.02f

float find_rotate(float debut, float fin)
{
	float alpha = fin - debut;
	alpha = fmodf(alpha, 2*PI); // Retour dans [-2*PI;2*PI]

	// Retour dans [-PI;PI] si neccessaire
	if (alpha > PI)
	{
		alpha -= 2*PI;
	}
	else if (alpha < -PI)
	{
		alpha += 2*PI;
	}

	return alpha;
}

void trajectory_init_rotate(struct trajectory* t, float angle)
{
	trapeze_reset(&t->trapeze_av, 0, 0);
	trapeze_reset(&t->trapeze_rot, 0, 0);

	t->pos_cons = location_get_position();

	t->angle = angle;
	t->dist = 0;
	t->dest_cons = t->pos_cons;
	t->dest_cons.alpha += angle;
	t->dest_cons.ca = cosf(t->dest_cons.alpha);
	t->dest_cons.sa = sinf(t->dest_cons.alpha);
}

void trajectory_init_rotate_to(struct trajectory* t, float angle)
{
	trapeze_reset(&t->trapeze_av, 0, 0);
	trapeze_reset(&t->trapeze_rot, 0, 0);

	t->pos_cons = location_get_position();

	t->angle = find_rotate(t->pos_cons.alpha, angle);
	t->dist = 0;
	t->dest_cons = t->pos_cons;
	t->dest_cons.alpha += t->angle;
	t->dest_cons.ca = cosf(t->dest_cons.alpha);
	t->dest_cons.sa = sinf(t->dest_cons.alpha);
}

void trajectory_init_straight(struct trajectory* t, float dist)
{
	trapeze_reset(&t->trapeze_av, 0, 0);
	trapeze_reset(&t->trapeze_rot, 0, 0);

	t->pos_cons = location_get_position();

	t->dist = dist;
	t->angle = 0;
	t->dest_cons = t->pos_cons;
	t->dest_cons.x += t->dest_cons.ca * dist;
	t->dest_cons.y += t->dest_cons.sa * dist;
}

void trajectory_init_goto(struct trajectory* t, float x, float y, float dist, enum trajectory_way sens)
{
	trapeze_reset(&t->trapeze_av, 0, 0);
	trapeze_reset(&t->trapeze_rot, 0, 0);

	t->pos_cons = location_get_position();

	float dx = x - t->pos_cons.x;
	float dy = y - t->pos_cons.y;
	t->dist = sqrtf(dx*dx+dy*dy) - dist;
	float a = atan2f(dy, dx);

	if(sens == TRAJECTORY_FORWARD)
	{
		t->angle = find_rotate(t->pos_cons.alpha, a);
	}
	else if(sens == TRAJECTORY_BACKWARD)
	{
		t->angle = find_rotate(t->pos_cons.alpha, a + PI);
		t->dist *= -1;
	}
	else
	{
		float angle_forward = find_rotate(t->pos_cons.alpha, a);
		float angle_backward = find_rotate(t->pos_cons.alpha, a + PI);

		if ( fabsf(angle_forward) > fabsf(angle_backward))
		{
			t->angle = angle_backward;
			t->dist *= -1;
		}
		else
		{
			t->angle = angle_forward;
		}
	}

	t->dest_cons.alpha = t->pos_cons.alpha + t->angle;
	t->dest_cons.ca = cosf(t->dest_cons.alpha);
	t->dest_cons.sa = sinf(t->dest_cons.alpha);
	t->dest_cons.x = t->pos_cons.x + t->dist * t->dest_cons.ca;
	t->dest_cons.y = t->pos_cons.y + t->dist * t->dest_cons.sa;
}

int trajectory_compute(struct trajectory* t, struct vect_pos* pos_mes)
{
	int res = 0;
	if(t->angle)
	{
		if(	fabsf(t->dest_cons.alpha - pos_mes->alpha) < TRAJECTORY_POS_REACHED_TOLERANCE_ALPHA)
		{
			t->angle = 0;
			t->pos_cons.alpha = t->dest_cons.alpha;
			t->pos_cons.ca = t->dest_cons.ca;
			t->pos_cons.sa = t->dest_cons.sa;
			trapeze_reset(&t->trapeze_rot, 0, 0);
		}
		else
		{
			trapeze_apply(&t->trapeze_rot, t->angle);
			t->pos_cons.alpha += t->trapeze_rot.v;
			t->pos_cons.ca = cosf(t->pos_cons.alpha);
			t->pos_cons.sa = sinf(t->pos_cons.alpha);
		}
	}
	else if(t->dist)
	{
		float ex = pos_mes->ca  * (t->dest_cons.x - pos_mes->x) + pos_mes->sa * (t->dest_cons.y - pos_mes->y);

		if( fabsf(ex) < TRAJECTORY_POS_REACHED_TOLERANCE_X)
		{
			t->dist = 0;
			t->pos_cons = t->dest_cons;
			trapeze_reset(&t->trapeze_av, 0, 0);
		}
		else
		{
			float distance = t->dist;
			if(distance > 0)
			{
				struct vect_pos front_obj;
				struct vect_pos front_obj_robot;
				detection_get_front_object(&front_obj);
				pos_table_to_robot(&t->pos_cons, &front_obj, &front_obj_robot);
				float dist = front_obj_robot.x - PARAM_LEFT_CORNER_X - 50;
				if(dist < 0)
				{
					dist = 0;
				}

				if(dist < ex)
				{
					distance = t->trapeze_av.s + dist;
				}
			}

			trapeze_apply(&t->trapeze_av, distance);

			t->pos_cons.x += t->trapeze_av.v * t->pos_cons.ca;
			t->pos_cons.y += t->trapeze_av.v * t->pos_cons.sa;
		}
	}
	else
	{
		res = 1;
	}

	return res;
}