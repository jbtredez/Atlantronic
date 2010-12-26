//! @file odometry.c
//! @brief Odometry
//! @author Jean-Baptiste Trédez

#include "location/odometry.h"
#include "io/encoders.h"
#include "module.h"
#include "robot_parameters.h"
#include "portmacro.h"

static uint16_t encoders_right;
static uint16_t encoders_left;

static float v_distance;   //!< en "m / unité de temps"
static float v_rotate;     //!< en "rd / unité de temps"

static struct vect_pos odometry_pos;

static int odometry_module_init()
{
	encoders_right = encoders_get(ENCODERS_MOT_RIGHT);
	encoders_left = encoders_get(ENCODERS_MOT_LEFT);

	odometry_pos.x = 0;
	odometry_pos.y = 0;
	odometry_pos.alpha = 0;
	odometry_pos.ca = 1;
	odometry_pos.sa = 0;

	v_distance = 0;
	v_rotate = 0;

	return 0;
};

module_init(odometry_module_init, INIT_ODOMETRY);

void odometry_update()
{
	portENTER_CRITICAL();
	uint16_t enc_right_old = encoders_right;
	uint16_t enc_left_old  = encoders_left;
	uint16_t enc_right = encoders_get(ENCODERS_MOT_RIGHT);
	uint16_t enc_left = encoders_get(ENCODERS_MOT_LEFT);
	portEXIT_CRITICAL();

	float delta_right = (float) ((int16_t) (enc_right - enc_right_old) );
	float delta_left  = (float) ((int16_t) (enc_left  - enc_left_old ) );

	delta_right *= PARAM_RIGHT_ODO_WHEEL_RADIUS * PARAM_RIGHT_ODO_WHEEL_WAY;
	delta_left  *= PARAM_LEFT_ODO_WHEEL_RADIUS  * PARAM_LEFT_ODO_WHEEL_WAY;

	float v_d = PARAM_DIST_ODO_GAIN * (delta_right + delta_left);
	float v_r = PARAM_ROT_ODO_GAIN  * (delta_right - delta_left);

	float dx = v_d * odometry_pos.ca;
	float dy = v_d * odometry_pos.sa;
	float da = v_r;

	// update
	portENTER_CRITICAL();
	encoders_right = enc_right;
	encoders_left  = enc_left;
	v_distance = v_d;
	v_rotate   = v_r;
	odometry_pos.x += dx;
	odometry_pos.y += dy;
	odometry_pos.alpha += da;
	odometry_pos.ca = cos(odometry_pos.alpha);
	odometry_pos.sa = sin(odometry_pos.alpha);
	portEXIT_CRITICAL();
}

struct vect_pos odometry_get_position()
{
	struct vect_pos p;
	portENTER_CRITICAL();
	p = odometry_pos;
	portEXIT_CRITICAL();
	return p;
}

float odometry_get_speed_curv_abs()
{
	float v;
	portENTER_CRITICAL();
	v = v_distance;
	portEXIT_CRITICAL();
	return v;
}

float odometry_get_speed_rot()
{
	float v;
	portENTER_CRITICAL();
	v = v_rotate;
	portEXIT_CRITICAL();
	return v;
}

