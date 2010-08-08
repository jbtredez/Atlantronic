//! @file robot_state.c
//! @brief State of the robot
//! @author Jean-Baptiste Trédez

#include "robot_state.h"
#include "io/encoders.h"
#include "module.h"
#include "robot_parameters.h"
#include "portmacro.h"

static uint16_t encoders_right;
static uint16_t encoders_left;

static float v_distance;   //!< en "m / unité de temps"
static float v_rotate;     //!< en "rd / unité de temps"

static struct vect_pos pos;

static int robot_state_module_init()
{
	encoders_right = encoders_get_right();
	encoders_left = encoders_get_left();

	pos.x = 0;
	pos.y = 0;
	pos.alpha = 0;

	return 0;
};

module_init(robot_state_module_init, INIT_ROBOT_STATE);

void robot_state_update_odometry()
{
	portENTER_CRITICAL();
	uint16_t enc_right_old = encoders_right;
	uint16_t enc_left_old  = encoders_left;
	portEXIT_CRITICAL();

	uint16_t enc_right = encoders_get_right();
	uint16_t enc_left = encoders_get_left();

	float delta_right = (float) ((int16_t) (enc_right - enc_right_old) );
	float delta_left  = (float) ((int16_t) (enc_left  - enc_left_old ) );

	delta_right *= PARAM_RIGHT_ODO_WHEEL_RADIUS * PARAM_RIGHT_ODO_WHEEL_WAY;
	delta_left  *= PARAM_LEFT_ODO_WHEEL_RADIUS  * PARAM_LEFT_ODO_WHEEL_WAY;

	float v_d = PARAM_DIST_ODO_GAIN * (delta_right + delta_left);
	float v_r = PARAM_ROT_ODO_GAIN  * (delta_right - delta_left);

	float ca = cos(pos.alpha);
	float dx = v_d * ca;
	float dy = v_d * ca;
	float da = v_r;

	// update
	portENTER_CRITICAL();
	encoders_right = enc_right;
	encoders_left  = enc_left;
	v_distance = v_d;
	v_rotate   = v_r;
	pos.x += dx;
	pos.y += dy;
	pos.alpha += da;
	portEXIT_CRITICAL();
}

struct vect_pos robot_state_get_position()
{
	struct vect_pos p;
	portENTER_CRITICAL();
	p = pos;
	portEXIT_CRITICAL();
	return p;
}
