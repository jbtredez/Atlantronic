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
static uint16_t encoders_right_old;
static uint16_t encoders_left_old;

static float v_distance;   //!< en "m / unité de temps"
static float v_rotate;     //!< en "rd / unité de temps"

static struct vect_pos pos;

static int robot_state_module_init()
{
	encoders_right = encoders_get_right();
	encoders_left = encoders_get_left();
	encoders_right_old = encoders_right;
	encoders_left_old = encoders_left;

	pos.x = 0;
	pos.y = 0;
	pos.alpha = 0;

	return 0;
};

module_init(robot_state_module_init, INIT_ROBOT_STATE);

void robot_state_update_odometry()
{
	//! @todo lock
	encoders_right = encoders_get_right();
	encoders_left = encoders_get_left();

	float delta_right = (float) ((int16_t) (encoders_right - encoders_right_old) );
	float delta_left  = (float) ((int16_t) (encoders_left  - encoders_left_old ) );

	delta_right *= PARAM_RIGHT_ODO_WHEEL_RADIUS * PARAM_RIGHT_ODO_WHEEL_WAY;
	delta_left  *= PARAM_LEFT_ODO_WHEEL_RADIUS  * PARAM_LEFT_ODO_WHEEL_WAY;

	v_distance = PARAM_DIST_ODO_GAIN * (delta_right + delta_left);
	v_rotate   = PARAM_ROT_ODO_GAIN  * (delta_right - delta_left);

	//! @todo voir / unités : en m/s, v_distance = v_distance / te
	//! @todo voir / unités : en m/s, v_rotate   = v_rotate / te

	encoders_right_old = encoders_right;
	encoders_left_old  = encoders_left;

	pos.x += v_distance * cos(pos.alpha);
	pos.y += v_distance * cos(pos.alpha);
	pos.alpha += v_rotate;
	//! @todo unlock
}

struct vect_pos robot_state_get_position()
{
	struct vect_pos p;
	//! @todo lock
	p = pos;
	//! @todo unlock
	return p;
}
