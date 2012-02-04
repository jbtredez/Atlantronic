//! @file odometry.c
//! @brief Odometry
//! @author Atlantronic

#include "location/odometry.h"
#include "encoders.h"
#include "kernel/module.h"
#include "kernel/robot_parameters.h"
#include "kernel/portmacro.h"
#include "kernel/math/trigo.h"

static uint16_t encoders_right;
static uint16_t encoders_left;

static int32_t odometry_v_dist;    //!< en "2^-16 m / unité de temps"
static int32_t odometry_v_rotate;  //!< en "2^-26 tours / unité de temps"

static struct fx_vect_pos odometry_pos;

static int odometry_module_init()
{
	encoders_right = encoders_get(ENCODERS_MOT_RIGHT);
	encoders_left = encoders_get(ENCODERS_MOT_LEFT);

	odometry_pos.x = 0;
	odometry_pos.y = 0;
	odometry_pos.alpha = 0;
	odometry_pos.ca = 1 << 16;
	odometry_pos.sa = 0;

	odometry_v_dist = 0;
	odometry_v_rotate = 0;

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

	// soustraction en 16 bits pour le debordement
	int32_t delta_right = (int32_t) ((int16_t) (enc_right - enc_right_old) );
	int32_t delta_left  = (int32_t) ((int16_t) (enc_left  - enc_left_old ) );

	delta_right *= PARAM_RIGHT_ODO_WHEEL_RADIUS_FX * PARAM_RIGHT_ODO_WHEEL_WAY;
	delta_left  *= PARAM_LEFT_ODO_WHEEL_RADIUS_FX  * PARAM_LEFT_ODO_WHEEL_WAY;

	// on evite de perdre en precision lors du calcul
	int32_t vd = ((int64_t)(delta_right + delta_left) * (int64_t)PI_FX29) >> ( 29 + PARAM_ENCODERS_BIT_RES);
	int32_t vr = ((int64_t)(delta_right - delta_left) * (int64_t)PARAM_INVERTED_VOIE_FX39) >> ( 29 + PARAM_ENCODERS_BIT_RES);

	// update
	portENTER_CRITICAL();
	encoders_right = enc_right;
	encoders_left  = enc_left;
	odometry_v_dist = vd;
	odometry_v_rotate = vr;
	odometry_pos.x += ((int64_t)vd * (int64_t)odometry_pos.ca) >> 30;
	odometry_pos.y += ((int64_t)vd * (int64_t)odometry_pos.sa) >> 30;
	odometry_pos.alpha += vr;
	odometry_pos.ca = fx_cos(odometry_pos.alpha);
	odometry_pos.sa = fx_sin(odometry_pos.alpha);
	portEXIT_CRITICAL();
}

struct fx_vect_pos odometry_get_position()
{
	struct fx_vect_pos p;
	portENTER_CRITICAL();
	p = odometry_pos;
	portEXIT_CRITICAL();
	return p;
}

void odometry_set_position(const struct fx_vect_pos pos)
{
	portENTER_CRITICAL();
	encoders_right = encoders_get(ENCODERS_MOT_RIGHT);
	encoders_left = encoders_get(ENCODERS_MOT_LEFT);
	odometry_pos = pos;
	portEXIT_CRITICAL();
}

int32_t odometry_get_speed_curv_abs()
{
	int32_t v;
	portENTER_CRITICAL();
	v = odometry_v_dist;
	portEXIT_CRITICAL();
	return v;
}

int32_t odometry_get_speed_rot()
{
	int32_t v;
	portENTER_CRITICAL();
	v = odometry_v_rotate;
	portEXIT_CRITICAL();
	return v;
}

