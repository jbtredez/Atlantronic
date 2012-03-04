//! @file odometry.c
//! @brief Odometry
//! @author Atlantronic

#include "location/odometry.h"
#include "encoders.h"
#include "kernel/FreeRTOS.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/robot_parameters.h"
#include "kernel/portmacro.h"
#include "kernel/math/trigo.h"

static uint16_t encoders_right;
static uint16_t encoders_left;
static xSemaphoreHandle odometry_mutex;
static struct kinematics odometry_kinematics;

static int odometry_module_init()
{
	encoders_right = encoders_get(ENCODERS_MOT_RIGHT);
	encoders_left = encoders_get(ENCODERS_MOT_LEFT);

	odometry_kinematics.x = 0;
	odometry_kinematics.y = 0;
	odometry_kinematics.alpha = 0;
	odometry_kinematics.ca = 1 << 30;
	odometry_kinematics.sa = 0;
	odometry_kinematics.v = 0;
	odometry_kinematics.w = 0;

	odometry_mutex = xSemaphoreCreateMutex();

	if( ! odometry_mutex )
	{
		return ERR_INIT_ODOMETRY;
	}

	return 0;
};

module_init(odometry_module_init, INIT_ODOMETRY);

void odometry_update()
{
	xSemaphoreTake(odometry_mutex, portMAX_DELAY);
	uint16_t enc_right_old = encoders_right;
	uint16_t enc_left_old  = encoders_left;
	encoders_right = encoders_get(ENCODERS_MOT_RIGHT);
	encoders_left = encoders_get(ENCODERS_MOT_LEFT);

	// soustraction en 16 bits pour le debordement
	int32_t delta_right = (int32_t) ((int16_t) (encoders_right - enc_right_old) );
	int32_t delta_left  = (int32_t) ((int16_t) (encoders_left  - enc_left_old ) );

	delta_right *= PARAM_RIGHT_ODO_WHEEL_RADIUS_FX * PARAM_RIGHT_ODO_WHEEL_WAY;
	delta_left  *= PARAM_LEFT_ODO_WHEEL_RADIUS_FX  * PARAM_LEFT_ODO_WHEEL_WAY;

	// on evite de perdre en precision lors du calcul
	odometry_kinematics.v = ((int64_t)(delta_right + delta_left) * (int64_t)PI_FX29) >> ( 29 + PARAM_ENCODERS_BIT_RES);
	odometry_kinematics.w = ((int64_t)(delta_right - delta_left) * (int64_t)PARAM_INVERTED_VOIE_FX39) >> ( 29 + PARAM_ENCODERS_BIT_RES);
	odometry_kinematics.x += ((int64_t)odometry_kinematics.v * (int64_t)odometry_kinematics.ca) >> 30;
	odometry_kinematics.y += ((int64_t)odometry_kinematics.v * (int64_t)odometry_kinematics.sa) >> 30;
	odometry_kinematics.alpha += odometry_kinematics.w;
	odometry_kinematics.ca = fx_cos(odometry_kinematics.alpha);
	odometry_kinematics.sa = fx_sin(odometry_kinematics.alpha);
	xSemaphoreGive(odometry_mutex);
}

struct kinematics odometry_get_kinematics()
{
	struct kinematics k;
	xSemaphoreTake(odometry_mutex, portMAX_DELAY);
	k = odometry_kinematics;
	xSemaphoreGive(odometry_mutex);
	return k;
}

void odometry_set_position(const int32_t x, const int32_t y, const int32_t alpha)
{
	int32_t ca = fx_cos(alpha);
	int32_t sa = fx_sin(alpha);

	xSemaphoreTake(odometry_mutex, portMAX_DELAY);
	encoders_right = encoders_get(ENCODERS_MOT_RIGHT);
	encoders_left = encoders_get(ENCODERS_MOT_LEFT);
	odometry_kinematics.x = x;
	odometry_kinematics.y = y;
	odometry_kinematics.alpha = alpha;
	odometry_kinematics.ca = ca;
	odometry_kinematics.sa = sa;
	xSemaphoreGive(odometry_mutex);
}