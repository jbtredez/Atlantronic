#define WEAK_MOTION
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "kernel/control.h"
#include "Motion.h"
#include "kernel/location/location.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/pwm.h"
#include "kernel/fault.h"
#include "kernel/driver/power.h"
#include "disco/robot_parameters.h"
#include "kernel/match.h"
#include "MotionDisabledState.h"
#include "MotionTryEnableState.h"
#include "MotionEnabledState.h"
#include "MotionActuatorKinematicsState.h"
#include "MotionSpeedState.h"
#include "MotionTrajectoryState.h"
#include "MotionInterruptingState.h"
#include "kernel/math/findRotation.h"

static MotionDisabledState motionDisabledState;
static MotionTryEnableState motionTryEnableState;
static MotionEnabledState motionEnabledState;
static MotionActuatorKinematicsState motionActuatorKinematicsState;
static MotionSpeedState motionSpeedState;
static MotionTrajectoryState motionTrajectoryState;
static MotionInterruptingState motionInterrputingState;

StateMachineState* Motion::m_motionStates[MOTION_MAX_STATE] = {
	&motionDisabledState,
	&motionTryEnableState,
	&motionEnabledState,
	&motionActuatorKinematicsState,
	&motionSpeedState,
	&motionTrajectoryState,
	&motionInterrputingState
};

int Motion::init(Detection* detection, Location* location, KinematicsModel* kinematicsModel, MotorInterface* motorLeft, MotorInterface* motorRight, EncoderInterface* encoderLeft, EncoderInterface* encoderRight)
{
	m_location = location;
	m_detection = detection;
	m_kinematicsModel = kinematicsModel;

	m_mutex = xSemaphoreCreateMutex();
	if( ! m_mutex )
	{
		return ERR_INIT_CONTROL;
	}

	m_linearSpeedCheck.init(1000, 100);
	m_xPid.init(0, 0, 0, 100);// TODO voir saturation
	m_yPid.init(0, 0, 0, 1);// TODO voir saturation + regler
	m_thetaPid.init(0, 0, 0, 1); // TODO voir saturation
	m_motionStateMachine.init(m_motionStates, MOTION_MAX_STATE, this);

	m_anticoOn = true;
	m_wantedState = MOTION_UNKNOWN_STATE;
	m_motionMotor[MOTION_MOTOR_LEFT] = motorLeft;
	m_motionMotor[MOTION_MOTOR_RIGHT] = motorRight;
	m_motionEncoder[MOTION_MOTOR_LEFT] = encoderLeft;
	m_motionEncoder[MOTION_MOTOR_RIGHT] = encoderRight;

	usb_add_cmd(USB_CMD_MOTION_SET_SPEED, &Motion::cmd_set_speed, this);
	usb_add_cmd(USB_CMD_MOTION_SET_MAX_CURRENT, &Motion::cmd_set_max_current, this);
	usb_add_cmd(USB_CMD_MOTION_ENABLE, &Motion::cmd_enable, this);
	usb_add_cmd(USB_CMD_MOTION_SET_ACTUATOR_KINEMATICS, &Motion::cmd_set_actuator_kinematics, this);
	usb_add_cmd(USB_CMD_MOTION_PRINT_PARAM, &Motion::cmd_print_param, this);
	usb_add_cmd(USB_CMD_MOTION_PARAM, &Motion::cmd_set_param, this);

	return 0;
}

void Motion::compute()
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);

// si odometrie sur roues motrices (utiliser uniquement pour tests sur cale)
#if 0
	int motor_mes_valid = 1;

	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		if( ! m_motionMotor[i]->is_op_enable() )
		{
			motor_mes_valid = 0;
		}

		m_kinematicsMes[i] = m_motionMotor[i]->kinematics;
	}

	if( motor_mes_valid )
	{
		// mise à jour de la position
		m_location->update(m_kinematicsMes, CONTROL_DT);
	}
#else
	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		m_motionEncoder[i]->update(CONTROL_DT);
		m_kinematicsMes[i].v = m_motionEncoder[i]->getSpeed();
	}

	m_location->update(m_kinematicsMes, CONTROL_DT);
#endif
	m_posMes = m_location->getPosition();
	m_speedMes = m_location->getSpeed();

	m_motionStateMachine.execute();

	xSemaphoreGive(m_mutex);
}

void Motion::motionUpdateMotors()
{
	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		if( m_kinematics[i].mode == KINEMATICS_SPEED )
		{
			m_motionMotor[i]->set_speed(m_kinematics[i].v);
		}
		else if( m_kinematics[i].mode == KINEMATICS_POSITION )
		{
			m_motionMotor[i]->set_position(m_kinematics[i].pos);
		}
	}

	if( match_get_go() )
	{
		// ambiance selon la vitesse des roues
		float pwm = fabsf(m_kinematics[RIGHT_WHEEL].v) / 1000;
		if( pwm > 1)
		{
			pwm = 1;
		}
		//pwm_set(PWM_1, pwm);
		pwm = fabsf(m_kinematics[LEFT_WHEEL].v) / 1000;
		if( pwm > 1)
		{
			pwm = 1;
		}
		//pwm_set(PWM_2, pwm);
	}

	m_speedCmd = m_kinematicsModel->computeSpeed(m_kinematics);
}

void Motion::enableAntico(bool enable)
{
	m_anticoOn = enable;
}

float Motion::motionComputeTime(float ds, KinematicsParameters param)
{
	ds = fabsf(ds);
	float ainv = 1 / param.aMax + 1 / param.dMax;
	float v = sqrtf( 2 * ds / ainv); // vitesse si pas de palier a vMax
	if( v  < param.vMax )
	{
		// on n'a pas le temps d'atteindre vMax
		return v * ainv;
	}

	return ds / param.vMax + 0.5f * param.vMax * ainv;
}

void Motion::cmd_print_param(void* arg, void* /*data*/)
{
	Motion* m = (Motion*) arg;
	log_format(LOG_INFO, "axe x     : kp %d ki %d kd %d", (int)(m->m_xPid.kp), (int)(m->m_xPid.ki), (int)(m->m_xPid.kd));
	log_format(LOG_INFO, "axe theta : kp %d ki %d kd %d", (int)(m->m_thetaPid.kp), (int)(m->m_thetaPid.ki), (int)(m->m_thetaPid.kd));
}

void Motion::cmd_set_param(void* arg, void* data)
{
	Motion* m = (Motion*) arg;
	struct motion_cmd_param_arg* cmd = (struct motion_cmd_param_arg*) data;
	m->m_xPid.kp = cmd->kp_av;
	m->m_xPid.ki = cmd->ki_av;
	m->m_xPid.kd = cmd->kd_av;
	m->m_thetaPid.kp = cmd->kp_rot;
	m->m_thetaPid.ki = cmd->ki_rot;
	m->m_thetaPid.kd = cmd->kd_rot;
}

void Motion::cmd_set_speed(void* arg, void* data)
{
	Motion* m = (Motion*) arg;
	struct motion_cmd_set_speed_arg* cmd = (struct motion_cmd_set_speed_arg*) data;
	m->setSpeed(cmd->u, cmd->v);
}

void Motion::cmd_set_max_current(void* arg, void* data)
{
	Motion* m = (Motion*) arg;
	struct motion_cmd_set_max_driving_current_arg* cmd = (struct motion_cmd_set_max_driving_current_arg*) data;
	m->setMaxDrivingCurrent(cmd->maxDrivingCurrent);
}

void Motion::cmd_enable(void* arg, void* data)
{
	Motion* m = (Motion*) arg;
	struct motion_cmd_enable_arg* cmd_arg = (struct motion_cmd_enable_arg*) data;
	m->enable(cmd_arg->enable != 0);
}

void Motion::cmd_set_actuator_kinematics(void* arg, void* data)
{
	Motion* m = (Motion*) arg;
	struct motion_cmd_set_actuator_kinematics_arg* cmd = (struct motion_cmd_set_actuator_kinematics_arg*) data;
	m->setActuatorKinematics(*cmd);
}

void Motion::enable(bool enable)
{	
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	if( enable && m_motionStateMachine.getCurrentState() == MOTION_DISABLED )
	{
		m_wantedState = MOTION_ENABLED;
	}
	else if( ! enable )
	{
		m_wantedState = MOTION_DISABLED;
	}
	xSemaphoreGive(m_mutex);
}

void Motion::setMaxDrivingCurrent(float maxCurrent)
{
	m_motionMotor[0]->set_max_current(maxCurrent);
	m_motionMotor[1]->set_max_current(maxCurrent);
}

void Motion::clearTrajectory()
{
	// arret au cas ou la trajectoire est en cours d'utilisation
	stop();
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_path.clear();
	xSemaphoreGive(m_mutex);
}

void Motion::addTrajectoryPoints(PathPoint* pt, int size)
{
	// pas d arret, on peut ajouter la fin de la trajectoire alors qu'elle est en cours d execution
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_path.add(pt, size);
	xSemaphoreGive(m_mutex);
}

void Motion::setTrajectory(PathPoint* pt, int size)
{
	// arret au cas ou la trajectoire est en cours d'utilisation avant le clear()
	stop();
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_path.clear();
	m_path.add(pt, size);
	xSemaphoreGive(m_mutex);
}

void Motion::startTrajectory(const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	if( m_path.getCount() < 2)
	{
		log_format(LOG_ERROR, "path is invalid (%d points)", m_path.getCount());
	}
	else
	{
		m_wantedState = MOTION_TRAJECTORY;
		m_wantedLinearParam = linearParam;
		m_wantedAngularParam = angularParam;
		m_status = MOTION_UPDATING_TRAJECTORY;
	}
	xSemaphoreGive(m_mutex);
}

VectPlan Motion::getLastPathPoint()
{
	VectPlan res;
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	if( m_path.getCount() )
	{
		res = m_path.getLastPoint();
	}
	else
	{
		res = m_posMes;
	}
	xSemaphoreGive(m_mutex);
	return res;
}

void Motion::setSpeed(VectPlan u, float v)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);

	m_wantedState = MOTION_SPEED;
	m_u = u;
	m_v = v;
	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		m_kinematics[i] = m_kinematicsMes[i];
	}

	xSemaphoreGive(m_mutex);
}

void Motion::stop()
{
	setSpeed(VectPlan(1,0,0), 0);
}

void Motion::setActuatorKinematics(struct motion_cmd_set_actuator_kinematics_arg cmd)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_wantedState = MOTION_ACTUATOR_KINEMATICS;
	m_wantedKinematics = cmd;

	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		if( cmd.mode[i] != KINEMATICS_POSITION && cmd.mode[i] != KINEMATICS_SPEED)
		{
			log_format(LOG_ERROR, "unknown mode %d for actuator %i", cmd.mode[i], i);
			m_wantedState = MOTION_UNKNOWN_STATE;
		}
	}

	xSemaphoreGive(m_mutex);
}

void Motion::getState(enum motion_state* state, enum motion_status* status, enum motion_state* wanted_state)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	*state = (enum motion_state)m_motionStateMachine.getCurrentState();
	*status = m_status;
	*wanted_state = m_wantedState;
	xSemaphoreGive(m_mutex);
}

void Motion::updateUsbData(struct control_usb_data* data)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);

	data->motion_state = m_motionStateMachine.getCurrentState();
	data->cons = m_path.getLastPosCmd();
	data->wanted_pos = m_path.getLastPoint();

	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		data->cons_motors_v[i] = m_kinematics[i].v;// * m_motionMotor[i]->inputGain;
		data->mes_motors[i] = m_kinematicsMes[i];
		//data->mes_motors[i].v *= m_motionMotor[i]->inputGain;
		data->mes_motor_current[i] = m_motionMotor[i]->current;
	}

	xSemaphoreGive(m_mutex);
}
