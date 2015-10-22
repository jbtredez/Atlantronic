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
#include "kernel/kinematics_model/kinematics_model.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/pwm.h"
#include "kernel/fault.h"
#include "kernel/driver/power.h"
#include "disco/robot_parameters.h"
#include "middleware/detection.h"
#include "kernel/match.h"
#include "MotionDisabledState.h"
#include "MotionTryEnableState.h"
#include "MotionEnabledState.h"
#include "MotionActuatorKinematicsState.h"
#include "MotionSpeedState.h"
#include "MotionTrajectoryState.h"
#include "MotionInterruptingState.h"

Motion motion;

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

static int motion_module_init()
{
	usb_add_cmd(USB_CMD_MOTION_GOTO, &Motion::cmd_goto);
	usb_add_cmd(USB_CMD_MOTION_SET_SPEED, &Motion::cmd_set_speed);
	usb_add_cmd(USB_CMD_MOTION_SET_MAX_CURRENT, &Motion::cmd_set_max_current);
	usb_add_cmd(USB_CMD_MOTION_ENABLE, &Motion::cmd_enable);
	usb_add_cmd(USB_CMD_MOTION_SET_ACTUATOR_KINEMATICS, &Motion::cmd_set_actuator_kinematics);
	usb_add_cmd(USB_CMD_MOTION_PRINT_PARAM, &Motion::cmd_print_param);
	usb_add_cmd(USB_CMD_MOTION_PARAM, &Motion::cmd_set_param);

	return motion.init();
}

module_init(motion_module_init, INIT_MOTION);

Motion::Motion() :
	m_linearSpeedCheck(100, 10),
	m_xPid(2, 1, 0, 100),// TODO voir saturation
	m_thetaPid(8, 1, 0, 1), // TODO voir saturation
	m_motionStateMachine(m_motionStates, MOTION_MAX_STATE, this)
{
	m_anticoOn = true;
	///C'est pour l'allumage des moteur mais c'est le Wanted State
	///m_enableWanted = MOTION_ENABLE_WANTED_UNKNOWN;
	m_wantedState = MOTION_UNKNOWN_STATE;

	m_canMotor[CAN_MOTOR_RIGHT].nodeId = CAN_MOTOR_RIGHT_NODEID;
	m_canMotor[CAN_MOTOR_RIGHT].inputGain = 60 * MOTOR_DRIVING2_RED / (float)(2 * M_PI * DRIVING2_WHEEL_RADIUS);
	m_canMotor[CAN_MOTOR_RIGHT].outputGain = 2 * M_PI * DRIVING2_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING2_RED);
	m_canMotor[CAN_MOTOR_RIGHT].name = "moteur droit";
	m_canMotor[CAN_MOTOR_RIGHT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_0;

	m_canMotor[CAN_MOTOR_LEFT].nodeId = CAN_MOTOR_LEFT_NODEID;
	m_canMotor[CAN_MOTOR_LEFT].inputGain = 60 * MOTOR_DRIVING1_RED / (float)(2 * M_PI * DRIVING1_WHEEL_RADIUS);
	m_canMotor[CAN_MOTOR_LEFT].outputGain = 2 * M_PI * DRIVING1_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING1_RED);
	m_canMotor[CAN_MOTOR_LEFT].name = "moteur gauche";
	m_canMotor[CAN_MOTOR_LEFT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_1;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		can_mip_register_node(&m_canMotor[i]);
	}
}

int Motion::init()
{
	m_mutex = xSemaphoreCreateMutex();

	if( ! m_mutex )
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

void Motion::compute()
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);

// si odometrie sur roues motrices (utiliser uniquement pour tests sur cale)
#if 0
	int motor_mes_valid = 1;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( ! m_canMotor[i].is_op_enable() )
		{
			motor_mes_valid = 0;
		}

		m_kinematicsMes[i] = m_canMotor[i].kinematics;
	}

	if( motor_mes_valid )
	{
		// mise à jour de la position
		location_update(VOIE_MOT_INV, m_kinematicsMes, CONTROL_DT);
	}
#else
	// mise à jour de la position
	uint16_t p1 = encoder_get(ENCODER_1);
	uint16_t p2 = encoder_get(ENCODER_2);
	m_kinematicsMes[0].v = (int16_t)((uint16_t) p1 - (uint16_t)m_kinematicsMes[0].pos);
	m_kinematicsMes[0].v *= ODO1_WAY * 2 * M_PI * ODO1_WHEEL_RADIUS / (float)(ODO_ENCODER_RESOLUTION * CONTROL_DT);
	m_kinematicsMes[1].v = (int16_t)((uint16_t) p2 - (uint16_t)m_kinematicsMes[1].pos);
	m_kinematicsMes[1].v *= ODO2_WAY * 2 * M_PI * ODO2_WHEEL_RADIUS / (float)(ODO_ENCODER_RESOLUTION * CONTROL_DT);
	m_kinematicsMes[0].pos = p1;
	m_kinematicsMes[1].pos = p2;

	location_update(VOIE_ODO_INV, m_kinematicsMes, CONTROL_DT);
#endif
	m_posMes = location_get_position();
	m_speedMes = location_get_speed();

	m_motionStateMachine.execute();

	xSemaphoreGive(m_mutex);
}

void Motion::motionUpdateMotors()
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( m_kinematics[i].mode == KINEMATICS_SPEED )
		{
			m_canMotor[i].set_speed(m_kinematics[i].v);
		}
		else if( m_kinematics[i].mode == KINEMATICS_POSITION )
		{
			m_canMotor[i].set_position(m_kinematics[i].pos);
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
		pwm_set(PWM_1, pwm);
		pwm = fabsf(m_kinematics[LEFT_WHEEL].v) / 1000;
		if( pwm > 1)
		{
			pwm = 1;
		}
		pwm_set(PWM_2, pwm);
	}

	m_speedCmd = kinematics_model_compute_speed(VOIE_MOT_INV, m_kinematics);
}

void Motion::enableAntico(bool enable)
{
	m_anticoOn = enable;
}

float Motion::findRotate(float start, float end)
{
	float dtheta = end - start;
	bool neg = dtheta < 0;

	// modulo 1 tour => retour dans [ 0 ; 2*M_PI [
	dtheta = fmodf(dtheta, 2*M_PI);
	if( neg )
	{
		dtheta += 2*M_PI;
	}

	// retour dans ] -M_PI ; M_PI ] tour
	if( dtheta > M_PI )
	{
		dtheta -= 2*M_PI;
	}

	return dtheta;
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

	//////////////////////////................. Cette méthode est appellé que dans deux classes ...? je me pose la question sur son utilitée dans cette classe (a déplacer dans les classe utilisatrice ou créer une classe intermédiaire pour héritage...(note dans une classe fille tu peux appeller une méthode de la classe mére =>
	//FILLE::METHODE { MAMMAN::METHODE(); .....}
//unsigned int Motion::motionStateGenericPowerTransition(unsigned int currentState)
//{
//	bool all_op_enable = true;
//
//	for(int i = 0; i < CAN_MOTOR_MAX; i++)
//	{
//		all_op_enable &= m_canMotor[i].is_op_enable();
//	}
	///En gros on veut éteindre les moteur cad passer dans l'etat DISABLE
	/// On par en DIsable si pas de puissance dans les moteur ou on veut eteindre les moteur en partant en DISABLE
//	if( power_get() || ! all_op_enable || m_wantedState == MOTION_DISABLED)
//	{
//		return MOTION_DISABLED;
//	}

	//On rentre dans ce If que dans l'état DISABLE ou TRY_ENABLE ou MOTION_ENABLE
	//Or cette méthode est appellée seulement par les ETATs autres que ces trois derniers....
	//Donc Code mort à supprimer
//	if( m_wantedState == MOTION_ENABLED && currentState != MOTION_ACTUATOR_KINEMATICS &&
//		currentState != MOTION_SPEED && currentState != MOTION_TRAJECTORY && currentState != MOTION_INTERRUPTING)
//	{
//		return MOTION_ENABLED;
//	}

//	return currentState;
//}

void Motion::cmd_print_param(void* /*arg*/)
{
	log_format(LOG_INFO, "axe x     : kp %d ki %d kd %d", (int)(motion.m_xPid.kp), (int)(motion.m_xPid.ki), (int)(motion.m_xPid.kd));
	log_format(LOG_INFO, "axe theta : kp %d ki %d kd %d", (int)(motion.m_thetaPid.kp), (int)(motion.m_thetaPid.ki), (int)(motion.m_thetaPid.kd));
}

void Motion::cmd_set_param(void* arg)
{
	struct motion_cmd_param_arg* cmd = (struct motion_cmd_param_arg*) arg;
	motion.m_xPid.kp = cmd->kp_av;
	motion.m_xPid.ki = cmd->ki_av;
	motion.m_xPid.kd = cmd->kd_av;
	motion.m_thetaPid.kp = cmd->kp_rot;
	motion.m_thetaPid.ki = cmd->ki_rot;
	motion.m_thetaPid.kd = cmd->kd_rot;
}

void Motion::cmd_goto(void* arg)
{
	struct motion_cmd_goto_arg* cmd = (struct motion_cmd_goto_arg*) arg;
	motion.goTo(cmd->dest, cmd->cp, (enum motion_way)cmd->way, (enum motion_trajectory_type)cmd->type, cmd->linearParam, cmd->angularParam);
}

void Motion::cmd_set_speed(void* arg)
{
	struct motion_cmd_set_speed_arg* cmd = (struct motion_cmd_set_speed_arg*) arg;
	motion.setSpeed(cmd->u, cmd->v);
}

void Motion::cmd_set_max_current(void* arg)
{
	struct motion_cmd_set_max_driving_current_arg* cmd = (struct motion_cmd_set_max_driving_current_arg*) arg;
	motion.setMaxDrivingCurrent(cmd->maxDrivingCurrent);
}

void Motion::cmd_enable(void* arg)
{
	struct motion_cmd_enable_arg* cmd_arg = (struct motion_cmd_enable_arg*) arg;

	motion.enable(cmd_arg->enable != 0);
}

void Motion::cmd_set_actuator_kinematics(void* arg)
{
	struct motion_cmd_set_actuator_kinematics_arg* cmd = (struct motion_cmd_set_actuator_kinematics_arg*) arg;
	motion.setActuatorKinematics(*cmd);
}
//En gros l'utilisatteur par cette méthode veut partir dans l'etat ENABLE .................
// Franchement ne vaut-il pas mieux d'utiliser la variable wantedState que d'utiliser une autre variable, non? 
void Motion::enable(bool enable)
{	
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	//Sur notre document nous résumant nos etats si on n'est pas les etats Disable on ne peut pas transiter vers l'etat ENABLE=> donc le seul etat ou on peut transiter vers ENABLE c'est l'état DISABLE
	//SI on est dans les autre état on a forcement de la puissance
	//if( enable && m_motionStateMachine.getCurrentState() != MOTION_ENABLED )
	if( enable && m_motionStateMachine.getCurrentState() == MOTION_DISABLED )
	{
		//m_enableWanted = MOTION_ENABLE_WANTED_ON;
		m_wantedState = MOTION_ENABLED;

	}
	///Pour le CAS de disable, on veut revenir dans quelque soit l'etat de la machine d'etat même disable dans l'état disable .......
	//else if( ! enable && m_motionStateMachine.getCurrentState() != MOTION_DISABLED )
	else if( ! enable )
	{
		//m_enableWanted = MOTION_ENABLE_WANTED_ON;
		m_wantedState = MOTION_DISABLED;
	}
	xSemaphoreGive(m_mutex);
}

void Motion::setMaxDrivingCurrent(float maxCurrent)
{
	m_canMotor[0].set_max_current(maxCurrent);
	m_canMotor[1].set_max_current(maxCurrent);
}

void Motion::goTo(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_wantedState = MOTION_TRAJECTORY;
	m_wantedDest = loc_to_abs(dest, -cp);
	m_wantedTrajectoryType = type;
	m_wantedWay = way;
	m_wantedLinearParam = linearParam;
	m_wantedAngularParam = angularParam;
	m_status = MOTION_UPDATING_TRAJECTORY;
	
	xSemaphoreGive(m_mutex);
}

void Motion::setSpeed(VectPlan u, float v)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);

	m_wantedState = MOTION_SPEED;
	m_u = u;
	m_v = v;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
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
	//Appele un chat un chat, le nom de la variable suffit non? on veut aller dans l'etat Wanted MOTION_ACTUATOR_KINEMATICS
	//m_wantedState = MOTION_WANTED_STATE_ACTUATOR_KINEMATICS;
	m_wantedState = MOTION_ACTUATOR_KINEMATICS;
	m_wantedKinematics = cmd;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( cmd.mode[i] != KINEMATICS_POSITION && cmd.mode[i] != KINEMATICS_SPEED)
		{
			log_format(LOG_ERROR, "unknown mode %d for actuator %i", cmd.mode[i], i);
			m_wantedState = MOTION_UNKNOWN_STATE;
		}
	}

	xSemaphoreGive(m_mutex);
}

void Motion::getState(enum motion_state* state, enum motion_status* status, enum motion_trajectory_step* step, enum motion_state* wanted_state)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	*state = (enum motion_state)m_motionStateMachine.getCurrentState();
	*status = m_status;
	*step = m_trajStep;
	*wanted_state = m_wantedState;
	xSemaphoreGive(m_mutex);
}

void Motion::updateUsbData(struct control_usb_data* data)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);

	data->motion_state = m_motionStateMachine.getCurrentState();
	data->cons = m_posCmdTh;

	data->wanted_pos = m_dest;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		data->cons_motors_v[i] = m_kinematics[i].v;
		data->mes_motors[i] = m_kinematicsMes[i];
		data->mes_motor_current[i] = m_canMotor[i].current;
	}

	xSemaphoreGive(m_mutex);
}
