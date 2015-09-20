
//#define WEAK_MOTION


#include "kernel/motion/new_state/CMotionEtat.h"
// machine a etats de motion
//Initialisation de l'etat'
#define MOTION_AUTO_ENABLE

#ifndef MOTION_AUTO_ENABLE
	motion_state MotionEtat::m_motion_Wanted_State = MOTION_STATE_DISABLED;
#else
	motion_state MotionEtat::m_motion_Wanted_State = MOTION_STATE_ENABLED;
#endif

Kinematics MotionEtat::m_motion_kinematics[CAN_MOTOR_MAX];
Kinematics MotionEtat::m_motion_kinematics_mes[CAN_MOTOR_MAX];

enum motion_status MotionEtat::m_motion_status;
enum motion_trajectory_step MotionEtat::m_motion_traj_step;
 VectPlan MotionEtat::m_motion_pos_mes;
 VectPlan MotionEtat::m_motion_speed_mes;

VectPlan MotionEtat::m_motion_pos_cmd_th;
 VectPlan MotionEtat::m_motion_u;
 float MotionEtat::m_motion_v;

 VectPlan MotionEtat::m_motion_dest;

bool MotionEtat::m_motion_antico_on;

xSemaphoreHandle MotionEtat::m_motion_mutex;

//VectPlan MotionEtat::m_motion_wanted_dest;


int MotionEtat::motion_module_init()
{
	m_motion_mutex = xSemaphoreCreateMutex();
	m_motion_status = MOTION_TARGET_REACHED;

	if( ! m_motion_mutex )
	{
		return ERR_INIT_CONTROL;
	}
	return 0;
}


MotionEtat::MotionEtat(char* name):Etat(name)
{
	
}


void MotionEtat::motion_update_motors()
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( m_motion_kinematics[i].mode == KINEMATICS_SPEED )
		{
			can_motor[i].set_speed(m_motion_kinematics[i].v);
		}
		else if( m_motion_kinematics[i].mode == KINEMATICS_POSITION )
		{
			can_motor[i].set_position(m_motion_kinematics[i].pos);
		}
	}

	if( match_get_go() )
	{
		// ambiance selon la vitesse des roues
		float pwm = fabsf(m_motion_kinematics[RIGHT_WHEEL].v) / 1000;
		if( pwm > 1)
		{
			pwm = 1;
		}
		pwm_set(PWM_1, pwm);
		pwm = fabsf(m_motion_kinematics[LEFT_WHEEL].v) / 1000;
		if( pwm > 1)
		{
			pwm = 1;
		}
		pwm_set(PWM_2, pwm);
	}


}



float MotionEtat::motion_find_rotate(float start, float end)
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

void MotionEtat::motion_enable_antico(bool enable)
{
	m_motion_antico_on = enable;
}

void MotionEtat::motion_cmd_goto(void* arg)
{
	struct motion_cmd_goto_arg* cmd = (struct motion_cmd_goto_arg*) arg;
	motion_goto(cmd->dest, cmd->cp, (motion_way)cmd->way, (motion_trajectory_type)cmd->type, cmd->linearParam, cmd->angularParam);
}

void MotionEtat::motion_cmd_set_speed(void* arg)
{
	struct motion_cmd_set_speed_arg* cmd = (struct motion_cmd_set_speed_arg*) arg;
	motion_set_speed(cmd->u, cmd->v);
}

void MotionEtat::motion_cmd_set_max_current(void* arg)
{
	struct motion_cmd_set_max_driving_current_arg* cmd = (struct motion_cmd_set_max_driving_current_arg*) arg;
	motion_set_max_driving_current(cmd->maxDrivingCurrent);
}

void MotionEtat::motion_cmd_enable(void* arg)
{
	struct motion_cmd_enable_arg* cmd_arg = (struct motion_cmd_enable_arg*) arg;

	motion_enable(cmd_arg->enable != 0);
}

void MotionEtat::motion_cmd_set_actuator_kinematics(void* arg)
{
	struct motion_cmd_set_actuator_kinematics_arg* cmd = (struct motion_cmd_set_actuator_kinematics_arg*) arg;
	motion_set_actuator_kinematics(*cmd);
}
void MotionEtat::GiveSem()
{
	xSemaphoreGive(m_motion_mutex);
}

void MotionEtat::TakeSem()
{
	xSemaphoreTake(m_motion_mutex, portMAX_DELAY);
}

void MotionEtat::motion_enable(bool enable)
{
	xSemaphoreTake(m_motion_mutex, portMAX_DELAY);
	if( enable && m_motion_State != MOTION_STATE_ENABLED )
	{
		m_motion_Wanted_State = MOTION_STATE_ENABLED;
	}
	else if( ! enable && m_motion_State != MOTION_STATE_DISABLED )
	{
		m_motion_Wanted_State = MOTION_STATE_DISABLED;
	}
	xSemaphoreGive(m_motion_mutex);
}

void MotionEtat::motion_set_max_driving_current(float maxCurrent)
{
	can_motor[0].set_max_current(maxCurrent);
	can_motor[1].set_max_current(maxCurrent);
}


///Normal dans un etat autre que enable ou trajectory on ne fais rien avec cette cmd
void MotionEtat::motion_goto(VectPlan dest, VectPlan cp, motion_way way, motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	log_format(LOG_INFO, "etat %s, motion Goto", this->getNameEtat());
}


void MotionEtat::motion_compute()
{
// si odometrie sur roues motrices (utiliser uniquement pour tests sur cale)
#if 0
	int motor_mes_valid = 1;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( ! can_motor[i].is_op_enable() )
		{
			motor_mes_valid = 0;
		}

		m_motion_kinematics_mes[i] = can_motor[i].kinematics;
	}

	if( motor_mes_valid )
	{
		// mise à jour de la position
		location_update(VOIE_MOT_INV, m_motion_kinematics_mes, CONTROL_DT);
	}
#else
	// mise à jour de la position
	uint16_t p1 = encoder_get(ENCODER_1);
	uint16_t p2 = encoder_get(ENCODER_2);
	m_motion_kinematics_mes[0].v = (int16_t)((uint16_t) p1 - (uint16_t)m_motion_kinematics_mes[0].pos);
	m_motion_kinematics_mes[0].v *= ODO1_WAY * 2 * M_PI * ODO1_WHEEL_RADIUS / (float)(ODO_ENCODER_RESOLUTION * CONTROL_DT);
	m_motion_kinematics_mes[1].v = (int16_t)((uint16_t) p2 - (uint16_t)m_motion_kinematics_mes[1].pos);
	m_motion_kinematics_mes[1].v *= ODO2_WAY * 2 * M_PI * ODO2_WHEEL_RADIUS / (float)(ODO_ENCODER_RESOLUTION * CONTROL_DT);
	m_motion_kinematics_mes[0].pos = p1;
	m_motion_kinematics_mes[1].pos = p2;

	location_update(VOIE_ODO_INV, m_motion_kinematics_mes, CONTROL_DT);
#endif
	m_motion_pos_mes = location_get_position();
	m_motion_speed_mes = location_get_speed();

}

void MotionEtat::motion_set_speed(VectPlan u, float v)
{
	xSemaphoreTake(m_motion_mutex, portMAX_DELAY);

	m_motion_Wanted_State = MOTION_STATE_SPEED;
	log_format(LOG_INFO, "m_WantedState  %d", m_motion_Wanted_State);
	m_motion_u = u;
	m_motion_v = v;


	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		
		m_motion_kinematics[i] = m_motion_kinematics_mes[i];
	}

	xSemaphoreGive(m_motion_mutex);
}

void MotionEtat::motion_stop()
{
	motion_set_speed(VectPlan(1,0,0), 0);
}

void MotionEtat::motion_set_actuator_kinematics(struct motion_cmd_set_actuator_kinematics_arg cmd)
{
	log_format(LOG_INFO, "etat %s,motion_set_actuator_kinematics", this->getNameEtat());
}

void MotionEtat::motion_get_state(motion_state* state,  motion_status* status,  motion_trajectory_step* step,  motion_state* wanted_state)
{
	xSemaphoreTake(m_motion_mutex, portMAX_DELAY);
	*state = m_motion_State;
	*status = m_motion_status;
	*step = m_motion_traj_step;
	*wanted_state = m_motion_Wanted_State;
	xSemaphoreGive(m_motion_mutex);
}

////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool MotionEtat::entry()
{

	log_format(LOG_INFO, "Entree dans l'etat %s", this->getNameEtat());

	m_motion_Wanted_State = MOTION_NONE_STATE;

	return true;
}



////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool MotionEtat::out()
{

	log_format(LOG_INFO, "Sortie de l'etat %s", this->getNameEtat());
	return true;
};


void MotionEtat::motion_update_usb_data(struct control_usb_data* data)
{
	xSemaphoreTake(m_motion_mutex, portMAX_DELAY);


	data->cons = m_motion_pos_cmd_th;
	data->motion_state = m_motion_State;
	data->wanted_pos = m_motion_dest;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		data->cons_motors_v[i] = m_motion_kinematics[i].v;
		data->mes_motors[i] = m_motion_kinematics_mes[i];
		data->mes_motor_current[i] = can_motor[i].current;
	}

	xSemaphoreGive(m_motion_mutex);
}
