/*
 * CStateMotionTrajectory.cxx
 *
 *  Created on: 24 août 2015
 *      Author: jul
 */

#include "CStateMotionTrajectory.h"

CStateMotionTrajectory::CStateMotionTrajectory():MotionEtat("MOTION_STATE_TRAJECTORY")
{
	// TODO Auto-generated constructor stub
	m_motion_State 				= MOTION_STATE_TRAJECTORY;

}

CStateMotionTrajectory::~CStateMotionTrajectory()
{
	// TODO Auto-generated destructor stub
}


void CStateMotionTrajectory::InitState(Etat * pMotionInterrupting, Etat * pMotionDisable, Pid * motion_x_pid,Pid * motion_theta_pid , motion_goto_parameter * pgotoparam)
{
	m_pMotionInterrupting = pMotionInterrupting;
	m_pMotionDisable = pMotionDisable;
	m_pmotion_x_pid =motion_x_pid;
	m_pmotion_theta_pid = motion_theta_pid;
	m_pgotoparam = pgotoparam;
	m_motion_linear_speed_check.Set(100, 10);

}

////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionTrajectory::run()
{
	Kinematics kinematics = m_motion_curvilinearKinematics;
	KinematicsParameters curvilinearKinematicsParam;
	float ds = 0;
	float k;
	float n;
	VectPlan u;
	VectPlan u_loc;
	VectPlan error_loc;
	Kinematics motion_kinematics_th[CAN_MOTOR_MAX];
	VectPlan v;
	VectPlan v_th;

	enum motion_check_speed res = m_motion_linear_speed_check.compute(m_motion_speed_cmd.x, m_motion_speed_mes.x);
	if( res != MOTION_SPEED_OK )
	{
		log(LOG_INFO, "MOTION_COLSISION");
		m_motion_status = MOTION_COLISION;

	}
	if(m_motion_status != MOTION_COLISION)
	{
		ds = m_motion_ds[m_motion_traj_step];
		if( m_motion_traj_step == MOTION_TRAJECTORY_PRE_ROTATE || m_motion_traj_step == MOTION_TRAJECTORY_ROTATE )
		{
			u = VectPlan(0, 0, 1);
			curvilinearKinematicsParam = m_pgotoparam->motion_wanted_angularParam;
			if( m_motion_antico_on )
			{
				float opponentMinDistance = detection_compute_opponent_distance(Vect2(m_motion_pos_mes.x, m_motion_pos_mes.y));
				if( opponentMinDistance < 1.5*PARAM_RIGHT_CORNER_X )
				{
					//reduction de la vitesse max de rotation si l'adversaire est tres proche
					curvilinearKinematicsParam.vMax = 1;
				}
				else if( opponentMinDistance < 2*PARAM_RIGHT_CORNER_X )
				{
					// reduction de la vitesse max de rotation si l'adversaire est proche
					curvilinearKinematicsParam.vMax /= 1.5;
				}
			}
		}
		else
			{
			u = m_motion_u;
			curvilinearKinematicsParam = m_pgotoparam->motion_wanted_linearParam;
			if( m_motion_antico_on )
			{
				// detection adverse
				float opponentMinDistance = detection_compute_opponent_in_range_distance(Vect2(m_motion_pos_mes.x, m_motion_pos_mes.y), Vect2(u.x, u.y));
				opponentMinDistance = opponentMinDistance - PARAM_RIGHT_CORNER_X - PARAM_FINGER_SIZE_X;
				opponentMinDistance /= 2; // facteur de securite

				// detection statique
				VectPlan dir = m_motion_pos_mes;
				if( u.x < 0 )
				{
					dir.theta += M_PI;
				}
				float tableBorderDistance = detection_compute_front_object(DETECTION_STATIC_OBJ, dir, NULL, NULL) + PARAM_NP_X;
				if( tableBorderDistance < 10 )
				{
					tableBorderDistance = 10;
				}
				float maxDistance = tableBorderDistance;
				if( opponentMinDistance < maxDistance)
				{
					maxDistance = opponentMinDistance;
				}

				if( maxDistance < 0 )
				{
					maxDistance = 0;
				}

				float corr = curvilinearKinematicsParam.dMax * CONTROL_DT / 2;
				float vMaxSlowDown = sqrtf(corr * corr + 2 * fabsf(maxDistance) * curvilinearKinematicsParam.dMax) - corr;
				if(vMaxSlowDown < curvilinearKinematicsParam.vMax )
				{
					curvilinearKinematicsParam.vMax = vMaxSlowDown;
				}

				if( vMaxSlowDown < EPSILON )
				{
					log(LOG_INFO, "MOTION_COLSISION");
					m_motion_status = MOTION_COLISION;
				}
			}
		}
	}
	if(m_motion_status != MOTION_COLISION )
	{
		error_loc = abs_to_loc(m_motion_pos_mes, m_motion_pos_cmd_th);

		// calcul de la commande theorique
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			motion_kinematics_th[i] = m_motion_kinematics[i];
		}
		kinematics.setPosition(ds, 0, curvilinearKinematicsParam, CONTROL_DT);
		u_loc = abs_to_loc_speed(m_motion_pos_cmd_th.theta, u);
		k = kinematics_model_compute_actuator_cmd(VOIE_MOT, u_loc, kinematics.v, CONTROL_DT, motion_kinematics_th);
		m_motion_curvilinearKinematics.v = k * kinematics.v;
		m_motion_curvilinearKinematics.pos += m_motion_curvilinearKinematics.v * CONTROL_DT;
		v_th = m_motion_curvilinearKinematics.v * u;
		m_motion_pos_cmd_th = m_motion_pos_cmd_th + CONTROL_DT * v_th;

		// correction en fonction de l'erreur
		v = abs_to_loc_speed(m_motion_pos_mes.theta, v_th);
		// TODO pas de correction laterale pour le moment
		v.x += m_pmotion_x_pid->compute(error_loc.x, CONTROL_DT);
		v.theta += m_pmotion_theta_pid->compute(error_loc.theta, CONTROL_DT);

		n = v.norm();
		if( n > EPSILON )
		{
			u_loc = v / n;
		}
		else
		{
			n = v.theta;
			u_loc = VectPlan(0, 0, 1);
		}
		kinematics_model_compute_actuator_cmd(VOIE_MOT, u_loc, n, CONTROL_DT, m_motion_kinematics);

		if(fabsf(m_motion_curvilinearKinematics.pos - ds) < EPSILON && fabsf(m_motion_curvilinearKinematics.v) < EPSILON )
		{
			if( m_motion_traj_step == MOTION_TRAJECTORY_PRE_ROTATE)
			{
				log_format(LOG_DEBUG1, "ds %d pos %d", (int)(1000*ds), (int)(1000*m_motion_curvilinearKinematics.pos));
				m_motion_curvilinearKinematics.reset();
				for(int i = 0; i < CAN_MOTOR_MAX; i++)
				{
					m_motion_kinematics[i].v = 0;
				}
				m_motion_traj_step = MOTION_TRAJECTORY_STRAIGHT;
			}
			else if( m_motion_traj_step == MOTION_TRAJECTORY_STRAIGHT)
			{
				m_motion_curvilinearKinematics.reset();
				for(int i = 0; i < CAN_MOTOR_MAX; i++)
				{
					m_motion_kinematics[i].v = 0;
				}
				m_motion_traj_step = MOTION_TRAJECTORY_ROTATE;
			}
			else
			{
				m_motion_curvilinearKinematics.v = 0;
				m_motion_curvilinearKinematics.a = 0;

				VectPlan err = m_motion_dest - m_motion_pos_mes;
				if( err.norm2() < MOTION_TARGET_REACHED_LIN_THRESHOLD_SQUARE && fabsf(err.theta) < MOTION_TARGET_REACHED_ANG_THRESHOLD )
				{
					log(LOG_INFO, "MOTION_TARGET_REACHED");
					m_motion_status = MOTION_TARGET_REACHED;
				}
				else
				{
					systime t = systick_get_time();
					if( m_motion_target_not_reached_start_time.ms == 0)
					{
						log(LOG_INFO, "MOTION_TARGET_NOT_REACHED ARMED");
						m_motion_target_not_reached_start_time = t;
					}
					else
					{
						systime detla = t - m_motion_target_not_reached_start_time;
						if( detla.ms > MOTION_TARGET_NOT_REACHED_TIMEOUT)
						{
							log_format(LOG_INFO, "MOTION_TARGET_NOT_REACHED error %d %d %d", (int)err.x, (int)err.y, (int)(err.theta * 180 / M_PI));
							m_motion_status = MOTION_TARGET_NOT_REACHED;
						}
					}
				}
			}
		}
	}
	//Dans les cs suivants Time_out, Colision ,Target REACHED -> on s'arrete et on passe dans l'etat interrupting
	if(m_motion_status == MOTION_TARGET_NOT_REACHED ||  m_motion_status == MOTION_COLISION || m_motion_status == MOTION_TARGET_REACHED)
	{

		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			m_motion_kinematics[i].v = 0;
		}

		m_motion_Wanted_State = MOTION_STATE_INTERRUPTING;
	}

	motion_update_motors();
	return true;
}


void CStateMotionTrajectory::motion_update_motors()
{
	MotionEtat::motion_update_motors();
	m_motion_speed_cmd = kinematics_model_compute_speed(VOIE_MOT_INV, m_motion_kinematics);
}

void CStateMotionTrajectory::motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(m_motion_mutex, portMAX_DELAY);
	m_motion_Wanted_State = MOTION_STATE_TRAJECTORY;
	m_pgotoparam->motion_wanted_dest = loc_to_abs(dest, -cp);
	m_pgotoparam->motion_wanted_trajectory_type = type;
	m_pgotoparam->motion_wanted_way = way;
	m_pgotoparam->motion_wanted_linearParam = linearParam;
	m_pgotoparam->motion_wanted_angularParam = angularParam;
	m_motion_status = MOTION_UPDATING_TRAJECTORY;
	xSemaphoreGive(m_motion_mutex);
}

////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionTrajectory::entry()
{

	log_format(LOG_INFO, "Entree dans l'etat %s", this->getNameEtat());

	//Prise en compte de la commande utilisateur de changement d'état
	m_motion_Wanted_State = MOTION_NONE_STATE;


	float dtheta1 = 0;
	float ds = 0;
	float dtheta2 = 0;
	float t = 0;

	if( m_pgotoparam->motion_wanted_trajectory_type == MOTION_AXIS_A)
	{
		m_pgotoparam->motion_wanted_dest.x = m_motion_pos_mes.x;
		m_pgotoparam->motion_wanted_dest.y = m_motion_pos_mes.y;
	}

	VectPlan ab = m_pgotoparam->motion_wanted_dest - m_motion_pos_mes;
	float nab = ab.norm();

	// distance minimale de translation TODO 5mm en dur
	if( nab > 5 )
	{
		float theta1 = atan2f(ab.y, ab.x);
		ds = nab;
		m_motion_u = ab / nab;
		m_motion_u.theta = 0;

		if(m_pgotoparam->motion_wanted_way == WAY_FORWARD)
		{
			dtheta1 = motion_find_rotate(m_motion_pos_mes.theta, theta1);
			if( m_pgotoparam->motion_wanted_trajectory_type == MOTION_AXIS_XYA )
			{
				dtheta2 = motion_find_rotate(m_motion_pos_mes.theta + dtheta1, m_pgotoparam->motion_wanted_dest.theta);
			}
		}
		else if( m_pgotoparam->motion_wanted_way == WAY_BACKWARD)
		{
			dtheta1 = motion_find_rotate(m_motion_pos_mes.theta, theta1 + M_PI);
			if( m_pgotoparam->motion_wanted_trajectory_type == MOTION_AXIS_XYA )
			{
				dtheta2 = motion_find_rotate(m_motion_pos_mes.theta + dtheta1, m_pgotoparam->motion_wanted_dest.theta);
			}
		}
		else
		{
			float dtheta1_forward = motion_find_rotate(m_motion_pos_mes.theta, theta1);
			float dtheta1_backward = motion_find_rotate(m_motion_pos_mes.theta, theta1 + M_PI);
			float dtheta2_forward = 0;
			float dtheta2_backward = 0;

			if( m_pgotoparam->motion_wanted_trajectory_type == MOTION_AXIS_XYA )
			{
				dtheta2_forward = motion_find_rotate(m_motion_pos_mes.theta + dtheta1_forward, m_pgotoparam->motion_wanted_dest.theta);
				dtheta2_backward = motion_find_rotate(m_motion_pos_mes.theta + dtheta1_backward, m_pgotoparam->motion_wanted_dest.theta);
			}

			if ( fabsf(dtheta1_forward) + fabsf(dtheta2_forward) > fabsf(dtheta1_backward) + fabsf(dtheta2_backward))
			{
				dtheta1 = dtheta1_backward;
				dtheta2 = dtheta2_backward;
			}
			else
			{
				dtheta1 = dtheta1_forward;
				dtheta2 = dtheta2_forward;
			}
		}
	}
	else if( m_pgotoparam->motion_wanted_trajectory_type != MOTION_AXIS_XY)
	{
		if( m_pgotoparam->motion_wanted_trajectory_type == MOTION_AXIS_A )
		{
			// rotation demandee explicitement. Pas d'optimisation de la rotation a faire.
			// utile pour calibration odometrie principalement
			dtheta2 =  m_pgotoparam->motion_wanted_dest.theta - m_motion_pos_mes.theta;
		}
		else
		{
			// optimisation de l'angle de rotation a faire
			dtheta2 = motion_find_rotate(m_motion_pos_mes.theta, m_pgotoparam->motion_wanted_dest.theta);
		}
	}

	if( m_pgotoparam->motion_wanted_trajectory_type == MOTION_AXIS_XY )
	{
		m_pgotoparam->motion_wanted_dest.theta = m_motion_pos_mes.theta + dtheta1;
	}

	t = motion_compute_time(dtheta1, m_pgotoparam->motion_wanted_angularParam);
	t += motion_compute_time(ds, m_pgotoparam->motion_wanted_linearParam);
	t += motion_compute_time(dtheta2, m_pgotoparam->motion_wanted_angularParam);
	// mise a jour de l'angle de destination en fonction des optimisations faites
	m_pgotoparam->motion_wanted_dest.theta = m_motion_pos_mes.theta + dtheta1 + dtheta2;

	log_format(LOG_INFO, "goto %d %d %d t %d ms : rotate %d translate %d, rotate %d",
			(int)m_pgotoparam->motion_wanted_dest.x, (int)m_pgotoparam->motion_wanted_dest.y, (int)(m_pgotoparam->motion_wanted_dest.theta*180/M_PI), (int)(1000*t),
			(int)(dtheta1 * 180 / M_PI), (int)ds, (int)(dtheta2 * 180 / M_PI));

	m_motion_ds[0] = dtheta1;
	m_motion_ds[1] = ds;
	m_motion_ds[2] = dtheta2;

	m_motion_curvilinearKinematics.reset();
	m_motion_traj_step = MOTION_TRAJECTORY_PRE_ROTATE;
	log(LOG_INFO, "IN_MOTION");
	m_motion_status = MOTION_IN_MOTION;
	m_motion_pos_cmd_th = m_motion_pos_mes;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m_motion_kinematics[i] = m_motion_kinematics_mes[i];
	}

	m_motion_dest = m_pgotoparam->motion_wanted_dest;
	m_motion_target_not_reached_start_time.ms = 0;
	m_motion_target_not_reached_start_time.ns = 0;
	m_pmotion_x_pid->reset();
	m_pmotion_theta_pid->reset();
	m_motion_linear_speed_check.reset();



	return true;
}



////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionTrajectory::out()
{

	log_format(LOG_INFO, "Sortie de l'etat %s", this->getNameEtat());
	return true;
}




////////////////////////////////////////
//méthode recupere l'etat suivant
//Param :
//retourne: Id de l'etat suivant
Etat * CStateMotionTrajectory::getProchainEtat()
{
	Etat * pFuturState = this;
	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}
	//Passage à l'état Disable à la fin du match ou demande utilisateur
	if( power_get() || ! all_op_enable || m_motion_Wanted_State == MOTION_STATE_DISABLED)
	{
		pFuturState = m_pMotionDisable;
	}
	//Passage à l'état interrupting suite à une demande utilisateur ENABLE, à l'arrivé au point Final, Time-out ou colision
	else if( m_motion_Wanted_State == MOTION_STATE_ENABLED || m_motion_Wanted_State == MOTION_STATE_INTERRUPTING)
	{
		pFuturState = m_pMotionInterrupting;
	}


	return pFuturState;
}

float CStateMotionTrajectory::motion_compute_time(float ds, KinematicsParameters param)
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
