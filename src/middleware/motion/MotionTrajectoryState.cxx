#include "MotionTrajectoryState.h"
#include "kernel/log.h"
#include "middleware/detection.h"
#include "disco/robot_parameters.h"
#include "kernel/control.h"
#include "kernel/kinematics_model/kinematics_model.h"

MotionTrajectoryState::MotionTrajectoryState() :
	MotionMoveState("MOTION_TRAJECTORY",MOTION_TRAJECTORY)
{

}

void MotionTrajectoryState::entry(void* data)
{
	
	Motion* m = (Motion*) data;

	//Satisfaction de la volonte operateur 
	m->m_wantedState = MOTION_UNKNOWN_STATE;

////
	float dtheta1 = 0;
	float ds = 0;
	float dtheta2 = 0;
	float t = 0;

	if( m->m_wantedTrajectoryType == MOTION_AXIS_A)
	{
		m->m_wantedDest.x = m->m_posMes.x;
		m->m_wantedDest.y = m->m_posMes.y;
	}

	VectPlan ab = m->m_wantedDest - m->m_posMes;
	float nab = ab.norm();

	// distance minimale de translation TODO 5mm en dur
	if( nab > 5 )
	{
		float theta1 = atan2f(ab.y, ab.x);
		ds = nab;
		m->m_u = ab / nab;
		m->m_u.theta = 0;

		if(m->m_wantedWay == WAY_FORWARD)
		{
			dtheta1 = m->findRotate(m->m_posMes.theta, theta1);
			if( m->m_wantedTrajectoryType == MOTION_AXIS_XYA )
			{
				dtheta2 = m->findRotate(m->m_posMes.theta + dtheta1, m->m_wantedDest.theta);
			}
		}
		else if( m->m_wantedWay == WAY_BACKWARD)
		{
			dtheta1 = m->findRotate(m->m_posMes.theta, theta1 + M_PI);
			if( m->m_wantedTrajectoryType == MOTION_AXIS_XYA )
			{
				dtheta2 = m->findRotate(m->m_posMes.theta + dtheta1, m->m_wantedDest.theta);
			}
		}
		else
		{
			float dtheta1_forward = m->findRotate(m->m_posMes.theta, theta1);
			float dtheta1_backward = m->findRotate(m->m_posMes.theta, theta1 + M_PI);
			float dtheta2_forward = 0;
			float dtheta2_backward = 0;

			if( m->m_wantedTrajectoryType == MOTION_AXIS_XYA )
			{
				dtheta2_forward = m->findRotate(m->m_posMes.theta + dtheta1_forward, m->m_wantedDest.theta);
				dtheta2_backward = m->findRotate(m->m_posMes.theta + dtheta1_backward, m->m_wantedDest.theta);
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
	else if( m->m_wantedTrajectoryType != MOTION_AXIS_XY)
	{
		if( m->m_wantedTrajectoryType == MOTION_AXIS_A )
		{
			// rotation demandee explicitement. Pas d'optimisation de la rotation a faire.
			// utile pour calibration odometrie principalement
			dtheta2 =  m->m_wantedDest.theta - m->m_posMes.theta;
		}
		else
		{
			// optimisation de l'angle de rotation a faire
			dtheta2 = m->findRotate(m->m_posMes.theta, m->m_wantedDest.theta);
		}
	}

	if( m->m_wantedTrajectoryType == MOTION_AXIS_XY )
	{
		m->m_wantedDest.theta = m->m_posMes.theta + dtheta1;
	}

	t = m->motionComputeTime(dtheta1, m->m_wantedAngularParam);
	t += m->motionComputeTime(ds, m->m_wantedLinearParam);
	t += m->motionComputeTime(dtheta2, m->m_wantedAngularParam);
	// mise a jour de l'angle de destination en fonction des optimisations faites
	m->m_wantedDest.theta = m->m_posMes.theta + dtheta1 + dtheta2;

	log_format(LOG_INFO, "goto %d %d %d t %d ms : rotate %d translate %d, rotate %d",
			(int)m->m_wantedDest.x, (int)m->m_wantedDest.y, (int)(m->m_wantedDest.theta*180/M_PI), (int)(1000*t),
			(int)(dtheta1 * 180 / M_PI), (int)ds, (int)(dtheta2 * 180 / M_PI));

	m->m_ds[0] = dtheta1;
	m->m_ds[1] = ds;
	m->m_ds[2] = dtheta2;

	m->m_curvilinearKinematics.reset();
	m->m_trajStep = MOTION_TRAJECTORY_PRE_ROTATE;
	log(LOG_INFO, "IN_MOTION");
	m->m_status = MOTION_IN_MOTION;
	m->m_posCmdTh = m->m_posMes;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i] = m->m_kinematicsMes[i];
	}

	m->m_dest = m->m_wantedDest;
	m->m_targetNotReachedStartTime.ms = 0;
	m->m_targetNotReachedStartTime.ns = 0;
	m->m_xPid.reset();
	m->m_thetaPid.reset();
	m->m_linearSpeedCheck.reset();
}
/*
void MotionTrajectoryState::run(void* data)
{
	Motion* m = (Motion*) data;
	Kinematics kinematics = m->m_curvilinearKinematics;
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

	enum motion_check_speed res = m->m_linearSpeedCheck.compute(m->m_speedCmd.x, m->m_speedMes.x);
	if( res != MOTION_SPEED_OK )
	{
		log(LOG_INFO, "MOTION_COLSISION");
		m->m_status = MOTION_COLSISION;
		goto error;
	}

	ds = m->m_ds[m->m_trajStep];
	if( m->m_trajStep == MOTION_TRAJECTORY_PRE_ROTATE || m->m_trajStep == MOTION_TRAJECTORY_ROTATE )
	{
		u = VectPlan(0, 0, 1);
		curvilinearKinematicsParam = m->m_wantedAngularParam;
		if( m->m_anticoOn )
		{
			float opponentMinDistance = detection_compute_opponent_distance(Vect2(m->m_posMes.x, m->m_posMes.y));
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
		u = m->m_u;
		curvilinearKinematicsParam = m->m_wantedLinearParam;
		if( m->m_anticoOn )
		{
			// detection adverse
			float opponentMinDistance = detection_compute_opponent_in_range_distance(Vect2(m->m_posMes.x, m->m_posMes.y), Vect2(u.x, u.y));
			opponentMinDistance = opponentMinDistance - PARAM_RIGHT_CORNER_X - PARAM_FINGER_SIZE_X;
			opponentMinDistance /= 2; // facteur de securite

			// detection statique
			VectPlan dir = m->m_posMes;
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
				m->m_status = MOTION_COLSISION;
				goto error;
			}
		}
	}

	error_loc = abs_to_loc(m->m_posMes, m->m_posCmdTh);

	// calcul de la commande theorique
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics_th[i] = m->m_kinematics[i];
	}
	kinematics.setPosition(ds, 0, curvilinearKinematicsParam, CONTROL_DT);
	u_loc = abs_to_loc_speed(m->m_posCmdTh.theta, u);
	k = kinematics_model_compute_actuator_cmd(VOIE_MOT, u_loc, kinematics.v, CONTROL_DT, motion_kinematics_th);
	m->m_curvilinearKinematics.v = k * kinematics.v;
	m->m_curvilinearKinematics.pos += m->m_curvilinearKinematics.v * CONTROL_DT;
	v_th = m->m_curvilinearKinematics.v * u;
	m->m_posCmdTh = m->m_posCmdTh + CONTROL_DT * v_th;

	// correction en fonction de l'erreur
	v = abs_to_loc_speed(m->m_posMes.theta, v_th);
	// TODO pas de correction laterale pour le moment
	v.x += m->m_xPid.compute(error_loc.x, CONTROL_DT);
	v.theta += m->m_thetaPid.compute(error_loc.theta, CONTROL_DT);

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
	kinematics_model_compute_actuator_cmd(VOIE_MOT, u_loc, n, CONTROL_DT, m->m_kinematics);

	if(fabsf(m->m_curvilinearKinematics.pos - ds) < EPSILON && fabsf(m->m_curvilinearKinematics.v) < EPSILON )
	{
		if( m->m_trajStep == MOTION_TRAJECTORY_PRE_ROTATE)
		{
			log_format(LOG_DEBUG1, "ds %d pos %d", (int)(1000*ds), (int)(1000*m->m_curvilinearKinematics.pos));
			m->m_curvilinearKinematics.reset();
			for(int i = 0; i < CAN_MOTOR_MAX; i++)
			{
				m->m_kinematics[i].v = 0;
			}
			m->m_trajStep = MOTION_TRAJECTORY_STRAIGHT;
		}
		else if( m->m_trajStep == MOTION_TRAJECTORY_STRAIGHT)
		{
			m->m_curvilinearKinematics.reset();
			for(int i = 0; i < CAN_MOTOR_MAX; i++)
			{
				m->m_kinematics[i].v = 0;
			}
			m->m_trajStep = MOTION_TRAJECTORY_ROTATE;
		}
		else
		{
			m->m_curvilinearKinematics.v = 0;
			m->m_curvilinearKinematics.a = 0;

			VectPlan err = m->m_dest - m->m_posMes;
			if( err.norm2() < MOTION_TARGET_REACHED_LIN_THRESHOLD_SQUARE && fabsf(err.theta) < MOTION_TARGET_REACHED_ANG_THRESHOLD )
			{
				log(LOG_INFO, "MOTION_TARGET_REACHED");
				m->m_status = MOTION_TARGET_REACHED;
			}
			else
			{
				systime t = systick_get_time();
				if( m->m_targetNotReachedStartTime.ms == 0)
				{
					log(LOG_INFO, "MOTION_TARGET_NOT_REACHED ARMED");
					m->m_targetNotReachedStartTime = t;
				}
				else
				{
					systime detla = t - m->m_targetNotReachedStartTime;
					if( detla.ms > MOTION_TARGET_NOT_REACHED_TIMEOUT)
					{
						log_format(LOG_INFO, "MOTION_TARGET_NOT_REACHED error %d %d %d", (int)err.x, (int)err.y, (int)(err.theta * 180 / M_PI));
						m->m_status = MOTION_TARGET_NOT_REACHED;
						goto error;
					}
				}
			}
		}
	}

	m->motionUpdateMotors();
	return;

error:
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].v = 0;
	}
	m->motionUpdateMotors();
}
*/


void MotionTrajectoryState::run(void* data)
{
	Motion* m = (Motion*) data;
	Kinematics kinematics = m->m_curvilinearKinematics;
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

	enum motion_check_speed res = m->m_linearSpeedCheck.compute(m->m_speedCmd.x, m->m_speedMes.x);
	if( res != MOTION_SPEED_OK )
	{
		log(LOG_INFO, "MOTION_COLSISION");
		m->m_status = MOTION_COLSISION;
		Stop(m);
		return;
	}

	ds = m->m_ds[m->m_trajStep];
	if( m->m_trajStep == MOTION_TRAJECTORY_PRE_ROTATE || m->m_trajStep == MOTION_TRAJECTORY_ROTATE )
	{
		u = VectPlan(0, 0, 1);
		curvilinearKinematicsParam = m->m_wantedAngularParam;
		if( m->m_anticoOn )
		{
			float opponentMinDistance = detection_compute_opponent_distance(Vect2(m->m_posMes.x, m->m_posMes.y));
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
		u = m->m_u;
		curvilinearKinematicsParam = m->m_wantedLinearParam;
		if( m->m_anticoOn )
		{
			// detection adverse
			float opponentMinDistance = detection_compute_opponent_in_range_distance(Vect2(m->m_posMes.x, m->m_posMes.y), Vect2(u.x, u.y));
			opponentMinDistance = opponentMinDistance - PARAM_RIGHT_CORNER_X - PARAM_FINGER_SIZE_X;
			opponentMinDistance /= 2; // facteur de securite

			// detection statique
			VectPlan dir = m->m_posMes;
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
				m->m_status = MOTION_COLSISION;
				Stop(m);
				return;
			}
		}
	}

	error_loc = abs_to_loc(m->m_posMes, m->m_posCmdTh);

	// calcul de la commande theorique
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics_th[i] = m->m_kinematics[i];
	}
	kinematics.setPosition(ds, 0, curvilinearKinematicsParam, CONTROL_DT);
	u_loc = abs_to_loc_speed(m->m_posCmdTh.theta, u);
	k = kinematics_model_compute_actuator_cmd(VOIE_MOT, u_loc, kinematics.v, CONTROL_DT, motion_kinematics_th);
	m->m_curvilinearKinematics.v = k * kinematics.v;
	m->m_curvilinearKinematics.pos += m->m_curvilinearKinematics.v * CONTROL_DT;
	v_th = m->m_curvilinearKinematics.v * u;
	m->m_posCmdTh = m->m_posCmdTh + CONTROL_DT * v_th;

	// correction en fonction de l'erreur
	v = abs_to_loc_speed(m->m_posMes.theta, v_th);
	// TODO pas de correction laterale pour le moment
	v.x += m->m_xPid.compute(error_loc.x, CONTROL_DT);
	v.theta += m->m_thetaPid.compute(error_loc.theta, CONTROL_DT);

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
	kinematics_model_compute_actuator_cmd(VOIE_MOT, u_loc, n, CONTROL_DT, m->m_kinematics);

	if(fabsf(m->m_curvilinearKinematics.pos - ds) < EPSILON && fabsf(m->m_curvilinearKinematics.v) < EPSILON )
	{
		if( m->m_trajStep == MOTION_TRAJECTORY_PRE_ROTATE)
		{
			log_format(LOG_DEBUG1, "ds %d pos %d", (int)(1000*ds), (int)(1000*m->m_curvilinearKinematics.pos));
			m->m_curvilinearKinematics.reset();
			m->m_trajStep = MOTION_TRAJECTORY_STRAIGHT;
			Stop(m);
			return;
			
		}
		else if( m->m_trajStep == MOTION_TRAJECTORY_STRAIGHT)
		{
			m->m_curvilinearKinematics.reset();
			m->m_trajStep = MOTION_TRAJECTORY_ROTATE;
			Stop(m);
			return;
		}
		else
		{
			m->m_curvilinearKinematics.v = 0;
			m->m_curvilinearKinematics.a = 0;

			VectPlan err = m->m_dest - m->m_posMes;
			if( err.norm2() < MOTION_TARGET_REACHED_LIN_THRESHOLD_SQUARE && fabsf(err.theta) < MOTION_TARGET_REACHED_ANG_THRESHOLD )
			{
				log(LOG_INFO, "MOTION_TARGET_REACHED");
				m->m_status = MOTION_TARGET_REACHED;
			}
			else
			{
				systime t = systick_get_time();
				if( m->m_targetNotReachedStartTime.ms == 0)
				{
					log(LOG_INFO, "MOTION_TARGET_NOT_REACHED ARMED");
					m->m_targetNotReachedStartTime = t;
				}
				else
				{
					systime detla = t - m->m_targetNotReachedStartTime;
					if( detla.ms > MOTION_TARGET_NOT_REACHED_TIMEOUT)
					{
						log_format(LOG_INFO, "MOTION_TARGET_NOT_REACHED error %d %d %d", (int)err.x, (int)err.y, (int)(err.theta * 180 / M_PI));
						m->m_status = MOTION_TARGET_NOT_REACHED;
						Stop(m);
						return;
					}
				}
			}
		}
	}

	m->motionUpdateMotors();
	return;

}

void MotionTrajectoryState::Stop(Motion* m)
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].v = 0;
	}
	m->motionUpdateMotors();
}

unsigned int MotionTrajectoryState::transition(void* data)
{
	Motion* m = (Motion*) data;
	unsigned int new_State = MotionMoveState::transition(data);
	if(new_State == MOTION_DISABLED)
	{
		return MOTION_DISABLED;
	}
	///Si le prochain etat n'est pas motion disabled et que le robot est arrete on passe forcement dans l'etat d'arret de moteur pour retomberr dans l'etat ENABLE, new State nesera jamais en DISABLED
	if( m->m_status != MOTION_IN_MOTION )
	{
		log(LOG_INFO, "new state MOTION_INTERRUPTING");
		return MOTION_INTERRUPTING;
	}

	return m_stateId;
}
