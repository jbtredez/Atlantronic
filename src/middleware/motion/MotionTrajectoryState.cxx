#include "MotionTrajectoryState.h"
#include "kernel/log.h"
#include "middleware/detection.h"
#include "disco/robot_parameters.h"
#include "kernel/control.h"
#include "kernel/math/findRotation.h"

MotionTrajectoryState::MotionTrajectoryState() :
	MotionMoveState("MOTION_TRAJECTORY",MOTION_TRAJECTORY)
{

}

void MotionTrajectoryState::entry(void* data)
{
	Motion* m = (Motion*) data;

	//Satisfaction de la volonte operateur
	m->m_wantedState = MOTION_UNKNOWN_STATE;
	m->m_path.planify(m->m_wantedLinearParam,  m->m_wantedAngularParam);

	// DEBUG : laisser commente
	//m->m_path.display();

	// TODO fonction computeTime dans path pour passer sur traj courbe
	/*t = m->motionComputeTime(dtheta1, m->m_wantedAngularParam);
	t += m->motionComputeTime(ds, m->m_wantedLinearParam);
	t += m->motionComputeTime(dtheta2, m->m_wantedAngularParam);*/

	log(LOG_INFO, "IN_MOTION");
	m->m_status = MOTION_IN_MOTION;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i] = m->m_kinematicsMes[i];
	}

	m->m_targetNotReachedStartTime.ms = 0;
	m->m_targetNotReachedStartTime.ns = 0;
	m->m_xPid.reset();
	m->m_thetaPid.reset();
	m->m_linearSpeedCheck.reset();
}

void MotionTrajectoryState::run(void* data)
{
	Motion* m = (Motion*) data;
	KinematicsParameters vParam = m->m_wantedLinearParam;
	KinematicsParameters wParam = m->m_wantedAngularParam;

	enum motion_check_speed res = m->m_linearSpeedCheck.compute(m->m_speedCmd.x, m->m_speedMes.x);
	if( res != MOTION_SPEED_OK )
	{
		log(LOG_INFO, "MOTION_COLSISION");
		m->m_status = MOTION_COLSISION;
		stop(m);
		return;
	}

	if( m->m_anticoOn )
	{
		float opponentMinDistance = m->m_detection->computeOpponentDistance(Vect2(m->m_posMes.x, m->m_posMes.y));
		if( opponentMinDistance < 1.5*PARAM_RIGHT_CORNER_X )
		{
			//reduction de la vitesse max de rotation si l'adversaire est tres proche
			wParam.vMax = 1;
		}
		else if( opponentMinDistance < 2*PARAM_RIGHT_CORNER_X )
		{
			// reduction de la vitesse max de rotation si l'adversaire est proche
			wParam.vMax /= 1.5;
		}
	}

	// TODO voir si on regarde uniquement l'erreur en dy/dtheta par rapport a la traj
	VectPlan errorLoc = abs_to_loc(m->m_posMes, m->m_path.getLastPosCmd());

	// calcul de la commande theorique
	VectPlan vTh = m->m_path.getNextCommand(m->m_posMes, CONTROL_DT, vParam, wParam);
	float nth2 = vTh.norm();
	if( nth2 > EPSILON )
	{
		m->m_u = vTh / nth2;
	}
	else if( vTh.theta > EPSILON )
	{
		m->m_u = VectPlan(0,0,1);
	}
	else if( vTh.theta < -EPSILON )
	{
		m->m_u = VectPlan(0,0,-1);
	}
	else
	{
		m->m_u = VectPlan();
	}

	if( m->m_anticoOn )
	{
		VectPlan u = m->m_u;
		float opponentMinDistance = m->m_detection->computeOpponentInRangeDistance(Vect2(m->m_posMes.x, m->m_posMes.y), Vect2(u.x, u.y));
		opponentMinDistance = opponentMinDistance - PARAM_RIGHT_CORNER_X - PARAM_FINGER_SIZE_X;
		opponentMinDistance /= 2; // facteur de securite

		// detection statique
		VectPlan dir = m->m_posMes;
		if( u.x < 0 )
		{
			dir.theta += M_PI;
		}
		float tableBorderDistance = m->m_detection->computeFrontObject(DETECTION_STATIC_OBJ, dir, NULL, NULL) + PARAM_NP_X;
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

		float corr = vParam.dMax * CONTROL_DT / 2;
		float vMaxSlowDown = sqrtf(corr * corr + 2 * fabsf(maxDistance) * vParam.dMax) - corr;
		if(vMaxSlowDown < vParam.vMax )
		{
			vParam.vMax = vMaxSlowDown;
		}

		if( vMaxSlowDown < EPSILON )
		{
			log(LOG_INFO, "MOTION_COLSISION");
			m->m_status = MOTION_COLSISION;
			stop(m);
			return;
		}
	}

	// correction en fonction de l'erreur
	VectPlan v = abs_to_loc_speed(m->m_posMes.theta, vTh);
	v.x += m->m_xPid.compute(errorLoc.x, CONTROL_DT);
	v.theta += m->m_thetaPid.compute(errorLoc.theta, CONTROL_DT);
	float dthetaCorr = m->m_yPid.compute(errorLoc.y, CONTROL_DT);
	if( v.x < 0 )
	{
		dthetaCorr *= -1;
	}
	v.theta += dthetaCorr;

	VectPlan u_loc;
	float n = v.norm();
	if( n > EPSILON )
	{
		u_loc = v / n;
	}
	else
	{
		n = v.theta;
		u_loc = VectPlan(0, 0, 1);
	}
	m->m_kinematicsModel->computeActuatorCmd(u_loc, n, CONTROL_DT, m->m_kinematics);

	VectPlan err = m->m_path.getLastPoint() - m->m_posMes;
	if( vTh == VectPlan() && err.norm2() < MOTION_TARGET_REACHED_LIN_THRESHOLD_SQUARE && fabsf(err.theta) < MOTION_TARGET_REACHED_ANG_THRESHOLD )
	{
		log(LOG_INFO, "MOTION_TARGET_REACHED");
		m->m_status = MOTION_TARGET_REACHED;
	}
#if 0
	// TODO timeout en fonction du temps prevu + marge ?
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
				stop(m);
				return;
			}
		}
	}
#endif
	m->motionUpdateMotors();
	return;

}

void MotionTrajectoryState::stop(Motion* m)
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

	// interruption de la trajectoire courante.
	if( m->m_status != MOTION_IN_MOTION )
	{
		log(LOG_INFO, "new state MOTION_INTERRUPTING");
		return MOTION_INTERRUPTING;
	}

	return m_stateId;
}
