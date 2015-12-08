//! @file Trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include <stdlib.h>
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/math/findRotation.h"
#include "disco/robot_parameters.h"
#include "Trajectory.h"
#include "kernel/math/poly7.h"

#define TRAJECTORY_STACK_SIZE       400
#define TRAJECTORY_APPROX_DIST      150      //!< distance d'approche d'un objet
#define TRAJECTORY_PERIOD            10

int Trajectory::init(Detection* detection, Motion* motion, Location* location)
{
	xTaskHandle xHandle;

	usb_add_cmd(USB_CMD_TRAJECTORY, &trajectoryCmd, this);

	m_location = location;
	m_detection = detection;
	m_detection->registerCallback(detectionCallback, this);
	m_motion = motion;

	m_trajectoryState = TRAJECTORY_STATE_NONE;
	m_hokuyoEnableCheck = true;
	m_staticCheckEnable = true;

	m_linearParam.vMax = 700;
	m_linearParam.aMax = 600;
	m_linearParam.dMax = 600;
	m_angularParam.vMax = 3;
	m_angularParam.aMax = 5;
	m_angularParam.dMax = 5;
	m_mutex = xSemaphoreCreateMutex();

	if(m_mutex == NULL)
	{
		return ERR_INIT_TRAJECTORY;
	}

	portBASE_TYPE err = xTaskCreate(&Trajectory::trajectory_task, "traj", TRAJECTORY_STACK_SIZE, this, PRIORITY_TASK_TRAJECTORY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TRAJECTORY;
	}

	return 0;
}

void Trajectory::trajectory_task(void* arg)
{
	Trajectory* traj = (Trajectory*) arg;
	traj->trajectoryTask();
}

void Trajectory::trajectoryTask()
{
	uint32_t wake_time = 0;
	enum motion_state motion_state, motion_wanted_state;
	enum motion_status motion_status;

	while(1)
	{
		m_pos = m_location->getPosition();
		m_motion->getState(&motion_state, &motion_status, &motion_wanted_state);

		if( m_newRequest )
		{
			update();
		}

		switch(m_trajectoryState)
		{
			default:
			case TRAJECTORY_STATE_UPDATING_TRAJECTORY:
			case TRAJECTORY_STATE_NONE:
			case TRAJECTORY_STATE_TARGET_REACHED:
			case TRAJECTORY_STATE_TARGET_NOT_REACHED:
			case TRAJECTORY_STATE_COLISION:
				break;
			case TRAJECTORY_STATE_MOVING_TO_DEST:
				switch( motion_status )
				{
					case MOTION_TARGET_REACHED:
						log(LOG_INFO, "TRAJECTORY_TARGET_REACHED");
						m_trajectoryState = TRAJECTORY_STATE_TARGET_REACHED;
						break;
					case MOTION_TARGET_NOT_REACHED:
						log(LOG_ERROR, "TRAJECTORY_TARGET_NOT_REACHED");
						m_trajectoryState = TRAJECTORY_STATE_TARGET_NOT_REACHED;
						break;
					case MOTION_COLSISION:
						switch(m_avoidanceType)
						{
							default:
							case AVOIDANCE_STOP:
								// pas d'évitement, fin de la trajectoire
								log(LOG_INFO, "collision -> stop");
								m_trajectoryState = TRAJECTORY_STATE_COLISION;
								break;
							case AVOIDANCE_GRAPH:
								log(LOG_INFO, "collision -> graph");
								computeGraph(DETECTION_FULL);
								break;
						}
						break;
					default:
					case MOTION_TIMEOUT:
						log(LOG_INFO, "timeout -> target not reached");
						m_trajectoryState = TRAJECTORY_STATE_TARGET_NOT_REACHED;
						break;
					case MOTION_IN_MOTION:
					case MOTION_UPDATING_TRAJECTORY:
						break;
				}
				break;
		}

		// TODO ne pas attendre si evenement
		vTaskDelayUntil(&wake_time, TRAJECTORY_PERIOD);
	}
}

void Trajectory::detectionCallback(void* /*arg*/)
{
	// TODO
}

void Trajectory::updateRequest()
{
	if( m_newRequest )
	{
		log(LOG_ERROR, "trajectory - overiding trajectory");
	}
	m_newRequest = true;
	m_trajectoryState = TRAJECTORY_STATE_UPDATING_TRAJECTORY;
}

void Trajectory::setKinematicsParam(KinematicsParameters linParam, KinematicsParameters angParam)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_linearParam = linParam;
	m_angularParam = angParam;
	xSemaphoreGive(m_mutex);
}

void Trajectory::getKinematicsParam(KinematicsParameters* linParam, KinematicsParameters* angParam)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	*linParam = m_linearParam;
	*angParam = m_angularParam;
	xSemaphoreGive(m_mutex);
}

void Trajectory::trajectoryCmd(void* arg, void* data)
{
	Trajectory* traj = (Trajectory*) arg;
	xSemaphoreTake(traj->m_mutex, portMAX_DELAY);
	memcpy(&traj->m_request, data, sizeof(struct trajectory_cmd_arg));
	traj->updateRequest();
	xSemaphoreGive(traj->m_mutex);
}

void Trajectory::freeWheel()
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_FREE;
	m_request.avoidance_type = AVOIDANCE_STOP;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::rotate(float theta)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_ROTATE;
	m_request.avoidance_type = AVOIDANCE_STOP;
	m_request.dest.theta = theta;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::rotateTo(float theta)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_ROTATE_TO;
	m_request.avoidance_type = AVOIDANCE_STOP;
	m_request.dest.theta = theta;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::straight(float dist)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_STRAIGHT;
	m_request.avoidance_type = AVOIDANCE_STOP;
	m_request.dist = dist;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::goToNearXy(float x, float y, float dist, TrajectoryWay way, enum avoidance_type avoidance_type)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_GOTO_XY;
	m_request.avoidance_type = avoidance_type;
	m_request.dest.x = x;
	m_request.dest.y = y;
	m_request.dist = dist;
	m_request.way = way;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::goToNear(VectPlan dest, float dist, TrajectoryWay way, enum avoidance_type avoidance_type)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_GOTO_XYA;
	m_request.avoidance_type = avoidance_type;
	m_request.dest = dest;
	m_request.dist = dist;
	m_request.way = way;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::goTo(VectPlan dest, TrajectoryWay way, enum avoidance_type avoidance_type)
{
	goToNear(dest, 0, way, avoidance_type);
}

void Trajectory::goToGraphNode(uint32_t node_id, float dist, TrajectoryWay way, enum avoidance_type avoidance_type)
{
	if( node_id >= GRAPH_NUM_NODE)
	{
		log_format(LOG_ERROR, "node_id inconnu : %d", (int)node_id);
		return;
	}

	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_GOTO_XY;
	m_request.avoidance_type = avoidance_type;
	m_request.dest = VectPlan(m_graph.getNode(node_id), 0);
	m_request.dist = dist;
	m_request.way = way;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

void Trajectory::goToGraph()
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_request.type = TRAJECTORY_GOTO_GRAPH;
	m_request.avoidance_type = AVOIDANCE_STOP;
	updateRequest();
	xSemaphoreGive(m_mutex);
}

////////////////////////////////////////////////
/// function    : trajectory_wait()
/// descrition  : Waiting function of trajectory move functions
/// param       : wanted_state = enum trajectory_state
/// param       : timeout = uint32_t time_out (<0 no time-out but buffer overflow!!!!)
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
// TODO faire mieux pour eviter le polling
int Trajectory::wait(enum trajectory_status wanted_status, uint32_t timeout)
{
	enum trajectory_status status = m_status;

	while(m_status != TRAJECTORY_STATE_COLISION && m_status != TRAJECTORY_STATE_TARGET_REACHED && m_status != TRAJECTORY_STATE_TARGET_NOT_REACHED && timeout)
	{
		vTaskDelay(1);
		timeout --;
		status = m_status;
	}

	if( status != wanted_status )
	{
		log_format(LOG_ERROR, "incorrect state : %d", status);
		return -1;
	}

	return 0;
}
