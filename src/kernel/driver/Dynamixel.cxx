//! @file dynamixel.cxx
//! @brief Gestion des dynamixel (ax12 et rx24)
//! @author Atlantronic

#include "Dynamixel.h"
#include "DynamixelManager.h"
#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/match.h"
#include "kernel/driver/power.h"
#include <stdlib.h>
#include <math.h>

#define DYNAMIXEL_MOVE_TIMEOUT         3000 // en ms

int Dynamixel::init(DynamixelManager* manager, int id, bool autoUpdate)
{
	m_manager = manager;
	m_id = id;
	m_lastError.transmit_error = 0xff;
	m_maxGoal = 0x3ff;
	m_goalPos = 0x1ff;
	m_targetReachedThreshold = 2;
	m_flags = DYNAMIXEL_FLAG_TARGET_REACHED | DYNAMIXEL_FLAG_CONTROL_OFF | DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;

	m_mutex = xSemaphoreCreateMutex();
	if( m_mutex == 0)
	{
		return -1;
	}

	if( autoUpdate )
	{
		manager->registerDynamixel(this);
	}

	return 0;
}

DynamixelError Dynamixel::setLed(uint8_t on)
{
	return write8(DYNAMIXEL_LED, on);
}

void Dynamixel::setId(uint8_t newId)
{
	DynamixelError error;

	if(newId <= 0xfd)
	{
		error = write8(DYNAMIXEL_ID, newId);
		if( error.transmit_error)
		{
			log_format(LOG_ERROR, "erreur de transmission dynamixel id %d", newId);
		}
		else
		{
			m_id = newId;
			log_format(LOG_INFO, "id %d -> %d - status = %#.2x", m_id, newId, error.internal_error);
		}
	}
	else
	{
		log(LOG_ERROR, "changement d'id par broadcast non autorisé");
	}
}

DynamixelError Dynamixel::setMovingSpeed(float speed)
{
	uint16_t speed16 = (uint16_t)fabsf(DYNAMIXEL_RD_TO_POS * speed);
	if( speed16 > 0x3ff )
	{
		speed16 = 0x3ff;
	}
	if( speed < 0 )
	{
		speed16 |= 1<<10;
	}

	DynamixelError error = write16(DYNAMIXEL_MOVING_SPEED_L, speed16);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission dynamixel id %d", m_id);
	}
	return error;
}

DynamixelError Dynamixel::setTorqueLimitEeprom(float torque_limit)
{
	if( torque_limit < 0)
	{
		torque_limit = 0;
	}

	if( torque_limit > 1)
	{
		torque_limit = 1;
	}

	return write16(DYNAMIXEL_MAX_TORQUE_L, torque_limit * DYNAMIXEL_MAX_TORQUE_LIMIT);
}

DynamixelError Dynamixel::setTorqueEnable(uint8_t enable)
{
	return write8(DYNAMIXEL_TORQUE_ENABLE, enable & 0x01);
}

DynamixelError Dynamixel::setCwAngleLimit(uint16_t val)
{
	DynamixelError error = write16(DYNAMIXEL_CW_ANGLE_LIMIT_L, val);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission dynamixel id %d", m_id);
	}
	else
	{
		log_format(LOG_INFO, "cw angle limit %d - status = %#.2x", val, error.internal_error);
	}

	return error;
}

DynamixelError Dynamixel::setCcwAngleLimit(uint16_t val)
{
	DynamixelError error = write16(DYNAMIXEL_CCW_ANGLE_LIMIT_L, val);
	if( error.transmit_error)
	{
		log_format(LOG_ERROR, "erreur de transmission dynamixel id %d", m_id);
	}
	else
	{
		log_format(LOG_INFO, "ccw angle limit %d - status = %#.2x", val, error.internal_error);
	}

	return error;
}

DynamixelError Dynamixel::setBaudRate(uint8_t bd)
{
	return write8(DYNAMIXEL_BAUD_RATE, bd);
}

void Dynamixel::update()
{
	struct DynamixelRequest req;

	// lecture de la position du dynamixel
	req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
	req.arg[0] = DYNAMIXEL_PRESENT_POSITION_L;
	req.arg[1] = 0x02;
	req.argc = 2;
	req.id = m_id;

	// pas de log d erreur de com si pas de puissance
	if( ! power_get() )
	{
		m_manager->send(&req);
	}
	else
	{
		req.status.error.transmit_error = ERR_DYNAMIXEL_POWER_OFF;
	}

	if( !req.status.error.transmit_error )
	{
		int pos = req.status.arg[0] + (req.status.arg[1] << 8);
		xSemaphoreTake(m_mutex, portMAX_DELAY);
		m_pos = pos;
		int goal_pos = m_goalPos;

		uint16_t pos_err = abs(pos - goal_pos);
		systime t = systick_get_time();
		if( m_flags & DYNAMIXEL_FLAG_CONTROL_OFF )
		{
			m_timeStartMovingMs = t.ms;
		}
		if( pos_err <= m_targetReachedThreshold)
		{
			m_flags |= DYNAMIXEL_FLAG_TARGET_REACHED;
			m_timeStartMovingMs = t.ms;
			if( m_flags & DYNAMIXEL_FLAG_STUCK )
			{
				log_format(LOG_ERROR, "%s id %d unstucked", pcTaskGetTaskName(NULL), m_id);
				m_flags &= ~DYNAMIXEL_FLAG_STUCK;
			}
		}
		else if( pos_err > m_targetReachedThreshold + 1)
		{
			if( m_flags & DYNAMIXEL_FLAG_TARGET_REACHED )
			{
				m_timeStartMovingMs = t.ms;
				m_flags &= ~DYNAMIXEL_FLAG_TARGET_REACHED;
			}

			if( t.ms - m_timeStartMovingMs > DYNAMIXEL_MOVE_TIMEOUT )
			{
				if( ! (m_flags & DYNAMIXEL_FLAG_STUCK) )
				{
					log_format(LOG_ERROR, "%s id %d stucked", pcTaskGetTaskName(NULL), m_id);
					m_flags |= DYNAMIXEL_FLAG_STUCK;
				}
			}
			else if( m_flags & DYNAMIXEL_FLAG_STUCK )
			{
				log_format(LOG_ERROR, "%s id %d unstucked", pcTaskGetTaskName(NULL), m_id);
				m_flags &= ~DYNAMIXEL_FLAG_STUCK;
			}
		}

		// renvoi du couple uniquement si ! control_off et torque_to_update
		if( (m_flags & (DYNAMIXEL_FLAG_TORQUE_TO_UPDATE | DYNAMIXEL_FLAG_CONTROL_OFF)) == DYNAMIXEL_FLAG_TORQUE_TO_UPDATE || m_manager->m_disabled)
		{
			uint16_t max_torque = 0;
			if( ! m_manager->m_disabled )
			{
				max_torque = m_maxTorque;
			}
			DynamixelError err = write16(DYNAMIXEL_TORQUE_LIMIT_L, max_torque);
			if( ! err.transmit_error && ! err.internal_error )
			{
				err = write8(DYNAMIXEL_TORQUE_ENABLE, 0x01);
				if( ! err.transmit_error )
				{
					m_flags &= ~DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
				}
			}
		}
		xSemaphoreGive(m_mutex);

		if( pos_err > 0 && ! (m_flags & DYNAMIXEL_FLAG_CONTROL_OFF) && ! m_manager->m_disabled)
		{
			// on va envoyer la position désirée
			req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
			req.arg[0] = DYNAMIXEL_GOAL_POSITION_L;
			req.arg[1] = (uint8_t) (goal_pos & 0xFF);
			req.arg[2] = (uint8_t) ((goal_pos >> 8) & 0xFF);
			req.argc = 3;
			m_manager->send(&req);
		}
	}

	xSemaphoreTake(m_mutex, portMAX_DELAY);
	if( m_lastError.transmit_error )
	{
		m_flags |= DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
	}
	if(m_lastError.transmit_error != req.status.error.transmit_error || m_lastError.internal_error != req.status.error.internal_error)
	{
		m_manager->print_error(req.id, req.status.error);
		m_lastError = req.status.error;
	}

	xSemaphoreGive(m_mutex);
}

void Dynamixel::updateUsbData(DynamixelUsbDeviceData* dynamixelDeviceData)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	dynamixelDeviceData->id = m_id;
	dynamixelDeviceData->pos = m_pos;
	dynamixelDeviceData->flags = m_flags;
	dynamixelDeviceData->error = m_lastError;
	xSemaphoreGive(m_mutex);
}

uint8_t Dynamixel::read8(uint8_t offset, DynamixelError* error)
{
	struct DynamixelRequest req;
	req.id = m_id;
	req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x01;
	req.argc = 2;

	m_manager->send(&req);
	*error = req.status.error;

	return req.status.arg[0];
}

uint16_t Dynamixel::read16(uint8_t offset, DynamixelError* error)
{
	struct DynamixelRequest req;
	req.id = m_id;
	req.instruction = DYNAMIXEL_INSTRUCTION_READ_DATA;
	req.arg[0] = offset;
	req.arg[1] = 0x02;
	req.argc = 2;

	m_manager->send(&req);
	*error = req.status.error;

	return req.status.arg[0] + (req.status.arg[1] << 8);
}

DynamixelError Dynamixel::write8(uint8_t offset, uint8_t data)
{
	struct DynamixelRequest req;
	req.id = m_id;
	req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = data;
	req.argc = 2;

	m_manager->send(&req);
	return req.status.error;
}

DynamixelError Dynamixel::write16(uint8_t offset, uint16_t data)
{
	struct DynamixelRequest req;
	req.id = m_id;
	req.instruction = DYNAMIXEL_INSTRUCTION_WRITE_DATA;
	req.arg[0] = offset;
	req.arg[1] = (uint8_t) (data & 0xFF);
	req.arg[2] = (uint8_t) ((data >> 8) & 0xFF);
	req.argc = 3;

	m_manager->send(&req);
	return req.status.error;
}

DynamixelError Dynamixel::setGoalPosition(float theta)
{
	DynamixelError err;
	// on met theta dans [-M_PI ; M_PI[
	theta = fmodf(theta, 2*M_PI);
	if( theta > M_PI)
	{
		theta -= 2*M_PI;
	}
	else if(theta < -M_PI)
	{
		theta += 2*M_PI;
	}

	// passage en unité dynamixel entre 0 et 0x3ff (de -150 degre a 150 degre)
	// zero au milieu qui vaut 0x1ff
	int32_t alpha = 0x1ff + theta * DYNAMIXEL_RD_TO_POS;

	// saturation
	if(alpha < m_minGoal)
	{
		alpha = m_minGoal;
	}
	else if( alpha > m_maxGoal)
	{
		alpha = m_maxGoal;
	}

	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_flags &= ~(DYNAMIXEL_FLAG_CONTROL_OFF | DYNAMIXEL_FLAG_STUCK);
	m_goalPos = alpha;
	if( abs(m_goalPos - m_pos) < m_targetReachedThreshold)
	{
		m_flags |= DYNAMIXEL_FLAG_TARGET_REACHED;
	}
	else
	{
		m_flags &= ~DYNAMIXEL_FLAG_TARGET_REACHED;
	}
	err = m_lastError;
	xSemaphoreGive(m_mutex);

	return err;
}

DynamixelError Dynamixel::setTorqueLimit(float torque_limit)
{
	DynamixelError err;

	if( match_end || torque_limit < 0)
	{
		torque_limit = 0;
	}

	if( torque_limit > 1)
	{
		torque_limit = 1;
	}

	// utilisation de la tache pour la mise à jour
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_maxTorque = torque_limit * DYNAMIXEL_MAX_TORQUE_LIMIT;
	m_flags |= DYNAMIXEL_FLAG_TORQUE_TO_UPDATE;
	err = m_lastError;
	xSemaphoreGive(m_mutex);

	return err;
}

void Dynamixel::setGoalLimits(float min, float max)
{
	min = 0x1ff + min * DYNAMIXEL_RD_TO_POS;
	max = 0x1ff + max * DYNAMIXEL_RD_TO_POS;

	if(min < 0)
	{
		min = 0;
	}
	else if(min > 0x3ff)
	{
		min = 0x3ff;
	}

	if( max < 0)
	{
		max = 0;
	}
	else if( max > 0x3ff)
	{
		max = 0x3ff;
	}

	m_minGoal = min;
	m_maxGoal = max;
}

void Dynamixel::setTargetReachedThreshold(float threshold)
{
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	m_targetReachedThreshold = threshold * DYNAMIXEL_RD_TO_POS;
	xSemaphoreGive(m_mutex);
}

bool Dynamixel::isFlagActive(uint32_t mask)
{
	bool res = false;
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	res = m_flags & mask;
	xSemaphoreGive(m_mutex);

	return res;
}

float Dynamixel::getPosition(DynamixelError* error)
{
	int32_t alpha = 0x1ff;
	xSemaphoreTake(m_mutex, portMAX_DELAY);
	alpha = m_pos;
	*error = m_lastError;
	xSemaphoreGive(m_mutex);

	// passage en rd
	// zero au milieu qui vaut "0x1ff"
	return (alpha - 0x1ff) * DYNAMIXEL_POS_TO_RD;
}
