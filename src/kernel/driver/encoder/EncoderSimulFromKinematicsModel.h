#ifndef ENCODER_SIMUL_FROM_KINEMATICS_MODEL_H
#define ENCODER_SIMUL_FROM_KINEMATICS_MODEL_H

#include "EncoderInterface.h"
#include "kernel/location/location.h"
#include "kernel/kinematics_model/KinematicsModelDiff.h"

class EncoderSimulFromKinematicsModel : public EncoderInterface
{
	public:
		void init(Location* location, KinematicsModelDiff* model, int wheelId);
		void update(float dt);
		float getPosition();
		float getSpeed();

	protected:
		Location* m_location;
		KinematicsModelDiff* m_model;
		Kinematics m_kinematics[2];
		int m_wheelId;
};

#endif
