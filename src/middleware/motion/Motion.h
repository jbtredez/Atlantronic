#ifndef MOTION_H
#define MOTION_H

//! @file Motion.h
//! @brief Gestion du deplacement du robot (asservissement)
//! @author Atlantronic

#include <stdint.h>

#include "kernel/systick.h"
#include "kernel/asm/asm_base_func.h"
#include "kernel/MotorInterface.h"
#include "kernel/math/VectPlan.h"
#include "middleware/state_machine/StateMachine.h"
#include "motion_speed_check.h"
#include "pid.h"
#include "middleware/detection.h"
#include "kernel/kinematics_model/KinematicsModel.h"
#include "kernel/driver/encoder/EncoderInterface.h"
#include "Path.h"

#ifndef WEAK_MOTION
#define WEAK_MOTION __attribute__((weak, alias("nop_function") ))
#endif

#define MOTION_TARGET_REACHED_LIN_THRESHOLD_SQUARE          (10*10)
#define MOTION_TARGET_REACHED_ANG_THRESHOLD                  0.02f
#define MOTION_TARGET_NOT_REACHED_TIMEOUT                      300

enum motion_state
{
	MOTION_DISABLED = 0,         //!< pas de puissance sur les moteurs
	MOTION_TRY_ENABLE,           //!< mise en puissance des moteurs
	MOTION_ENABLED,              //!< moteurs avec puissance
	MOTION_ACTUATOR_KINEMATICS,  //!< pilotage des vitesses ou position des moteurs (debug)
	MOTION_SPEED,                //!< robot pilote en vitesse (mode manuel)
	MOTION_TRAJECTORY,           //!< trajectoire en cours
	MOTION_INTERRUPTING,         //!< arret en cours
//	MOTION_BACK_TO_WALL,         //!< pas d'asservissement, les deux roues en marche arrière, pwm à x %. Arrêt quand le robot ne bouge plus
	MOTION_MAX_STATE,
	MOTION_UNKNOWN_STATE,	     //!< etat neutre permet d'indiquer que l'utilisateur ne veut pas changer d'état
};

enum motion_status
{
	MOTION_UPDATING_TRAJECTORY = 0,  //!< mise a jour de la trajectoire en cours
	MOTION_TARGET_REACHED,           //!< cible atteinte
	MOTION_TARGET_NOT_REACHED,       //!< cible non atteinte
	MOTION_COLSISION,                //!< collision
	MOTION_TIMEOUT,                  //!< timeout
	MOTION_IN_MOTION,                //!< trajectorie en cours
};

struct motion_cmd_param_arg
{
	float kp_x;
	float ki_x;
	float kd_x;
	float kp_y;
	float ki_y;
	float kd_y;
	float kp_theta;
	float ki_theta;
	float kd_theta;
}  __attribute__((packed));

struct motion_cmd_max_speed_arg
{
	uint32_t vmax_av;
	uint32_t vmax_rot;
}  __attribute__((packed));

struct motion_cmd_set_speed_arg
{
	VectPlan u;
	float v;
}  __attribute__((packed));

struct motion_cmd_set_actuator_kinematics_arg
{
	int mode[MOTION_MOTOR_MAX];
	float val[MOTION_MOTOR_MAX];
}  __attribute__((packed));

struct motion_cmd_enable_arg
{
	uint8_t enable;
}  __attribute__((packed));

struct motion_cmd_set_max_driving_current_arg
{
	float maxDrivingCurrent;
}  __attribute__((packed));

#define MOTION_AUTO_ENABLE

#ifndef LINUX
class Motion
{
	public:
		int init(Detection* detection, Location* location, KinematicsModel* kinematicsModel, MotorInterface* motorLeft, MotorInterface* motorRight, EncoderInterface* encoderLeft, EncoderInterface* encoderRight);

		void getState(enum motion_state* state, enum motion_status* status, enum motion_state* wanted_state);

		void enable(bool enable);

		//!< demande de trajectoire
		void clearTrajectory();
		void addTrajectoryPoints(PathPoint* pt, int size);
		void setTrajectory(PathPoint* pt, int size);
		void startTrajectory(const KinematicsParameters &linearParam, const KinematicsParameters &angularParam);
		VectPlan getLastPathPoint();

		//! arret du mouvement en cours
		void stop();

		//!< demande de vitesse
		void setSpeed(VectPlan u, float v);

		//!< demande de cinematique actionneur (debug)
		void setActuatorKinematics(struct motion_cmd_set_actuator_kinematics_arg cmd);

		//!< mise a jour de l'asservissement
		void compute() WEAK_MOTION;

		void updateUsbData(struct control_usb_data* data) WEAK_MOTION;

		void setMaxDrivingCurrent(float maxCurrent);

		void enableAntico(bool enable);

	protected:
		friend class MotionDisabledState;
		friend class MotionTryEnableState;
		friend class MotionEnabledState;
		friend class MotionMoveState;
		friend class MotionActuatorKinematicsState;
		friend class MotionSpeedState;
		friend class MotionTrajectoryState;
		friend class MotionInterruptingState;

		// interface usb
		static void cmd_goto(void* arg, void* data);
		static void cmd_set_speed(void* arg, void* data);
		static void cmd_enable(void* arg, void* data);
		static void cmd_set_actuator_kinematics(void* arg, void* data);
		static void cmd_set_max_current(void* arg, void* data);
		static void cmd_print_param(void* arg, void* data);
		static void cmd_set_param(void* arg, void* data);

		void motionUpdateMotors();
		float motionComputeTime(float ds, KinematicsParameters param);
		unsigned int motionStateGenericPowerTransition(unsigned int currentState);

		enum motion_state m_wantedState;
		enum motion_status m_status;
		struct motion_cmd_set_actuator_kinematics_arg m_wantedKinematics; // cinematique desiree (mode MOTION_ACTUATOR_KINEMATICS)
		Kinematics m_kinematics[MOTION_MOTOR_MAX];
		Kinematics m_kinematicsMes[MOTION_MOTOR_MAX];
		xSemaphoreHandle m_mutex;
		KinematicsParameters m_wantedLinearParam;
		KinematicsParameters m_wantedAngularParam;
		float m_v;
		VectPlan m_u;
		VectPlan m_speedCmd;
		VectPlan m_posMes;
		VectPlan m_speedMes;
		systime m_targetNotReachedStartTime;
		MotionSpeedCheck m_linearSpeedCheck[MOTION_MOTOR_MAX];
		Pid m_xPid;
		Pid m_yPid;
		Pid m_thetaPid;
		bool m_anticoOn;
		static StateMachineState* m_motionStates[MOTION_MAX_STATE];
		StateMachine m_motionStateMachine;
		MotorInterface* m_motionMotor[MOTION_MOTOR_MAX];
		EncoderInterface* m_motionEncoder[MOTION_MOTOR_MAX];
		Detection* m_detection;
		Location* m_location;
		KinematicsModel* m_kinematicsModel;
		Path m_path;
};

#endif

#endif
