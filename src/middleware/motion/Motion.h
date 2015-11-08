#ifndef MOTION_H
#define MOTION_H

//! @file Motion.h
//! @brief Gestion du deplacement du robot (asservissement)
//! @author Atlantronic

#include <stdint.h>

#include "kernel/systick.h"
#include "kernel/asm/asm_base_func.h"
#include "kernel/CanMipMotor.h"
#include "kernel/math/VectPlan.h"
#include "middleware/state_machine/StateMachine.h"
#include "motion_speed_check.h"
#include "pid.h"
#include "middleware/detection.h"
#include "kernel/kinematics_model/KinematicsModel.h"

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

enum motion_trajectory_step
{
	MOTION_TRAJECTORY_PRE_ROTATE = 0,
	MOTION_TRAJECTORY_STRAIGHT,
	MOTION_TRAJECTORY_ROTATE,
};

enum motion_way
{
	WAY_BACKWARD = -1,    //!< marche arriere
	WAY_ANY  = 0,         //!< marche avant ou marche arriere (selon le plus rapide)
	WAY_FORWARD  = 1,     //!< marche avant
};

enum motion_trajectory_type
{
	MOTION_AXIS_XYA = 0,   //!< aller a la position x,y, alpha en ligne droite (=> rotation puis avance puis rotation)
	MOTION_AXIS_A,         //!< rotation sur place
	MOTION_AXIS_XY,        //!< aller a la position x,y en ligne droite (=> rotation puis avance)
};

struct motion_cmd_param_arg
{
	float kp_av;
	float ki_av;
	float kd_av;
	float kp_rot;
	float ki_rot;
	float kd_rot;
}  __attribute__((packed));

struct motion_cmd_max_speed_arg
{
	uint32_t vmax_av;
	uint32_t vmax_rot;
}  __attribute__((packed));

struct motion_cmd_goto_arg
{
	VectPlan dest;
	VectPlan cp;
	int8_t way;
	int8_t type;
	KinematicsParameters linearParam;
	KinematicsParameters angularParam;
}  __attribute__((packed));

struct motion_cmd_set_speed_arg
{
	VectPlan u;
	float v;
}  __attribute__((packed));

struct motion_cmd_set_actuator_kinematics_arg
{
	int mode[CAN_MOTOR_MAX];
	float val[CAN_MOTOR_MAX];
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
		int init(Detection* detection, Location* location, KinematicsModel* kinematicsModel);

		void getState(enum motion_state* state, enum motion_status* status, enum motion_trajectory_step* step, enum motion_state* wanted_state);

		void enable(bool enable);

		//!< demande de trajectoire
		void goTo(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam);

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

		float findRotate(float start, float end);

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
		enum motion_trajectory_step m_trajStep;
		struct motion_cmd_set_actuator_kinematics_arg m_wantedKinematics; // cinematique desiree (mode MOTION_ACTUATOR_KINEMATICS)
		Kinematics m_kinematics[CAN_MOTOR_MAX];
		Kinematics m_kinematicsMes[CAN_MOTOR_MAX];
		xSemaphoreHandle m_mutex;
		VectPlan m_wantedDest;
		enum motion_way m_wantedWay;
		enum motion_trajectory_type m_wantedTrajectoryType;
		KinematicsParameters m_wantedLinearParam;
		KinematicsParameters m_wantedAngularParam;
		float m_ds[3];
		float m_v;
		VectPlan m_u;
		Kinematics m_curvilinearKinematics;
		VectPlan m_posCmdTh;
		VectPlan m_speedCmd;
		VectPlan m_posMes;
		VectPlan m_speedMes;
		VectPlan m_dest;  //!< destination
		systime m_targetNotReachedStartTime;
		MotionSpeedCheck m_linearSpeedCheck;
		Pid m_xPid;
		Pid m_thetaPid;
		bool m_anticoOn;
		static StateMachineState* m_motionStates[MOTION_MAX_STATE];
		StateMachine m_motionStateMachine;
		CanMipMotor m_canMotor[CAN_MOTOR_MAX];
		Detection* m_detection;
		Location* m_location;
		KinematicsModel* m_kinematicsModel;
};

#endif

#endif
