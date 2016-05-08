#include "bot.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
void Bot::init()
{
	usb_add_cmd(USB_CMD_ODO_WHEEL, &Bot::cmd_set_odo_wheel_radius, this);
	usb_add_cmd(USB_CMD_ODO_WAY, &Bot::cmd_set_odo_voie, this);
}


void Bot::cmd_set_odo_voie(void* arg, void* data)
{
	Bot* m = (Bot*) arg;
struct motion_cmd_odo_voie_arg* cmd = (struct motion_cmd_odo_voie_arg*) data;
	m->voieOdoPositif = cmd->voieOdoPostif;
	m->voieOdoNegatif = cmd->voieOdoNegatif;
	odoWheelKinematicsModelDiff.setOdoVoie(cmd->voieOdoPostif,cmd->voieOdoNegatif);
}
void Bot::cmd_set_odo_wheel_radius(void* arg, void* data)
{
	Bot* m = (Bot*) arg;
	struct Bot_cmd_odo_wheel_radius_arg* cmd = (struct Bot_cmd_odo_wheel_radius_arg*) data;

	m->odo1WheelRadius = cmd->odo1WheelRadius;
	m->odo2WheelRadius = cmd->odo2WheelRadius;
	motionEncoders[MOTION_MOTOR_LEFT].setOutputFactor( m->odo1Way * 2 * M_PI * m->odo1WheelRadius / (float)(m->odoEncoderResolution ));
    motionEncoders[MOTION_MOTOR_RIGHT].setOutputFactor(m->odo2Way * 2 * M_PI * m->odo2WheelRadius / (float)(m->odoEncoderResolution ));

}
void Bot::cmd_print_odo_wheel_radius(void* arg, void* /*data*/)
{
	Bot* m = (Bot*) arg;
	log_format(LOG_INFO, "Taille roue odo : %d (*1000), %d (*1000)", (int)(m->odo1WheelRadius * 1000),(int)(m->odo2WheelRadius * 1000));
}

void Bot::cmd_print_odo_voie(void* arg, void*/* data*/)
{
	Bot* m = (Bot*) arg;
	log_format(LOG_INFO, "Voie odo (Pos,Neg) : %d (*1000), %d (*1000)", (int)(m->voieOdoPositif * 1000),(int)(m->voieOdoNegatif * 1000));
}

float Bot::halfLength;
float Bot::halfWidth;
int Bot::rearOmronRange;
int Bot::leftWheel;
int Bot::rightWheel;
float Bot::xKP;
float Bot::xKI;
float Bot::xKD;
float Bot::xMax;
float Bot::yKP;
float Bot::yKI;
float Bot::yKD;
float Bot::yMax;
float Bot::tethaKP;
float Bot::tethaKI;
float Bot::tethaKD;
float Bot::tethaMax;
float Bot::voieMot;
float Bot::voieOdoPositif;
float Bot::voieOdoNegatif;
float Bot::driving1WheelRadius;
float Bot::driving2WheelRadius;
float Bot::motorDriving1Red;
float Bot::motorDriving2Red;
float Bot::motorRpmToVolt;
float Bot::odo1WheelRadius;
float Bot::odo2WheelRadius;
int Bot::odo1Way;
int Bot::odo2Way;
int Bot::odoEncoderResolution;

Trajectory trajectory;

Motion motion;
