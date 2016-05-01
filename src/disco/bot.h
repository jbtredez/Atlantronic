#ifndef SRC_DISCO_BOT_H_
#define SRC_DISCO_BOT_H_

class Bot
{
	public:
		static void init();

	public:
		static int leftWheel;	// Numero de moteur gauche
		static int rightWheel;	// Numero de moteur droit

		static float voieMot;
		static float voieOdo;
		static float driving1WheelRadius;
		static float driving2WheelRadius;
		static float motorDriving1Red;
		static float motorDriving2Red;
		static float motorRpmToVolt;

		static float odo1WheelRadius;
		static float odo2WheelRadius;
		static int odo1Way;
		static int odo2Way;
		static int odoEncoderResolution;

		static float halfLength;
		static float halfWidth;

		static int rearOmronRange;
};

#endif /* SRC_DISCO_BOT_H_ */
