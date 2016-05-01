#ifndef SRC_DISCO_BOT_H_
#define SRC_DISCO_BOT_H_

class Bot
{
	public:
		static void init();

	public:
		// Dimensions
		static float halfLength;
		static float halfWidth;
		static int rearOmronRange;

		static int leftWheel;	// Numero de moteur gauche
		static int rightWheel;	// Numero de moteur droit

		// Gains asservissement
		static float xKP;
		static float xKI;
		static float xKD;
		static float xMax;

		static float yKP;
		static float yKI;
		static float yKD;
		static float yMax;

		static float tethaKP;
		static float tethaKI;
		static float tethaKD;
		static float tethaMax;

		// Parametres moteurs et roues
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
};

#endif /* SRC_DISCO_BOT_H_ */
