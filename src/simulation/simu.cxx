#include <irrlicht/irrlicht.h>
#include <irrlicht/driverChoice.h>
#include "Environnement.h"
#include "log.h"
#include "Robot.h"

int main(int argc, char* argv[])
{
	(void) argc;
	(void) argv;

	initLog();

	Environnement env;

	env.configure(0,0,0);
	env.start("bin/arm_cm3/test_control", "bin/arm_cm3/test_control");
	env.loop();

	return 0;
}

