#include <irrlicht/irrlicht.h>
#include <irrlicht/driverChoice.h>
#include <getopt.h>
#include "Environnement.h"
#include "log.h"
#include "Robot.h"

int main(int argc, char* argv[])
{
	char* prog_b = NULL;
	char* prog_r = NULL;

	initLog();

	if(argc > 1)
	{
		int option = -1;
		while( (option = getopt(argc, argv, "b:r:")) != -1)
		{
			switch(option)
			{
				case 'b':
					prog_b = optarg;
					break;
				case 'r':
					prog_r = optarg;
					break;
				default:
					meslog(_erreur_, "option %c inconnue", (char)option);
					return 1;
					break;
			}
		}
	}

	if(prog_b == NULL)
	{
		meslog(_erreur_, "indiquer le programme bleu (option -b)");
		return 2;
	}

	if(prog_r == NULL)
	{
		meslog(_erreur_, "indiquer le programme rouge (option -r)");
		return 3;
	}

	if(argc > optind)
	{
		meslog(_erreur_, "trop de parametres");
		return 4;
	}

	Environnement env;

	env.configure(0,0,0);
	env.start(prog_b, prog_r);
	env.loop();

	return 0;
}

