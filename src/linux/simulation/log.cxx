#include "log.h"

//! @file log.cxx
//! @brief Contient l'initialisation temporelle des log
//! @author Atlantronic

struct timespec debutProgramme;
FILE* _logStdout;
FILE* _logStderr;

int initLog()
{
	static int init = 0;
	if( ! init)
	{
		init++;
		_logStdout = stdout;
		_logStderr = stderr;
		clock_gettime(0, &debutProgramme);
	}

	return 0;
}

