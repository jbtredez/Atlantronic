#include "log.h"
#include "module.h"

//! @file log.cxx
//! @brief Contient l'initialisation temporelle des log
//! @author Jean-Baptiste Trédez

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
		clock_gettime(NULL, &debutProgramme);
	}

	return 0;
}

