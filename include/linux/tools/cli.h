#ifndef CLI_H
#define CLI_H

#include <readline/readline.h>

enum
{
	CMD_SUCESS,
	CMD_ERROR,
	CMD_QUIT,
	CMD_UNKNOWN
};

typedef struct
{
	char *name;			/* User printable name of the function. */
	Function *func;		/* Function to call to do the job. */
	char *doc;			/* Documentation for this function.  */
} COMMAND;

int cli_init(COMMAND* cmd);

#endif