#ifndef CLI_H
#define CLI_H

#include <readline/readline.h>
#include <errno.h>

enum
{
	CMD_SUCESS,
	CMD_ERROR,
	CMD_QUIT,
	CMD_UNKNOWN
};

typedef struct
{
	char *name;			// User printable name of the function
	Function *func;		// Function to call to do the job
	char *doc;			// Documentation for this function
} COMMAND;

int cli_init(COMMAND* cmd);

void cli_log(char* msg, ...) __attribute__(( format(printf, 1, 2) ));

extern char cli_prompt[];

#define log_error(msg, arg ...) \
	do \
	{ \
		if( rl_line_buffer ) \
		{ \
			cli_log("\r%10s:%i: "msg"\n%s%s", __FUNCTION__, __LINE__, ##arg, cli_prompt, rl_line_buffer);\
		}\
		else \
		{ \
			cli_log("\r%10s:%i: "msg"\n%s", __FUNCTION__, __LINE__, ##arg, cli_prompt);\
		} \
	}while(0)

#define log_error_errno(msg, arg ...) \
	do \
	{ \
		extern int cli_last_error; \
		if( errno != cli_last_error) \
		{ \
			cli_last_error = errno; \
			if( rl_line_buffer ) \
			{ \
				cli_log("\r%10s:%i: "msg" : %s (%d)\n%s%s", __FUNCTION__, __LINE__, ##arg, strerror(errno), errno, cli_prompt, rl_line_buffer);\
			} \
			else \
			{ \
				cli_log("\r%10s:%i: "msg" : %s (%d)\n%s", __FUNCTION__, __LINE__, ##arg, strerror(errno), errno, cli_prompt);\
			} \
		}\
	}while(0)

#define log_info(msg, arg ...) \
	do \
	{ \
		if( rl_line_buffer ) \
		{ \
			cli_log("\r"msg"\n%s%s", ##arg, cli_prompt, rl_line_buffer);\
		} \
		else \
		{ \
			cli_log("\r"msg"\n%s", ##arg, cli_prompt);\
		} \
	}while(0)

#endif