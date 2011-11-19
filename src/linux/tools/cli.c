#include <stdio.h>
#include <stdlib.h>
#include <readline/history.h>
#include "foo/control/control.h"
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include "linux/tools/cli.h"

void* cli_task(void* arg);

COMMAND *cli_commands = NULL;

char * stripwhite(char * string)
{
	char *s, *t;

	for (s = string; whitespace (*s); s++)
	{
		;
	}

	if (*s == 0)
	{
		return (s);
	}

	t = s + strlen (s) - 1;
	while (t > s && whitespace (*t))
	{
		t--;
	}

	*++t = '\0';

	return s;
}

char * dupstr (char* s)
{
	char *r;

	r = malloc (strlen (s) + 1);
	strcpy (r, s);
	return r;
}

char * cmd_generator(const char* text, int state)
{
	static int list_index, len;
	char *name;

	// If this is a new word to complete, initialize now.  This includes
	// saving the length of TEXT for efficiency, and initializing the index
	// variable to 0.
	if (!state)
	{
		list_index = 0;
		len = strlen (text);
	}

	// Return the next name which partially matches from the command list.
	while( (name = cli_commands[list_index].name) )
	{
		list_index++;

		if (strncmp (name, text, len) == 0)
			return (dupstr(name));
	}

	// If no names matched, then return NULL.
	return NULL;
}

char **completion(char* text, int start, int end)
{
	char **matches = NULL;
	(void) end;

	// If this word is at the start of the line, then it is a command
	// to complete.  Otherwise it is the name of a file in the current
	// directory.
	if (start == 0)
	{
		matches = rl_completion_matches(text, &cmd_generator);
	}

	return matches;
}

// Look up NAME as the name of a command, and return a pointer to that
// command.  Return a NULL pointer if NAME isn't a command name.
COMMAND * find_command (char* name)
{
	int i;

	for (i = 0; cli_commands[i].name; i++)
	{
		if (strcmp (name, cli_commands[i].name) == 0)
		{
			return (&cli_commands[i]);
		}
	}

	return NULL;
}

// Execute a command line.
int execute_line (char* line)
{
	int i;
	COMMAND *command;
	char *word;
	char sav = 0;

	/* Isolate the command word. */
	i = 0;
	while (line[i] && whitespace (line[i]))
	{
		i++;
	}

	word = line + i;

	while (line[i] && !whitespace (line[i]))
	{
		i++;
	}

	if (line[i])
	{
		sav = line[i];
		line[i] = '\0';
	}

	command = find_command (word);

	if(sav)
	{
		line[i] = sav;
		i++;
	}

	if (!command)
	{
		return CMD_UNKNOWN;
	}

	// Get argument to command, if any.
	while (whitespace (line[i]))
	{
		i++;
	}

	word = line + i;

	// Call the function.
	return ((*(command->func)) (word));
}

int cli_init(COMMAND* cmd, const char* prompt)
{
	pthread_t tid;

	if(cmd == NULL)
	{
		return -1;
	}

	cli_commands = cmd;

	return pthread_create(&tid, NULL, cli_task, (void*)prompt);
}

void* cli_task(void* arg)
{
	char *buf;
	(void) arg;

	rl_attempted_completion_function = (CPPFunction *)completion;

	// activation auto-complete
	rl_bind_key('\t',rl_complete);

	while((buf = readline(arg)) != NULL)
	{
		stripwhite(buf);

		if ( buf[0] )
		{
			add_history(buf);
			int res = execute_line(buf);

			switch(res)
			{
				case CMD_UNKNOWN:
					if(system(buf) == -1 )
					{
						printf("Erreur system\n");
					}
					break;
				case CMD_QUIT:
					goto end_free;
					break;
			}
		}
	}

end_free:
	free(buf);

	return 0;
}
