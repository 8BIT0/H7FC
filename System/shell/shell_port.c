#include "shell.h"
#include <stdio.h>
#include <stdarg.h>
#include "shell_port.h"

#define SHELL_BUFF_SIZE 512

static Shell shell;
static char shell_buff[SHELL_BUFF_SIZE];

/*can print the characters to the terminal with this function */
void Shell_Printf(const int *ch, ...)
{
	shellWriteString(&shell, ch);
}

static void bt_printf(const char *fmt, ...)
{
	va_list args;
	static char *bt_log_buf;
	int i = 0;
	int x = 0;
	va_start(args, fmt);

	vsnprintf(bt_log_buf, strlen(bt_log_buf) - 1, fmt, args);
	Shell_Printf(bt_log_buf);
	va_end(args);
}

void Shell_Init(Shell_Write_Callback callback)
{
	shell.write = callback;
	shellInit(&shell, shell_buff, sizeof(shell_buff));
}

Shell *Shell_GetInstence(void)
{
	return &shell;
}