#include "shell.h"
#include <stdio.h>
#include <stdarg.h>
#include "shell_port.h"

static Shell shell;

void Shell_Init(Shell_Write_Callback callback, uint8_t *p_buff, uint16_t size)
{
	shell.write = callback;
	shellInit(&shell, p_buff, size);
}

Shell *Shell_GetInstence(void)
{
	return &shell;
}