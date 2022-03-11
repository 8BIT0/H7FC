#ifndef __SHELL_PORT_H
#define __SHELL_PORT_H

#include "shell.h"

typedef int (*Shell_Write_Callback)(const uint8_t *ch, uint16_t len);

void Shell_Init(Shell_Write_Callback callback);
Shell *Shell_GetInstence(void);
void user_shell_write_byte(const int *ch, ...);
void Shell_Printf(const int *ch, ...);

#endif
