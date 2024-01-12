#ifndef __SHELL_PORT_H
#define __SHELL_PORT_H

#include "shell.h"

typedef int (*Shell_Write_Callback)(const uint8_t *ch, uint16_t len);

void Shell_Init(Shell_Write_Callback callback, uint8_t *p_buff, uint16_t size);
Shell *Shell_GetInstence(void);

#endif
