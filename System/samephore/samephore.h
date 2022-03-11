#ifndef __SAMEPHORE_H
#define __SAMEPHORE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef uint32_t Samephore_Handler;

bool Samephore_CreateBin();
bool Samephore_CreateCnt();
bool Samephore_CreateMutex();
bool Samephore_Send();
bool Samepjore_Wait();

#endif
