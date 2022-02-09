#include "debug_util.h"

bool CreateDebugPin()
{
}

bool DebugPin_Ctl(bool state)
{
}

void assert(bool state)
{
    if (state)
        while (true)
            ;
}
