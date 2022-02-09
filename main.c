#include "runtime.h"

void main(void)
{
    uint32_t i = 0;

    Runtime_Config(RUNTIME_TICK_FRQ_20K);

    while (1)
    {
        i++;
    }
}
