#include "kernel.h"
#include "Task_Manager.h"

int main(void)
{
    bool kernel_init_state = false;
    
    kernel_init_state = Kernel_Init();

    /* if kernel init failed do infinity loop */
    while(!kernel_init_state);

    /* create task down below */
    Task_Manager_Init();
    /* create task up top */

    return 0;
}
