#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>


int main() {
    //modules_init();
    mtk_platform_init();

    platform_and_wlan_init_done();
	
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following line
    will never be reached.  If the following line does execute, then there was
    insufficient FreeRTOS heap memory available for the idle and/or timer tasks
    to be created.  See the memory management section on the FreeRTOS web site
    for more details. */
    for ( ;; );
}
