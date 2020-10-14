#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include.h"
#include "timer.h"
#include "task.h"
#include "comfunc.h"
#include "sec_tick.h"
#include "cmd.h"

//---------------------------------------------------------------------------------------
volatile uint32_t sec_jiffies = INITIAL_JIFFIES;
//---------------------------------------------------------------------------------------
static void sec_tick_handle(ULONG arg)
{
    extern struct task soft_timer_task;

    sec_jiffies++;
    task_start(&soft_timer_task);
}
//---------------------------------------------------------------------------------------
int sec_tick_init(void)
{
    estimer_t tick_tmr = timer_create(TICKS_PER_SEC / 10, TICKS_PER_SEC / 10, TMR_RUNNING,
		TMR_PERIODIC, sec_tick_handle, NULL, "sec_tick_tmr");
    if (!tick_tmr)
    {
        esprintf("creat sec tick timer failed!\n");
        return -1;
    }

    return 0;
}

unsigned int jiffies_to_msecs(const unsigned long j)
{
	return (1000 / SEC_HZ) * j;
}
//---------------------------------------------------------------------------------------
