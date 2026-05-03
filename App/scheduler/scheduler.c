#include "scheduler.h"
#include "../utils/time.h"

/*
 * Microsecond cooperative scheduler.
 * - All times are uint32 µs from DWT (wraps ~50 s @ 84 MHz).
 *   Compare via signed difference so wrap is handled naturally.
 * - Missed cycles are skipped (no catch-up bursts): if we slip past one or
 *   more periods we realign next_run_us to the present and bump overruns.
 */

static sched_task_t s_tasks[SCHED_MAX_TASKS];
static uint32_t     s_task_count;

void sched_init(void)
{
    s_task_count = 0;
    for (uint32_t i = 0; i < SCHED_MAX_TASKS; ++i) {
        s_tasks[i].name          = 0;
        s_tasks[i].period_us     = 0;
        s_tasks[i].fn            = 0;
        s_tasks[i].next_run_us   = 0;
        s_tasks[i].run_count     = 0;
        s_tasks[i].overrun_count = 0;
        s_tasks[i].last_exec_us  = 0;
        s_tasks[i].max_exec_us   = 0;
    }
}

int sched_add_task(const char *name, uint32_t period_us, sched_task_fn fn)
{
    if (fn == 0 || period_us == 0) return -1;
    if (s_task_count >= SCHED_MAX_TASKS) return -1;

    sched_task_t *t = &s_tasks[s_task_count];
    t->name        = name;
    t->period_us   = period_us;
    t->fn          = fn;
    t->next_run_us = micros() + period_us;
    return (int)s_task_count++;
}

void sched_run(void)
{
    for (uint32_t i = 0; i < s_task_count; ++i) {
        sched_task_t *t = &s_tasks[i];

        uint32_t now   = micros();
        int32_t  delta = (int32_t)(now - t->next_run_us);
        if (delta < 0) {
            continue;  /* not due yet */
        }

        /* Skip missed cycles: if we are more than one period late,
         * realign to "now" and count an overrun. */
        if ((uint32_t)delta >= t->period_us) {
            t->overrun_count++;
            t->next_run_us = now + t->period_us;
        } else {
            t->next_run_us += t->period_us;
        }

        uint32_t t0 = micros();
        t->fn();
        uint32_t dt = micros() - t0;

        t->last_exec_us = dt;
        if (dt > t->max_exec_us) {
            t->max_exec_us = dt;
        }
        t->run_count++;
    }
}

uint32_t sched_task_count(void)
{
    return s_task_count;
}

const sched_task_t *sched_get_task(uint32_t index)
{
    if (index >= s_task_count) return 0;
    return &s_tasks[index];
}
