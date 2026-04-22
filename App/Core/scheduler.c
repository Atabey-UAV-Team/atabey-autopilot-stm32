#include "scheduler.h"
#include "../Utils/time.h"
#include "stm32f4xx_hal.h"

static sched_task_t s_tasks[SCHED_MAX_TASKS];
static uint32_t     s_task_count;

void sched_init(void)
{
    s_task_count = 0;
    for (uint32_t i = 0; i < SCHED_MAX_TASKS; ++i) {
        s_tasks[i].name          = 0;
        s_tasks[i].period_ms     = 0;
        s_tasks[i].fn            = 0;
        s_tasks[i].last_run_ms   = 0;
        s_tasks[i].run_count     = 0;
        s_tasks[i].overrun_count = 0;
        s_tasks[i].last_exec_us  = 0;
        s_tasks[i].max_exec_us   = 0;
    }
}

int sched_add_task(const char *name, uint32_t period_ms, sched_task_fn fn)
{
    if (fn == 0 || period_ms == 0) {
        return -1;
    }
    if (s_task_count >= SCHED_MAX_TASKS) {
        return -1;
    }

    sched_task_t *t = &s_tasks[s_task_count];
    t->name          = name;
    t->period_ms     = period_ms;
    t->fn            = fn;
    t->last_run_ms   = HAL_GetTick();
    t->run_count     = 0;
    t->overrun_count = 0;
    t->last_exec_us  = 0;
    t->max_exec_us   = 0;

    return (int)s_task_count++;
}

void sched_run(void)
{
    for (uint32_t i = 0; i < s_task_count; ++i) {
        sched_task_t *t = &s_tasks[i];

        uint32_t now     = HAL_GetTick();
        uint32_t elapsed = now - t->last_run_ms;

        if (elapsed < t->period_ms) {
            continue;
        }

        /* Overrun: scheduled slot missed by a full period or more.
         * Skip catch-up; realign to current time. */
        if (elapsed >= 2U * t->period_ms) {
            t->overrun_count++;
            t->last_run_ms = now;
            continue;
        }

        t->last_run_ms += t->period_ms;

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
    if (index >= s_task_count) {
        return 0;
    }
    return &s_tasks[index];
}
