#ifndef APP_CORE_SCHEDULER_H
#define APP_CORE_SCHEDULER_H

#include <stdint.h>

#ifndef SCHED_MAX_TASKS
#define SCHED_MAX_TASKS 16
#endif

typedef void (*sched_task_fn)(void);

typedef struct {
    const char   *name;
    uint32_t      period_ms;
    sched_task_fn fn;

    uint32_t      last_run_ms;
    uint32_t      run_count;
    uint32_t      overrun_count;
    uint32_t      last_exec_us;
    uint32_t      max_exec_us;
} sched_task_t;

void sched_init(void);
int  sched_add_task(const char *name, uint32_t period_ms, sched_task_fn fn);
void sched_run(void);

uint32_t             sched_task_count(void);
const sched_task_t  *sched_get_task(uint32_t index);

#endif /* APP_CORE_SCHEDULER_H */
