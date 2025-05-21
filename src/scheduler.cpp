#include "scheduler.h"

#define MAX_TASKS 4

typedef struct {
    task_func_t task;
    uint32_t period_ms;
    uint32_t next_run;
    uint8_t enabled;
} scheduler_task_t;

static scheduler_task_t tasks[MAX_TASKS];
static uint8_t task_count = 0;

extern volatile uint32_t system_millis;

// Add a task to the scheduler
int scheduler_add_task(task_func_t func, uint32_t period_ms)
{
    if (task_count >= MAX_TASKS) {
        return -1; // no space
    }
    tasks[task_count].task = func;
    tasks[task_count].period_ms = period_ms;
    tasks[task_count].next_run = system_millis + period_ms;
    tasks[task_count].enabled = 1;
    task_count++;
    return 0;
}

// Run scheduler loop (call this in main)
void scheduler_run(void)
{
    uint32_t now = system_millis;
    for (uint8_t i = 0; i < task_count; i++) {
        if (tasks[i].enabled && ((int32_t)(now - tasks[i].next_run) >= 0)) {
            tasks[i].task();
            tasks[i].next_run += tasks[i].period_ms;
        }
    }
}