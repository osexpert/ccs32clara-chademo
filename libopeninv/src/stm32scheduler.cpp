/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2017 Johannes Huebner <contact@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "stm32scheduler.h"
#include "printf.h"
#include <libopencm3/stm32/rcc.h>

 /* return CCRc of TIMt */
#define TIM_CCR(t,c) (*(uint32_t *)(&TIM_CCR1(t) + (c)))

const enum tim_oc_id Stm32Scheduler::ocMap[MAX_TASKS] = { TIM_OC1, TIM_OC2, TIM_OC3, TIM_OC4 };

Stm32Scheduler::Stm32Scheduler(uint32_t timer)
{
    this->timer = timer;
    /* Setup timers upcounting and auto preload enable */
    timer_enable_preload(timer);
    timer_direction_up(timer);

    /* Set prescaler to count at 100 kHz. Most timers run at 2xAPB1 frequency */
    uint32_t clk = rcc_get_timer_clk_freq(timer);
    printf("Clock for scheduler timer: %d\r\n", clk);
    //timer_set_prescaler(timer, (rcc_apb2_frequency) / 100000 - 1);
    timer_set_prescaler(timer, (clk / 100000) - 1);

    /* Maximum counter value */
    timer_set_period(timer, 0xFFFF);

    nextTask = 0;
}

/** @brief Add a periodic task, can be called up to 4 times
   * @param function the task function
   * @param period The calling period in ms, maximum 655
   */
void Stm32Scheduler::AddTask(void (*function)(void), uint16_t period)
{
    if (nextTask >= MAX_TASKS)
    {
        printf("can't add more than MAX_TASK!");
        return;
    }

    if (period > 655)
        printf("period is more than max 655 and will run faster!");

    /* Disable timer */
    timer_disable_counter(timer);

    timer_set_oc_mode(timer, ocMap[nextTask], TIM_OCM_ACTIVE);
    timer_set_oc_value(timer, ocMap[nextTask], 0);

    /* Assign task function and period */
    functions[nextTask] = function;
    periods[nextTask] = period * 100;

    /* Enable interrupt for that channel */
    timer_enable_irq(timer, TIM_DIER_CC1IE << nextTask);

    /* Reset counter */
    timer_set_counter(timer, 0);

    /* Enable timer */
    timer_enable_counter(timer);

    nextTask++;
}

void Stm32Scheduler::Run()
{
    for (int i = 0; i < MAX_TASKS; i++)
    {
        if (i < nextTask && timer_get_flag(timer, TIM_SR_CC1IF << i))
        {
            uint16_t start = timer_get_counter(timer);

            TIM_CCR(timer, i) += periods[i];
            functions[i]();
            execTicks[i] = timer_get_counter(timer) - start;
            timer_clear_flag(timer, TIM_SR_CC1IF << i);
        }
        else if (i >= nextTask)
        {
            //Also clear flags of unused channels, they seem to fire the interrupt as well...
            timer_clear_flag(timer, TIM_SR_CC1IF << i);
        }
    }
}


int Stm32Scheduler::GetCpuLoad()
{
    int totalLoad = 0;
    for (int i = 0; i < nextTask; i++)
    {
        int load = (1000 * execTicks[i]) / periods[i];
        totalLoad += load;
    }
    return totalLoad;
}
