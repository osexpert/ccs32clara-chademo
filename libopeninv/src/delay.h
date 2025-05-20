/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2021 David J. Fiddes <D.J@fiddes.net>
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
#ifndef DELAY_H
#define DELAY_H

/**
 * \brief Blocking delay for a period
 *
 * \param[in] period Length of the delay in micro-seconds
 */
 /*
inline void uDelay(int period)
{
    // Empirically determined constant by measurement of GPIO toggle
    // of 1000 uS delay on a 72MHz STM32F103 processor
    static const int CyclesPerMicroSecond = 12;

    int iterations = period * CyclesPerMicroSecond;

    for (int i = 0; i < iterations; i++)
    {
        __asm__("nop");
    }
}
*/
#endif // DELAY_H
