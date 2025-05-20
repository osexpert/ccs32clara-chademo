/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
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

#ifndef HWINIT_H_INCLUDED
#define HWINIT_H_INCLUDED


#ifdef __cplusplus
extern "C"
{
#endif

void can_setup(void);
void clock_setup(void);
void nvic_setup(void);
void rtc_setup(void);
uint32_t rtc_get_ms(void);
void tim_setup(int variant);
//void write_bootloader_pininit();

#ifdef __cplusplus
}
#endif

#endif // HWINIT_H_INCLUDED
