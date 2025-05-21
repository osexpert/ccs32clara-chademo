/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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
#include "digio.h"

#define DIG_IO_OFF 0
#define DIG_IO_ON  1

#undef DIG_IO_ENTRY
#define DIG_IO_ENTRY(name, port, pin, mode) DigIo DigIo::name;
DIG_IO_LIST

void DigIo::Configure(uint32_t port, uint16_t pin, PinMode::PinMode pinMode)
{
    uint8_t mode = GPIO_MODE_INPUT;
    uint8_t pull = GPIO_PUPD_NONE;

    _port = port;
    _pin = pin;

    switch (pinMode)
    {
    default:
    case PinMode::INPUT_PD:
        pull = GPIO_PUPD_PULLDOWN;
        break;
    case PinMode::INPUT_PU:
        pull = GPIO_PUPD_PULLUP;
        break;
    case PinMode::INPUT_FLT:
    case PinMode::INPUT_AIN:
        /* use defaults */
        break;
    case PinMode::OUTPUT:
        mode = GPIO_MODE_OUTPUT;
        break;
    }

    // todo: high or lowe speed??

    gpio_mode_setup(port, mode, pull, pin);

   // if (mode == GPIO_MODE_OUTPUT)
    {
    //    gpio_set_output_options(port, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, pin);
    }

}
