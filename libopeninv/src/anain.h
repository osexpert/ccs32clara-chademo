/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef ANAIO_H_INCLUDED
#define ANAIO_H_INCLUDED

#include <stdint.h>
#include "anain_prj.h"


class AnaIn
{
public:
	AnaIn(int chan) : firstValue(&values[chan]) {}

#define ANA_IN_ENTRY(name, port, pin) static AnaIn name;
	ANA_IN_LIST
#undef ANA_IN_ENTRY

#define ANA_IN_ENTRY(name, port, pin) +1
		static const int ANA_IN_COUNT = ANA_IN_LIST;
#undef ANA_IN_ENTRY

	static void Start();
	void Configure(uint32_t port, uint8_t pin);
	uint16_t Get();

private:
	static uint16_t values[];
	static uint8_t channel_array[];

	uint16_t GetIndex() { return firstValue - values; }
	static uint8_t AdcChFromPort(uint32_t command_port, int command_bit);
	static int median3(int a, int b, int c);

	uint16_t* const firstValue;
};

//Configure all AnaIn objects from the given list
#define ANA_IN_ENTRY(name, port, pin) AnaIn::name.Configure(port, pin);
#define ANA_IN_CONFIGURE(l) l

#endif // ANAIO_H_INCLUDED