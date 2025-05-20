///*
// * This file is part of the libopeninv project.
// *
// * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
// *
// * This program is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with this program.  If not, see <http://www.gnu.org/licenses/>.
// */
//
//#include <libopencm3/stm32/flash.h>
//#include <libopencm3/stm32/desig.h>
//#include <libopencm3/stm32/crc.h>
//#include "params.h"
//#include "param_save.h"
//#include "hwdefs.h"
//#include "my_string.h"
//
////#define NUM_PARAMS ((PARAM_BLKSIZE - 8) / sizeof(PARAM_ENTRY))
////#define PARAM_WORDS (PARAM_BLKSIZE / 4)
////
////typedef struct
////{
////   uint16_t key;
////   uint8_t dummy;
////   uint8_t flags;
////   uint32_t value;
////} PARAM_ENTRY;
////
////typedef struct
////{
////   PARAM_ENTRY data[NUM_PARAMS];
////   uint32_t crc;
////   uint32_t padding;
////} PARAM_PAGE;
////
