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
#ifndef MY_MATH_H_INCLUDED
#define MY_MATH_H_INCLUDED

#define ABS(a)   ((a) < 0?(-(a)) : (a))
#define SIGN(a)  ((a) < 0? -1 : 1)
#define MIN(a,b) ((a) < (b)?(a):(b))
#define MAX(a,b) ((a) > (b)?(a):(b))
#define RAMPUP(current, target, rate) ((target < current || (current + rate) > target) ? target : current + rate)
#define RAMPDOWN(current, target, rate) ((target > current || (current - rate) < target) ? target : current - rate)
#define IIRFILTER(l,n,c) (((n) + ((l) << (c)) - (l)) >> (c))
#define IIRFILTERF(l,n,c) (((n) + (l) * ((1 << (c)) - 1)) / (1 << (c)))
#define MEDIAN3(a,b,c)  ((a) > (b) ? ((b) > (c) ? (b) : ((a) > (c) ? (c) : (a))) \
                                   : ((a) > (c) ? (a) : ((b) > (c) ? (c) : (b))))
#define CHK_BIPOLAR_OFS(ofs) ((ofs < (2048 - 512)) || (ofs > (2048 + 512)))

#endif // MY_MATH_H_INCLUDED
