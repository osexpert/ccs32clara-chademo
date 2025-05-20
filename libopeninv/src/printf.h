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
#ifndef PRINTF_H_INCLUDED
#define PRINTF_H_INCLUDED

#if T_DEBUG
#define debugf(...) printf(__VA_ARGS__)
#else
#define debugf(...)
#endif

class IPutChar
{
public:
   virtual void PutChar(char c) = 0;
};


int printf(const char *format, ...);
int sprintf(char *out, const char *format, ...);
int fprintf(IPutChar* put, const char *format, ...);


#endif // PRINTF_H_INCLUDED
