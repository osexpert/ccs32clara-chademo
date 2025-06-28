/*
 * This file is part of the slibopeninv project.
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

#include <stdint.h>
#include "my_string.h"

int my_strcmp(const char *str1, const char *str2)
{
   int res = 0;
   for (; *str1 > 0 && *str2 > 0; str1++, str2++)
   {
      if (*str1 != *str2)
      {
         break;
      }
   }
   if (*str1 != *str2)
   {
      res = 1;
   }
   return res;
}

void my_strcat(char *str1, const char *str2)
{
   for (; *str1 > 0; str1++);
   my_strcpy(str1, str2);
}

void my_strcpy(char *str1, const char *str2)
{
   for (; *str2 > 0; str1++, str2++)
      *str1 = *str2;
   *str1 = 0;
}

int my_strlen(const char *str)
{
   int len = 0;
   for (; *str > 0; str++, len++);
   return len;
}

const char *my_strchr(const char *str, const char c)
{
   for (; *str > 0 && *str != c; str++);
   return str;
}

int my_ltoa(char *buf, int val, int base)
{
   char *start = buf;
   char temp;
   int len = 0;

   if (val < 0)
   {
      *buf = '-';
      buf++;
      start++;
      len++;
      val = -val;
   }
   else if (0 == val)
   {
      *buf = '0';
      *(buf + 1) = 0;
      return 1;
   }

   for (; val > 0; val /= base, buf++, len++)
   {
      *buf = (val % base) + '0';
   }
   *buf = 0;
   buf--;
   for (; buf > start; buf--, start++)
   {
      temp = *start;
      *start = *buf;
      *buf = temp;
   }
   return len;
}

int my_atoi(const char *str)
{
   int Res = 0;
   int sign = 1;
   if ('-' == *str)
   {
      sign = -1;
      str++;
   }
   for (; *str >= '0' && *str <= '9'; str++)
   {
      Res *= 10;
      Res += *str - '0';
   }

   return sign * Res;
}

#define UTOA_FRACDEC 100 // 2 decimal digits

char* my_ftoa(char* buf, float f)
{
    int sign = (f < 0.0f) ? -1 : 1;
    float absval = sign * f;

    int nat = (int)absval;
    float fracf = absval - nat;

    // Multiply and round to nearest integer
    uint32_t frac = (uint32_t)((fracf * UTOA_FRACDEC) + 0.5f);

    char* p = buf;
    if (sign < 0)
        *p++ = '-';

    p += my_ltoa(p, nat, 10);

    *p++ = '.';

    // Pad leading zeros in the fractional part
    for (uint32_t dec = UTOA_FRACDEC / 10; dec > 1; dec /= 10)
    {
        if ((frac / dec) == 0)
            *p++ = '0';
        else
            break;
    }

    my_ltoa(p, frac, 10);

    return buf;
}

char *my_trim(char *str)
{
  char *end;

  // Trim leading space
  while (' ' == *str || '\n' == *str || '\r' == *str) str++;

  if(0 == *str)  // All spaces?
     return str;

  // Trim trailing space
  end = str + my_strlen(str) - 1;
  while(end > str && (' ' == *end || '\n' == *end || '\r' == *end)) end--;

  // Write new null terminator
  *(end+1) = 0;

  return str;
}

void memcpy32(int* target, int *source, int length)
{
   while (length--)
      *target++ = *source++;
}

void memset32(int* target, int value, int length)
{
   while (length--)
      *target++ = value;
}
