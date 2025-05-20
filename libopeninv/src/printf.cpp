/*
	Copyright 2001, 2002 Georges Menie (www.menie.org)
	stdarg version contributed by Christian Ettinger

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*
	putchar is the only external dependency for this file,
	if you have a working putchar, leave it commented out.
	If not, uncomment the define below and
	replace outbyte(c) by your own function call.

#define putchar(c) outbyte(c)
*/

#include <stdarg.h>
#include "printf.h"
#include "my_fp.h"

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 14
#define PAD_RIGHT 1
#define PAD_ZERO 2

extern "C" void putchar(char c);

class ExternPutChar: public IPutChar
{
public:
   void PutChar(char c)
   {
      putchar(c);
   }
};

class StringPutChar: public IPutChar
{
public:
   StringPutChar(char *s) : s(s) {}
   void PutChar(char c) { *(s++) = c; }

private:
   char *s;
};

static int prints(IPutChar* put, const char *string, int width, int pad)
{
	int pc = 0, padchar = ' ';

	if (width > 0) {
		int len = 0;
		const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			put->PutChar(padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		put->PutChar(*string);
		++pc;
	}
	for ( ; width > 0; --width) {
		put->PutChar(padchar);
		++pc;
	}

	return pc;
}

static int printi(IPutChar* put, int i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	char *s;
	int t, neg = 0, pc = 0;
	unsigned int u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (put, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			put->PutChar('-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + prints (put, s, width, pad);
}

static int printfp(IPutChar* put, int i, int width, int pad)
{
	char print_buf[PRINT_BUF_LEN];

   fp_itoa(print_buf, i);

	return prints (put, print_buf, width, pad);
}

static int print(IPutChar* put, const char *format, va_list args )
{
	int width, pad;
	int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				char *s = (char *)va_arg( args, int );
				pc += prints (put, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) { // *format == 'd' || *format == 'i' 
				pc += printi (put, va_arg( args, int ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (put, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (put, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (put, va_arg( args, int ), 10, 0, width, pad, 'a');
				continue;
			}
			if ( *format == 'f' ) {
				pc += printfp (put, va_arg( args, int ), width, pad);
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += prints (put, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			put->PutChar(*format);
			++pc;
		}
	}
	va_end( args );
	return pc;
}

int printf(const char *format, ...)
{
   ExternPutChar pc;
   va_list args;

   va_start( args, format );
   return print( &pc, format, args );
}

int sprintf(char *out, const char *format, ...)
{
   StringPutChar pc(out);
   va_list args;

   va_start( args, format );

   int ret = print( &pc, format, args );

   pc.PutChar(0);

   return ret;
}

int fprintf(IPutChar* put, const char *format, ...)
{
   va_list args;

   va_start( args, format );

   return print( put, format, args );
}
