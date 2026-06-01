/*
 * Copyright (C) 2007-2018 Siemens AG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*******************************************************************
 *
 * @author Daniel.Peintner.EXT@siemens.com
 * @author Sebastian.Kaebisch@siemens.com
 * @version 0.9.4
 * @contact Richard.Kuntschke@siemens.com
 *
 * <p>Switch for sample programs: EXI codec only or for entire V2G example</p>
 *
 ********************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "main.h"

int main(int argc, char *argv[]) {
    /* disable buffering of output, especially when piped or redirected */
    setvbuf(stdout, NULL, _IONBF, 0);

#if CODE_VERSION == CODE_VERSION_EXI
	/* EXI codec only */
	return main_databinder(argc, argv);
#elif CODE_VERSION == CODE_VERSION_SAMPLE
	/* V2G client / service example  */
	return main_example(argc, argv);
#endif

}

