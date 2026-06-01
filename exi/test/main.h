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
 *
 ********************************************************************/


#define CODE_VERSION_EXI 1
#define CODE_VERSION_SAMPLE 2
#define CODE_VERSION CODE_VERSION_SAMPLE

#ifndef MAIN_H_
#define MAIN_H_

#if CODE_VERSION == CODE_VERSION_EXI
int main_databinder(int argc, char *argv[]);
#elif CODE_VERSION == CODE_VERSION_SAMPLE
int main_example(int argc, char *argv[]);
#endif

#endif
