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

#ifndef PARAM_H_INCLUDED
#define PARAM_H_INCLUDED

#include "param_prj.h"
#include <stdint.h>

namespace Param
{
   #define PARAM_ENTRY(category, name, unit, min, max, def, id) name,
   #define TESTP_ENTRY(category, name, unit, min, max, def, id) name,
   #define VALUE_ENTRY(name, unit, id) name,
   typedef enum
   {
       PARAM_LIST
       PARAM_LAST,
       PARAM_INVALID
   } PARAM_NUM;
   #undef PARAM_ENTRY
   #undef TESTP_ENTRY
   #undef VALUE_ENTRY

   int    GetInt(PARAM_NUM ParamNum);
   bool   GetBool(PARAM_NUM ParamNum);
   void   SetInt(PARAM_NUM ParamNum, int ParamVal);
}

#endif //PARAM_H_INCLUDED
