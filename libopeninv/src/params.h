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

   typedef enum
   {
      FLAG_NONE = 0,
      FLAG_HIDDEN = 1
   } PARAM_FLAG;

   typedef enum
   {
      TYPE_PARAM,
      TYPE_TESTPARAM,
      TYPE_SPOTVALUE,
   } PARAM_TYPE;

   typedef struct
   {
      char const *category;
      char const *name;
      char const *unit;
      float min;
      float max;
      float def;
      uint16_t id;
      uint16_t type;
   } Attributes;

   //int    SetFixedPoint_RangeCheck_ChangeNotify(PARAM_NUM ParamNum, s32fp ParamVal);
//   s32fp  GetFixedPoint(PARAM_NUM ParamNum);
   int    GetInt(PARAM_NUM ParamNum);
   float  GetFloat(PARAM_NUM ParamNum);
   bool   GetBool(PARAM_NUM ParamNum);
   void   SetInt(PARAM_NUM ParamNum, int ParamVal);
   //void   SetFixedPoint(PARAM_NUM ParamNum, s32fp ParamVal);
   void   SetFloat(PARAM_NUM ParamNum, float ParamVal);
   PARAM_NUM NumFromString(const char *name);
   PARAM_NUM NumFromId(uint32_t id);
   const Attributes *GetAttrib(PARAM_NUM ParamNum);
   void LoadDefaults();
   void SetFlagsRaw(PARAM_NUM param, uint8_t rawFlags);
   void SetFlag(PARAM_NUM param, PARAM_FLAG flag);
   void ClearFlag(PARAM_NUM param, PARAM_FLAG flag);
   PARAM_FLAG GetFlag(PARAM_NUM param);
   PARAM_TYPE GetType(PARAM_NUM param);
   uint32_t GetIdSum();

   //User defined callback
   //void Change(Param::PARAM_NUM ParamNum);
}

#endif //PARAM_H_INCLUDED
