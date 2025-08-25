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

#include "params.h"
#include "my_string.h"

namespace Param
{

#define PARAM_ENTRY(category, name, unit, min, max, def, id) (def),
#define TESTP_ENTRY(category, name, unit, min, max, def, id) (def),
#define VALUE_ENTRY(name, unit, id) 0,
static int values[] =
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef TESTP_ENTRY
#undef VALUE_ENTRY

//Duplicate ID check
#define PARAM_ENTRY(category, name, unit, min, max, def, id) ITEM_##id,
#define TESTP_ENTRY(category, name, unit, min, max, def, id) ITEM_##id,
#define VALUE_ENTRY(name, unit, id) ITEM_##id,
enum _dupes
{
    PARAM_LIST
};
#undef PARAM_ENTRY
#undef TESTP_ENTRY
#undef VALUE_ENTRY


/**
* Get a parameters integer value
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
int GetInt(PARAM_NUM ParamNum)
{
    return values[ParamNum];
}

/**
* Get a parameters boolean value, 1.00=True
*
* @param[in] ParamNum Parameter index
* @return Parameters value
*/
bool GetBool(PARAM_NUM ParamNum)
{
    return values[ParamNum] == 1;
}

/**
* Set a parameters digit value
*
* @param[in] ParamNum Parameter index
* @param[in] ParamVal New value of parameter
*/
void SetInt(PARAM_NUM ParamNum, int ParamVal)
{
    values[ParamNum] = ParamVal;
}



}
