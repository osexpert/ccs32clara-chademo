
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
 * @author Sebastian.Kaebisch@siemens.com
 * @author Daniel.Peintner.EXT@siemens.com
 * @version 0.9.4
 * @contact Richard.Kuntschke@siemens.com
 *
 ********************************************************************/

/*
 * This file implements the v2gtp header writer and reader.
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "v2gtp.h"


int write_v2gtpHeader(uint8_t* outStream, uint32_t outStreamLength, uint16_t payloadType)
{

	/* write v2gtp version number 1=byte */
	outStream[0]=V2GTP_VERSION;

	/* write inverse v2gtp version */
	outStream[1]=V2GTP_VERSION_INV;


	/* write payload type */
	outStream[3] = (uint8_t)(payloadType & 0xFF);
	outStream[2] = (uint8_t)(payloadType >> 8 & 0xFF);

	/* write payload length */
	outStream[7] = (uint8_t)(outStreamLength & 0xFF);
	outStream[6] = (uint8_t)(outStreamLength>>8 & 0xFF);
	outStream[5] = (uint8_t)(outStreamLength>>16 & 0xFF);
	outStream[4] = (uint8_t)(outStreamLength>>24 & 0xFF);

	return 0;
}

int read_v2gtpHeader(uint8_t* inStream, uint32_t* payloadLength)
{
	uint16_t payloadType=0;


	/* check, if we support this v2gtp version */
	if(inStream[0]!=V2GTP_VERSION || inStream[1]!=V2GTP_VERSION_INV)
		return -1;


	/* check, if we support this payload type*/
	payloadType = inStream[2];
	payloadType = (payloadType << 8 | inStream[3]);

	if(payloadType != V2GTP_EXI_TYPE)
		return -1;


	/* determine payload length*/
	*payloadLength = inStream[4];
	*payloadLength = (*payloadLength << 8 | inStream[5]);
	*payloadLength = (*payloadLength << 8 | inStream[6]);
	*payloadLength = (*payloadLength << 8 | inStream[7]);

	return 0;
}

