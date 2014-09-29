/*
  Copyright (c) 2013 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

 /**
   *  \file
   *
   *  Include Defines & macros for the SN65HVD234.
   */

#ifndef _CAN_SN65HVD234_
#define _CAN_SN65HVD234_

#include "variant.h"

class SSN65HVD234
{
private:
    /** Rs Pin on PIO */
    uint32_t dwPin_Rs ;

    /** EN Pin on PIO */
    uint32_t dwPin_EN ;
public:
	uint32_t Init() ;
	uint32_t SetRs(uint32_t Rs ) ;
	uint32_t SetEN(uint32_t EN ) ;

	uint32_t Enable();
	uint32_t Disable();

	uint32_t EnableLowPower();
	uint32_t DisableLowPower();
	SSN65HVD234(uint32_t Rs, uint32_t En);
};

#endif /* _CAN_SN65HVD234_ */
