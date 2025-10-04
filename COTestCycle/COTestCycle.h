/*
 * Copyright (c) 2025 by Andreas Wagener (AW)
 * CANopen central device library for Arduino UNO R4.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.

 */
 
#ifndef CO_TESTCYCLE_H
#define CO_TESTCYCLE_H

/*-----------------------------------------------------
 * implements a class which uses a single instance of an MCDrive
 * to test the communication implemented there
 *
 * 2025-09-04 AW
 *
 *-----------------------------------------------------*/
 
//---includes ---

#include <stdint.h>

#include <CO402Drive.h>

const uint32_t MaxProfileSpeed = 1500;
const uint32_t MinProfileSpeed = 500;
const uint32_t DeltaProfileSpeed = 100;

class COTestCycle {
	public:
		COTestCycle(int8_t);
		COTestCycle(int8_t, uint8_t);
	
	  void ResetCycle();
	  
	  bool AlignDrive(CO402Drive *);
	  bool FirstMove(CO402Drive *);
	  uint32_t DoCycle(CO402Drive *, uint32_t);

	  uint32_t stepTime;	
	  uint32_t maxStepTime = 2000;
	
	  uint32_t MaxStep;
	  uint16_t ErrorWord;
	  uint8_t DigInStatus;
	
	private:
		uint32_t AccessStep = 0;
	  uint32_t reachedStep = 0;
	
	  int8_t homingMethod;
	  uint8_t refSwitch;
	  bool hasRefSwitch = false;
	
    int32_t ActTestMove = 3600*3;
    uint32_t ActProfileSpeed = MaxProfileSpeed;
    uint32_t ActProfileAcc = 1500;
    uint32_t ActProfileDec = 500;
    uint32_t ActDeltaProfileSpeed = DeltaProfileSpeed;
};

#endif
