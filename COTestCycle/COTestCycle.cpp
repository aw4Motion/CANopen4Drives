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
 
/*---------------------------------------------------
 * COTestCycle.cpp
 * implements the class to test the drive
 *
 * 2025-09-04 AW Frame
 * 2025-09-17 AW added ref-switch
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <COTestCycle.h>

//--- local defines ---

COTestCycle::COTestCycle(int8_t method)
{
	homingMethod = method;
	MaxStep = 11;
}
	
COTestCycle::COTestCycle(int8_t method, uint8_t input)
{
	homingMethod = method;
	refSwitch = input;
	hasRefSwitch = true;
	
	MaxStep = 11;
}

void COTestCycle::ResetCycle()
{
	AccessStep = 0;
	reachedStep = 0;
}
	  
bool COTestCycle::AlignDrive(CO402Drive *Drive)
{
	bool returnValue = false;

	switch(AccessStep)
	{
		case 0:  //--set homing method--
			if(Drive->SetHomingMethod(homingMethod) == eCO_DriveDone)
			{
				AccessStep++;
			}       
			break;
		case 1:  //--enable--
			if(Drive->Enable() == eCO_DriveDone)
			{
				AccessStep++;
			}       
			break;
		case 2:  //--enable--
			if(Drive->SetOpMode(6) == eCO_DriveDone)
			{
				AccessStep++;
				stepTime = millis();
				pinMode(refSwitch, OUTPUT);
			}       
			break;
		case 3:  //--doHoming--
			if(Drive->DoHoming() == eCO_DriveDone)
			{
				AccessStep = 0;
				returnValue = true;
				Serial.print("TestCycle: Drive ");
				Serial.print(Drive->GetNodeId());
				Serial.println(" is aligned");
			}       
			break;
		default:
			Serial.print("Cycle: align: unexpected step @ NodeId ");
		  Serial.println(Drive->GetNodeId());
		  break;
	}
	if(hasRefSwitch)
	{
		if((millis() - stepTime) > maxStepTime)
		{
			digitalWrite(refSwitch, HIGH);
		}
	}
		
	return returnValue;
}

bool COTestCycle::FirstMove(CO402Drive *Drive)
{
  bool returnValue = false;

  switch(AccessStep)
	{		
		case 0:  //-- set OpMode PP --
		  if(Drive->SetOpMode(1) == eCO_DriveDone)
			{
				AccessStep++;
			}       
		  break;
	  case 1:  //--set target = 0 ---
		  if(Drive->SetTargetPos(0) == eCO_DriveDone)
			{
				AccessStep++;
			}       
		  break;
	  case 2:  //--move ---
		  if(Drive->StartMoveAbs(false) == eCO_DriveDone)
			{
				AccessStep++;
			}       
		  break;
	  case 3:
		  if(Drive->isInPos())
			{
				AccessStep = 0;
				returnValue = true;
			}       
		  break;
		default:
			Serial.print("Cycle: 1st: unexpected step @ NodeId ");
		  Serial.println(Drive->GetNodeId());
		  break;
	}
	return returnValue;
}

uint32_t COTestCycle::DoCycle(CO402Drive *Drive, uint32_t targetStep)
{
  if(reachedStep != targetStep)
	{		
		switch(targetStep)
		{
			case 1:  //--set target = positive ---
			  if(Drive->SetTargetPos(ActTestMove) == eCO_DriveDone)
			  {
				  reachedStep = targetStep;
			  }       
			  break;
		  case 2:  //--move ---
			  if(Drive->StartMoveAbs(false) == eCO_DriveDone)
			  {
				  reachedStep = targetStep;
			  }       
			  break;
			case 3:
				if(Drive->isInPos())
				{
					reachedStep = targetStep;
				}       
				break;
			case 4:  //--set target = negative ---
				if(Drive->SetTargetPos(-ActTestMove) == eCO_DriveDone)
				{
					reachedStep = targetStep;
				}       
				break;
			case 5:  //--move ---
				if(Drive->StartMoveAbs(false) == eCO_DriveDone)
				{
					reachedStep = targetStep;
				}       
				break;
			case 6:
				if(Drive->isInPos())
				{
					reachedStep = targetStep;
				}       
				break;
			case 7:  //--disable---  
				if(Drive->DisableVoltage() == eCO_DriveDone)
				{
					reachedStep = targetStep;
					 

					if(ActProfileSpeed >= MaxProfileSpeed)
						ActDeltaProfileSpeed = - DeltaProfileSpeed;
					else if(ActProfileSpeed <= MinProfileSpeed)
						ActDeltaProfileSpeed = DeltaProfileSpeed;

					ActProfileSpeed += ActDeltaProfileSpeed;
				}       
				break;
			case 8:  //--change profile
				if(Drive->UpdateProfile(ActProfileAcc, ActProfileSpeed, ActProfileDec) == eCO_DriveDone)
				{
					reachedStep = targetStep;
				}       
				break;
			case 9:  //--enable--
				if(Drive->Enable() == eCO_DriveDone)
				{
					reachedStep = targetStep;
				}       
				break;				
			case 10:
				if(Drive->GetErrorWord(&ErrorWord) == eCO_DriveDone)
				{
					#if 0
					Serial.print("Cycle: Error @ Node ");
					Serial.print(Drive->GetNodeId());
					Serial.print(":");
					Serial.println(ErrorWord,HEX);
					#endif
					
					reachedStep = targetStep;
				}       
        break;				
			case 11:
				if(Drive->GetDigInStatus(&DigInStatus) == eCO_DriveDone)
				{
					#if 0
					Serial.print("Cycle: DigIn @ Node ");
					Serial.print(Drive->GetNodeId());
					Serial.print(":");
					Serial.println(DigInStatus,HEX);
					#endif
					
					reachedStep = targetStep;
				}       
					
				break;
			default:
				Serial.print("Cycle: cycle: unexpected step ");
			  Serial.print(targetStep);
			  Serial.print(" @ NodeId ");
				Serial.println(Drive->GetNodeId());
				break;
	  }
	}
  return reachedStep;
}
