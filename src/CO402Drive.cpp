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
 * CO402Drive.cpp
 * implements the class to produce provide CiA 402 drive related function
 *
 * 2025-07-24 AW Frame
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <CO402Drive.h>

//--- local defines ---

#define DEBUG_DRIVE_TO			    0x0001
#define DEBUG_DRIVE_ERROR		    0x0002
#define DEBUG_DRIVE_Init        0x0004
#define DEBUG_DRIVE_WRITEOBJ    0x0010
#define DEBUG_DRIVE_READOBJ     0x0020
#define DEBUG_DRIVE_CW          0x0040

#define DEBUG_DRIVE (DEBUG_DRIVE_TO | DEBUG_DRIVE_ERROR | DEBUG_DRIVE_Init) 

//--- local definitions ---------

const uint16_t TSWWarningMask        = 0x0080;
const uint16_t TSWErrorMask          = 0x0100;
const uint16_t TSWTargetReachedMask  = 0x0400;
const uint16_t TSWLimitActiveMask    = 0x0800;
const uint16_t TSWSetPointAckMask    = 0x1000;

const uint16_t TSWIsSpeed0Mask       = 0x1000;
const uint16_t TSWIsSpeedReachedMask = 0x0400;

const uint16_t TSWIsHomingSpeed0Mask = 0x0400;
const uint16_t TSWIsHomingDone       = 0x1000;
const uint16_t TSWIsHomingError      = 0x2000;

const uint16_t TSWIsFaultState       = 0x0008;

const uint16_t TCWStartBit           = 0x0010;
const uint16_t TCWIsImmediateBit     = 0x0020;
const uint16_t TCWIsRelativeBit      = 0x0040;
const uint16_t TCWResetFaultMask     = 0x0080;


//--- public functions ---

/*---------------------------------------------------------------------
 * CO402Drive::CODrive(uint8_t thisId)
 * as of now there is noting to intialized when created
 * 
 * 2025-03-09 AW Done
 * ------------------------------------------------------------------*/

CO402Drive::CO402Drive(uint8_t thisId)
{
	nodeId = thisId;
	
  MapTxPDO1 = {3,{(ODEntry *)&OdActPos, (ODEntry *)&OdSW, (ODEntry *)&OdModesOfOpDisp,NULL}};
  MapRxPDO1 = {3,{(ODEntry *)&OdTargetPos, (ODEntry *)&OdCW, (ODEntry *)&OdModesOfOp,NULL}};

  MapTxPDO2 = {2,{(ODEntry *)&OdActSpeed, (ODEntry *)&OdActTorque, NULL, NULL}};
  MapRxPDO2 = {1,{(ODEntry *)&OdTargetSpeed, NULL, NULL, NULL}};

	uint8_t AccessStep = 0;
	
	for(uint8_t iter = 0; iter < 4; iter++)
	{
	  for(uint8_t charIdx = 0; charIdx < 32; charIdx++)
		{
	    ((ODEntryString *)IdentityEntries[iter])->Value[charIdx] = 0;
		}
		((ODEntryString *)IdentityEntries[iter])->len = 0;
	}
}


/*-------------------------------------------------------------------
 * void CO402Drive::init(COMsgHandler *ThisHandler)
 * 
 * store the instance to the MsgHandler
 * no need to be regsitered as we don't receive messages
 *
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

void CO402Drive::init(COMsgHandler *ThisHandler)
{
	uint8_t NodeHandle;
	
	MsgHandler = ThisHandler;

	NodeHandle = MsgHandler->RegisterNode(nodeId);
	
  Node.init(MsgHandler, nodeId, NodeHandle);
  
  //init the nodes PDO handler
  PDOHandler.init(MsgHandler, &Node, nodeId, NodeHandle);

	PDOHandler.PresetRxPDOTransmission(0, 255);  //parameters are the PDO# and the transmission type
	PDOHandler.PresetTxPDOTransmission(0, 255, 0, 0);  //parameters are the PDO#, the transmission type, the inhibit time and the EvtTimer
	
	PDOHandler.PresetRxPDOMapping(0, MapRxPDO1.NrEntries, MapRxPDO1.Entries);  //parameters are the PDO#, the number of actually mapped entries and the pointer to the entries
  PDOHandler.PresetTxPDOMapping(0, MapTxPDO1.NrEntries, MapTxPDO1.Entries);  //parameters are the PDO#, the number of actually mapped entries and the pointer to the entries

  PDOHandler.PresetRxPDOisValid(0,true);
  PDOHandler.PresetTxPDOisValid(0,true);
  
	PDOHandler.PresetRxPDOTransmission(1, 255);  //parameters are the PDO# and the transmission type
	PDOHandler.PresetTxPDOTransmission(1, 255, 0, 0);  //parameters are the PDO#, the transmission type, the inhibit time and the EvtTimer

	PDOHandler.PresetRxPDOMapping(1, MapRxPDO2.NrEntries, MapRxPDO2.Entries);  //parameters are the PDO#, the number of actually mapped entries and the pointer to the entries
  PDOHandler.PresetTxPDOMapping(1, MapTxPDO2.NrEntries, MapTxPDO2.Entries);  //parameters are the PDO#, the number of actually mapped entries and the pointer to the entries

  PDOHandler.PresetRxPDOisValid(1,true);
  PDOHandler.PresetTxPDOisValid(1,true);
}


/*-------------------------------------------------------------------
 * void CO402Drive::ResetComState()
 * 
 * reset the ComTate to the default Idle
 *
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/
void CO402Drive::ResetComState()
{
	AccessStep = 0;
}

/*-------------------------------------------------------------------
 * uint8_t CO402Drive::GetNodeId()
 * 
 * return the nodeId of this one
 *
 * 25-09-03 AW 
 *
 *-------------------------------------------------------------------*/
uint8_t CO402Drive::GetNodeId()
{
	return nodeId;
}


/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::Update(uint32_t actTime, COSyncState syncState)
 * 
 * cyclic update where the actTime is compared to the last time-stamp of either HB
 * or Sync and related messages are sent
 * these are trhow away messages - unconfirmed
 * for the HB the current node stante of this device is added
 * 
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

NMTNodeState CO402Drive::Update(uint32_t actTime, COSyncState syncState)
{
	CODriveCommStates returnValue = eCO_DriveIdle;
	
  NMTNodeState NodeState = Node.Update(actTime);
	
	if(NodeState < eNMTStateReset)
	{
	  isPDOsConfigured = false;
		PDOHandler.FlagPDOsInvalid();
		
		//might have seen a re-boot
		//invalidate the CW / SW values
		CWValue = 0;	
    SWValue = 0;
	}
	
	if(NodeState == eNMTStatePreOp)
	{		
		if(isPDOsConfigured == false)
		{
			if(reConfigPDOs == true)
      {
        if(InitPDOs(actTime) == eCO_DriveDone)
        {
				  #if(DEBUG_DRIVE & DEBUG_DRIVE_Init)
          Serial.println("Drive: PDO-Config re-established");
				  #endif
				
          ResetComState();
        }
		  }
		}
		else
		{
			if(autoEnable)
			{
			  //will automatically enable the drive only if preset accordingly
			  if(Node.SendStartNode() == eCO_NodeDone)
			  {
				  #if(DEBUG_DRIVE & DEBUG_DRIVE_Init)
				  Serial.println("Drive Update: Node auto-started");
				  #else
				  ;
				  #endif
				}
			}
		}
	}	
	
	if(NodeState == eNMTStateOperational)
    PDOHandler.Update(actTime, syncState);  
	

  return NodeState;	
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::InitNode(uint32_t actTime)
 * 
 * Init the remote node based on the preset configuration
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::InitNode(uint32_t actTime)
{
	CODriveCommStates returnValue = eCO_DriveBusy;
	
	if(Node.InitRemoteNode(actTime) == eNMTStatePreOp)
    returnValue = eCO_DriveDone;
			
  return returnValue;	
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::InitPDOs(uint32_t actTime)
 * 
 * init the PDO config of the remote node based on the prest configuration
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::InitPDOs(uint32_t actTime)
{
	CODriveCommStates returnValue = eCO_DriveBusy;
	
	if(PDOHandler.ConfigurePresetPDOs(actTime) == eCO_PDODone)
	{
    returnValue = eCO_DriveDone;
		isPDOsConfigured = true;
	}
	
  return returnValue;	
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::Enable()
 * 
 * step through the state machine to enable the drive
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/
	
CODriveCommStates CO402Drive::Enable()
{
	uint16_t SWCommandBits = SWValue & 0x007F;
  CODriveCommStates returnValue = eCO_DriveBusy;
	uint16_t newCWValue = CWValue;
	
	if(auteResetErrors == true)
		ResetError();

	//step for step state machine until enabled
	switch(SWCommandBits)
	{
		case 0x0000:
		case 0x0040:
			//is not ready to switch on
			newCWValue = 0x0006;  //--> ready to switch on
			break;
		case 0x0021:
			//is ready to switch on
			newCWValue = 0x0007;  //--> switchted on
			break;
		case 0x0023:
			//is switched on
		case 0x0007:
			//QS active
			newCWValue = 0x000F;
			break;
		case 0x0027:
			newCWValue = 0x000F;
			returnValue = eCO_DriveDone;
		  break;
		default:
		  //Fault still missing
		break;
	}
  CheckCWForTx(newCWValue);
	
	return returnValue;
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::Disable()
 * 
 * simply fores the state machine off
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::Disable()
{
	uint16_t SWCommandBits = SWValue & 0x007F;
  CODriveCommStates returnValue = eCO_DriveBusy;
	uint16_t newCWValue = 0;
	
  newCWValue = 0x0007;  //--> disable control ony
	
	//check for switched-on state
	if(SWCommandBits == 0x0023)
	{
		returnValue = eCO_DriveDone;
	}		
  CheckCWForTx(newCWValue);
	
	return returnValue;
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::Stop()
 * 
 * send a QS command
 * could end in stopped or ins switch on disabled
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::Stop()
{
  uint16_t SWCommandBits = SWValue & 0x007F;
  CODriveCommStates returnValue = eCO_DriveBusy;
	uint16_t newCWValue = 0;
	
  newCWValue = 0x0002;  //--> disable control ony
	
	//check for either QS active or switch-on disabled
	if((SWCommandBits == 0x0007) || (SWCommandBits == 0x0040))
	{
		returnValue = eCO_DriveDone;
	}		
  CheckCWForTx(newCWValue);
	
	return returnValue;
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::ResetError()
 * 
 * rest the state machine from fault state
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::ResetError()
{	
	CODriveCommStates returnValue = eCO_DriveBusy;

	//done only when the fault flag is gone
	if(SWValue & TSWIsFaultState)
		resetFault = true;
	else
	{
		resetFault = false;
		returnValue = eCO_DriveDone;
	}
	
	return returnValue;		
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::DisableVoltage()
 * 
 * simply shut it off
 * 
 * 25-07-26 AW 
 * 25-08-06 AW added fault reset
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::DisableVoltage()
{
	uint16_t SWCommandBits = SWValue & 0x007F;
  CODriveCommStates returnValue = eCO_DriveBusy;
	uint16_t newCWValue = 0;
	
  newCWValue = 0x0000;  //--> switch it off

	//check whether in Fault state an reset it if so
	if(SWValue & TSWIsFaultState)
		resetFault = true;
  else
    resetFault = false;		
	
	//check for switch-on disabled being reached
	if(SWCommandBits == 0x0040)
	{
		returnValue = eCO_DriveDone;
	}		
  CheckCWForTx(newCWValue);

	return returnValue;
}
	
	  
//--------- motion related commands ----------------------------------

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::UpdateProfile(uint32_t newPAcc, uint32_t newPSpeed, uint32_t newPDec)
 * 
 * update the profile parameters and check whether this can be done by PDO or is to be done by SDO
 * 
 * 25-08-07 AW 
 * 25-08-08 AW extracted the SetNumObject() 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::UpdateProfile(uint32_t newPAcc, uint32_t newPSpeed, uint32_t newPDec)
{
	CODriveCommStates returnValue = eCO_DriveBusy;
	
	if(AccessStep == 0)
	{
		returnValue = SetNumObject(&OdProfileSpeed, newPSpeed);
		if(returnValue == eCO_DriveDone)
		{
			AccessStep = 1;
			#if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			Serial.println("Drive: ProfileSpeed updated");
			#endif
			
			returnValue = eCO_DriveBusy;
		}
	}
	else if(AccessStep == 1)
	{
		returnValue = SetNumObject(&OdProfileAcc, newPAcc);
		if(returnValue == eCO_DriveDone)
		{
			AccessStep = 2;
			#if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			Serial.println("Drive: ProfileAcc updated");
      #endif
			
			returnValue = eCO_DriveBusy;
		}
	}
	else if(AccessStep == 2)
	{
		returnValue = SetNumObject(&OdProfileDec, newPDec);
		if(returnValue == eCO_DriveDone)
		{
			#if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			Serial.println("Drive: ProfileDec updated");
			#endif
			
			AccessStep = 0;
		}
	}
	else
	  returnValue = eCO_DriveError;	
	
	return returnValue;
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::StartMoveAbs(bool isImmediate)
 * 
 * start a profile based abs move
 * 
 * 25-08-01 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::StartMoveAbs(bool isImmediate)
{
	return MovePP(false, isImmediate);
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::StartMoveRel(bool isImmediate)
 * 
 * start a profile based rel move
 * 
 * 25-08-01 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::StartMoveRel(bool isImmediate)
{
	return MovePP(true, isImmediate);
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::SetTargetPos(int32_t TPos)
 * 
 * update the targetPos either by PDO or by SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::SetTargetPos(int32_t TPos)
{
	return SetNumObject(&OdTargetPos,(uint32_t)TPos);
}

/*-------------------------------------------------------------------
 * int32_t CO402Drive::GetActPos()
 * 
 * return the last received ActPos
 * requires the ActPos to be updated by PDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

int32_t CO402Drive::GetActPos()
{
	return ActPos;
}

/*-------------------------------------------------------------------
 * bool CO402Drive::isInPos()
 * 
 * check whether the TargetReached flag is set in SW
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

bool CO402Drive::isInPos()
{
	return (SWValue & TSWTargetReachedMask);
}
	  
/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::SetTargetSpeed(int32_t TSpeed)
 * 
 * update the targetSpeed either by PDO or by SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::SetTargetSpeed(int32_t TSpeed)
{
	return SetNumObject(&OdTargetSpeed,(uint32_t)TSpeed);
}

/*-------------------------------------------------------------------
 * int32_t CO402Drive::GetActSpeed()
 * 
 * return the last received ActSpeed
 * requires the ActSpeed to be updated by PDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

int32_t CO402Drive::GetActSpeed()
{
	return ActSpeed;
}

/*-------------------------------------------------------------------
 * bool CO402Drive::isSpeedReached()
 * 
 * check whether the target speed is reached by checking the flag in SW
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

bool CO402Drive::isSpeedReached()
{
	return (SWValue & TSWIsSpeedReachedMask);
}
	
/*-------------------------------------------------------------------
 * bool CO402Drive::isSpeed0()
 * 
 * check whether the target speed is 0 by checking the flag in SW
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

bool CO402Drive::isSpeed0()
{
	return (SWValue & TSWIsSpeed0Mask);
}
	
/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::SetTargetTorque(int16_t TTorque)
 * 
 * update the TargetTorque either by PDO or by SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::SetTargetTorque(int16_t TTorque)
{
	return SetNumObject(&OdTargetTorque,(uint16_t)TTorque);
}

/*-------------------------------------------------------------------
 * int16_t CO402Drive::GetActTorque()
 * 
 * return the last received ActTorque
 * requires the ActTorque to be updated by PDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

int16_t CO402Drive::GetActTorque()
{
	return ActTorque;
}
		
/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::SetHomingMethod(int8_t Method)
 * 
 * configure the HomingMethod of this drive
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::SetHomingMethod(int8_t Method)
{	
	return SetNumObject(&OdHomingMethod,(uint8_t)Method);
}

/*-------------------------------------------------------------------
 * CODriveCommStates  CO402Drive::DoHoming()
 * 
 * execute the configured homing
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::DoHoming()
{
	CODriveCommStates returnValue = eCO_DriveBusy;
	uint16_t SWCommandBits = SWValue & 0x007F;
	uint16_t newCWValue = CWValue;

	switch(AccessStep)
	{
		case 0:
      //reset start bit	
			newCWValue &= ~TCWStartBit;
	    CheckCWForTx(newCWValue);

		  AccessStep++;
			break;
		case 1:
	    //set Op-Mode Homing
		  if(SetOpMode(OpModeHoming) == eCO_DriveDone)
				AccessStep++;
			
			break;
		case 2:
			//set start bit
		  newCWValue = CWValue  | TCWStartBit;
			CheckCWForTx(newCWValue);

		  AccessStep++;
			break;
		case 3:
			if(isHomingFinished())
			{
				AccessStep = 4;
			}
			if((SWValue & (TSWIsHomingError)) == TSWIsHomingError)
			{
				returnValue = eCO_DriveError;
			}			
			break;
		case 4:
      //reset start bit	
			newCWValue &= ~TCWStartBit;
	    CheckCWForTx(newCWValue);
			
		  returnValue = eCO_DriveDone;

		  AccessStep = 0;
			break;
		default:
	    returnValue = eCO_DriveError;	
		  break;
	}	
	return returnValue;
}

/*-------------------------------------------------------------------
 * bool CO402Drive::isHomingFinished()
 * 
 * check whether the homing has been successful
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

bool CO402Drive::isHomingFinished()
{
	bool returnValue = false;
	
	if((SWValue & (TSWIsHomingSpeed0Mask | TSWIsHomingDone)) == (TSWIsHomingSpeed0Mask | TSWIsHomingDone))
	{
		returnValue = true;
	}
	return returnValue;
}
	
/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::SetTargetPos(int32_t TPos)
 * 
 * update the targetPos either by PDO or by SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::SetOpMode(int8_t TOpMode)
{
	return SetNumObject(&OdModesOfOp, TOpMode);
}

/*-------------------------------------------------------------------
 * int16_t CO402Drive::GetActTorque()
 * 
 * return the last received ActTorque
 * requires the ActTorque to be updated by PDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

int8_t CO402Drive::GetOpMode()
{
	return ModesOfOpDispValue;
}

/*-------------------------------------------------------------------
* CODriveCommStates CO402Drive::GetErrorWord(uint16_t *value)
 * 
 * read the latest error word 0x2320.00
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::GetErrorWord(uint16_t *value)
{
	CODriveCommStates returnValue = GetNumObject(&OdDriveError);
	
	if(returnValue == eCO_DriveDone)
    *value = *(OdDriveError.Value);
	
	return returnValue;	
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::GetDigInStatus(uint8_t *value)
 * 
 * read the latest DigIn status @ 0x2311
 * 
 * 25-09-14 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::GetDigInStatus(uint8_t *value)
{
	CODriveCommStates returnValue = GetNumObject(&OdDigitalInStatus);
  
	if(returnValue == eCO_DriveDone)
	  *value = *(OdDigitalInStatus.Value);
	
	return returnValue;	
}

    
/*-------------------------------------------------------------------
 * uint16_t CO402Drive::GetStatusWord()
 * 
 * return the last SW
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

uint16_t CO402Drive::GetStatusWord()
{
	return SWValue;
}

/*-------------------------------------------------------------------
 * bool CO402Drive::isWarningSet()
 * 
 * check in the lates received SW whether the drive flagged a warning
 * 
 * 25-09-11 AW frame 
 *-------------------------------------------------------------------*/

bool CO402Drive::isWarningSet()
{
	return (SWValue & TSWWarningMask);
}

/*-------------------------------------------------------------------
 * bool CO402Drive::isErrorActive()
 * 
 * check in the lates received SW whether the drive is flagged to
 * have the FAULHABER error bit set
 * 
 * 25-09-11 AW frame 
 *-------------------------------------------------------------------*/

bool CO402Drive::isErrorActive()
{
	return (SWValue & TSWErrorMask);
}

/*-------------------------------------------------------------------
 * bool CO402Drive::isLimited()
 * 
 * check in the lates received SW whether the drive is flagged to be limited
 * 
 * 25-09-11 AW frame 
 *-------------------------------------------------------------------*/

bool CO402Drive::isLimited()
{
	return (SWValue & TSWLimitActiveMask);
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::IdentifyDrive()
 * 
 * set a list of identification objects and request them to be uploaded
 * 
 * 25-09-11 AW frame
 * 25-09-12 AW implemented and tested
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::IdentifyDrive()
{
	CODriveCommStates returnValue = eCO_DriveBusy;
	
  if(Node.RWSDO.ReadObjects(IdentityEntries, 4) == eCO_SDODone)
		returnValue = eCO_DriveDone;

	return returnValue;
}	
	
/*-------------------------------------------------------------------
 * ODEntry **CO402Drive::GetIdentityEntries()
 * 
 * return a pointer to the array of identiy objects
 * 
 * 25-09-12 AW implemented and tested
 *
 *-------------------------------------------------------------------*/

ODEntry **CO402Drive::GetIdentityEntries()
{
	return IdentityEntries;
}


/*-------------------------------------------------------------------
 * void CO402Drive::PrintIdentityObjects()
 * 
 * directly print the identiy strings to Serial
 * 
 * 25-09-12 AW implemented and tested
 *
 *-------------------------------------------------------------------*/

void CO402Drive::PrintIdentityObjects()
{
	for(uint8_t iter = 0; iter < 4; iter++ )
	{
		switch(iter)
		{
			case 0:
				Serial.print("DeviceName: ");
			  break;
			case 1:
				Serial.print("HwVersion : ");
			  break;
			case 2:
				Serial.print("SwVersion : ");
			  break;
			case 3:
				Serial.print("MotorName : ");
			  break;
      default:
        break;			
		}
		for(uint32_t charIdx = 0; charIdx < IdentityEntries[iter]->len; charIdx++)
		{
			Serial.write(((ODEntryString *)IdentityEntries[iter])->Value[charIdx]);
		}
		Serial.println(";");
	}
}

//----------------------------------------------------------------------------------
//--- private functions ---

/*-------------------------------------------------------------------
 * void CO402Drive::CheckCWForTx(uint16_t newCWValue)
 * 
 * check whether the CW needs to be updated
 * does only work with PDO
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/
		
void CO402Drive::CheckCWForTx(uint16_t newCWValue)
{
	if(resetFault)
		newCWValue = TCWResetFaultMask;
	else
		newCWValue &= ~TCWResetFaultMask;
	
	if(newCWValue != CWValue)
  {
	  #if(DEBUG_DRIVE & DEBUG_DRIVE_CW)
    Serial.print("new CW: ");
    Serial.println(newCWValue, HEX);
    #endif

    //assign the new value first
    //otherwise the Tx might be to fast - really?		
    CWValue  = newCWValue;
    PDOHandler.TxPDOsAsync((ODEntry *)&OdCW);
  }	
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::MovePP(bool isRelMove, bool isImmediate)
 * 
 * start a profile based move by stepping throug the handshakes in SW / CW
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::MovePP(bool isRelMove, bool isImmediate)
{
  CODriveCommStates returnValue = eCO_DriveBusy;
	uint16_t newCWValue = CWValue;
		
	switch(AccessStep)
	{
		case 0:
			//if STA Bit set is SW, reset the start bit in CW
		  if(SWValue & TSWSetPointAckMask)
				newCWValue &= ~TCWStartBit;
			else
				AccessStep++;
			break;
		case 1:
			//now command a new move by a rising edge
		  //done when the STA bit flags the command being received
		  if(SWValue & TSWSetPointAckMask)
				AccessStep++;
			else
			{
				newCWValue = CWValue  | TCWStartBit;
				if(isRelMove)
					newCWValue |= TCWIsRelativeBit;
				if(isImmediate)
					newCWValue |= TCWIsImmediateBit;					
			}
			break;
		case 2:
			//reset the start bit again and wait until the STA bit is reset too
		  if(SWValue & TSWSetPointAckMask)
				newCWValue &= ~(TCWStartBit | TCWIsRelativeBit | TCWIsImmediateBit);
			else
			{
				AccessStep = 0;
				returnValue = eCO_DriveDone;
			}
			break;
	}
	CheckCWForTx(newCWValue);

	return returnValue;
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::SetNumObject(ODEntry08 *Object, uint8_t Value)
 * 
 * update a uint8_t object in the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::SetNumObject(ODEntry08 *Object, uint8_t Value)
{
	CODriveCommStates returnValue = eCO_DriveBusy;

	//do we need to update the value?	
  if(*(Object->Value) != Value)
	{
		//1st: update the local copy
		*(Object->Value) = Value;		
		
		//check for is being mapped to PDO
		if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
		{
			#if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			Serial.print("Drive: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" is mapped");
			#endif
			
			returnValue = eCO_DriveDone;
		}
		else
		{
			//is not mapped
			//fixed lenght of 1 byte
			COSDOCommStates SDOState = Node.RWSDO.WriteSDO((ODEntry *)Object);

			if(SDOState == eCO_SDODone)
			{
			  #if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			  Serial.print("Drive: Idx ");
			  Serial.print(Object->Idx,HEX);
			  Serial.println(" updated via SDO");
				#endif

			  returnValue = eCO_DriveDone;
			}
			else if(SDOState == eCO_SDOError)
			{
				returnValue = eCO_DriveError;
			}
		}
	}
	else
	{
		//no need to update anything
		returnValue = eCO_DriveDone;
	}

	return returnValue;
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::SetNumObject(ODEntry16 *Object, uint16_t Value)
 * 
 * update a uint16_t object in the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::SetNumObject(ODEntry16 *Object, uint16_t Value)
{
	CODriveCommStates returnValue = eCO_DriveBusy;

	//do we need to update the value?	
  if(*(Object->Value) != Value)
	{
		//1st: update the local copy
		*(Object->Value) = Value;		
		
		//check for is being mapped to PDO
		if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
		{
			#if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			Serial.print("Drive: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" is mapped");
			#endif
			
			returnValue = eCO_DriveDone;
		}
		else
		{
			//is not mapped
			//fixed lenght of 2 bytes
			COSDOCommStates SDOState = Node.RWSDO.WriteSDO((ODEntry *)Object);

			if(SDOState == eCO_SDODone)
			{
			  #if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			  Serial.print("Drive: Idx ");
			  Serial.print(Object->Idx,HEX);
			  Serial.println(" updated via SDO");
				#endif

			  returnValue = eCO_DriveDone;
			}
			else if(SDOState == eCO_SDOError)
			{
				returnValue = eCO_DriveError;
			}
		}
	}
	else
	{
		//no need to update anything
		returnValue = eCO_DriveDone;
	}

	return returnValue;
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::SetNumObject(ODEntry32 *Object, uint32_t Value)
 * 
 * update a uint32_t object in the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::SetNumObject(ODEntry32 *Object, uint32_t Value)
{
	CODriveCommStates returnValue = eCO_DriveBusy;

	//do we need to update the value?	
  if(*(Object->Value) != Value)
	{
		//1st: update the local copy
		*(Object->Value) = Value;		
		
		//check for is being mapped to PDO
		if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
		{
			#if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			Serial.print("Drive: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" is mapped");
      #endif
			
			returnValue = eCO_DriveDone;
		}
		else
		{
			//is not mapped
			COSDOCommStates SDOState = Node.RWSDO.WriteSDO((ODEntry *)Object);
			
			if(SDOState == eCO_SDODone)
			{
			  #if(DEBUG_DRIVE & DEBUG_DRIVE_WRITEOBJ)
			  Serial.print("Drive: Idx ");
			  Serial.print(Object->Idx,HEX);
			  Serial.println(" updated via SDO");
				#endif
				
			  returnValue = eCO_DriveDone;
			}
			else if(SDOState == eCO_SDOError)
			{
				returnValue = eCO_DriveError;
			}
		}
	}
	else
	{
		//no need to update anything
		returnValue = eCO_DriveDone;
	}

	return returnValue;
}


/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::GetNumObject(ODEntry08 *Object)
 * 
 * read a uint8_t object from the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-09-13 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::GetNumObject(ODEntry08 *Object)
{
	CODriveCommStates returnValue = eCO_DriveBusy;
	
	//check for is being mapped to PDO
	if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
	{
		#if(DEBUG_DRIVE & DEBUG_DRIVE_READBJ)
		Serial.print("Drive: Idx ");
		Serial.print(Object->Idx,HEX);
		Serial.println(" is mapped");
		#endif
		
		//PDO won't be triggered while this is executed
    //we will get the value received last
		returnValue = eCO_DriveDone;
	}
	else
	{
		//is not mapped
		COSDOCommStates SDOState = Node.RWSDO.ReadSDO((ODEntry *)Object);
		if((SDOState == eCO_SDODone) && (Object->len = 1))
		{
			#if(DEBUG_DRIVE & DEBUG_DRIVE_READOBJ)
			Serial.print("Drive: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" updated via SDO");
			#endif

			returnValue = eCO_DriveDone;
		}
		else if(SDOState == eCO_SDOError)
		{
			returnValue = eCO_DriveError;
		}
	}
	return returnValue;
}

	
/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::GetNumObject(ODEntry16 *Object)
 * 
 * read a uint16_t object from the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-09-13 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::GetNumObject(ODEntry16 *Object)
{
	CODriveCommStates returnValue = eCO_DriveBusy;
	
	//check for is being mapped to PDO
	if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
	{
		#if(DEBUG_DRIVE & DEBUG_DRIVE_READBJ)
		Serial.print("Drive: Idx ");
		Serial.print(Object->Idx,HEX);
		Serial.println(" is mapped");
		#endif
		
		//PDO won't be triggered while this is executed
    //we will get the value received last
		returnValue = eCO_DriveDone;
	}
	else
	{
		//is not mapped
		COSDOCommStates SDOState = Node.RWSDO.ReadSDO((ODEntry *)Object);
		if((SDOState == eCO_SDODone) && (Object->len = 2))
		{
			#if(DEBUG_DRIVE & DEBUG_DRIVE_READOBJ)
			Serial.print("Drive: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" updated via SDO");
			#endif

			returnValue = eCO_DriveDone;
		}
		else if(SDOState == eCO_SDOError)
		{
			returnValue = eCO_DriveError;
		}
	}
	return returnValue;
}

/*-------------------------------------------------------------------
 * CODriveCommStates CO402Drive::GetNumObject(ODEntry32 *Object)
 * 
 * read a uint32_t object from the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-09-13 AW 
 *
 *-------------------------------------------------------------------*/

CODriveCommStates CO402Drive::GetNumObject(ODEntry32 *Object)
{
	CODriveCommStates returnValue = eCO_DriveBusy;
	
	//check for is being mapped to PDO
	if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
	{
		#if(DEBUG_DRIVE & DEBUG_DRIVE_READBJ)
		Serial.print("Drive: Idx ");
		Serial.print(Object->Idx,HEX);
		Serial.println(" is mapped");
		#endif
		
		//PDO won't be triggered while this is executed
    //we will get the value received last
		returnValue = eCO_DriveDone;
	}
	else
	{
		//is not mapped
		COSDOCommStates SDOState = Node.RWSDO.ReadSDO((ODEntry *)Object);
		if((SDOState == eCO_SDODone) && (Object->len = 4))
		{
			#if(DEBUG_DRIVE & DEBUG_DRIVE_READOBJ)
			Serial.print("Drive: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" updated via SDO");
			#endif

			returnValue = eCO_DriveDone;
		}
		else if(SDOState == eCO_SDOError)
		{
			returnValue = eCO_DriveError;
		}
	}
	return returnValue;
}
		
