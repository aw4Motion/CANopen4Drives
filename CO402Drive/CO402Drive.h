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

#ifndef CO_DRIVE_H
#define CO_DRIVE_H

/*--------------------------------------------------------------------
 * interface for class COSynchandler
 * will send the SYNC periodically and
 * produce a global HB message if configured
 *
 * 2025-03-09 AW Frame
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---

#include <COObjects.h>
#include <CONode.h>
#include <COPDOHandler.h>
#include <COSyncHandler.h>

#include <stdint.h>

//--- local definitions ---

const int8_t OpModePP = 1;
const int8_t OpModePV = 3;
const int8_t OpModeHoming = 6;

const uint8_t NumDriveIdentityObjects = 4;
const uint8_t DriveOdStringLen = 32;

typedef enum CODriveCommStates {
	eCO_DriveIdle,
	eCO_DriveWaiting,
	eCO_DriveRetry,
	eCO_DriveBusy,
	eCO_DriveDone,
	eCO_DriveError
} CODriveCommStates;


class CO402Drive {
	public:
		CO402Drive(uint8_t);           //the NodeId for this node
		void init(COMsgHandler *);  //the MsgHandler of course
	                             
    CONode Node;
	  COPDOHandler PDOHandler;
	
	  uint8_t GetNodeId();
	
	  void ResetComState();
		
	  CODriveCommStates Enable();
	  CODriveCommStates Disable();
		CODriveCommStates Stop();
		CODriveCommStates ResetError();
	  CODriveCommStates DisableVoltage();
	
	  uint16_t GetStatusWord();
	  
		CODriveCommStates UpdateProfile(uint32_t, uint32_t, uint32_t);

    CODriveCommStates StartMoveAbs(bool);
	  CODriveCommStates StartMoveRel(bool);
	  
		CODriveCommStates SetTargetPos(int32_t);
	  int32_t GetActPos();
	  bool isInPos();
	  
	  CODriveCommStates SetTargetSpeed(int32_t);
	  int32_t GetActSpeed();
	  bool isSpeedReached();
		bool isSpeed0();
	
	  CODriveCommStates SetTargetTorque(int16_t);
	  int16_t GetActTorque();
		
		CODriveCommStates DoHoming();
		CODriveCommStates DoHoming(int8_t);	
		CODriveCommStates SetHomingMethod(int8_t);
		
		bool isHomingFinished();
	
	  CODriveCommStates SetOpMode(int8_t);
	  int8_t GetOpMode();

    CODriveCommStates GetErrorWord(uint16_t *);
    CODriveCommStates GetDigInStatus(uint8_t *);
		
		bool isWarningSet();
		bool isErrorActive();
		bool isLimited();
		
		CODriveCommStates InitNode(uint32_t);
		CODriveCommStates InitPDOs(uint32_t);
		
	  NMTNodeState Update(uint32_t, COSyncState); //parameters are actTime and SyncState
		
		CODriveCommStates IdentifyDrive();
		ODEntry **GetIdentityEntries();
		void PrintIdentityObjects();

	  CODriveCommStates SetNumObject(ODEntry08 *, uint8_t);
	  CODriveCommStates SetNumObject(ODEntry16 *, uint16_t);
	  CODriveCommStates SetNumObject(ODEntry32 *, uint32_t);
		
	  CODriveCommStates GetNumObject(ODEntry08 *);
	  CODriveCommStates GetNumObject(ODEntry16 *);
	  CODriveCommStates GetNumObject(ODEntry32 *);
				
		bool autoEnable = false;
	  bool isPDOsConfigured = false;
		bool reConfigPDOs = true;
		bool autoResetErrors = true;
		
    //--- the actual default OD etnries -------------------------
    ODEntryString OdDevice = {0x1008,0x00,DeviceName,32};
    ODEntryString OdHwVersion = {0x1009,0x00,HwVersion,32};
    ODEntryString OdSwVersion = {0x100a,0x00,SwVersion,32};
    ODEntryString OdMotor = {0x6403,0x00,MotorName,32};
		
		ODEntry *IdentityEntries[NumDriveIdentityObjects] = {(ODEntry *)&OdDevice, 
		                               (ODEntry *)&OdHwVersion, 
		                               (ODEntry *)&OdSwVersion, 
		                               (ODEntry *)&OdMotor};

    ODEntry08 OdModesOfOpDisp = {0x6061,0x00, &ModesOfOpDispValue, 1};
    ODEntry08 OdModesOfOp = {0x6060,0x00,&ModesOfOpTarget,1};

    ODEntry16 OdCW = {0x6040,0x00,&CWValue,2};
    ODEntry16 OdSW = {0x6041,0x00,&SWValue,2};

    ODEntry16 OdErrorWord = {0x6041,0x00,&ErrorWord,2};

    ODEntry32 OdTargetPos = {0x607A,0x00,(uint32_t *)&TargetPos,4};
    ODEntry32 OdActPos = {0x6064,0x00,(uint32_t *)&ActPos,4};

    ODEntry32 OdTargetSpeed = {0x60FF,0x00,(uint32_t *)&TargetSpeed,4};
    ODEntry32 OdActSpeed = {0x606C,0x00,(uint32_t *)&ActSpeed,4};

    ODEntry16 OdTargetTorque = {0x6071,0x00,(uint16_t *)&TargetTorque,2};
    ODEntry16 OdActTorque = {0x6077,0x00,(uint16_t *)&ActTorque,2};
		
		ODEntry08 OdHomingMethod = {0x6098,0x00,(uint8_t *)&DriveHomingMethod,1};

    ODEntry32 OdProfileSpeed = {0x6081,0x00,&ProfileSpeed,4};
    ODEntry32 OdProfileAcc = {0x6083,0x00,&ProfileAcc,4};
    ODEntry32 OdProfileDec = {0x6084,0x00,&ProfileDec,4};
		
		ODEntry16 OdDriveError = {0x2320, 0x00, &ErrorWord, 2};

		ODEntry08 OdDigitalInStatus = {0x2311, 0x01, &DigInStatus, 1};
		
	private:
		uint8_t nodeId;
		COMsgHandler *MsgHandler;

	  CODriveCommStates MovePP(bool, bool);
	  uint8_t AccessStep = 0;
	
	  bool resetFault = false;
	
	  void CheckCWForTx(uint16_t);  //check the CW for a required update
	  CODriveCommStates StartMove();
    
	  char DeviceName[32] = "";
	  char HwVersion[32] = "";
	  char SwVersion[32] = "";
    char MotorName[32] = "2250BX4 3692";
	
	  uint8_t ModesOfOpDispValue = 0;
    uint8_t ModesOfOpTarget = 1;
    
	  uint16_t CWValue = 0;	
    uint16_t SWValue = 0;

    int32_t TargetPos = 0;
    int32_t ActPos;
    
	  int32_t TargetSpeed = 0;
    int32_t ActSpeed;
    
	  int16_t TargetTorque = 0;
	  int16_t ActTorque;

    uint32_t ProfileSpeed = 500;
    uint32_t ProfileAcc = 6000;
    uint32_t ProfileDec = 2000;
		
		int8_t DriveHomingMethod = 0;
		
		uint16_t ErrorWord;
		uint8_t DigInStatus;
		
		PDOMapping MapTxPDO1 = {3,{(ODEntry *)&OdActPos, (ODEntry *)&OdSW, (ODEntry *)&OdModesOfOpDisp,NULL}};
    PDOMapping MapRxPDO1 = {3,{(ODEntry *)&OdTargetPos, (ODEntry *)&OdCW, (ODEntry *)&OdModesOfOp,NULL}};

    PDOMapping MapTxPDO2 = {2,{(ODEntry *)&OdActSpeed, (ODEntry *)&OdActTorque, NULL, NULL}};
    PDOMapping MapRxPDO2 = {1,{(ODEntry *)&OdTargetSpeed, NULL, NULL, NULL}};

};
 


#endif
