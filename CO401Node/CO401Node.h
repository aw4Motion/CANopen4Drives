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

#ifndef CO_IONode_H
#define CO_IONode_H

/*--------------------------------------------------------------------
 * a class covering the CiA401 I/O Nodes
 *
 * 2025-10-28 AW Frame
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---

#include <COObjects.h>
#include <CONode.h>
#include <COPDOHandler.h>
#include <COSyncHandler.h>

#include <stdint.h>

//--- local definitions ---

const uint8_t NumDigInObjects = 1;
const uint8_t NumDigOutObjects = 1;
const uint8_t NumAnIn08Objects = 1;
const uint8_t NumAnIn16Objects = 1;
const uint8_t NumAnOut08Objects = 1;
const uint8_t NumAnOut16Objects = 1;

const uint8_t NumNodeIdentityObjects = 3;
const uint8_t NodeOdStringLen = 32;

typedef enum COIONodeCommStates {
	eCO_IOIdle,
	eCO_IOWaiting,
	eCO_IORetry,
	eCO_IOBusy,
	eCO_IODone,
	eCO_IOError
} COIONodeCommStates;


class CO401Node {
	public:
		CO401Node(uint8_t);           //the NodeId for this node
		void init(COMsgHandler *);  //the MsgHandler of course
	                             
    CONode Node;
	  COPDOHandler PDOHandler;
	
	  uint8_t GetNodeId();
	  void ResetComState();
				
		COIONodeCommStates InitNode(uint32_t);
		COIONodeCommStates InitPDOs(uint32_t);
		
	  NMTNodeState Update(uint32_t, COSyncState); //parameters are actTime and SyncState
		
		COIONodeCommStates IdentifyIONode();
		ODEntry **GetIdentityEntries();
		void PrintIdentityObjects();
	
   	COIONodeCommStates SetDigOut(uint8_t);
	  COIONodeCommStates GetDigInStatus(uint8_t *);

	  COIONodeCommStates SetRemoteAnOut(uint8_t, int16_t);
	  COIONodeCommStates GetRemoteAnIn(uint8_t, int16_t *);

    COIONodeCommStates SetNumObject(ODEntry08 *, uint8_t);
	  COIONodeCommStates SetNumObject(ODEntry16 *, uint16_t);
	  COIONodeCommStates SetNumObject(ODEntry32 *, uint32_t);
		
	  COIONodeCommStates GetNumObject(ODEntry08 *);
	  COIONodeCommStates GetNumObject(ODEntry16 *);
	  COIONodeCommStates GetNumObject(ODEntry32 *);
						
		bool autoEnable = false;
	  bool isPDOsConfigured = false;
		bool reConfigPDOs = true;
		
    //--- the actual default OD etnries -------------------------
    ODEntryString OdDevice = {0x1008,0x00,DeviceName,32};
    ODEntryString OdHwVersion = {0x1009,0x00,HwVersion,32};
    ODEntryString OdSwVersion = {0x100a,0x00,SwVersion,32};
		
		ODEntry *IdentityEntries[NumNodeIdentityObjects] = {(ODEntry *)&OdDevice, 
		                               (ODEntry *)&OdHwVersion, 
		                               (ODEntry *)&OdSwVersion};

		ODEntry08 OdDigInStatus = {0x6000, 0x01, &(DigInStatus[0]), 1};
		ODEntry08 OdDigOutStatus = {0x6200, 0x01, &(DigOutStatus[0]), 1};

		ODEntry16 OdAnInStatus = {0x6401, 0x01, (uint16_t *)&(AnInStatus16[0]), 2};
		ODEntry16 OdAnOutStatus = {0x6411, 0x01, (uint16_t *)&(AnOutStatus16[0]), 2};
		
	private:
		uint8_t nodeId;
		COMsgHandler *MsgHandler;

	  uint8_t AccessStep = 0;
	
	  bool resetFault = false;
	
	  char DeviceName[32] = "";
	  char HwVersion[32] = "";
	  char SwVersion[32] = "";
	
		uint8_t DigInStatus[NumDigInObjects];
		uint8_t DigOutStatus[NumDigOutObjects];
		int8_t AnInStatus08[NumAnIn08Objects];
		int16_t AnInStatus16[NumAnIn16Objects];
		int8_t AnOutStatus08[NumAnOut08Objects];
		int16_t AnOutStatus16[NumAnOut16Objects];
		
		PDOMapping MapTxPDO1 = {1,{(ODEntry *)&(OdDigInStatus), NULL, NULL, NULL}}; 
    PDOMapping MapRxPDO1 = {1,{(ODEntry *)&(OdDigOutStatus), NULL, NULL, NULL}};

    PDOMapping MapTxPDO2 = {1,{(ODEntry *)&(OdAnInStatus), NULL, NULL, NULL}};
    PDOMapping MapRxPDO2 = {1,{(ODEntry *)&(OdAnOutStatus), NULL, NULL, NULL}};

};
 
#endif
