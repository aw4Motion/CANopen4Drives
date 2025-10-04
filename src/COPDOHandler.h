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
 
#ifndef COPDOHandler_H
#define COPDOHandler_H

/*--------------------------------------------------------------
 * class PDOHandler
 * implements the CiA 301 PDO services
 *
 * 2025-03-19 AW Frame
 *
 *-------------------------------------------------------------*/
 
//--- inlcudes ----

#include <COObjects.h>
#include <COMsgHandler.h>
#include <CONode.h>
#include <COSyncHandler.h>

#include <stdint.h>

// the node needs the typical states too

//define node states which should relate to the

const uint8_t MaxPDOMappingEntries = 8;
const uint8_t NrPDOs = 4;

typedef enum COPDOCommStates {
	eCO_PDOIdle,
	eCO_PDOWaiting,
	eCO_PDORetry,
	eCO_PDOBusy,
	eCO_PDODone,
	eCO_PDOError
} COPDOCommStates;
	 
typedef struct PDOMapping {
	uint8_t NrEntries;
	ODEntry *Entries[MaxPDOMappingEntries];
} PDOMapping;

typedef struct PDOTransmType {
	uint16_t COBId;        //subIdx 01
	bool isValid;
	uint8_t pending;       //to be incremented, if this PDO shall be sent (Rx only) // decremented if sent
	uint32_t sentAt;
	uint8_t TransmType;    //subIdx 02
	bool hasInhibitTime;
	uint16_t inhibitTime;  //subIdx 03
	bool hasEventTimer;
	uint16_t eventTimer;   //subIdx 05
} PDOTransmType;

typedef enum PDODir {
	eCO_PDORx,
	eCO_PDOTx
} PDODir;

	
class COPDOHandler {
	public:
		COPDOHandler();
	  void init(COMsgHandler *, CONode *, int8_t, int8_t);

  	void RegisterSDOHandler(COSDOHandler *);

	  void PresetRxPDOTransmission(uint8_t, uint8_t);  //paramters are the PDO# and the transmission type
	  void PresetTxPDOTransmission(uint8_t, uint8_t, uint16_t = 0, uint16_t = 0);  //paramters are the PDO#, the transmission type, the inhibit time and the EvtTimer
	
	  void PresetRxPDOMapping(uint8_t, uint8_t, ODEntry **);
	  void PresetTxPDOMapping(uint8_t, uint8_t, ODEntry **);
	
	  void PresetRxPDOisValid(uint8_t,bool);
	  void PresetTxPDOisValid(uint8_t,bool);
	
	  COPDOCommStates ConfigurePresetPDOs(uint32_t);
	  void FlagPDOsInvalid();
    
    COPDOCommStates ConfigureRxTxPDO(uint8_t, PDODir, uint32_t);
    COPDOCommStates Update(uint32_t, COSyncState);
	
	  //todo?
	  COPDOCommStates ModifyRxPDOTransmission(uint8_t, uint8_t);  //paramters are the PDO# and the transmission type
	  COPDOCommStates ModifyTxPDOTransmission(uint8_t, uint8_t, uint16_t);  //paramters are the PDO#, the transmission type and an inhibit time
		
	  //todo?
	  COPDOCommStates ModifyRxPDOMapping(uint8_t, uint8_t, ODEntry **);
	  COPDOCommStates ModifyTxPDOMapping(uint8_t, uint8_t, ODEntry **);
	
	  bool TxPDOsAsync(ODEntry *);
	
		void ResetComState(); 
		void ResetSDOState();
		
		COSDOCommStates  GetSDOComState();
		
		void SetTORetryMax(uint8_t);
		void SetBusyRetryMax(uint8_t);

		static void OnPdoMsgRxCb(void *op,void *p) {
			((COPDOHandler *)op)->OnRxHandler((CANMsg *)p);
		};
					
	private:
		void OnRxHandler(CANMsg *);
    void OnTimeOut();
		
	  bool TransmitPdo(uint8_t);
   	bool SendRequest(CANMsg *);
	
		uint32_t RequestSentAt;
		uint32_t actTime;
	  bool isTimerActive = false;
	
		uint8_t TORetryCounter = 0;
		uint8_t TORetryMax = 1;

		uint8_t BusyRetryCounter = 0;
		uint8_t BusyRetryMax = 1;
	
	  uint8_t nextTx = 0;
	
	  COPDOCommStates ConfigureRxTxPDO(uint16_t, PDOTransmType *, PDOMapping *, uint32_t);

	  COPDOCommStates SetRxPDOInvalid(uint8_t);
	  COPDOCommStates SetTxPDOInvalid(uint8_t);

  	COPDOCommStates SetRxPDOValid(uint8_t);
	  COPDOCommStates SetTxPDOValid(uint8_t);
	
	  COPDOCommStates WriteRxPDOMapping(uint8_t);
	  COPDOCommStates WriteTxPDOMapping(uint8_t);
		
		COPDOCommStates WriteObject(uint16_t, uint8_t, uint32_t *, uint32_t);
			
		uint8_t PDOConfigSequenceAccessStep = 0;
		uint8_t PDOConfigSingleStepAccessStep = 0;
		uint8_t PDOsConfigured = 0;
		
		uint8_t Channel = InvalidSlot;
		int8_t nodeId = invalidNodeId;
	
	  PDOTransmType RxPDOSettings[NrPDOs];
    PDOMapping RxPDOMapping[NrPDOs];
		uint8_t RxPDOLength[NrPDOs];
		
	  PDOTransmType TxPDOSettings[NrPDOs];
	  PDOMapping TxPDOMapping[NrPDOs];
		uint8_t TxPDOLength[NrPDOs];
	
		COMsgHandler *Handler;
		CONode *Node;
		
		COSDOHandler *RWSDO;
		COSDOCommStates SDORxTxState = eCO_SDOUnknown;
    	
	  CANMsg TxPDO;  //this is the generic PDO to be sent
		
		COPDOCommStates RequestState = eCO_PDOIdle;
};

#endif

