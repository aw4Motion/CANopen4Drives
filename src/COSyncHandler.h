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
 
#ifndef CO_SYNC_HANDLER_H
#define CO_SYNC_HANDLER_H

/*--------------------------------------------------------------------
 * interface for class COSynchandler
 * will send the SYNC periodically and
 * produce a global HB message if configured
 *
 * 2025-03-09 AW Frame
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---
 
#include <COMsgHandler.h>
#include <stdint.h>

//--- local definitions ---

typedef enum SyncMasterState {
  eSyncStateOffline = -1,
  eSyncStatePreOp = 127,
  eSyncStateOperational = 5,
  eSyncStateStopped = 4
}	SyncMasterState;

typedef enum COSyncCommStates {
	eCO_SyncIdle,
	eCO_SyncBusy,
	eCO_SyncRetry,
	eCO_SyncError
} COSyncCommStates;

typedef enum COSyncState {
	eSyncIdle,
	eSyncSyncSent
} COSyncState;


class COSyncHandler {
	public:
		COSyncHandler(uint8_t);     //the NodeId to be used for HB producer
		void init(COMsgHandler *);  //the MsgHandler of course
	                              //prepares the guarding and the Sync Msg

	  COSyncState Update(uint32_t); //generate the HB and the Sync depending on time and state
	
	  void SetState(SyncMasterState); //force the com op-mode to init / Pre-op / op
	
	  COSyncCommStates SendResetNodes();
	  COSyncCommStates SendStartNodes();
	
	  uint16_t ProducerHBTime = 0;
	  uint16_t SyncInterval = 100;
		
	private:
	  bool SendRequest(CANMsg *);
    
	  uint8_t HBProducerId = 127;  //used for HB message
	
		CANMsg HBMessage;    //well the prepared HB message
		CANMsg SyncMessage;  //the prepared Sync Msg
		NMTMsg NmtCommand;
		
		COMsgHandler *Handler;
	
	  COSyncCommStates SyncTxState = eCO_SyncIdle;
		
	  uint32_t lastSync = 0;
	  uint32_t lastHB = 0;
	
	  SyncMasterState SyncState = eSyncStateOffline;
	
		uint8_t BusyRetryCounter = 0;
		uint8_t BusyRetryMax = 1;

};
 


#endif
