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

#ifndef CONODE_H
#define CONODE_H

/*--------------------------------------------------------------
 * class CONode
 * implements the node specific NMT services of a CANopen device
 *
 * 2025-01-11 AW Frame
 *
 *-------------------------------------------------------------*/
 
//--- inlcudes ----

#include <COMsgHandler.h>
#include <COSDOHandler.h>
#include <COObjects.h>
#include <stdint.h>

// the node needs the typical states too

//define node states which should relate to the
//expected states of the remote CANopen stack
typedef enum NMTNodeState {
  eNMTStateOffline = -128,
	eNMTWaitForBoot = -3,
	eNMTBootMsgReceived = -2,
	eNMTBooting = -1,
  eNMTStateReset = 0,
  eNMTStatePreOp = 127,
  eNMTStateOperational = 5,
  eNMTStateStopped = 4
}	NMTNodeState;

typedef enum CONodeCommStates {
	eCO_NodeIdle,
	eCO_NodeWaiting,
	eCO_NodeRetry,
	eCO_NodeBusy,
	eCO_NodeDone,
	eCO_NodeError,
	eCO_NodeGuardingFailed
} CONodeCommStates;

typedef enum COGuardingState {
	eCO_GuardingOff,
	eCO_GuardingConfigured,
	eCO_GuardingExpected,
	eCO_GuardingWaiting,
	eCO_GuardingReceivedIntime,
	eCO_GuardingTimeOut,
	eCO_GuardingError
} COGuardingState;

const uint8_t NMTCommandFrameLength = 2;
const uint8_t NMTGuardingFrameLength = 1;

typedef struct NMTMsg {
   uint32_t Id;
	 uint8_t len;
	 bool isRTR;
	 COService serviceType;
	 uint8_t command;
	 uint8_t nodeId;
   } NMTMsg;

	
class CONode {
	public:
		CONode();
	  void init(COMsgHandler *, int8_t, int8_t);  //the MsgHandler, the NodeId and the MsgHandle of this Node 

		void SetNodeId(uint8_t);
	  int16_t GetNodeId();
	
    NMTNodeState InitRemoteNode(uint32_t);  //confire the remote node via SDO	
	  NMTNodeState Update(uint32_t);          //set the time and trigger Guarding / node identification
		
		void ResetComState(); 
	  void RestartNode();
		void ResetSDOState();
	
		void SetTORetryMax(uint8_t);
		void SetBusyRetryMax(uint8_t);
	 
	  CONodeCommStates ConfigureGuarding(uint16_t, uint8_t);
	  CONodeCommStates ConfigureRemoteHeartbeatProducer(uint16_t);
	  CONodeCommStates ConfigureRemoteHeartbeatConsumer(uint8_t, uint16_t);
	
	  void PresetHBMissedTime(uint16_t);
	  void Register_OnNodeStateChangeCb(pfunction_holder *);

		//COSDOCommStates ReadSDO(uint16_t, uint8_t, void *, uint32_t *);
		//COSDOCommStates WriteSDO(uint16_t, uint8_t, void *,uint32_t);

		CONodeCommStates SendResetNode();
		CONodeCommStates SendResetCom();
		CONodeCommStates SendStartNode();
		CONodeCommStates SendStopNode();
		CONodeCommStates SendPreopNode();
		
		void forceNodeState(NMTNodeState);

	  COSDOHandler RWSDO;
		COSDOCommStates GetSDOState();

		bool IsLive();

		static void OnSysMsgRxCb(void *op,void *p) {
			((CONode *)op)->OnRxHandler((CANMsg *)p);
		};
			
		static void OnEmcyMsgRxCb(void *op,void *p) {
			((CONode *)op)->EmcyHandler((CANMsg *)p);
		};

		uint16_t EmcyCode = 0;
	  uint16_t FAULHABERErrorWord = 0;
	  uint8_t CiA301ErrorWord = 0;

	private:
		void OnRxHandler(CANMsg *);
	  void EmcyHandler(CANMsg *);
	
    void OnTimeOut();
	  CONodeCommStates SendRequest(CANMsg *);
	
	  CONodeCommStates ActivateGuarding();
	  CONodeCommStates ActivateHeartbeat();
	
	  void PrintEMCY();
		
		uint8_t ConfigStep = 0;
		
		uint8_t Channel = InvalidSlot;
		int16_t NodeId = invalidNodeId;
					
		COMsgHandler *Handler;
	  pfunction_holder OnNodeStateChangeCb;
	
	  CANMsg GuardingRequest;  //this is the guarding request - a remote frame

    //some Objects used here
    ODEntry16 ODGuardTime = {0x100C, 0x00, NULL, 2};
    ODEntry08 ODLiveTimeFactor = {0x100D, 0x00, NULL, 1};
    ODEntry16 ODProducerHeartbeatTime = {0x1017, 0x00, NULL, 2};
    ODEntry32 ODConsumerHeartbeatTime = {0x1016, 0x01, NULL, 4};
	
	  uint16_t GuardTime = 0;
	  uint8_t LiveTimeFactor = 0;
	  uint16_t HeartbeatProducerTime = 0;
		uint32_t HeartbeatConsumerTime = 0;
		
		uint32_t RemoteHBMissedTime = 0;
		
    bool isGuardingActive = false;
		uint32_t GuardRequestSentAt;
		uint8_t NumGuardRequestsOpen = 0;
		uint8_t expectedToggleBit = 0;
		
	  bool isHeartbeatActive = false;
		uint32_t HeatbeatReceivedAt;
		
	  COGuardingState GuardingState = eCO_GuardingOff;
		
	  bool isTimerActive = false;
	
	  NMTMsg NmtCommand;
	
		//CONodeCommStates NodeRxTxState = eCO_NodeIdle;
		CONodeCommStates RequestState  = eCO_NodeIdle;
		CONodeCommStates ConfigState   = eCO_NodeIdle;

		uint32_t actTime;

		uint8_t TORetryCounter = 0;
		uint8_t TORetryMax = 1;

		uint8_t BusyRetryCounter = 0;
		uint8_t BusyRetryMax = 1;

	  NMTNodeState NodeState = eNMTStateOffline;
		NMTNodeState ReportedState = eNMTStateReset;
		bool isLive = false;
		
		uint32_t RequestTime = 0;
		
		ODEntry ODRemoteNodeType = {0x1000, 0x00, NULL, 4};
		uint32_t RemoteNodeTypeValue;
		uint8_t RemoteNodeTypeDataLength = 4;
};
 

#endif
