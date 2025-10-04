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
 * COSyncHandler.cpp
 * implements the class to produce Sync and global HB
 *
 * 2025-03-09 AW Frame
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <stdint.h>
#include <CONode.h>
#include <COSyncHandler.h>

//--- local defines ---

#define DEBUG_SYNC_TO			      0x0001
#define DEBUG_SYNC_ERROR		    0x0002
#define DEBUG_SYNC_TXMsg	      0x0004
#define DEBUG_SYNC_ConfigGuard	0x0008
#define DEBUG_SYNC_Init         0x0010
#define DEBUG_SYNC_StateChange  0x0100

#define DEBUG_SYNC (DEBUG_SYNC_TO | DEBUG_SYNC_ERROR | DEBUG_SYNC_ConfigGuard | DEBUG_SYNC_Init) 

//--- local definitions ---------

//  definitions for the NMT service
//  commands are defined as const uint8_t which simplified them being 
//  accepted in assignments

const uint8_t NMT_StartRemoteNode = 0x01;
const uint8_t NMT_StopRemoteNode = 0x02;
const uint8_t NMT_EnterPreop = 0x80;
const uint8_t NMT_ResetRemoteNode = 0x81;
const uint8_t NMT_ResetComRemoteNode = 0x82;



//--- public functions ---

/*---------------------------------------------------------------------
 * COSyncHandler::COSynchandler()
 * as of now there is noting to intialized when created
 * 
 * 2025-03-09 AW Done
 * ------------------------------------------------------------------*/

COSyncHandler::COSyncHandler(uint8_t thisId)
{
  uint32_t lastSync = 0;
	uint32_t lastHB = 0;
	
	HBProducerId = thisId;
	
	SyncMasterState SyncState = eSyncStateOffline;
}


/*-------------------------------------------------------------------
 * COSyncHandler::init(COMsgHandler *MsgMandler;
 * 
 * store the instance to the MsgHandler
 * no need to be regsitered as we don't receive messages
 *
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

void COSyncHandler::init(COMsgHandler *ThisHandler)
{
	Handler = ThisHandler;
	
	HBMessage.Id = eCANGuarding | HBProducerId;
	HBMessage.len = 1;
	HBMessage.isRTR = false;
  HBMessage.serviceType = eCANGuarding;
	HBMessage.payload[0] = 0;  //is filled when sent
			
	SyncMessage.Id = eCANSyncEmcy;
	SyncMessage.len = 0;
	SyncMessage.isRTR = false;
  SyncMessage.serviceType = eCANSyncEmcy;	
		
	NmtCommand.Id = eCANNMT;
	NmtCommand.nodeId = 0;  //nodeId 0 in an NMT command is "all nodes"
	NmtCommand.len = NMTCommandFrameLength;
	NmtCommand.isRTR = false;
	NmtCommand.serviceType = eCANNMT;

}

/*-------------------------------------------------------------------
 * void COSyncHandler::SetState(SyncMasterState newState)
 * 
 * simply force the SyncState to the expected one to control
 * generation of HB and Sync
 * 
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

void COSyncHandler::SetState(SyncMasterState newState)
{
	SyncState = newState;
}

/*-------------------------------------------------------------------
 * COSyncState COSyncHandler::update(uint32_t actTime)
 * 
 * cyclic update where the actTime is compared to the last time-stamp of either HB
 * or Sync and related messages are sent
 * these are trhow away messages - unconfirmed
 * for the HB the current node stante of this device is added
 * 
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

COSyncState COSyncHandler::Update(uint32_t actTime)
{
	COSyncState returnValue = eSyncIdle;
	
	//in pre-op it's HB only
  if(SyncState == eSyncStatePreOp)
	{
		//simply send the HB message related to the given nodeId
		//add the "node state" of this service
		if(ProducerHBTime > 0)
		{
			if((actTime - lastHB) >= ProducerHBTime)
			{
				HBMessage.payload[0] = (uint8_t)SyncState;

			  if(SendRequest(&HBMessage))
				{
					lastHB = actTime;
				}
			}
		}
	}
	//in operational it's HB and Sync to be checked
  else if(eSyncStateOperational)
	{
		//simply send the HB message related to the given nodeId
		//add the "node state" of this service
		if(ProducerHBTime > 0)
		{
			if((actTime - lastHB) >= ProducerHBTime)
			{
				HBMessage.payload[0] = (uint8_t)SyncState;

			  if(SendRequest(&HBMessage))
				{
					lastHB = actTime;
				}
			}
		}
    //send the sync message when timed out
		if(SyncInterval > 0)
		{
			if((actTime - lastSync) >= SyncInterval)
			  if(SendRequest(&SyncMessage))
				{
					lastSync = actTime;
					returnValue = eSyncSyncSent;
				}
		}
	}
  return returnValue;	
}

//--- global NMT commands --------------------------------------------

/*-------------------------------------------------------------------
 * COSyncCommStates SendResetNodes();
 *
 * send a NMT global command to reset all nodes
 * reset the sync state to PreOp
 *
 * 2025-09-14 AW adapted from CONode
 *
 *--------------------------------------------------------------------*/
COSyncCommStates COSyncHandler::SendResetNodes()
{  
COSyncCommStates returnValue = eCO_SyncBusy;
	
	switch(SyncTxState)
	{
		//only in case of being idele a new message is composed
		case eCO_SyncIdle:
	    NmtCommand.command = NMT_ResetRemoteNode;
		
		  #if(DEBUG_NODE & DEBUG_SYNC_StateChange)
		  Serial.println("Sync: global Reset Node requested");
		  #endif
		  //no break here
		case eCO_SyncRetry:					
			//send the data		
		  if(SendRequest((CANMsg *)&NmtCommand))
			{
				returnValue = eCO_SyncIdle;
			  SyncState = eSyncStatePreOp;
  
		    #if(DEBUG_NODE & DEBUG_SYNC_StateChange)
			  Serial.println("Sync: global switch remote state --> reset");
		    #endif
			}
		  break;
		default:
		  #if(DEBUG_NODE & DEBUG_SYNC_StateChange)
		  Serial.println("Sync: Reset Node state unexpected");
		  #endif
			break;
	}
	return returnValue;
}

/*-------------------------------------------------------------------
 * COSyncCommStates SendResetNodes();
 *
 * send a NMT global command to start all nodes
 * reset the sync state to PreOp
 *
 * 2025-09-14 AW adapted from CONode
 *
 *--------------------------------------------------------------------*/
COSyncCommStates COSyncHandler::SendStartNodes()
{  
COSyncCommStates returnValue = eCO_SyncBusy;
	
	switch(SyncTxState)
	{
		//only in case of being idele a new message is composed
		case eCO_SyncIdle:
	    NmtCommand.command = NMT_StartRemoteNode;
		
		  #if(DEBUG_NODE & DEBUG_SYNC_StateChange)
		  Serial.println("Sync: global start Node requested");
		  #endif
		  //no break here
		case eCO_SyncRetry:					
			//send the data		
		  if(SendRequest((CANMsg *)&NmtCommand))
			{
				returnValue = eCO_SyncIdle;
			  SyncState = eSyncStateOperational;
  
		    #if(DEBUG_NODE & DEBUG_SYNC_StateChange)
			  Serial.println("Sync: global switch remote state --> started");
		    #endif
			}
		  break;
		default:
		  #if(DEBUG_NODE & DEBUG_SYNC_StateChange)
		  Serial.println("Sync: Start Node state unexpected");
		  #endif
			break;
	}
	return returnValue;
}

//--- private functions ---
		
/*-------------------------------------------------------------------
 * bool COSyncHandler::SendRequest(CANMsg *Msg)
 * 
 * proess and send a CANMsg
 * enter a retry when momentarily blocked
 * 
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

bool COSyncHandler::SendRequest(CANMsg *Msg)
{
	bool result = Handler->SendMsg(Msg);	
	
	if(result)
	{
		//if we were able to send, the service is done
	  SyncTxState = eCO_SyncIdle;
		BusyRetryCounter = 0;				

		#if(DEBUG_SYNC & DEBUG_SYNC_TXMsg)
		Serial.print("Sync: TX ");
		Serial.println(Msg->Id,HEX);
		#endif
	}
	else
	{
		BusyRetryCounter++;
		if(BusyRetryCounter > BusyRetryMax)
		{
			SyncTxState = eCO_SyncError;

		  #if(DEBUG_SYNC & DEBUG_SYNC_TXMsg)
		  Serial.print("Sync: TX ");
			Serial.print(Msg->Id,HEX);
			Serial.println(" TxReq failed");
			#endif
		}
		else
		{
			SyncTxState = eCO_SyncRetry;
		  
		  #if(DEBUG_SYNC & DEBUG_SYNC_TXMsg)
		  Serial.print("Sync: TX ");
			Serial.print(Msg->Id,HEX);
			Serial.println(" TxReq failed");
			#endif
		}
	}
	return result;
}