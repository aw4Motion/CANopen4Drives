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
 
/*-------------------------------------------------------------------
 * COMsgHandler.cpp
 * Implementation of the COMsgHandler Class
 * used for sending Msgs and distribution of received messages
 *
 * 2024-11-16 AW Frame
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---
 
#include <COMsgHandler.h>

#define DEBUG_ONRX		0x0001
#define DEBUG_REGNODE	0x0002
#define DEBUG_TXMSG		0x0004
#define DEBUG_ONINT   0x0008
#define DEBUG_REGHandler 0x0010

//--- definitions ---

#define DEBUG_COMSGHandler 0 // (DEBUG_ONINT | DEBUG_ONRX | DEBUG_TXMSG)


//--- implementation ---

/*------------------------------------------------------
 * COMsgHandler()
 * constuctor. Register the callback at my instance of
 * the Uart
 * 
 * 2020-05-15 AW Rev A
 * 
 * ----------------------------------------------------*/
 
COMsgHandler::COMsgHandler(int const can_tx_pin, int const can_rx_pin,CanBitRate BR):
	 can(can_tx_pin, can_rx_pin),
   can_bitrate(BR)
{
	//now set default values for no node registered
	for(uint8_t iter = 0; iter < MsgHandler_MaxNodes; iter++)
	{
		nodeId[iter] = invalidNodeId;
	}
	
	for(uint8_t iter = 0; iter < NumRxBuffers; iter++)
  {
		//invalidate all buffers
    CORxVector[iter].serviceType = eCANNone;
	}
}

/*------------------------------------------------------
 * void set_can_bitrate(CanBitRate bitrate)
 *
 * set the CAN rate to be used within Open()
 *
 * 2024-11-17 AW
 *-----------------------------------------------------*/

void COMsgHandler::set_can_bitrate(CanBitRate bitrate)
{
  can_bitrate = bitrate;	
	can.set_can_bitrate(can_bitrate);
}

/*------------------------------------------------------
 * Open()
 * Open the serial interface at the set rate
 * 
 * 2020-05-15 AW Rev A
 * 
 * ----------------------------------------------------*/
 
void COMsgHandler::COMsgHandler::Open()
{
	pfunction_holder Cb;
	//register Cb
	Cb.callback = (pfunction_pointer_t)COMsgHandler::OnMsgRxCb;
	Cb.op = (void *)this;
	
  can.set_callback(&Cb);            						// register our handler for CAN bus events

	can.set_can_bitrate(can_bitrate);           	// limited to BR_125k, BR_250k, BR_500k, BR_1000k
  bool ok = can.begin();                        // start the CAN bus peripheral

  Serial.print("MSG: > CAN begin returns ");
  Serial.println(ok);

  Serial.println("MSG: > end of setup");
  Serial.println();
	
	TxStatus = eCOTxIdle;
}

/*------------------------------------------------------
 * Update()
 * needed to call the Update of the underlying Uart as there
 * is no real interrupt driven Rx or Tx here
 * If the Msghandler has been locked for a too long time
 * it will e unlocked here to give the system a chance to recover
 * 
 * 2020-05-15 AW Rev A
 * 
 * ----------------------------------------------------*/
 
void COMsgHandler::COMsgHandler::Update(uint32_t timeNow)
{
	actTime = timeNow;

	if(CORxNextWrite != CORxNextRead)
	//while(CORxNextWrite != CORxNextRead)
	{
	  CANMsg *RxMsg = &(CORxVector[CORxNextRead]);
		uint8_t thisNodeId = RxMsg->Id & 0x7F;
    uint8_t NodeHandle = FindNode(thisNodeId);
				
	  if(NodeHandle != InvalidSlot)
	  {
		  switch(RxMsg->serviceType)
	    {
			  case eCANNone:
	      case eCANNMT:
	      case eCANSdoReq:
			  case eCANRPDO1:
			  case eCANRPDO2:
			  case eCANRPDO3:
			  case eCANRPDO4:
				  break;
			  case eCANSyncEmcy:
				  #if(DEBUG_COMSGHandler & DEBUG_ONRX)
				  Serial.print("MSG: Rx EMCY: ");
				  Serial.println(RxMsg->Id, HEX);
				  #endif
				  if(OnRxEMCYCb[NodeHandle].callback != NULL)
					  OnRxEMCYCb[NodeHandle].callback(OnRxEMCYCb[NodeHandle].op,(void *)RxMsg);
				  break;
			  case eCANSdoResp:
				  #if(DEBUG_COMSGHandler & DEBUG_ONRX)
				  Serial.print("MSG: Rx SDO Response: ");
				  Serial.println(RxMsg->Id, HEX);
				  #endif
				  if(OnRxSDOCb[NodeHandle].callback != NULL)
					  OnRxSDOCb[NodeHandle].callback(OnRxSDOCb[NodeHandle].op,(void *)RxMsg);
				  break;
			  case eCANTPDO1:
			  case eCANTPDO2:
			  case eCANTPDO3:
			  case eCANTPDO4:
				  #if(DEBUG_COMSGHandler & DEBUG_ONRX)
			    Serial.print("MSG: Rx PDO: ");
			    Serial.println(RxMsg->Id, HEX);
				  #endif
				  if(OnRxPDOCb[NodeHandle].callback != NULL)
					  OnRxPDOCb[NodeHandle].callback(OnRxPDOCb[NodeHandle].op,(void *)RxMsg);
					else
						Serial.println("Msg: no PDO handler present");
			    break;
        case eCANGuarding:
				  #if(DEBUG_COMSGHandler & DEBUG_ONRX & 0)
			    Serial.print("MSG: Rx Guarding: ");
			    Serial.println(RxMsg->Id, HEX);
				  #endif
				  if(OnRxNmtCb[NodeHandle].callback != NULL)
					  OnRxNmtCb[NodeHandle].callback(OnRxNmtCb[NodeHandle].op,(void *)RxMsg);
					else
						Serial.println("Msg: no Nmt handler present");
					
				  break;
			  default:
				  break;
		  }	// end of switch case
	  } // end of processing for Node is registered
		CORxNextRead++;
    if(CORxNextRead == NumRxBuffers)
			CORxNextRead = 0;

		NumProcessedMessages++;
  } //end of processing when NextRead != NextWrite
}

/*------------------------------------------------------
 * Reset()
 * to be called, when to upper layers run into a TO
 * does not delete pending messages
 * MsgHandler itself doesn't really have states do be reset
 * so it's a call to reset the Uart only//end of swtich case
 // end of processing for Node is registered * 
 * 2020-07-25 AW Rev A
 * 
 * ----------------------------------------------------*/
void COMsgHandler::COMsgHandler::Reset()
{
	#if 0
	Uart.ResetUart();
	#else
	;
	#endif
}

/*------------------------------------------------------
 * OnRxHandler(can_callback_args_t *p_args)
 * react to a received Msg
 * this is interrupt context and we should not use Serial.print out of this
 * >> will enter the contents of any received message in the prepared CORxVector
 *    no special flag used - Update will reacte when (CORxNextWrite != CORxNextRead)
 * >> will flag a Tx being done when receiving the indication
 * 
 * 2020-05-15 AW Rev A
 * 
 * ----------------------------------------------------*/

void COMsgHandler::OnRxHandler(can_callback_args_t *p_args)
{
  switch (p_args->event) 
	{
    case CAN_EVENT_TX_COMPLETE:
			//SendMsg left the TxStatus @ TxStatus = eCOTxBusy
		  //and would send only when TxStatus == eCOTxIdle
		  //only now we can take new commands
      TxStatus = eCOTxIdle;
      break;

    case CAN_EVENT_RX_COMPLETE:
			CORxVector[CORxNextWrite].Id = p_args->frame.id;
		  CORxVector[CORxNextWrite].len = p_args->frame.data_length_code;
		  if(p_args->frame.type == CAN_FRAME_TYPE_REMOTE)
			  CORxVector[CORxNextWrite].isRTR = true;
      else
			{
        CORxVector[CORxNextWrite].isRTR = false;
				memcpy((void *)CORxVector[CORxNextWrite].payload, p_args->frame.data, p_args->frame.data_length_code);
			}
     	//now determine the service type
			CORxVector[CORxNextWrite].serviceType = (COService)(p_args->frame.id & 0xFF80);
      
			#if(DEBUG_COMSGHandler & DEBUG_ONINT)
			sprintf(IntBuff, "Int: rx: [%lX] [%d]: s: %X @ %d ", p_args->frame.id, p_args->frame.data_length_code,CORxVector[CORxNextWrite].serviceType,CORxNextWrite);
			#endif
			
      NumRxMessages++;
			CORxNextWrite++;
      if(CORxNextWrite == NumRxBuffers)
				CORxNextWrite = 0;
      break;

    case CAN_EVENT_ERR_WARNING:          /* error warning event */
    case CAN_EVENT_ERR_PASSIVE:          /* error passive event */
    case CAN_EVENT_ERR_BUS_OFF:          /* error bus off event */
    case CAN_EVENT_BUS_RECOVERY:         /* Bus recovery error event */
    case CAN_EVENT_MAILBOX_MESSAGE_LOST: /* overwrite/overrun error event */
    case CAN_EVENT_ERR_BUS_LOCK:         /* Bus lock detected (32 consecutive dominant bits). */
    case CAN_EVENT_ERR_CHANNEL:          /* Channel error has occurred. */
    case CAN_EVENT_TX_ABORTED:           /* Transmit abort event. */
    case CAN_EVENT_ERR_GLOBAL:           /* Global error has occurred. */
    case CAN_EVENT_TX_FIFO_EMPTY:        /* Transmit FIFO is empty. */
      #if 0
      Serial.print("> handler: error = ");
      Serial.println(p_args->event);
      #endif
      break;
		default:
		  #if(DEBUG_COMSGHandler & DEBUG_ONINT)
			sprintf(IntBuff, "Int: rx: [%lX] [%d]: s: %X @ %d, ? ", p_args->frame.id, p_args->frame.data_length_code,CORxVector[CORxNextWrite].serviceType,CORxNextWrite);
			#endif
			;
		  break;
  }
}

/*----------------------------------------------------------
 * char FindNode(char)
 * find the NodeHandle in the vector of adresses
 * 
 * 2020-05-16 AW Header
 * 
 * --------------------------------------------------------*/
uint8_t COMsgHandler::FindNode(uint8_t NodeId)
{
	uint8_t i = 0;
	uint8_t slot = InvalidSlot;
	while(i < MsgHandler_MaxNodes)
	{
		if((uint8_t)nodeId[i] == NodeId)
			slot = i;
		i++;
	}
	return slot;	
}

/*----------------------------------------------------------
 * uint8_t RegisterNode(char)
 * try to register a node with it's node ID.
 * the handerl shall be used as a regference for furhter calls
 * 
 * 2020-05-16 AW Header
 * 
 * --------------------------------------------------------*/

uint8_t COMsgHandler::RegisterNode(uint8_t thisNodeId)
{
	uint8_t i = 0;
	uint8_t slot = InvalidSlot;
	while(i < MsgHandler_MaxNodes)
	{
		if(nodeId[i] == invalidNodeId)
		{
			slot = i;
			break;
		}
		i++;
	}
	if(slot != InvalidSlot)
		nodeId[slot]=(int16_t)thisNodeId;
	
	#if(DEBUG_COMSGHandler & DEBUG_REGNODE)
	Serial.print("MSG: Reg ");
	Serial.print(thisNodeId, DEC);
	Serial.print(" at ");
	Serial.println(slot);
	#endif
	
	return slot;
}

/*----------------------------------------------------------
 *  int8_t MsgHandler::GetNodeId(uint8_t)
 *  read the nodeId of a regsitered node back
 * 
 * 2020-11-01 AW
 * ----------------------------------------------------------*/ 

int8_t COMsgHandler::GetNodeId(uint8_t NodeHandle)
{
int8_t thisNodeId = -1;

	if(NodeHandle < MsgHandler_MaxNodes)
	{
		thisNodeId = (uint8_t) nodeId[NodeHandle];
	}
	return thisNodeId;
}
		

/*----------------------------------------------------------
 * void UnRegisterNode(char)
 * remove the entry for a given node
 * 
 * 2020-05-16 AW Header
 * 
 * --------------------------------------------------------*/

void COMsgHandler::UnRegisterNode(uint8_t NodeHandle)
{
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		nodeId[NodeHandle] = invalidNodeId;
		OnRxSDOCb[NodeHandle].callback = NULL;
		OnRxSDOCb[NodeHandle].op = NULL;
		OnRxNmtCb[NodeHandle].callback = NULL;
		OnRxNmtCb[NodeHandle].op = NULL;
		OnRxEMCYCb[NodeHandle].callback = NULL;
		OnRxEMCYCb[NodeHandle].op = NULL;
		OnRxPDOCb[NodeHandle].callback = NULL;
		OnRxPDOCb[NodeHandle].op = NULL;
		
	}
}
		
/*----------------------------------------------------------
 * SendMsg(MCMsg *TxMsg)
 *
 * if possible send the Msg directly
 * takes a pointer to a CAN msg and copies the contents in a 
 * CAN frame used by the low level
 * this local instance is used for send - a syncronous call
 *
 * 2025-01-01 AW
 * 
 * --------------------------------------------------------*/

bool COMsgHandler::SendMsg(CANMsg *msg)
{
	bool returnValue = false;
	
	if(TxStatus == eCOTxIdle)
  {
    can_frame_t TxMsg;
    
    TxMsg.id = msg->Id;
		TxMsg.id_mode = CAN_ID_MODE_STANDARD;
		if(msg->isRTR)
		{
			TxMsg.type = CAN_FRAME_TYPE_REMOTE;
			TxMsg.data_length_code = 0;
		}
		else
		{
		  TxMsg.type = CAN_FRAME_TYPE_DATA;
		  TxMsg.data_length_code = msg->len;
      memcpy(TxMsg.data, msg->payload, 8);
		}
		//indicate busy before sending
		TxStatus = eCOTxBusy;
		returnValue = can.send(&TxMsg);
   
		#if ((DEBUG_COMSGHandler & DEBUG_TXMSG) > 0)
    Serial.print("Msg> CAN write of frame returns: ");
    Serial.println((returnValue ? "ok" : "fail"));
    #endif
	
	}
	else
	{
		#if ((DEBUG_COMSGHandler & DEBUG_TXMSG) > 0)
    Serial.println("Msg> CAN write still busy");
    #else
		;
		#endif
	}

	return returnValue;
}

/*----------------------------------------------------------
 * COTxStatus GetTxStatus()
 * return the status of the Tx channel
 * 
 * 2024-11-20 AW 
 * 
 * --------------------------------------------------------*/

COTxStatus COMsgHandler::GetTxStatus()
{
	return TxStatus;
}

/*----------------------------------------------------------
 * Register_onRxCb(function_holder *cb)
 * store the function and object pointer for the callback
 * called in case of a successful Rx
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

void COMsgHandler::Register_OnRxSDOCb(uint8_t NodeHandle, pfunction_holder *Cb)
{
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		OnRxSDOCb[NodeHandle].callback = Cb->callback;
		OnRxSDOCb[NodeHandle].op = Cb->op;
		
		#if(DEBUG_COMSGHandler & DEBUG_REGHandler)
		Serial.print("registered SDO Handler @ ");
		Serial.println(NodeHandle);
		#endif
	}
}

/*----------------------------------------------------------
 * Register_onRxNmtCb(function_holder *cb)
 * store the function and object pointer for the callback
 * called in case of a successful Rx
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

void COMsgHandler::Register_OnRxNmtCb(uint8_t NodeHandle, pfunction_holder *Cb)
{
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		OnRxNmtCb[NodeHandle].callback = Cb->callback;
		OnRxNmtCb[NodeHandle].op = Cb->op;
		
		#if(DEBUG_COMSGHandler & DEBUG_REGHandler)
		Serial.print("registered Nmt Handler @ ");
		Serial.println(NodeHandle);
		#endif
	}
}

/*----------------------------------------------------------
 * Register_onRxEMCYCb(function_holder *cb)
 * store the function and object pointer for the callback
 * called in case of a successful Rx
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

void COMsgHandler::Register_OnRxEMCYCb(uint8_t NodeHandle, pfunction_holder *Cb)
{
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		OnRxEMCYCb[NodeHandle].callback = Cb->callback;
		OnRxEMCYCb[NodeHandle].op = Cb->op;
		
		#if(DEBUG_COMSGHandler & DEBUG_REGHandler)
		Serial.print("registered EMCY Handler @ ");
		Serial.println(NodeHandle);
		#endif
	}
}

/*----------------------------------------------------------
 * Register_onRxPDOCb(function_holder *cb)
 * store the function and object pointer for the callback
 * called in case of a successful Rx
 * 
 * 2020-05-10 AW Header
 * 
 * --------------------------------------------------------*/

void COMsgHandler::Register_OnRxPDOCb(uint8_t NodeHandle, pfunction_holder *Cb)
{
	if(NodeHandle < MsgHandler_MaxNodes)
	{
		OnRxPDOCb[NodeHandle].callback = Cb->callback;
		OnRxPDOCb[NodeHandle].op = Cb->op;

		#if(DEBUG_COMSGHandler & DEBUG_REGHandler)
		Serial.print("registered PDO Handler @ ");
		Serial.println(NodeHandle);
		#endif
	}
}
