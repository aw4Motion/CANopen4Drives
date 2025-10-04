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

#ifndef CO_MSG_HANDLER_H
#define CO_MSG_HANDLER_H

/*--------------------------------------------------------------------
 * interface for class COMsgHandler
 * is used to send requests via CAN lib and receive responses.
 * Depending on their serive they will be distributed
 *
 * 2024-11-16 AW Frame
 *
 *-------------------------------------------------------------------*/
 
//--- includes ---

#include <UNOR4CAN.h>
#include <MC_Helpers.h>

#include <stdint.h>

const uint8_t MsgHandler_MaxNodes = 10;
const int16_t invalidNodeId = -1;
const uint8_t InvalidSlot = 0xff;

const uint8_t NumRxBuffers = 20;
const uint8_t IntRxBufferLen = 40;

const int R4WiFiTx = 10;
const int R4WiFiRx = 13;
const int R4MinimaTx = 4;
const int R4MinimaRx = 5;

const uint32_t MaxMsgTime = 2; //max expected send time for a message

//--- status codes to be know by others

typedef enum COServices {
	eCANNone = 0xFFFF,
	eCANNMT 			=  	0x000,
	eCANSyncEmcy	=   0x080,
	eCANSdoResp		=   0x580,
	eCANSdoReq		=   0x600,
	eCANTPDO1			=   0x180,
	eCANRPDO1			=   0x200,
	eCANTPDO2			=   0x280,
	eCANRPDO2			=   0x300,
	eCANTPDO3			=   0x380,
	eCANRPDO3			=   0x400,
	eCANTPDO4			=   0x480, 
	eCANRPDO4			=   0x500, 
	eCANGuarding	=   0x700
  } COService;

typedef enum COTxStatus {
	eCOTxOffline,
	eCOTxIdle,
	eCOTxBusy,
	eCOTxTimeOut
  } COTxStatus;
	
//--- the CAN message structure
typedef struct CANMsg {
   uint32_t Id;
	 uint8_t len;
	 bool isRTR;
	 COService serviceType;
	 uint8_t payload[8];
   } CANMsg;
	 
class COMsgHandler {
	public:
		COMsgHandler(int const can_tx_pin = R4WiFiTx, int const can_rx_pin = R4WiFiRx,CanBitRate bitrate = CanBitRate::BR_250k);
	  void set_can_bitrate(CanBitRate bitrate);
  
	  void Open();
		void Update(uint32_t);
	  void Reset();
	
		uint8_t RegisterNode(uint8_t);
		void UnRegisterNode(uint8_t);
		int8_t GetNodeId(uint8_t);
		
	  bool SendMsg(CANMsg *);
	  COTxStatus GetTxStatus();
	
		void Register_OnRxSDOCb(uint8_t,pfunction_holder *);
		void Register_OnRxNmtCb(uint8_t,pfunction_holder *);
		void Register_OnRxEMCYCb(uint8_t,pfunction_holder *);
		void Register_OnRxPDOCb(uint8_t,pfunction_holder *);
	
	  char IntBuff[IntRxBufferLen];
	
	  //todo: den Datenzeiger auf CAN Msg anpassen
		static void OnMsgRxCb(void *op,void *p) {
			((COMsgHandler *)op)->OnRxHandler((can_callback_args_t *)p);
		};

	private:
	  //todo: den Datenzeiger auf CAN Msg anpassen
	  void OnRxHandler(can_callback_args_t *);
		uint8_t FindNode(uint8_t);
	
	  //a local copy of the bitrate
	  CanBitRate can_bitrate;
				
	  UNOR4CAN can;   // CAN bus object´
	  //a vector of buffers to be used for any received Msg to be copied out of the interrupt context
    CANMsg CORxVector[NumRxBuffers];
	  uint8_t CORxNextRead = 0;
	  uint8_t CORxNextWrite = 0;
	  uint16_t NumRxMessages = 0;
	  uint16_t NumProcessedMessages = 0;
	
	  COTxStatus TxStatus = eCOTxOffline;
	
	  int16_t nodeId[MsgHandler_MaxNodes];
		pfunction_holder OnRxSDOCb[MsgHandler_MaxNodes];
		pfunction_holder OnRxNmtCb[MsgHandler_MaxNodes];	
		pfunction_holder OnRxEMCYCb[MsgHandler_MaxNodes];	
		pfunction_holder OnRxPDOCb[MsgHandler_MaxNodes];	
		
		uint32_t actTime;
};


#endif
