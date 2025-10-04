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
 
#ifndef CO_SDOHANDLER_H
#define CO_SDOHANDLER_H

/*--------------------------------------------------------------
 * class CO_SDOHandler
 * handles R/W of parameters via SDO
 * uses an already existing instance of the MsgHandler
 *
 * 2024-11-28 AW derived from RS Msghandler
 *
 *-------------------------------------------------------------*/
 
//--- inlcudes ----
 
#include <COMsgHandler.h>
#include <COObjects.h>

#include <stdint.h>

//--- SDO service defines ---

const uint8_t SDOInitUploadReq = 2;
const uint8_t SDOInitUploadResponse = 2;
const uint8_t SDOUploadSegReq = 3;
const uint8_t SDOUploadSegResp = 0;

const uint8_t SDOInitDownloadReq = 1;
const uint8_t SDOInitDownloadResp = 3;
const uint8_t SDODownloadSegReq = 0;
const uint8_t SDODownloadSegResp = 1;

const uint8_t SDOErrorReqResp = 4;

#define ExpDataLen 4
#define SegDataLen 7

//--- definitions arround the SDO messages

//basic SDO request/response
//no need to go for a packed struct here

typedef struct  __attribute__((packed)) SDO_CS_Init {
	uint8_t s  : 1;
	uint8_t e  : 1;
	uint8_t n  : 2;
	uint8_t x  : 1;
	uint8_t cs : 3;
} SDO_CS_Init;

typedef struct  __attribute__((packed)) SDO_CS_Seg {
	uint8_t c  : 1;
	uint8_t n  : 3;
	uint8_t t  : 1;
	uint8_t cs : 3;
} SDO_CS_Seg;

typedef union {
			uint8_t u8[4];
			uint16_t u16[2];
			uint32_t u32;
} ExpeditedData;

typedef struct __attribute__((packed)) COSDOMsgExp {
   SDO_CS_Init control;
   uint16_t  Idx;
   uint8_t SubIdx;
   ExpeditedData Data;
} COSDOMsgExp;

typedef struct __attribute__((packed)) COSDOMsgSeg {
   SDO_CS_Seg control;
   uint8_t Data[SegDataLen];
} COSDOMsgSeg;

typedef union COSDO {
	COSDOMsgExp MsgExp;
	COSDOMsgSeg MsgSeg;
	uint32_t words[2];
	uint8_t bytes[8];
} COSDO;


//define the enum with the Comm states

typedef enum COSDOCommStates {
	eCO_SDOUnknown,  //meant for intialization where the used SDO service has to be rest first
	eCO_SDOIdle,
	eCO_SDOWaiting,
	eCO_SDOBusy,
	eCO_SDODone,
	eCO_SDOError,
	eCO_SDORetry,
	eCO_SDOTimeout
} COSDOCommStates;

typedef enum COSDORequestType {
	eSDONoRequest,
	eSDOReadRequestInit, //the first is allways segement, only the response will tell
	eSDOReadRequestSeg, 
	eSDOWriteRequestExp,
	eSDOWriteRequestSeg  
} COSDORequestType;
//define the class itself

class COSDOHandler {
	public:
		COSDOHandler();
		void init(COMsgHandler *,int8_t,int8_t);
		void SetActTime(uint32_t);
		
		COSDOCommStates ReadSDO(uint16_t, uint8_t, void *, uint32_t *);
	  COSDOCommStates ReadSDO(ODEntry *Object);
		COSDOCommStates WriteSDO(uint16_t, uint8_t, void *,uint32_t);
	  COSDOCommStates WriteSDO(ODEntry *Object);
	
		COSDOCommStates ReadObjects(ODEntry **, uint8_t);
		COSDOCommStates WriteObjects(ODEntry **, uint8_t);

		COSDOCommStates GetComState();
		void ResetComState(); 
		void SetTORetryMax(uint8_t);
		void SetBusyRetryMax(uint8_t);
		
		//handler to be registered at the Msghandler instance
		static void OnCOSDOMsgRxCb(void *op,void *p) {
			((COSDOHandler *)op)->OnRxHandler((CANMsg *)p);
		};
		
	private:
		void OnRxHandler(CANMsg *);
    void OnTimeOut();
	  bool SendRequest(CANMsg *);
    
		int8_t nodeId = invalidNodeId;

	  CANMsg SDORequestMsg;
	
		COSDO *SDOReqData;
	  uint8_t nextToggle = 0;

		COSDOCommStates SDORxTxState = eCO_SDOIdle;
	
	  uint8_t RWObjectsAccessStep = 0;

		uint32_t MaxRxLen = 0;
	  uint32_t ExpectedRxTxLen = 0;
		uint32_t ActRxTxLen = 0;

	  uint8_t *ClientDataPtr;
		
	  COSDORequestType requestedService = eSDONoRequest;
		uint16_t requestedIdx;
		uint8_t requestedSub;
		
		COMsgHandler *Handler;
		
		uint32_t RequestSentAt;
		uint32_t actTime;
	  bool isTimerActive = false;
				
		uint8_t TORetryCounter = 0;
		uint8_t TORetryMax = 1;
		uint8_t BusyRetryCounter = 0;
		uint8_t BusyRetryMax = 5;
};
 

#endif
