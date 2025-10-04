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
 * SDOHandler.cpp
 * handels R/W access to deive parameters via SDO services
 * does itself no interpreation
 *
 * 2024-11-28 AW Frame derived from RS SDOHandler.cpp
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <COSDOHandler.h>

#define DEBUG_RXMSG 	0x0001
#define DEBUG_WREQ		0x0002
#define DEBUG_RREQ		0x0004
#define DEBUG_ERROR		0x0008
#define DEBUG_TO		  0x0010
#define DEBUG_INIT    0x0020
#define DEBUG_BUSY    0x8000

#define DEBUG_SDO (DEBUG_INIT | DEBUG_TO | DEBUG_ERROR | DEBUG_BUSY)
//#define DEBUG_SDO (DEBUG_TO | DEBUG_ERROR | DEBUG_BUSY)

//--- implementation ---

//this is an ultimate measure
//might be delayed due to the Serial commands 
const uint32_t SDORespTimeOut = 20;

//--- public calls ---

/*---------------------------------------------------
 * SDOHandler()
 * init the instance by at least initializing the RxLen
 * 
 * 2020-11-18 AW Done
 *---------------------------------------------------*/
 
COSDOHandler::COSDOHandler()
{
	ActRxTxLen = 0;
}

/*-------------------------------------------------------
 * void init(MsgHandler *,uint8_t)
 * create a functor to register this instance of a SDOHandler
 * at the Msghander which is referred to.
 * The SDOHandler will store the pointer to the Msghandler for further
 * use. Needs to be given the handle under which the node is registered
 * at the MsgHandler and uses this to finally regsiter the Call-back for
 * SDO messages
 * 
 * 2020-11-18 AW Done
 * ---------------------------------------------------------------*/

void COSDOHandler::init(COMsgHandler *MsgHandler, int8_t ThisNode, int8_t MsgHandle)
{
	nodeId = ThisNode;
	Handler = MsgHandler;
		
	if(MsgHandle != InvalidSlot)
	{
	  //register Cb
		pfunction_holder Cb;
		
		SDORequestMsg.Id = eCANSdoReq | ThisNode;
		SDORequestMsg.len = 8;
		SDORequestMsg.isRTR = false;
    SDORequestMsg.serviceType = eCANSdoReq;
    //don't care for the contents of the payload
		SDOReqData = (COSDO *)&(SDORequestMsg.payload[0]);

		Cb.callback = (pfunction_pointer_t)COSDOHandler::OnCOSDOMsgRxCb;
		Cb.op = (void *)this;

	  Handler->Register_OnRxSDOCb(MsgHandle,&Cb);
	  SDORxTxState = eCO_SDOIdle;	

    #if (DEBUG_SDO & DEBUG_INIT)
    Serial.print("SDO: Handler registered @ Msg. Node: ");
		Serial.print(nodeId);
		Serial.print(" Handle: ");
		Serial.println(MsgHandle);		
		#endif
	}
	else
	{
		SDORxTxState = eCO_SDOError;	
		
    #if (DEBUG_SDO & DEBUG_ERROR)
    Serial.print("SDO: Could not get a handle for node ");
		Serial.println(ThisNode);
		#endif
	}
	requestedService = eSDONoRequest;
}

/*---------------------------------------------------------------
 * SDOCommStates GetComState()
 * return the state of either the Rx or Tx of an SDO
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------------*/
 
COSDOCommStates COSDOHandler::GetComState()
{
	return SDORxTxState;
}

/*--------------------------------------------------------------
 * void SetTORetryMax(uint8_t)
 * An attempted transfer can run into a timeout either while trying to send
 * or by waiting for a response. The Time-out counter is incremented if 
 * a consecutive occurs. the call here cen be used to modify the
 * default max value for the number of time-outs in a row
 * 
 * 2020-11-18 AW Done
 * --------------------------------------------------------------*/ 

void COSDOHandler::SetTORetryMax(uint8_t value)
{
	TORetryMax = value;
}

/*--------------------------------------------------------------
 * void SetBusyRetryMax(uint8_t)
 * An attempted transfer can fail because the Msghandler and Uart are blocked
 * either while trying to send a request. The Time-out counter is incremented if 
 * a consecutive occurs. the call here cen be used to modify the
 * default max value for the number of time-outs in a row
 * 
 * 2020-11-18 AW Done
 * --------------------------------------------------------------*/
 
void COSDOHandler::SetBusyRetryMax(uint8_t value)
{
	BusyRetryMax = value;
}

/*----------------------------------------------
 * void SDOHandler::ResetComState()
 * to be called after each interaction to 
 * move the SDORxTxState from eDone to eIdle
 * 
 * 2020-10-16 AW inital
 * ---------------------------------------------*/

void COSDOHandler::ResetComState()
{
	SDORxTxState = eCO_SDOIdle;
	requestedService = eSDONoRequest;

	TORetryCounter = 0;
	BusyRetryCounter = 0;
	//Handler should not be reset, as it could be used by different
	//instances of the Drive
}

/*-------------------------------------------------------------
 * SDOCommStates ReadSDO(uint16_t Idx, uint8_t SubIdx)
 * Try to read a drive parameter identified by its Idx and SubIdx.
 * Is using a step sequence of sendig a request, waiting for an answer
 * and giving it a retry if not sucessfull.
 * Reception fo the responses is via the OnRxHandler().
 * SDORxTxState is used to handle the steps and as the central feedback
 * ReadSDO will end up in eDone state to indicate the requested value
 * has been received and can be read.
 * So actuall reading the value will reset the communication state to eIdle.
 * 
 * As any access to the MsgHandler ReadSDO will lock the Msghandler and
 * will only unlock it the actual service failed.
 * Successful service will unlock in OnRxHandler().

 * 
 * 2024-12-02 AW Done
 * -------------------------------------------------------------*/

COSDOCommStates COSDOHandler::ReadSDO(uint16_t Idx, uint8_t SubIdx, void *dataptr, uint32_t *length)
{
	switch(SDORxTxState)
	{
		case eCO_SDOIdle:
			//fill the SDO read request message
      SDOReqData->MsgExp.Idx = Idx;
      SDOReqData->MsgExp.SubIdx = SubIdx;
		  SDOReqData->MsgExp.Data.u32 = 0;
		  SDOReqData->MsgExp.control.cs = SDOInitUploadReq; //upload request
      SDOReqData->MsgExp.control.x = 0;
		  SDOReqData->MsgExp.control.n = 0;
		  //with the init upload it's not yet known, whether expedited or segmented
		  SDOReqData->MsgExp.control.e = 0;
		  //no size given
		  SDOReqData->MsgExp.control.s = 0;
				  			
		  //now store the requested object
		  requestedIdx = Idx;
		  requestedSub = SubIdx;
		  MaxRxLen = *length;
		  ActRxTxLen = 0;

		  nextToggle = 0;
		  //save the pointer to the data
		  ClientDataPtr = (uint8_t *)dataptr;
      requestedService = eSDOReadRequestInit;
		  //no break here;
		
		case eCO_SDORetry:
		{
			//try to send the data
			if(SendRequest(&SDORequestMsg))
			{
				SDORxTxState = eCO_SDOWaiting;

				BusyRetryCounter = 0;
				
				#if(DEBUG_SDO & DEBUG_RREQ)
				Serial.print("SDO: N ");
				Serial.print(nodeId, DEC);
				Serial.print(" RxReq ok: ");
				Serial.print(Idx, HEX);
				Serial.println(" --> eSDOWaiting");
				#endif
				
				//register a timeout handler
				RequestSentAt = actTime;
				isTimerActive = true;
			}
			else
			{
				//was busy
				BusyRetryCounter++;
				if(BusyRetryCounter > BusyRetryMax)
				{
					SDORxTxState = eCO_SDOError;
					#if(DEBUG_SDO & DEBUG_ERROR)
					Serial.print("SDO: N ");
					Serial.print(nodeId, DEC);
					Serial.println(" RxReq failed --> eError");
					#endif
				}
				else
				{
					SDORxTxState = eCO_SDORetry;
					#if(DEBUG_SDO & (DEBUG_RREQ | DEBUG_BUSY))
					Serial.print("SDO: N ");
					Serial.print(nodeId,DEC);
					Serial.print("Idx: ");
					Serial.print(Idx,HEX);
					Serial.print(".");
					Serial.print(SubIdx,HEX);
					Serial.println(" RxReq busy --> eRetry");
					#endif
				}
			}
		}
		break;

		case eCO_SDODone:
		  //assumption is - we received a response which set the RxTxState to Done
		  //data has already been copied into the target
			#if(DEBUG_SDO & DEBUG_RREQ)
		  Serial.print("SDO: Read Done: ");			    
			for(uint8_t iter = 0; iter < ActRxTxLen; iter++)
			{
		    Serial.print(ClientDataPtr[iter],HEX);
				Serial.print(".");
			}
			#endif
			*length = ActRxTxLen;
			requestedService = eSDONoRequest;
		  break;

	}
	return SDORxTxState;
}

/*-------------------------------------------------------------------
 * COSDOCommStates COSDOHandler::ReadSDO(ODEntry *)
 *
 * read an entry given by an ODEntry struct
 * return _SDODone when successful
 *
 * 2025-09-13 AW
 *-------------------------------------------------------------------*/

COSDOCommStates COSDOHandler::ReadSDO(ODEntry *Object)
{
	COSDOCommStates returnValue = ReadSDO(Object->Idx, Object->SubIdx, Object->Value, &(Object->len));

	if(returnValue == eCO_SDODone)
     ResetComState();

return returnValue;
}	

	

/*-------------------------------------------------------------
 * SDOCommStates WriteSDO(uint16_t Idx, uint8_t SubIdx,uint32_t *Data,uint8_t len)
 * Try to write a drive parameter identified by its Idx and SubIdx.
 * Additional parameters are the value itself and the size of the parameter in bytes.
 * Is using a step sequence of sendig a request, waiting for an answer
 * and giving it a retry if not sucessfull.
 * Reception fo the responses is via the OnRxHandler().
 * SDORxTxState is used to handle the steps and as the central feedback
 * WriteSDO will end up in eDone state to indicate the requested value
 * has been sent. Nedes to be reset explictily by calling ResetComState().
 * 
 * As any access to the MsgHandler WriteSDO will lock the Msghandler and
 * will only unlock it the actual service failed.
 * Successful servie will unlock in OnRxHandler().
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------------*/

COSDOCommStates COSDOHandler::WriteSDO(uint16_t Idx, uint8_t SubIdx,void *Data, uint32_t len)
{
	switch(SDORxTxState)
	{
		case eCO_SDOIdle:
			//fill the SDO read request message
      SDOReqData->MsgExp.Idx = Idx;
      SDOReqData->MsgExp.SubIdx = SubIdx;
			
		  //now register the expected lenght
		  ExpectedRxTxLen = len;
		  ActRxTxLen = 0;
		
		  //upload request
		  //same for epedited and standard
		  SDOReqData->MsgExp.control.cs = SDOInitDownloadReq; 

		  if(len <= 4)
			{
		    SDOReqData->MsgExp.control.e = 1;
		    SDOReqData->MsgExp.control.n = 4-len;
				SDOReqData->MsgExp.control.s = 1;
        SDOReqData->MsgExp.control.x = 0;
				
		    //clear the contents first	  
        SDOReqData->MsgExp.Data.u32 = 0;

				//have to copy the data into the message
				if(len == 1)
			    SDOReqData->MsgExp.Data.u8[0] = *((uint8_t *)Data);
			  else if(len == 2)
			    SDOReqData->MsgExp.Data.u16[0] = *((uint16_t *)Data);				
			  else if(len == 4)
          SDOReqData->MsgExp.Data.u32 = *((uint32_t *)Data);

			}
			else
			{
				//this is segmented, but the request has the same contents as an expedited one
		    SDOReqData->MsgExp.control.e = 0;
		    SDOReqData->MsgExp.control.n = 0;
		    SDOReqData->MsgExp.control.s = 1;
				
				//in the expedited case it's the number of bytes to be donwloaded
				SDOReqData->MsgExp.Data.u32 = len;
        ClientDataPtr = (uint8_t *)Data;
			  
				nextToggle = 0;				
			}
			//the init request is the same for both
			requestedService = eSDOWriteRequestExp;

      //now store the requested object
		  requestedIdx = Idx;
		  requestedSub = SubIdx;
			
		  //no break here
		case eCO_SDORetry:			
			//send the data
			if(SendRequest(&SDORequestMsg))
			{
				SDORxTxState = eCO_SDOWaiting;
				BusyRetryCounter = 0;
				
				#if(DEBUG_SDO & DEBUG_WREQ)
				Serial.print("SDO: N ");
				Serial.print(nodeId ,DEC);
				Serial.print(" TxReq ok ");
				Serial.println(Idx, HEX);
				#endif

				RequestSentAt = actTime;
				isTimerActive = true;
			}
			else
			{
				BusyRetryCounter++;
				if(BusyRetryCounter > BusyRetryMax)
				{
					SDORxTxState = eCO_SDOError;
					#if(DEBUG_SDO & DEBUG_ERROR)
					Serial.print("SDO: N ");
					Serial.print(nodeId ,DEC);
					Serial.println(" TxReq failed --> eError");
					#endif
				}
				else
				{
					SDORxTxState = eCO_SDORetry;
					#if(DEBUG_SDO & (DEBUG_WREQ | DEBUG_BUSY))
					Serial.print("SDO: N ");
					Serial.print(nodeId,DEC);
					Serial.print("Idx: ");
					Serial.print(Idx,HEX);
					Serial.print(".");
					Serial.print(SubIdx,HEX);
					Serial.println(" TxReq busy --> eRetry");
					#endif
				}
			}
		  break;

		case eCO_SDODone:
		  //assumption is - we received a response which set the RxTxState to Done
		  //data has already been copied into the target
			requestedService = eSDONoRequest;
			
		  #if(DEBUG_SDO & DEBUG_WREQ)
      Serial.println("Write Done: ");			    
      #endif
		
		  break;

		//no need to handle eCO_SDODone here
	} //end of switch (SDORxTxState)
	return SDORxTxState;
}

/*-------------------------------------------------------------------
 * COSDOCommStates COSDOHandler::WriteSDO(ODEntry *)
 *
 * read an entry given by an ODEntry struct
 * return _SDODone when successful
 *
 * 2025-09-13 AW
 *-------------------------------------------------------------------*/

COSDOCommStates COSDOHandler::WriteSDO(ODEntry *Object)
{
	COSDOCommStates returnValue = WriteSDO(Object->Idx, Object->SubIdx, Object->Value, Object->len);

	if(returnValue == eCO_SDODone)
     ResetComState();

return returnValue;
}


/*-------------------------------------------------------------------
 * COSDOCommStates COSDOHandler::ReadObjects(ODEntry **, uint8_t nrEntries)
 *
 * read a complete set of objects given oin an vector
 * return _SDODone when all have been read
 *
 *
 * 2025-09-11 AW
 *-------------------------------------------------------------------*/

COSDOCommStates COSDOHandler::ReadObjects(ODEntry **Objects, uint8_t nrEntries)
{
	COSDOCommStates returnValue = eCO_SDOBusy;
	COSDOCommStates stepResult;
	uint32_t actLength;
	
	stepResult = ReadSDO(Objects[RWObjectsAccessStep]->Idx,
		                   Objects[RWObjectsAccessStep]->SubIdx,
	                     Objects[RWObjectsAccessStep]->Value,
	                     &actLength);
	
	//we can only increment the pointr when a preceeding one is done
	if(stepResult == eCO_SDODone)
	{
		Objects[RWObjectsAccessStep]->len = actLength;

		RWObjectsAccessStep++;
		ResetComState();
	
		//was this the last one?
		if(RWObjectsAccessStep == nrEntries)
		{
			returnValue = eCO_SDODone;	
			RWObjectsAccessStep = 0;
		}
	}
	//if Read results in an error
	if(stepResult == eCO_SDOError)
		returnValue = eCO_SDOError;		
	
	return returnValue;	
}


/*-------------------------------------------------------------------
 * COSDOCommStates COSDOHandler::WriteObjects(ODEntry **, uint8_t nrEntries)
 *
 * write a complete set of objects given oin an vector
 * return _SDODone when all have been written
 *
 *
 * 2025-09-11 AW
 *-------------------------------------------------------------------*/

COSDOCommStates COSDOHandler::WriteObjects(ODEntry **Objects, uint8_t nrEntries)
{
	COSDOCommStates returnValue = eCO_SDOBusy;
	COSDOCommStates stepResult;

  //for an object to write to the len is a given	
	stepResult = 	WriteSDO(Objects[RWObjectsAccessStep]->Idx,
		                     Objects[RWObjectsAccessStep]->SubIdx,
	                       Objects[RWObjectsAccessStep]->Value,
	                       Objects[RWObjectsAccessStep]->len);
	
	//we can only increment the pointr when a preceeding one is done
	if(stepResult == eCO_SDODone)
	{
		RWObjectsAccessStep++;
		ResetComState();
		
		//was this the last one?
	  if(RWObjectsAccessStep == nrEntries)
	  {
	    returnValue = eCO_SDODone;	
		  RWObjectsAccessStep = 0;
	  }
	}
	
	//if Write results in an error
	if(stepResult == eCO_SDOError)
		returnValue = eCO_SDOError;		

	return returnValue;	
}


//-------------------------------------------------------------------
//--- private calls ---

/*-------------------------------------------------------------------
 * bool SendRequest(CANMsg *)
 *
 * send the request - the onyl direct Tx interface to the COMsghandler
 *
 *
 * 2025-01-05 AW
 *-------------------------------------------------------------------*/

bool COSDOHandler::SendRequest(CANMsg *Msg)
{
  return Handler->SendMsg(Msg);	
}

/*-------------------------------------------------------------------
 * void OnRxHandler(MCMsg *Msg)
 * The actual handler for any SDO services received by the MsgHandler
 * Checks whether the received response belongs to any open
 * request and will switch these to eDone.
 * The received message can be used without explicitely coping it
 * onyl the payload will have to be copied
 *
 * CANMsg is defined in COMsghandler.h and is still a complete raw CAN
 * msg.
 * Playload should be casted to a SDO
 * 
 * 2020-11-18 AW Done
 * -----------------------------------------------------------------*/

void COSDOHandler::OnRxHandler(CANMsg *Msg)
{
	//fist check payload[0] for being a correct response type and if not an error
	//payload [1...3] for having the expected object
	COSDO *Response = (COSDO *)(Msg->payload);
	
  if(Response->MsgExp.control.cs == SDOErrorReqResp)
  {
	  SDORxTxState = eCO_SDOError;
		Serial.println("SDO: Error: Server sent cancellation");	
    isTimerActive = false;
  }
  else
  {
    //this might have an index or sub-index	
    if((requestedService == eSDOReadRequestInit) && (Response->MsgExp.control.cs == SDOInitUploadResponse))	
    {
			if((Response->MsgExp.Idx == requestedIdx) && (Response->MsgExp.SubIdx == requestedSub))
      {
		    // the upload request ended with a response which has to be examined to determine
		    // whether being done (expedited) or requiring segments
				if((Response->MsgExp.control.e == 1) && (Response->MsgExp.control.s == 1))
				{
					ActRxTxLen = 4 - Response->MsgExp.control.n;
					//this is expedited and size is given
					if(ActRxTxLen == 1)
						*(uint8_t *)ClientDataPtr = Response->MsgExp.Data.u8[0];
					else if(ActRxTxLen == 2)
						*(uint16_t *)ClientDataPtr = Response->MsgExp.Data.u16[0];				
					else if(ActRxTxLen == 4)
						*(uint32_t *)ClientDataPtr = Response->MsgExp.Data.u32;

					#if(DEBUG_SDO  & DEBUG_RXMSG)
				  Serial.print("SDO: Rx Idx ");
				  Serial.print(requestedIdx, HEX);
				  Serial.print(" :");
				  Serial.println(Response->MsgExp.Data.u32, HEX);				
				  #endif

          isTimerActive = false;
					SDORxTxState = eCO_SDODone;
				}
				else if((Response->MsgExp.control.e == 0) && (Response->MsgExp.control.s == 1))
				{
					ExpectedRxTxLen = Response->MsgExp.Data.u32;

					//todo
					//check against MaxRxLen
					
					ActRxTxLen = 0;
					//compose the next request
					SDOReqData->MsgSeg.control.cs = SDOUploadSegReq;
					//toggle bit will be inverted with the correct response
					SDOReqData->MsgSeg.control.t = nextToggle;
					//clear the data vector
					for(uint8_t iter = 0; iter < 7; iter++)
						SDOReqData->MsgSeg.Data[iter] = 0;
					
					requestedService = eSDOReadRequestSeg;
			
					#if(DEBUG_SDO  & DEBUG_RXMSG)
				  Serial.print("SDO: Rx Idx ");
				  Serial.print(requestedIdx, HEX);
				  Serial.print(" :");
				  Serial.println("Segmented upload response");				
				  #endif

          //try to send the data
					//if this blocks we are stuck
					//then move the request to ReadSDO
			    if(SendRequest(&SDORequestMsg))
					{
						SDORxTxState = eCO_SDOWaiting;
					  #if(DEBUG_SDO  & DEBUG_RXMSG)
						Serial.println("next segment requested");
						#endif
						RequestSentAt = actTime;
						BusyRetryCounter = 0;
					}
					else
					{
						SDORxTxState = eCO_SDORetry;
            isTimerActive = false;
					  #if(DEBUG_SDO  & DEBUG_ERROR)
						Serial.println("SDO Error: Seg Upload Request blocked! --> retry");		
            #endif						
					}					
				}	//end of case start segement
			}  // end of handling for correct Idx/Sub in response      
			else			
			{
			  SDORxTxState = eCO_SDOError;
			  #if(DEBUG_SDO  & DEBUG_ERROR)
				Serial.println("SDO: Error: wrong Idx/Sub in response!");	
        #endif				
			}					
    }  // end of handling of UploadInit response
    else if((requestedService == eSDOReadRequestSeg) && (Response->MsgSeg.control.cs == SDOUploadSegResp))	
    {
      if(Response->MsgSeg.control.t == nextToggle)
			{
				uint8_t length = 7 - Response->MsgSeg.control.n;

			  #if(DEBUG_SDO  & DEBUG_RXMSG)
				Serial.println("SDO: Segmented upload response");	
        Serial.print("Expect: ");
				Serial.print(length);
				Serial.println(" bytes");
				#endif

			  #if(DEBUG_SDO  & DEBUG_RXMSG)
				Serial.print(">>:");
				#endif
				for(uint8_t iter = 0; iter < length; iter++)
				{
				  ClientDataPtr[ActRxTxLen + iter] = Response->MsgSeg.Data[iter];
			    #if(DEBUG_SDO  & DEBUG_RXMSG)
				  Serial.print(ClientDataPtr[ActRxTxLen + iter]);
				  Serial.print("-");
					#endif
				}
			  #if(DEBUG_SDO  & DEBUG_RXMSG)
				Serial.println(".");
				#endif

				ActRxTxLen += length;
				
				if(Response->MsgSeg.control.c == 0)
				{
					//more to come
				  if(nextToggle)
					  nextToggle = 0;
				  else
					  nextToggle = 1;
					//update the toggle bit in the request
				  SDOReqData->MsgSeg.control.t = nextToggle;
			  
					//try to send the data
				  //if this blocks we are stuck
				  //then move the request to ReadSDO
			    if(SendRequest(&SDORequestMsg))
			    {
				    SDORxTxState = eCO_SDOWaiting;
						RequestSentAt = actTime;
				    BusyRetryCounter = 0;
          }
          else
				  {
					  SDORxTxState = eCO_SDORetry;
						isTimerActive = false;
				    Serial.println("SDO Error: Seg Upload Request blocked!");			
				  }
				} //end of repeated segmented request
				else
				{
				  // no more data to be received
          isTimerActive = false;
          SDORxTxState = eCO_SDODone;
				}					
			}//and of action when correct toggle received
      else
			{
			  SDORxTxState = eCO_SDOError;
				#if(DEBUG_SDO  & DEBUG_ERROR)
				Serial.println("SDO Error: wrong toggle bit in response!");			
				#endif
			}
    }
    else if((requestedService == eSDOWriteRequestExp) && (Response->MsgExp.control.cs == SDOInitDownloadResp))	
    {
			if((Response->MsgExp.Idx == requestedIdx) && (Response->MsgExp.SubIdx == requestedSub))
			{
				if(ExpectedRxTxLen <= 4)
        {
			    SDORxTxState = eCO_SDODone;
					isTimerActive = false;
					ActRxTxLen = ExpectedRxTxLen;
					
					#if(DEBUG_SDO  & DEBUG_TXMSG)
				  Serial.print("SDO: Tx Idx ");
				  Serial.print(requestedIdx, HEX);
				  Serial.print(" :");
				  Serial.println("confirmed");				
				  #endif

				}  // end of handling a response to a expedited download
        else
				{
					//calculate the length for this segment
					uint32_t length = 7;
					
					//indicate more segments to follow
					SDOReqData->MsgSeg.control.c = 0;
					//unless there are max 7 bytes left
					if(ExpectedRxTxLen < 8)
					{
						length = ExpectedRxTxLen;
						SDOReqData->MsgSeg.control.c = 1;
					}

 					#if(DEBUG_SDO  & DEBUG_TXMSG)
				  Serial.print("SDO: Tx Idx ");
				  Serial.print(requestedIdx, HEX);
				  Serial.print(" :");
				  Serial.println("Segment confirmed");				
				  #endif

					//now start the segemented download
					//compose the next request
					SDOReqData->MsgSeg.control.cs = SDODownloadSegReq;
					//toggle bit will be inverted with the correct response
					nextToggle = 0;
					SDOReqData->MsgSeg.control.t = nextToggle;
					
					//clear the contens
					for(uint8_t iter = 0; iter < SegDataLen; iter++)
						SDOReqData->MsgSeg.Data[iter] = ClientDataPtr[ActRxTxLen + iter];

					//copy the data into the message
					for(uint8_t iter = 0; iter < length; iter++)
						SDOReqData->MsgSeg.Data[iter] = ClientDataPtr[ActRxTxLen + iter];
					
					//denote the amount of date already packed
					ActRxTxLen = length;
					//adjust the service
					requestedService = eSDOWriteRequestSeg;
			
					//try to send the data
					//if this blocks we are stuck
					//then move the request to ReadSDO
			    if(SendRequest(&SDORequestMsg))
					{
						SDORxTxState = eCO_SDOWaiting;
						RequestSentAt = actTime;
						BusyRetryCounter = 0;
					}
					else
					{
						SDORxTxState = eCO_SDORetry;
						isTimerActive = false;

						#if(DEBUG_SDO  & DEBUG_ERROR)
						Serial.println("SDO Error: Seg Download Request blocked! --> retry");			
						#endif
					}					
				}	// end of handling a response to a init segmented download request
			}  // end of handling for correct Idx/Sub in response      
			else			
			{
			  SDORxTxState = eCO_SDOError;
				#if(DEBUG_SDO  & DEBUG_ERROR)
				Serial.println("SDO Error: wrong Idx/Sub in response!");	
        #endif				
			}  				
    }
    else if((requestedService == eSDOWriteRequestSeg) && (Response->MsgSeg.control.cs == SDODownloadSegResp))	
    {
		  uint32_t length = ExpectedRxTxLen - ActRxTxLen;
      
			//anything left to be tranfered?
			if(length > 0)
      {	
			  if(nextToggle)
		      nextToggle = 0;
			  else
			    nextToggle = 1;
				
				//update the toggle bit
			  SDOReqData->MsgSeg.control.t = nextToggle;
				
				if(length < 8)
				{
				  SDOReqData->MsgSeg.control.c = 1;
				}
        else
				{
				  SDOReqData->MsgSeg.control.c = 0;
	        length = 7;
				}
        SDOReqData->MsgSeg.control.n = 7-length;
				
				//clear the contens
				for(uint8_t iter = 0; iter < SegDataLen; iter++)
					SDOReqData->MsgSeg.Data[iter] = ClientDataPtr[ActRxTxLen + iter];

				//copy the data into the vector
				for(uint8_t iter = 0; iter < length; iter++)
					SDOReqData->MsgSeg.Data[iter] = ClientDataPtr[ActRxTxLen + iter];
				
				ActRxTxLen += length;
							
				//try to send the data
				//if this blocks we are stuck
				//then move the request to ReadSDO
			  if(SendRequest(&SDORequestMsg))
				{
					SDORxTxState = eCO_SDOWaiting;
					RequestSentAt = actTime;
					BusyRetryCounter = 0;
				}
				else
				{
					SDORxTxState = eCO_SDORetry;
					isTimerActive = false;
					Serial.println("SDO Error: Seg Request blocked!");			
				}
			}
			else //no more data left
			{
			  SDORxTxState = eCO_SDODone;
				isTimerActive = false;

			}	//end of case start segement
    }
	}
}

/*----------------------------------------------------
 * void SetActTime(uint32_t time)
 * Soft-Update of the internal time in case of no HW timer being used.
 * use the updated time to check whether any of the responses
 * is timed out and call the OnTimeOut() if so.
 * 
 * 2020-11-18 AW Done
 * -----------------------------------------------------------*/

void COSDOHandler::SetActTime(uint32_t time)
{	
	actTime = time;
	
	if((isTimerActive) && ((RequestSentAt + SDORespTimeOut) < actTime))	
	{	
		OnTimeOut();
		isTimerActive = false;
	}

}

/*----------------------------------------------------------
 * void OnTimeOut()
 * In case of a time-out either detected by the HW-tiemr or by the 
 * soft-timer swtich the communication either to a retry and increment
 * the retry counter or switch to final state eTimeout.
 * 
 * 2020-11-18 AW Done
 * -------------------------------------------------------------*/

void COSDOHandler::OnTimeOut()
{
	#if(DEBUG_SDO & DEBUG_TO)
	Serial.print("SDO: Timeout ");
	#endif
	
	if(TORetryCounter < TORetryMax)
	{
		SDORxTxState = eCO_SDORetry;
		TORetryCounter++;

		#if(DEBUG_SDO & DEBUG_TO)
		Serial.println("retry");
		#endif
	}
	else
	{	
		SDORxTxState = eCO_SDOTimeout;
		TORetryCounter = 0;

		#if(DEBUG_SDO & DEBUG_TO)
		Serial.println("final");
		#endif
	}
}


