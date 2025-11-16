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
 * COPDOHandler.cpp
 * implements the class to produce handle and configure the PDOs
 *
 * 2025-03-09 AW Frame
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <stdint.h>
#include <COPDOHandler.h>

//--- local defines ---

#define DEBUG_PDO_ERROR		0x0001
#define DEBUG_PDO_TXAsync	0x0002
#define DEBUG_PDO_RXSync	0x0004
#define DEBUG_PDO_TXMsg   0x0008

#define DEBUG_PDO_RX	    0x0010
#define DEBUG_PDO_Config  0x0020
#define DEBUG_PDO_Init    0x0040
#define DEBUG_PDO_BUSY    0x0080

#define DEBUG_PDO (DEBUG_PDO_ERROR | DEBUG_PDO_BUSY) 

//--- local definitions ---------

const uint16_t RxPDOTransmTypeBaseIndex =  0x1400;
const uint16_t RxPDOMappingTypeBaseIndex = 0x1600;

const uint16_t TxPDOTransmTypeBaseIndex =  0x1800;
const uint16_t TxPDOMappingTypeBaseIndex = 0x1A00;

const uint8_t PDOComSettingsSubIdxCobId = 01;
const uint8_t PDOComSettingsSubIdxTType = 02;
const uint8_t PDOComSettingsSubIdxInhTime = 03;
const uint8_t PDOComSettingsSubIdxEvtTimer = 05;

const uint32_t PDOInvalidFlag = 0x80000000;

//--- public functions ---

/*---------------------------------------------------------------------
 * COPDOHandler::COPDOHandler()
 * as of now there is noting to intialized when created
 * 
 * 2025-03-11 AW Done
 * ------------------------------------------------------------------*/

COPDOHandler::COPDOHandler()
{
	for(uint8_t iter = 0; iter < NrPDOs; iter++)
	{
	  RxPDOMapping[iter].NrEntries = 0;
	  TxPDOMapping[iter].NrEntries = 0;
		
		RxPDOSettings[iter].isValid = false;
		RxPDOSettings[iter].pending = 0;
		RxPDOSettings[iter].sentAt = 0;
		
		TxPDOSettings[iter].isValid = false;
		TxPDOSettings[iter].pending = 0;
		TxPDOSettings[iter].sentAt = 0;
		TxPDOSettings[iter].hasInhibitTime = false;
		TxPDOSettings[iter].hasEventTimer = false;
		
		
		RxPDOLength[iter] = 0;
		TxPDOLength[iter] = 0;
	}
}


/*-------------------------------------------------------------------
 * COPDOHandler::init((COMsgHandler *ThisHandler, int8_t ThisNode, int8_t MsgHandle)
 * 
 * store the pointer to the MsgHandler
 * and the local Nodeid + Handler at the MsgHandler
 * register the CB at exact this handle
 *
 * 25-03-11 AW 
 *+6
 *-------------------------------------------------------------------*/

void COPDOHandler::init(COMsgHandler *MsgHandler, CONode *MyNode, int8_t ThisNode, int8_t MsgHandle)
{
	nodeId = ThisNode;
	Handler = MsgHandler;
	Node = MyNode;
	
	PDOConfigSequenceAccessStep = 0;
	PDOConfigSingleStepAccessStep = 0;

		
	if(MsgHandle != InvalidSlot)
	{
	  //register Cb
		pfunction_holder Cb;
		
		Cb.callback = (pfunction_pointer_t)COPDOHandler::OnPdoMsgRxCb;
		Cb.op = (void *)this;

	  Handler->Register_OnRxPDOCb(MsgHandle,&Cb);
	  //PDORxTxState = eCO_PDOIdle;
		RequestState = eCO_PDOIdle;
    SDORxTxState = eCO_SDOUnknown;		
	}
	else
	{
		//PDORxTxState = eCO_PDOError;	
		RequestState = eCO_PDOError;
		
    #if (DEBUG_PDO & DEBUG_PDO_ERROR)
    Serial.print("Could not get an handle for node ");
		Serial.println(ThisNode);
		#endif
  }
	
}

/*-------------------------------------------------------------------
 * COPDOHandler::RegisterSDOHandler(COSDOHandler *SDOHandler)
 * 
 * store the pointer to the SDOHandler
 *
 * 25-03-11 AW 
 *
 *-------------------------------------------------------------------*/

void COPDOHandler::RegisterSDOHandler(COSDOHandler *SDOHandler)
{
	RWSDO = SDOHandler;
}


/*----------------------------------------------
 * void PDOHandler::GetSDOComState()
 * check the state of the used SDOHandler
 *
 * 2025-03-14 AW inital
 * ---------------------------------------------*/

COSDOCommStates COPDOHandler::GetSDOComState()
{
  Node->RWSDO.GetComState();	
}


/*----------------------------------------------
 * void PDOHandler::ResetComState()
 * to be called only if the PDO Handler was stuck
 *
 * 2025-03-14 AW inital
 * ---------------------------------------------*/

void COPDOHandler::ResetSDOState()
{
  Node->RWSDO.ResetComState();
}
 
/*----------------------------------------------
 * void PDOHandler::ResetComState()
 * to be called after each interaction to 
 * move the PDORxTxState from eDone to eIdle
 * 
 * 2025-03-14 AW inital
 * ---------------------------------------------*/

void COPDOHandler::ResetComState()
{
	//PDORxTxState = eCO_PDOIdle;
	RequestState = eCO_PDOIdle;

	TORetryCounter = 0;
	BusyRetryCounter = 0;
	
	PDOConfigSequenceAccessStep = 0;
	PDOConfigSingleStepAccessStep = 0;

	//Handler should not be reset, as it could be used by different
	//instances of the Drive
}


/*----------------------------------------------
 * void COPDOHandler::SetPDOConfigTimeout(uint32_t value)
 * allow for longer response times for slow devices
 * 
 * 2025-11-11 AW
 * ---------------------------------------------*/
void COPDOHandler::SetPDOConfigTimeout(uint32_t value)
{
  PDOConfigTimeout = value;
}	

/*----------------------------------------------
 * void COPDOHandler::FlagPDOsInvalid()
 * to be called when a bbot msg was received to flag all
 * the pods to be unconfigured
 * 
 * 2025-07-27 AW inital
 * ---------------------------------------------*/

void COPDOHandler::FlagPDOsInvalid()
{
	PDOsConfigured = 0;
}

/*--------------------------------------------------------------
 * void SetTORetryMax(uint8_t)
 * An attempted transfer can run into a timeout either while trying to send
 * or by waiting for a response. The Time-out counter is incremented if 
 * a consecutive occurs. the call here cen be used to modify the
 * default max value for the number of time-outs in a row
 * 
 * 2025-03-14 AW inital
 * --------------------------------------------------------------*/ 

void COPDOHandler::SetTORetryMax(uint8_t value)
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
 * 2025-03-14 AW done
 * --------------------------------------------------------------*/
 
void COPDOHandler::SetBusyRetryMax(uint8_t value)
{
	BusyRetryMax = value;
}
	
/*--------------------------------------------------------------
 * void PresetRxPDOTransmission(uint8_t PDONr, uint8_t TransmType)
 *
 * store the tranmission type in the requested PDO settings
 * will not be transferred now
 * paramters are the PDO# and the transmission type
 * 
 * 2025-03-14 AW frame
 * --------------------------------------------------------------*/

void COPDOHandler::PresetRxPDOTransmission(uint8_t PDONr, uint8_t TransmType)
{	
	RxPDOSettings[PDONr].TransmType = TransmType;
	switch(PDONr)
	{
		case 0:
	    RxPDOSettings[PDONr].COBId = (uint16_t)eCANRPDO1 | nodeId;
		  break;
		case 1:
	    RxPDOSettings[PDONr].COBId = (uint16_t)eCANRPDO2 | nodeId;
		  break;
    case 2:
	    RxPDOSettings[PDONr].COBId = (uint16_t)eCANRPDO3 | nodeId;
		  break;
    case 3:
	    RxPDOSettings[PDONr].COBId = (uint16_t)eCANRPDO4 | nodeId;
		  break;
	}		
  #if (DEBUG_PDO & DEBUG_PDO_Config) 
	Serial.print("PDO: Preset Rx #");
	Serial.print(PDONr);
	Serial.print(" with Id ");
	Serial.print(RxPDOSettings[PDONr].COBId,HEX);
	Serial.print(" T-type: ");
	Serial.println(RxPDOSettings[PDONr].TransmType);
	#endif
}

/*--------------------------------------------------------------
 * void PresetTxPDOTransmission(uint8_t PDONr, uint8_t TransmType, uint16_t inhibitTime, uint16_t EvtTimer)
 *
 * store the tranmission type in the requested PDO settings
 * will not be transferred now
 * paramters are the PDO#, the transmission type, the inhibit time and the EventTimer
 *
 * inihibit time nor supported than uesed so far
 * 
 * 2025-03-14 AW frame
 * 2025-10-03 AW add handling of EvtTimer and inhibit time
 * --------------------------------------------------------------*/

void COPDOHandler::PresetTxPDOTransmission(uint8_t PDONr, uint8_t TransmType, uint16_t InhibitTime, uint16_t EvtTimer)
{
	TxPDOSettings[PDONr].TransmType = TransmType;
	TxPDOSettings[PDONr].inhibitTime = InhibitTime;
	TxPDOSettings[PDONr].eventTimer = EvtTimer;	

	if(InhibitTime > 0)
		TxPDOSettings[PDONr].hasInhibitTime = true;
	if(EvtTimer > 0)
		TxPDOSettings[PDONr].hasEventTimer = true;
	
	switch(PDONr)
	{
		case 0:
	    TxPDOSettings[PDONr].COBId = (uint16_t)eCANTPDO1 | nodeId;
		  break;
		case 1:
	    TxPDOSettings[PDONr].COBId = (uint16_t)eCANTPDO2  | nodeId;
		  break;
    case 2:
	    TxPDOSettings[PDONr].COBId = (uint16_t)eCANTPDO3 | nodeId;
		  break;
    case 3:
	    TxPDOSettings[PDONr].COBId = (uint16_t)eCANTPDO4 | nodeId;
		  break;
	}			
  #if (DEBUG_PDO & DEBUG_PDO_Config) 
	Serial.print("PDO: Preset Tx #");
	Serial.print(PDONr);
	Serial.print(" with Id ");
	Serial.print(TxPDOSettings[PDONr].COBId,HEX);
	Serial.print(" T-type: ");
	Serial.println(TxPDOSettings[PDONr].TransmType);
  #endif
}
	
/*--------------------------------------------------------------
 * void PresetRxPDOMapping(uint8_t PDONr, unit8_t NrEntries, ODEntry **Entries)
 *
 * store the the mapping entires of a single PDO
 * will not be transferred now
 * paramters are the PDO#, number of active entries and a vector of ODEntries
 * 
 * 2025-03-14 AW frame
 * --------------------------------------------------------------*/

void COPDOHandler::PresetRxPDOMapping(uint8_t PDONr, uint8_t NrEntries, ODEntry **Entries)
{
	RxPDOMapping[PDONr].NrEntries = NrEntries;
	for(uint8_t iter = 0; iter < NrEntries; iter++)
	{
	  RxPDOMapping[PDONr].Entries[iter] = Entries[iter];
		RxPDOLength[PDONr] += (Entries[iter])->len;
	}

  #if (DEBUG_PDO & DEBUG_PDO_Config) 
	Serial.print("PDO: Preset Rx #");
	Serial.print(PDONr);
	Serial.print(" with # ");
	Serial.print(RxPDOMapping[PDONr].NrEntries);
	Serial.print(" entries #");
	Serial.print(RxPDOLength[PDONr]);
	Serial.println(" bytes");
  #endif
}

/*--------------------------------------------------------------
 * void PresetTxPDOMapping(uint8_t PDONr, unit8_t NrEntries, ODEntry **Entries)
 *
 * store the the mapping entires of a single PDO
 * will not be transferred now
 * paramters are the PDO#, number of active entries and a vector of ODEntries
 * 
 * 2025-03-14 AW frame
 * --------------------------------------------------------------*/

void COPDOHandler::PresetTxPDOMapping(uint8_t PDONr, uint8_t NrEntries, ODEntry **Entries)
{
	TxPDOMapping[PDONr].NrEntries = NrEntries;
	for(uint8_t iter = 0; iter < NrEntries; iter++)
	{
	  TxPDOMapping[PDONr].Entries[iter] = Entries[iter];
		TxPDOLength[PDONr] += (Entries[iter])->len;
	}

  #if (DEBUG_PDO & DEBUG_PDO_Config) 
	Serial.print("PDO: Preset Tx #");
	Serial.print(PDONr);
	Serial.print(" with # ");
	Serial.print(TxPDOMapping[PDONr].NrEntries);
	Serial.print(" entries #");
	Serial.print(RxPDOLength[PDONr]);
	Serial.println(" bytes");
  #endif
}

/*--------------------------------------------------------------
 * void FlagRxPDOvalid(uint8_t PDONr)
 *
 * flag the PDO to be ready for use by setting it valid
 * 
 * 2025-07-08 AW implemented
 * --------------------------------------------------------------*/

void COPDOHandler::PresetRxPDOisValid(uint8_t PdoNr,bool isValid)
{
	RxPDOSettings[PdoNr].isValid = isValid;
}

/*--------------------------------------------------------------
 * void FlagTxPDOvalid(uint8_t PDONr)
 *
 * flag the PDO to be ready for use by setting it valid
 * 
 * 2025-07-08 AW implemented
 * --------------------------------------------------------------*/

void COPDOHandler::PresetTxPDOisValid(uint8_t PdoNr,bool isValid)
{
	TxPDOSettings[PdoNr].isValid = isValid;
}

/*--------------------------------------------------------------
 * COPDOCommStates COPDOHandler::ConfigurePresetPDOs(uint32_t)
 *
 * configure the preset PDOs at the remote node
 * 
 * 2025-04-26 AW implementation
 * --------------------------------------------------------------*/

COPDOCommStates COPDOHandler::ConfigurePresetPDOs(uint32_t timestamp)
{
	COPDOCommStates PDOAccessState;
	COPDOCommStates returnValue = eCO_PDOBusy;	
	
	//first configure the Rx
	if(PDOsConfigured < NrPDOs)
	{
	  if((PDOAccessState = ConfigureRxTxPDO(PDOsConfigured, eCO_PDORx, timestamp)) == eCO_PDODone)
		  PDOsConfigured++;
  }
	else if(PDOsConfigured == (2 * NrPDOs))
	{
	  returnValue = eCO_PDODone;			
	}
	else
	{
	  if((PDOAccessState = ConfigureRxTxPDO(PDOsConfigured - NrPDOs, eCO_PDOTx, timestamp)) == eCO_PDODone)
		  PDOsConfigured++;
	}
	
  return returnValue;	
}
	
/*--------------------------------------------------------------
 * COPDOCommStates COPDOHandler::Update(uint32_t)
 *
 * update the timestamp and return the current state
 * to be called after the Pre-Op
 * 
 * 2025-04-26 AW implementation
 * --------------------------------------------------------------*/

COPDOCommStates COPDOHandler::Update(uint32_t time, COSyncState synchState)
{
  actTime = time;
	
	//check for sync ones
	if(synchState == eSyncSyncSent)
	{
	  for(uint8_t iterPDO = 0; iterPDO < NrPDOs; iterPDO++)
	  {
			//handling for Sync but non 1 is missing
	    if((RxPDOSettings[iterPDO].TransmType == 1))
			{
				//add this one to the list for sending
			  RxPDOSettings[iterPDO].pending = true;
			}
		}
	}
	//now check the PDO one in focus in this turn for being pendig
	if(RxPDOSettings[nextTx].pending > 0)
	{
		//if it is pending we try to send
		//will only proceed with the next in list, when Tx did work
		
		if(TransmitPdo(nextTx))
		{
		  RxPDOSettings[nextTx].sentAt = actTime;
      //several source can falg this one for Tx
		  if(RxPDOSettings[nextTx].pending > 0)
			{
			  RxPDOSettings[nextTx].pending--;
			}
	
			nextTx++;			
			if(nextTx == NrPDOs)
			{
				nextTx = 0;
			}
		}
	}
	else
	{
		//not pendig, check the next
		nextTx++;			
		if(nextTx == NrPDOs)
		{
			nextTx = 0;
		}
	}
	return RequestState;
}

/*--------------------------------------------------------------
 * COPDOCommStates ModifyRxPDOTransmission(uint8_t PDONr, uint8_t TransmType)
 *
 * store the tranmission type in the requested PDO settings
 * and modify the PDO settings directly
 * paramters are the PDO# and the transmission type
 * 
 * 2025-03-14 AW frame
 * --------------------------------------------------------------*/

COPDOCommStates COPDOHandler::ModifyRxPDOTransmission(uint8_t PDONr, uint8_t TransmType)
{
	//should reconfigure the transmission type setting for the given RxPDO
	//1st: set the PDO to invalid
	//2nd: write the new setting
	//3rd: set the PDO active again
	; 
}

/*--------------------------------------------------------------
 * COPDOCommStates ModifyTxPDOTransmission(uint8_t PDONr, uint8_t TransmType)
 *
 * store the tranmission type in the requested PDO settings
 * and modify the PDO settings directly
 * paramters are the PDO#, the transmission type and an inhibit time
 * 
 * 2025-03-14 AW frame
 * --------------------------------------------------------------*/

COPDOCommStates COPDOHandler::ModifyTxPDOTransmission(uint8_t PDONr, uint8_t TransmType, uint16_t InhibitTime)
{
	//should reconfigure the transmission type setting for the given TxPDO
	//1st: set the PDO to invalid
	//2nd: write the new setting
	//3rd: set the PDO active again
	;
}

/*--------------------------------------------------------------
 * COPDOCommStates ModifyRxPDOMapping((unit8_t PDONr, uint8_t NrEntries, ODEntry **Entries)
 *
 * store the tranmission type in the requested PDO settings
 * and modify the PDO settings directly
 * paramters are the PDO#, number of active entries and a vector of ODEntries
 * 
 * 2025-03-14 AW frame
 * --------------------------------------------------------------*/

COPDOCommStates COPDOHandler::ModifyRxPDOMapping(uint8_t PDONr, uint8_t NrEntries, ODEntry **Entries)
{
	//should reconfigure the contents of the given RxPDO
	//1st: set the PDO to invalid
	//2nd: entry by entry download new mappings and update the nr of mapped onjects
	//3rd: set the PDO active again
	;
}

/*--------------------------------------------------------------
 * COPDOCommStates ModifyTxPDOMapping((unit8_t PDON, uint8_t NrEntries, ODEntry **Entries)
 *
 * store the tranmission type in the requested PDO settings
 * and modify the PDO settings directly
 * paramters are the PDO#, number of active entries and a vector of ODEntries
 * 
 * 2025-03-14 AW frame
 * --------------------------------------------------------------*/

COPDOCommStates COPDOHandler::ModifyTxPDOMapping(uint8_t PDONr, uint8_t NrEntries, ODEntry **Entries)
{
	//should reconfigure the contents of the given TxPDO
	//1st: set the PDO to invalid
	//2nd: entry by entry download new mappings and update the nr of mapped onjects
	//3rd: set the PDO active again
	;
}	
			
/*--------------------------------------------------------------
 * COPDOCommStates COPDOHandler::ConfigureRxTxPDO(uint8_t PdoNr, PDODir Dir, uint32_t time)
 *
 * configure a single preset PDO at the remote node
 * 
 * 2025-04-26 AW implementation
 * --------------------------------------------------------------*/

COPDOCommStates COPDOHandler::ConfigureRxTxPDO(uint8_t PdoNr, PDODir Dir, uint32_t time)
{
  COPDOCommStates StepState;
	COPDOCommStates returnValue = eCO_PDOBusy;
	
  actTime = time;
	
	switch(PDOConfigSequenceAccessStep)
	{
		case 0:
		  //set the PDO invalid
		  if(Dir == eCO_PDORx)
			  StepState = SetRxPDOInvalid(PdoNr);
		  else
			  StepState = SetTxPDOInvalid(PdoNr);
		  break;
		case 1:
			//write the mapping
		  if(Dir == eCO_PDORx)
			  StepState = WriteRxPDOMapping(PdoNr);
		  else
			  StepState = WriteTxPDOMapping(PdoNr);
		  break;
		case 2:
      //set the PDO valid again
		  if(Dir == eCO_PDORx)
			  StepState = SetRxPDOValid(PdoNr);
		  else
			  StepState = SetTxPDOValid(PdoNr);
		  break;
		case 3:
			//we are done
	    returnValue = eCO_PDODone;
		  PDOConfigSequenceAccessStep = 0;
      #if (DEBUG_PDO & DEBUG_PDO_Init) 
		  Serial.print("PDO Config RxTx # ");
		  Serial.print(PdoNr);
		  Serial.println("completed");
		  #endif
		  break;
		default:
			//??
		  break;
	}
	
  //check the result
	if(StepState == eCO_PDODone)
	{
	  PDOConfigSequenceAccessStep++;
		PDOConfigSingleStepAccessStep = 0;
    //reset the SDO Com State as we checked on this

    #if (DEBUG_PDO & DEBUG_PDO_Init) 
		Serial.print("PDO Config RxTx # ");
		Serial.print(PdoNr);
		Serial.print(" done. Next: ");
		Serial.println(PDOConfigSequenceAccessStep);
		#endif
	}
  else if(StepState == eCO_PDOError)
	{
    returnValue = eCO_PDOError;
		
		#if (DEBUG_PDO & DEBUG_PDO_Init) 
		Serial.print("PDO Config RxTx # ");
		Serial.print(PdoNr);
		Serial.println(" has an error. Stopped!");
		#endif
	}	
	if((actTime - RequestSentAt) > PDOConfigTimeout)
	  returnValue = eCO_PDOError;	
  
	return returnValue;
}

/*--------------------------------------------------------------
 * COPDOCommStates COPDOHandler::TxPDOsAsync(ODEntry *entry)
 *
 * check whether entry is mapped into a RxPDO and triger its transmission if async
 * to have it transmitted it gets entered into a list of PDOs to be sent for this node
 * 
 * 2025-07-12 AW frame
 * --------------------------------------------------------------*/

bool COPDOHandler::TxPDOsAsync(ODEntry *entry)
{
	bool returnValue = false;
	
  for(uint8_t iterPDO = 0; iterPDO < NrPDOs; iterPDO++)
	{
		#if (DEBUG_PDO & DEBUG_PDO_TXAsync)
		Serial.print("PDO: Check RxPDO");
		Serial.println(iterPDO+1);
		#endif
		
		//now check every mapped entry
		for(uint8_t iterEntry = 0; iterEntry < RxPDOMapping[iterPDO].NrEntries; iterEntry++)
		{
			#if (DEBUG_PDO & DEBUG_PDO_TXAsync)
			Serial.print("PDO: Check Entry ");
			Serial.print(iterEntry);
			Serial.print(":");
			Serial.println((RxPDOMapping[iterPDO].Entries[iterEntry])->Idx, HEX);
			#endif
			if(RxPDOMapping[iterPDO].Entries[iterEntry] == entry)
			{
				//add this one to the list for sending
				//only PDOs configured to be asyc get flagged
		    if(RxPDOSettings[iterPDO].TransmType == TPDOTTypeAsync)
				  RxPDOSettings[iterPDO].pending++;
				
				returnValue = true;
				
				#if (DEBUG_PDO & DEBUG_PDO_TXAsync)
				Serial.print("PDO: will Tx RxPDO");
				Serial.println(iterPDO+1);  
				#endif
			}
		}
	}
	return returnValue;
}

/*--------------------------------------------------------------
 * bool COPDOHandler::RxPDOIsSync(ODEntry *entry)
 *
 * check whether entry is mapped into a sync TxPDO 
 * 
 * 2025-11-16 AW frame
 * --------------------------------------------------------------*/

bool COPDOHandler::RxPDOIsSync(ODEntry *entry)
{
	bool returnValue = false;
	
  for(uint8_t iterPDO = 0; iterPDO < NrPDOs; iterPDO++)
	{
		#if (DEBUG_PDO & DEBUG_PDO_RXSync)
		Serial.print("PDO: Check TxPDO");
		Serial.println(iterPDO+1);
		#endif
		
		//now check every mapped entry
		for(uint8_t iterEntry = 0; iterEntry < TxPDOMapping[iterPDO].NrEntries; iterEntry++)
		{
			#if (DEBUG_PDO & DEBUG_PDO_RXSync)
			Serial.print("PDO: Check Entry ");
			Serial.print(iterEntry);
			Serial.print(":");
			Serial.println((TxPDOMapping[iterPDO].Entries[iterEntry])->Idx, HEX);
			#endif
			if(TxPDOMapping[iterPDO].Entries[iterEntry] == entry)
			{				
				returnValue = true;
				
				#if (DEBUG_PDO & DEBUG_PDO_RXSync)
				Serial.print("PDO: will be Rx sync");
				Serial.println(iterPDO+1);  
				#endif
			}
		}
	}
	return returnValue;
}
	
// - private functions ---
	
/*-------------------------------------------------------------------
 * bool COPDOHandler::SendRequest(CANMsg *)
 * 
 * proess and send a CANMsg
 * enter a retry when momentarily blocked
 * 
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/
		
bool COPDOHandler::SendRequest(CANMsg *Msg)
{
	bool result = Handler->SendMsg(Msg);	
	
	if(result)
	{
		//if we were able to send, the service is done
	  RequestState = eCO_PDOIdle;
		BusyRetryCounter = 0;				

		#if(DEBUG_PDO & DEBUG_PDO_TXMsg)
		Serial.print("PDO: TX ");
		Serial.print(Msg->Id,HEX);
		Serial.print(" len:");
		Serial.println(Msg->len);
		#endif
	}
	else
	{
		BusyRetryCounter++;
		if(BusyRetryCounter > BusyRetryMax)
		{
			RequestState = eCO_PDOError;

		  #if(DEBUG_PDO & DEBUG_PDO_ERROR)
		  Serial.print("PDO: TX ");
			Serial.print(Msg->Id,HEX);
			Serial.println(" TxReq failed");
			#endif
		}
		else
		{
			RequestState = eCO_PDORetry;
		  
		  #if(DEBUG_PDO & DEBUG_PDO_BUSY)
		  Serial.print("PDO: TX ");
			Serial.print(Msg->Id,HEX);
			Serial.println(" TxReq busy, retry");
			#endif
		}
	}
	return result;
}


/*-------------------------------------------------------------------
 * COPDOCommStates COPDOHandler::SetRxPDOInvalid(uint8_t PdoNr)
 * 
 * set this RxPDO to invalid
 * uses the CobId stored in the master
 * is intended to be called cyclically until the SDO access reports eCO_SDODone
 * 
 * 25-04-27 AW 
 *
 *-------------------------------------------------------------------*/

COPDOCommStates COPDOHandler::SetRxPDOInvalid(uint8_t PdoNr)
{
	uint32_t ObjValue = RxPDOSettings[PdoNr].COBId | PDOInvalidFlag;
	uint32_t ObjLen = 4;
	
  return WriteObject(RxPDOTransmTypeBaseIndex + PdoNr,PDOComSettingsSubIdxCobId, &ObjValue, ObjLen);	
}

/*-------------------------------------------------------------------
 * COPDOCommStates COPDOHandler::SetTxPDOInvalid(uint8_t PdoNr)
 * 
 * set this TxPDO to invalid
 * uses the CobId stored in the master
 * is intended to be called cyclically until the SDO access reports eCO_SDODone
 * PdoNr is valid starting from 0
 * 
 * 25-04-27 AW 
 *
 *-------------------------------------------------------------------*/

COPDOCommStates COPDOHandler::SetTxPDOInvalid(uint8_t PdoNr)
{
	uint32_t ObjValue = TxPDOSettings[PdoNr].COBId | PDOInvalidFlag;
	uint32_t ObjLen = 4;

  return WriteObject(TxPDOTransmTypeBaseIndex + PdoNr,PDOComSettingsSubIdxCobId, &ObjValue, ObjLen);	
}

/*-------------------------------------------------------------------
 * COPDOCommStates COPDOHandler::SetRxPDOValid(uint8_t PdoNr)
 * 
 * set this RxPDO to valid state
 * does not check the mapping stored in the node
 * uses the CobId stored in the master
 * is intended to be called cyclically until the SDO access reports eCO_SDODone
 * 
 * 25-04-27 AW 
 *
 *-------------------------------------------------------------------*/

COPDOCommStates COPDOHandler::SetRxPDOValid(uint8_t PdoNr)
{
	uint32_t ObjValue = RxPDOSettings[PdoNr].COBId;
	uint32_t ObjLen = 4;

	//todo: will I have to re-use the isValid for internal purposes?
	if((RxPDOMapping[PdoNr].NrEntries == 0) || (!(RxPDOSettings[PdoNr].isValid)))
	  ObjValue = ObjValue | PDOInvalidFlag;	
	
  return WriteObject(RxPDOTransmTypeBaseIndex + PdoNr,PDOComSettingsSubIdxCobId, &ObjValue, ObjLen);	
}

/*-------------------------------------------------------------------
 * COPDOCommStates COPDOHandler::SetTxPDOValid(uint8_t PdoNr)
 * 
 * set this TxPDO to valid state
 * does not check the mapping stored in the node
 * uses the CobId stored in the master
 * is intended to be called cyclically until the SDO access reports eCO_SDODone
 * 
 * 25-04-27 AW 
 *
 *-------------------------------------------------------------------*/

COPDOCommStates COPDOHandler::SetTxPDOValid(uint8_t PdoNr)
{
	uint32_t ObjValue = TxPDOSettings[PdoNr].COBId;
	uint32_t ObjLen = 4;
	
	//todo: will I have to re-use the isValid for internal purposes?
	if((TxPDOMapping[PdoNr].NrEntries == 0) || (!(TxPDOSettings[PdoNr].isValid)))
	  ObjValue = ObjValue | PDOInvalidFlag;	
					
  return WriteObject(TxPDOTransmTypeBaseIndex + PdoNr,PDOComSettingsSubIdxCobId, &ObjValue, ObjLen);	
}
	
/*-------------------------------------------------------------------
 * COPDOCommStates COPDOHandler::WriteRxPDOMapping(uint8_t PdoNr)
 * 
 * write the locally stored mappings to the referred PDO
 * has to reset the number of mapped entires to 0 first
 * then write the valid mappings
 * finally write the number of mapped objects
 * is intended to be called cyclically until the final SDO access reports eCO_SDODone
 * 
 * 25-04-27 AW 
 *
 *-------------------------------------------------------------------*/

COPDOCommStates COPDOHandler::WriteRxPDOMapping(uint8_t PdoNr)
{
  //the return value of this method
	COPDOCommStates returnValue = eCO_PDOBusy;
	bool doTransmit = true;

	uint16_t ObjIdx;
	uint8_t ObjSubIdx;
	uint32_t ObjValue;
	uint32_t ObjLen;
  
	switch(PDOConfigSingleStepAccessStep)
	{
	  case 0:
		  //set the number of mapped objects to 0
			ObjValue = 0;
			ObjLen = 1;
		  ObjIdx = RxPDOMappingTypeBaseIndex + PdoNr;
		  ObjSubIdx = 0;
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		{
			uint8_t entry = PDOConfigSingleStepAccessStep - 1;
			if(RxPDOMapping[PdoNr].Entries[entry] != NULL)
			{
			  ObjValue = (RxPDOMapping[PdoNr].Entries[entry]->Idx << 16) | 
									 (RxPDOMapping[PdoNr].Entries[entry]->SubIdx << 8) |
									 (RxPDOMapping[PdoNr].Entries[entry]->len * 8);
				ObjLen = 4;
			}
			else
				//skip this access
				doTransmit = false;		
			
		  ObjIdx = RxPDOMappingTypeBaseIndex + PdoNr;
		  ObjSubIdx = PDOConfigSingleStepAccessStep;
			break;
		}
		case 5:
			//set the number of mepped objects to the actual one
			ObjValue = RxPDOMapping[PdoNr].NrEntries;
			ObjLen = 1;
		  ObjIdx = RxPDOMappingTypeBaseIndex + PdoNr;
		  ObjSubIdx = 0;
			break;
		case 6:
			//set the trasnmission type
			ObjValue = RxPDOSettings[PdoNr].TransmType;
			ObjLen = 1;
		  ObjIdx = RxPDOTransmTypeBaseIndex + PdoNr;
		  ObjSubIdx = PDOComSettingsSubIdxTType;
			break;
		case 7:
			//we are done with this one
			PDOConfigSingleStepAccessStep = 0;
			returnValue = eCO_PDODone;	
		  doTransmit = false;
		
		  #if (DEBUG_PDO & DEBUG_PDO_Init) 
		  Serial.print("RxPDO config # ");
			Serial.print(PdoNr);
			Serial.println(" complete --> valid ");
		  #endif
		
      break;
		default:
			break;
	}
  if(doTransmit)
	{
	  COPDOCommStates StepState = WriteObject(ObjIdx,ObjSubIdx, &ObjValue, ObjLen);	
		switch(StepState)
		{
			case eCO_PDODone:
		    #if (DEBUG_PDO & DEBUG_PDO_Init) 
	      Serial.print("RxPDO config # ");
			  Serial.print(PdoNr);
			  Serial.print(" step ");
			  Serial.print(PDOConfigSingleStepAccessStep);
			  Serial.println(" done");
			  #endif
			
        PDOConfigSingleStepAccessStep++;
			  Node->RWSDO.ResetComState(); 
			  //SDORxTxState = eCO_SDOIdle;
			  break;
			case eCO_PDOError:
			  returnValue = eCO_PDOError;
        break;
      default:
        // the busy case
        break;			
		}
	}
	else
	{
		//no transmission
	  //step forward directly
	  PDOConfigSingleStepAccessStep++;
	}
	return returnValue;			
}

/*-------------------------------------------------------------------
 * COPDOCommStates COPDOHandler::WriteTxPDOMapping(uint8_t PdoNr)
 * 
 * write the locally stored mappings to the referred PDO
 * has to reset the number of mapped entires to 0 first
 * then write the valid mappings
 * finally write the number of mapped objects
 * is intended to be called cyclically until the final SDO access reports eCO_SDODone
 * 
 * 25-04-27 AW 
 *
 *-------------------------------------------------------------------*/

COPDOCommStates COPDOHandler::WriteTxPDOMapping(uint8_t PdoNr)
{
  //the return value of this method
	COPDOCommStates returnValue = eCO_PDOBusy;
	bool doTransmit = true;

	uint16_t ObjIdx;
	uint8_t ObjSubIdx;
	uint32_t ObjValue;
	uint32_t ObjLen;
  
	switch(PDOConfigSingleStepAccessStep)
	{
	  case 0:
		  //set the number of mapped objects to 0
			ObjValue = 0;
			ObjLen = 1;
		  ObjIdx = TxPDOMappingTypeBaseIndex + PdoNr;
		  ObjSubIdx = 0;
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		{
			uint8_t entry = PDOConfigSingleStepAccessStep - 1;
			if(TxPDOMapping[PdoNr].Entries[entry] != NULL)
			{
			  ObjValue = (TxPDOMapping[PdoNr].Entries[entry]->Idx << 16) | 
									 (TxPDOMapping[PdoNr].Entries[entry]->SubIdx << 8) |
									 (TxPDOMapping[PdoNr].Entries[entry]->len * 8);
				ObjLen = 4;
			}
			else
				//skip this access
				doTransmit = false;		
			
		  ObjIdx = TxPDOMappingTypeBaseIndex + PdoNr;
		  ObjSubIdx = PDOConfigSingleStepAccessStep;
			break;
		}
		case 5:
			//set the number of mepped objects to 0
			ObjValue = TxPDOMapping[PdoNr].NrEntries;
			ObjLen = 1;
		  ObjIdx = TxPDOMappingTypeBaseIndex + PdoNr;
		  ObjSubIdx = 0;
			break;
		case 6:
			//set the trasnmission type
			ObjValue = TxPDOSettings[PdoNr].TransmType;
			ObjLen = 1;
		  ObjIdx = TxPDOTransmTypeBaseIndex + PdoNr;
		  ObjSubIdx = PDOComSettingsSubIdxTType;
			break;
		case 7:
			//inhibit time - only if supported
			ObjValue = TxPDOSettings[PdoNr].inhibitTime;
			ObjLen = 2;
		  ObjIdx = TxPDOTransmTypeBaseIndex + PdoNr;
		  ObjSubIdx = PDOComSettingsSubIdxInhTime;

		  if(TxPDOSettings[PdoNr].hasInhibitTime == false)
				  doTransmit = false;
			
			break;
		case 8:
			//event timer - only if supported
			ObjValue = TxPDOSettings[PdoNr].eventTimer;
			ObjLen = 2;
		  ObjIdx = TxPDOTransmTypeBaseIndex + PdoNr;
		  ObjSubIdx = PDOComSettingsSubIdxEvtTimer;

		  if(TxPDOSettings[PdoNr].hasEventTimer == false)
				  doTransmit = false;
			
			break;
		case 9:
			//we are done with this one
			PDOConfigSingleStepAccessStep = 0;
			returnValue = eCO_PDODone;	
		  doTransmit = false;
		
		  #if (DEBUG_PDO & DEBUG_PDO_Init) 
		  Serial.print("TxPDO config # ");
			Serial.print(PdoNr);
			Serial.println(" complete --> valid");
		  #endif
		
    default:
			break;
	}
  if(doTransmit)
	{
	  COPDOCommStates StepState = WriteObject(ObjIdx,ObjSubIdx, &ObjValue, ObjLen);	
		switch(StepState)
		{
			case eCO_PDODone:
		    #if (DEBUG_PDO & DEBUG_PDO_Init) 
    		Serial.print("TxPDO Mapping # ");
			  Serial.print(PdoNr);
			  Serial.print(" step ");
			  Serial.print(PDOConfigSingleStepAccessStep);
			  Serial.println(" done");
        #endif
			
        PDOConfigSingleStepAccessStep++;
			  //SDORxTxState = eCO_SDOIdle;
			  break;
			case eCO_PDOError:
			  returnValue = eCO_PDOError;
        break;
      default:
        // the busy case
        break;			
		}
	}
	else
	{
		//no transmission
	  //step forward directly
	  PDOConfigSingleStepAccessStep++;
	}
	return returnValue;			
}

/*-------------------------------------------------------------------
 * COPDOCommStates COPDOHandler::WriteObject(uint16_t Idx, uint8_t SubIdx, uint32_t *ObjValue, uint32_t ObjLen)
 * 
 * write to an entry for PDO config
 * is intended to be called cyclically until the final SDO access reports eCO_SDODone
 * 
 * 25-04-27 AW 
 *
 *-------------------------------------------------------------------*/

COPDOCommStates COPDOHandler::WriteObject(uint16_t Idx, uint8_t SubIdx, uint32_t *ObjValue, uint32_t ObjLen)
{
	COPDOCommStates returnValue = eCO_PDOBusy;
  ODEntry32 Object = {Idx, SubIdx, ObjValue, ObjLen};
	
	switch(SDORxTxState)
	{
		case eCO_SDOUnknown:
			Node->RWSDO.ResetComState(); 
			SDORxTxState = eCO_SDOIdle;
		  //no break necessary here
		case eCO_SDOIdle:
		case eCO_SDORetry:
			RequestSentAt = actTime;
		  #if (DEBUG_PDO & DEBUG_PDO_Init) 
		  Serial.print("PDO: write to ");
		  Serial.print(Idx,HEX),
		  Serial.print(" ");
		  Serial.print(SubIdx, HEX);
		  Serial.print(":");
		  Serial.println(*ObjValue,HEX);
		  #endif
		  //no break here
		case eCO_SDOWaiting:
		case eCO_SDOBusy:
			//the object to be written is the 0x1600 for TxPDO1, ...
		  //todo: check whether ObjValue and ObjLenght would have to be class members
			SDORxTxState = Node->RWSDO.WriteSDO((ODEntry *)&Object);
		  #if (DEBUG_PDO & DEBUG_PDO_Init) 
  		Serial.print(".. returns ");
		  Serial.println(SDORxTxState);
		  #endif
		
			break;
		case eCO_SDODone:
			//access was successful
			//reset the SDO state and write the next object
			SDORxTxState = eCO_SDOIdle;
		  returnValue = eCO_PDODone;
			break;
		default:
	    returnValue = eCO_PDOError;	
			break;
	} //end of the configuration of a single RxPDO		
				
	if((actTime - RequestSentAt) > PDOConfigTimeout)
	  returnValue = eCO_PDOError;	
		
	return returnValue;
}

/*-------------------------------------------------------------------
 * bool COPDOHandler::TransmitPdo(uint8_t PdoNr);
 *
 * transmit a single RxPDO identified by its index
 * needs to have a step sequence to have the chance to send until it's done 
 *
 * 25-07-06 AW frame added 
 *
 *-------------------------------------------------------------------*/

bool COPDOHandler::TransmitPdo(uint8_t PdoNr)
{
	bool returnValue = false;
	
	//write to the TxPDO structure only when PDORxTxState == eCO_PDOIdle
	//indicating the last SendRequest being closed
	//otherwise simply re-trigger the last one
	if(RequestState == eCO_PDOIdle)
	{
	  uint8_t writeIdx = 0;

		//add Id and service type and rtr
		#if(DEBUG_PDO & DEBUG_PDO_TXMsg)
		Serial.print("PDO: Tx RxPDO");
		Serial.print(PdoNr+1);
		Serial.print(": ");
    #endif
		
		//fill in the data	
		if((RxPDOSettings[PdoNr].isValid) &&(RxPDOMapping[PdoNr].NrEntries > 0))
		{
			//check the mapping 
			for(uint8_t iter = 0; iter < RxPDOMapping[PdoNr].NrEntries; iter++)
			{
				#if(DEBUG_PDO & DEBUG_PDO_TXMsg)
    		Serial.print(iter);
				Serial.print(": ");
				#endif
				//copy data according to length into a temp buffer
				switch((RxPDOMapping[PdoNr].Entries[iter])->len)
				{
					case 1:
					{
						uint8_t tempValue = *(((ODEntry08 *)(RxPDOMapping[PdoNr].Entries[iter]))->Value);

						#if(DEBUG_PDO & DEBUG_PDO_TXMsg)
						Serial.print(tempValue);
						Serial.print(" 1b | ");
						#endif

						//copy the value into the payload
						TxPDO.payload[writeIdx++] = tempValue;
						
						break;
					}
					case 2:
					{
						//1st copy the value
						uint16_t tempValue = *(((ODEntry16 *)(RxPDOMapping[PdoNr].Entries[iter]))->Value);

						#if(DEBUG_PDO & DEBUG_PDO_TXMsg)
						Serial.print(tempValue);
						Serial.print(" 2b | ");
						#endif

						//lsb is transferred first
						TxPDO.payload[writeIdx++] = (uint8_t)tempValue;
						//msb is next
						tempValue = tempValue >> 8;
						TxPDO.payload[writeIdx++] = (uint8_t)tempValue;

						break;
					}
					case 4:
					{
						//1st copy the value
						uint32_t tempValue = *(((ODEntry32 *)(RxPDOMapping[PdoNr].Entries[iter]))->Value);

						#if(DEBUG_PDO & DEBUG_PDO_TXMsg)
						Serial.print(tempValue);
						Serial.print(" 4b | ");
						#endif

						//lsb is transferred first
						TxPDO.payload[writeIdx++] = (uint8_t)tempValue;
						tempValue = tempValue >> 8;
						TxPDO.payload[writeIdx++] = (uint8_t)tempValue;
						tempValue = tempValue >> 8;
						TxPDO.payload[writeIdx++] = (uint8_t)tempValue;
						//msb is last
						tempValue = tempValue >> 8;
						TxPDO.payload[writeIdx++] = (uint8_t)tempValue;

						break;
					}
					default:
						#if(DEBUG_PDO & DEBUG_PDO_ERROR)
						Serial.print("Mapping entry # ");
						Serial.print(iter);
						Serial.println(" odd");
					  #endif
						break;
				}
			}
			#if(DEBUG_PDO & DEBUG_PDO_TXMsg)
			Serial.println(".");
			#endif
		}
		else
		{
			Serial.print("PDO: PDO is not valid");
		}	
		//add the length
		TxPDO.len = writeIdx;
		//add the COB-Id
		TxPDO.Id = RxPDOSettings[PdoNr].COBId;
		
	}
	//send it
	if(SendRequest(&TxPDO))
	{
	  returnValue = true;
	}
	
	return returnValue;
}

/*-------------------------------------------------------------------
 * void COPDOHandler::OnRxHandler(CANMsg *)
 * 
 * COMsgHander received a PDO - is handled here bases on the locally stored mappings
 * 
 * 25-04-27 AW frame added
 * 25-07-05 AW implemented
 *
 *-------------------------------------------------------------------*/

void COPDOHandler::OnRxHandler(CANMsg *RxMsg)
{
	uint16_t PdoNr;
	uint8_t readIdx = 0;
	
	#if(DEBUG_PDO & DEBUG_PDO_RX)
	Serial.print("PDO: Rx PDO @ ");
	Serial.print(RxMsg->Id);
	Serial.print(" : ");
	Serial.print(RxMsg->len);
	Serial.println(" bytes");
	#endif
	
	//1st: determine the PDO Idx
	//but be safe here CoMsghandelr should only call with Ids 0x18x, 0x28x, 0x38x and 0x48x	
	PdoNr = (RxMsg->Id>>8)-1;
	
	if((TxPDOSettings[PdoNr].isValid) &&(TxPDOMapping[PdoNr].NrEntries > 0))
	{
		//check the mapping 
		for(uint8_t iter = 0; iter < TxPDOMapping[PdoNr].NrEntries; iter++)
		{
		  //copy data according to length into a temp buffer
			switch((TxPDOMapping[PdoNr].Entries[iter])->len)
			{
				case 1:
				{
					//1st copy the value
					uint8_t tempValue = RxMsg->payload[readIdx++];

					#if(DEBUG_PDO & DEBUG_PDO_RX)
					Serial.print("PDO: entry ");
					Serial.print(iter);
					Serial.print(" = ");
					Serial.println(tempValue,HEX);
					#endif

				  //then write it to the object
				  *(((ODEntry08 *)(TxPDOMapping[PdoNr].Entries[iter]))->Value) = tempValue;
					break;
				}
				case 2:
				{
					//1st copy the value
				  //lsb is transferred first
					uint16_t tempValue = (uint16_t)RxMsg->payload[readIdx++];
				  //msb is next
				  tempValue |= (uint16_t)((RxMsg->payload[readIdx++]) * 256);

					#if(DEBUG_PDO & DEBUG_PDO_RX)
					Serial.print("PDO: entry ");
					Serial.print(iter);
					Serial.print(" = ");
					Serial.println(tempValue,HEX);
					#endif

				  //then write it to the object
				  *(((ODEntry16 *)(TxPDOMapping[PdoNr].Entries[iter]))->Value) = tempValue;
					break;
				}
				case 4:
				{
					//1st copy the value
				  //lsb is transferred first
					uint32_t tempValue = (uint32_t)RxMsg->payload[readIdx++];
				  tempValue |= (uint32_t)((RxMsg->payload[readIdx++]) * 256);
				  tempValue |= (uint32_t)((RxMsg->payload[readIdx++]) * 256 *256);
				  //msb is last
				  tempValue |= (uint32_t)((RxMsg->payload[readIdx++]) * 256 * 256 * 256);

					#if(DEBUG_PDO & DEBUG_PDO_RX)
					Serial.print("PDO: entry ");
					Serial.print(iter);
					Serial.print(" = ");
					Serial.println(tempValue,HEX);
					#endif

				  //then write it to the object
				  *(((ODEntry32 *)(TxPDOMapping[PdoNr].Entries[iter]))->Value) = tempValue;
					break;
				}
				default:
					Serial.print("Mapping entry # ");
				  Serial.print(iter);
				  Serial.println(" odd");
					break;
			}
		}
	}
	else
	{
		Serial.println("PDO: PDO is not valid");
	}
}

/*-------------------------------------------------------------------
 * void COPDOHandler::OnTimeOut()
 * 
 * actions when an expected response timed out
 * these can either be a SDO timed out or a sync PDO was not recieved
 * 
 * 25-04-27 AW frame added
 *
 *-------------------------------------------------------------------*/

void COPDOHandler::OnTimeOut()
{
	;
}
		
