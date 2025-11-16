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
 * CO401IO.cpp
 * implements the class to produce provide CiA 401 drive related function
 *
 * 2025-07-24 AW Frame
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <CO401Node.h>

//--- local defines ---

#define DEBUG_IO_TO			    0x0001
#define DEBUG_IO_ERROR		    0x0002
#define DEBUG_IO_Init        0x0004
#define DEBUG_IO_WRITEOBJ    0x0010
#define DEBUG_IO_READOBJ     0x0020
#define DEBUG_IO_CW          0x0040

#define DEBUG_IO (DEBUG_IO_TO | DEBUG_IO_ERROR) 

//--- local definitions ---------


//--- public functions ---

/*---------------------------------------------------------------------
 * CO401Node::CO401Node(uint8_t thisId)
 * as of now there is noting to intialized when created
 * 
 * 2025-03-09 AW Done
 * ------------------------------------------------------------------*/

CO401Node::CO401Node(uint8_t thisId)
{
	nodeId = thisId;
	
  //default mapping should not be needed here

	uint8_t AccessStep = 0;
	
	for(uint8_t iter = 0; iter < NumNodeIdentityObjects; iter++)
	{
	  for(uint8_t charIdx = 0; charIdx < NodeOdStringLen; charIdx++)
		{
	    ((ODEntryString *)IdentityEntries[iter])->Value[charIdx] = 0;
		}
		((ODEntryString *)IdentityEntries[iter])->len = 0;
	}
}


/*-------------------------------------------------------------------
 * void CO401Node::init(COMsgHandler *ThisHandler)
 * 
 * store the instance to the MsgHandler
 * no need to be regsitered as we don't receive messages
 *
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

void CO401Node::init(COMsgHandler *ThisHandler)
{
	uint8_t NodeHandle;
	
	MsgHandler = ThisHandler;

	NodeHandle = MsgHandler->RegisterNode(nodeId);
	
  Node.init(MsgHandler, nodeId, NodeHandle);
  
  //init the nodes PDO handler
  PDOHandler.init(MsgHandler, &Node, nodeId, NodeHandle);

	PDOHandler.PresetRxPDOTransmission(0, 255);  //parameters are the PDO# and the transmission type
	PDOHandler.PresetTxPDOTransmission(0, 255, 0, 0);  //parameters are the PDO#, the transmission type, the inhibit time and the EvtTimer
	
	PDOHandler.PresetRxPDOMapping(0, MapRxPDO1.NrEntries, MapRxPDO1.Entries);  //parameters are the PDO#, the number of actually mapped entries and the pointer to the entries
  PDOHandler.PresetTxPDOMapping(0, MapTxPDO1.NrEntries, MapTxPDO1.Entries);  //parameters are the PDO#, the number of actually mapped entries and the pointer to the entries

  PDOHandler.PresetRxPDOisValid(0,true);
  PDOHandler.PresetTxPDOisValid(0,true);
  
	PDOHandler.PresetRxPDOTransmission(1, 255);  //parameters are the PDO# and the transmission type
	PDOHandler.PresetTxPDOTransmission(1, 255, 0, 0);  //parameters are the PDO#, the transmission type, the inhibit time and the EvtTimer

	PDOHandler.PresetRxPDOMapping(1, MapRxPDO2.NrEntries, MapRxPDO2.Entries);  //parameters are the PDO#, the number of actually mapped entries and the pointer to the entries
  PDOHandler.PresetTxPDOMapping(1, MapTxPDO2.NrEntries, MapTxPDO2.Entries);  //parameters are the PDO#, the number of actually mapped entries and the pointer to the entries

  PDOHandler.PresetRxPDOisValid(1,false);
  PDOHandler.PresetTxPDOisValid(1,false);
}


/*-------------------------------------------------------------------
 * void CO401Node::ResetComState()
 * 
 * reset the ComTate to the default Idle
 *
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/
void CO401Node::ResetComState()
{
	AccessStep = 0;
}

/*-------------------------------------------------------------------
 * uint8_t CO401Node::GetNodeId()
 * 
 * return the nodeId of this one
 *
 * 25-09-03 AW 
 *
 *-------------------------------------------------------------------*/
uint8_t CO401Node::GetNodeId()
{
	return nodeId;
}


/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::Update(uint32_t actTime, COSyncState syncState)
 * 
 * cyclic update where the actTime is compared to the last time-stamp of either HB
 * or Sync and related messages are sent
 * these are trhow away messages - unconfirmed
 * for the HB the current node stante of this device is added
 * 
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

NMTNodeState CO401Node::Update(uint32_t actTime, COSyncState syncState)
{
	COIONodeCommStates returnValue = eCO_IOIdle;
	
  NMTNodeState NodeState = Node.Update(actTime);
	
	if(NodeState < eNMTStateReset)
	{
	  isPDOsConfigured = false;
		PDOHandler.FlagPDOsInvalid();
		
		//might have seen a re-boot
		//invalidate the CW / SW values
		
		//todo - what needs to be reset here?
	}
	
	if(NodeState == eNMTStatePreOp)
	{		
		if(isPDOsConfigured == false)
		{
			if(reConfigPDOs == true)
      {
        if(InitPDOs(actTime) == eCO_IODone)
        {
				  #if(DEBUG_IO & DEBUG_IO_Init)
          Serial.println("IONode: PDO-Config re-established");
				  #endif
				
          ResetComState();
        }
		  }
		}
		else
		{
			if(autoEnable)
			{
			  //will automatically enable the drive only if preset accordingly
			  if(Node.SendStartNode() == eCO_NodeDone)
			  {
				  #if(DEBUG_IO & DEBUG_IO_Init)
				  Serial.println("IONode Update: Node auto-started");
				  #else
				  ;
				  #endif
				}
			}
		}
	}	
	
	if(NodeState == eNMTStateOperational)
    PDOHandler.Update(actTime, syncState);  
	

  return NodeState;	
}

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::InitNode(uint32_t actTime)
 * 
 * Init the remote node based on the preset configuration
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::InitNode(uint32_t actTime)
{
	COIONodeCommStates returnValue = eCO_IOBusy;
	
	if(Node.InitRemoteNode(actTime) == eNMTStatePreOp)
    returnValue = eCO_IODone;
			
  return returnValue;	
}

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::InitPDOs(uint32_t actTime)
 * 
 * init the PDO config of the remote node based on the prest configuration
 * 
 * 25-07-26 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::InitPDOs(uint32_t actTime)
{
	COIONodeCommStates returnValue = eCO_IOBusy;
	
	if(PDOHandler.ConfigurePresetPDOs(actTime) == eCO_PDODone)
	{
    returnValue = eCO_IODone;
		isPDOsConfigured = true;
	}
	
  return returnValue;	
}

//---- specific for the IO-Node -----------------------------------------------

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::SetDigOut(uint8_t value)
 * 
* update the DigOut status of the first 8 bit register
 * 
 * 25-10-28 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::SetDigOut(uint8_t value)
{
	return SetNumObject(&(OdDigOutStatus),value);
}

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::GetDigInStatus(uint8_t *value)
 * 
 * read the latest DigIn status 0x6200.01
 * 
 * 25-09-14 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::GetDigInStatus(uint8_t *value)
{
	COIONodeCommStates returnValue = GetNumObject(&(OdDigInStatus));
  
	if(returnValue == eCO_IODone)
	  *value = *(OdDigInStatus.Value);
	
	return returnValue;	
}

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::UpdateRemoteAnOut(uint8_t output, uin16_t value)
 * 
 * update the analog value to be transmitted to the remote node
 * 
 * 25-11-15 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::SetRemoteAnOut(uint8_t output, int16_t value)
{
	return SetNumObject(&(OdAnOutStatus),(uint16_t)value);	
}


/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::GetRemoteAnIn(uint8_t input, uint16_t * value)
 * 
 * update the analog value received frommthe remote I/O
 * 
 * 25-11-16 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::GetRemoteAnIn(uint8_t input, int16_t * value)
{
	COIONodeCommStates returnValue = GetNumObject(&(OdAnInStatus));
  
	if(returnValue == eCO_IODone)
	  *value = (int16_t)(*(OdAnInStatus.Value));
	
	return returnValue;	
}
/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::IdentifyIONode()
 * 
 * set a list of identification objects and request them to be uploaded
 * 
 * 25-09-11 AW frame
 * 25-09-12 AW implemented and tested
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::IdentifyIONode()
{
	COIONodeCommStates returnValue = eCO_IOBusy;
	
  if(Node.RWSDO.ReadObjects(IdentityEntries, NumNodeIdentityObjects) == eCO_SDODone)
		returnValue = eCO_IODone;

	return returnValue;
}	
	
/*-------------------------------------------------------------------
 * ODEntry **CO401Node::GetIdentityEntries()
 * 
 * return a pointer to the array of identiy objects
 * 
 * 25-09-12 AW implemented and tested
 *
 *-------------------------------------------------------------------*/

ODEntry **CO401Node::GetIdentityEntries()
{
	return IdentityEntries;
}


/*-------------------------------------------------------------------
 * void CO401Node::PrintIdentityObjects()
 * 
 * directly print the identiy strings to Serial
 * 
 * 25-09-12 AW implemented and tested
 *
 *-------------------------------------------------------------------*/

void CO401Node::PrintIdentityObjects()
{
	for(uint8_t iter = 0; iter < NumNodeIdentityObjects; iter++ )
	{
		switch(iter)
		{
			case 0:
				Serial.print("DeviceName: ");
			  break;
			case 1:
				Serial.print("HwVersion : ");
			  break;
			case 2:
				Serial.print("SwVersion : ");
			  break;
      default:
        break;			
		}
		for(uint32_t charIdx = 0; charIdx < IdentityEntries[iter]->len; charIdx++)
		{
			Serial.write(((ODEntryString *)IdentityEntries[iter])->Value[charIdx]);
		}
		Serial.println(";");
	}
}

//----------------------------------------------------------------------------------
//--- private functions ---

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::SetNumObject(ODEntry08 *Object, uint8_t Value)
 * 
 * update a uint8_t object in the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::SetNumObject(ODEntry08 *Object, uint8_t Value)
{
	COIONodeCommStates returnValue = eCO_IOBusy;

	//do we need to update the value?	
  if(*(Object->Value) != Value)
	{
		//1st: update the local copy
		*(Object->Value) = Value;		
		
		//check for is being mapped to PDO
		if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
		{
			#if(DEBUG_IO & DEBUG_IO_WRITEOBJ)
			Serial.print("IONode: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" is mapped");
			#endif
			
			returnValue = eCO_IODone;
		}
		else
		{
			//is not mapped
			//fixed lenght of 1 byte
			COSDOCommStates SDOState = Node.RWSDO.WriteSDO((ODEntry *)Object);

			if(SDOState == eCO_SDODone)
			{
			  #if(DEBUG_IO & DEBUG_IO_WRITEOBJ)
			  Serial.print("IONode: Idx ");
			  Serial.print(Object->Idx,HEX);
			  Serial.println(" updated via SDO");
				#endif

			  returnValue = eCO_IODone;
			}
			else if(SDOState == eCO_SDOError)
			{
				returnValue = eCO_IOError;
			}
		}
	}
	else
	{
		//no need to update anything
		returnValue = eCO_IODone;
	}

	return returnValue;
}

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::SetNumObject(ODEntry16 *Object, uint16_t Value)
 * 
 * update a uint16_t object in the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::SetNumObject(ODEntry16 *Object, uint16_t Value)
{
	COIONodeCommStates returnValue = eCO_IOBusy;

	//do we need to update the value?	
  if(*(Object->Value) != Value)
	{
		//1st: update the local copy
		*(Object->Value) = Value;		
		
		//check for is being mapped to PDO
		if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
		{
			#if(DEBUG_IO & DEBUG_IO_WRITEOBJ)
			Serial.print("IONode: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" is mapped");
			#endif
			
			returnValue = eCO_IODone;
		}
		else
		{
			//is not mapped
			//fixed lenght of 2 bytes
			COSDOCommStates SDOState = Node.RWSDO.WriteSDO((ODEntry *)Object);

			if(SDOState == eCO_SDODone)
			{
			  #if(DEBUG_IO & DEBUG_IO_WRITEOBJ)
			  Serial.print("IONode: Idx ");
			  Serial.print(Object->Idx,HEX);
			  Serial.println(" updated via SDO");
				#endif

			  returnValue = eCO_IODone;
			}
			else if(SDOState == eCO_SDOError)
			{
				returnValue = eCO_IOError;
			}
		}
	}
	else
	{
		//no need to update anything
		returnValue = eCO_IODone;
	}

	return returnValue;
}

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::SetNumObject(ODEntry32 *Object, uint32_t Value)
 * 
 * update a uint32_t object in the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-08-08 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::SetNumObject(ODEntry32 *Object, uint32_t Value)
{
	COIONodeCommStates returnValue = eCO_IOBusy;

	//do we need to update the value?	
  if(*(Object->Value) != Value)
	{
		//1st: update the local copy
		*(Object->Value) = Value;		
		
		//check for is being mapped to PDO
		if(PDOHandler.TxPDOsAsync((ODEntry *)Object))
		{
			#if(DEBUG_IO & DEBUG_IO_WRITEOBJ)
			Serial.print("IONode: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" is mapped");
      #endif
			
			returnValue = eCO_IODone;
		}
		else
		{
			//is not mapped
			COSDOCommStates SDOState = Node.RWSDO.WriteSDO((ODEntry *)Object);
			
			if(SDOState == eCO_SDODone)
			{
			  #if(DEBUG_IO & DEBUG_IO_WRITEOBJ)
			  Serial.print("IONode: Idx ");
			  Serial.print(Object->Idx,HEX);
			  Serial.println(" updated via SDO");
				#endif
				
			  returnValue = eCO_IODone;
			}
			else if(SDOState == eCO_SDOError)
			{
				returnValue = eCO_IOError;
			}
		}
	}
	else
	{
		//no need to update anything
		returnValue = eCO_IODone;
	}

	return returnValue;
}


/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::GetNumObject(ODEntry08 *Object)
 * 
 * read a uint8_t object from the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-09-13 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::GetNumObject(ODEntry08 *Object)
{
	COIONodeCommStates returnValue = eCO_IOBusy;
	
	//check for is being mapped to PDO
	if(PDOHandler.RxPDOIsSync((ODEntry *)Object))
	{
		#if(DEBUG_IO & DEBUG_IO_READBJ)
		Serial.print("IONode: Idx ");
		Serial.print(Object->Idx,HEX);
		Serial.println(" is mapped");
		#endif
		
		//PDO won't be triggered while this is executed
    //we will get the value received last
		returnValue = eCO_IODone;
	}
	else
	{
		//is not mapped
		COSDOCommStates SDOState = Node.RWSDO.ReadSDO((ODEntry *)Object);
		if((SDOState == eCO_SDODone) && (Object->len = 1))
		{
			#if(DEBUG_IO & DEBUG_IO_READOBJ)
			Serial.print("IONode: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" updated via SDO");
			#endif

			returnValue = eCO_IODone;
		}
		else if(SDOState == eCO_SDOError)
		{
			returnValue = eCO_IOError;
		}
	}
	return returnValue;
}

	
/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::GetNumObject(ODEntry16 *Object)
 * 
 * read a uint16_t object from the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-09-13 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::GetNumObject(ODEntry16 *Object)
{
	COIONodeCommStates returnValue = eCO_IOBusy;
	
	//check for is being mapped to PDO
	if(PDOHandler.RxPDOIsSync((ODEntry *)Object))
	{
		#if(DEBUG_IO & DEBUG_IO_READBJ)
		Serial.print("IONode: Idx ");
		Serial.print(Object->Idx,HEX);
		Serial.println(" is mapped");
		#endif
		
		//PDO won't be triggered while this is executed
    //we will get the value received last
		returnValue = eCO_IODone;
	}
	else
	{
		//is not mapped
		COSDOCommStates SDOState = Node.RWSDO.ReadSDO((ODEntry *)Object);
		if((SDOState == eCO_SDODone) && (Object->len = 2))
		{
			#if(DEBUG_IO & DEBUG_IO_READOBJ)
			Serial.print("IONode: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" updated via SDO");
			#endif

			returnValue = eCO_IODone;
		}
		else if(SDOState == eCO_SDOError)
		{
			returnValue = eCO_IOError;
		}
	}
	return returnValue;
}

/*-------------------------------------------------------------------
 * COIONodeCommStates CO401Node::GetNumObject(ODEntry32 *Object)
 * 
 * read a uint32_t object from the remote drive either by PDO if it is mapped
 * or by explicit usage of an SDO
 * 
 * 25-09-13 AW 
 *
 *-------------------------------------------------------------------*/

COIONodeCommStates CO401Node::GetNumObject(ODEntry32 *Object)
{
	COIONodeCommStates returnValue = eCO_IOBusy;
	
	//check for is being mapped to PDO
	if(PDOHandler.RxPDOIsSync((ODEntry *)Object))
	{
		#if(DEBUG_IO & DEBUG_IO_READBJ)
		Serial.print("IONode: Idx ");
		Serial.print(Object->Idx,HEX);
		Serial.println(" is mapped");
		#endif
		
		//PDO won't be triggered while this is executed
    //we will get the value received last
		returnValue = eCO_IODone;
	}
	else
	{
		//is not mapped
		COSDOCommStates SDOState = Node.RWSDO.ReadSDO((ODEntry *)Object);
		if((SDOState == eCO_SDODone) && (Object->len = 4))
		{
			#if(DEBUG_IO & DEBUG_IO_READOBJ)
			Serial.print("IONode: Idx ");
			Serial.print(Object->Idx,HEX);
			Serial.println(" updated via SDO");
			#endif

			returnValue = eCO_IODone;
		}
		else if(SDOState == eCO_SDOError)
		{
			returnValue = eCO_IOError;
		}
	}
	return returnValue;
}
		
