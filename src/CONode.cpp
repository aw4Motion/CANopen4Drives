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
 * CONode.cpp
 * implements the class to handle NMT and Guarding 
 *
 * 2025-01-11 AW Frame
 *
 *--------------------------------------------------------------*/
 
//--- includes ---

#include <CONode.h>
#include <COObjects.h>


//--- local defines ---

#define DEBUG_NMT_RXMSG		0x0001
#define DEBUG_NMT_TO			0x0002
#define DEBUG_NMT_ERROR		0x0004
#define	DEBUG_NMT_TXCS		0x0008
#define DEBUG_NMT_RXGuard	0x0010
#define DEBUG_NMT_TXGuard	0x0020
#define DEBUG_NMT_ConfigGuard	0x0040
#define DEBUG_NMT_Init    0x0080
#define DEBUG_NMT_StateChange 0x0100
#define DEBUG_NMT_BUSY    0x0200
#define DEBUG_NMT_EMCY    0x0400
#define DEBUG_NMT_BOOTING 0x0800

#define DEBUG_NODE (DEBUG_NMT_TO | DEBUG_NMT_ERROR | DEBUG_NMT_EMCY | DEBUG_NMT_BOOTING | DEBUG_NMT_Init) 

#define NODE_PrintEMCY 1

//--- local definitions ---------

//  defintions for the NMT service
//  commands are defined as const uint8_t which simplified them being 
//  accepted in assignments

const uint8_t NMT_StartRemoteNode = 0x01;
const uint8_t NMT_StopRemoteNode = 0x02;
const uint8_t NMT_EnterPreop = 0x80;
const uint8_t NMT_ResetRemoteNode = 0x81;
const uint8_t NMT_ResetComRemoteNode = 0x82;

//define a timeout for SDORequests of this module
const uint32_t SDORequestTimeout = 200;

//--- public functions ---

/*---------------------------------------------------------------------
 * CONode::CONode()
 * as of now there is noting to intialized when created
 * 
 * 2025-02-22 AW Done
 * ------------------------------------------------------------------*/

CONode::CONode()
{
  ODGuardTime.Value = &GuardTime;
	ODLiveTimeFactor.Value = &LiveTimeFactor;
	ODProducerHeartbeatTime.Value = &HeartbeatProducerTime;
	ODConsumerHeartbeatTime.Value = &HeartbeatConsumerTime;
	
	ODRemoteNodeType.Value = (void *)&RemoteNodeTypeValue;
}


/*-------------------------------------------------------------------
 * CONode::init(COMsgHandler *MsgMandler,int8_t);
 * 
 * connect the instane to the MsgHandler and do the same for the
 * instance of the COSDOHandler
 *
 * 25-01-11 AW 
 *
 *-------------------------------------------------------------------*/

void CONode::init(COMsgHandler *MsgHandler, int8_t ThisNode, int8_t MsgHandle)
{
	NodeId = ThisNode;
	Handler = MsgHandler;
		
	if(MsgHandle != InvalidSlot)
	{
	  //register Cb
		pfunction_holder Cb;
		
		//we do have a handle for the COMsghandler - register the COSDOHandler
  	RWSDO.init(MsgHandler, ThisNode, MsgHandle);

		GuardingRequest.Id = eCANGuarding | NodeId;
		GuardingRequest.len = 0;
		GuardingRequest.isRTR = true;
    GuardingRequest.serviceType = eCANGuarding;

    //don't care for the contents of the guarding payload
		
		NmtCommand.Id = eCANNMT;
		NmtCommand.nodeId = NodeId;
	  NmtCommand.len = NMTCommandFrameLength;
	  NmtCommand.isRTR = false;
    NmtCommand.serviceType = eCANNMT;

		//create the functor for the NMT Cb
		Cb.callback = (pfunction_pointer_t)CONode::OnSysMsgRxCb;
		Cb.op = (void *)this;

	  Handler->Register_OnRxNmtCb(MsgHandle,&Cb);
	  ConfigState = eCO_NodeIdle;
		RequestState = eCO_NodeIdle;
		ConfigStep = 0;
		
		//create the dunctor for the EmcyHandler and register it
		Cb.callback = (pfunction_pointer_t)CONode::OnEmcyMsgRxCb;
	  Handler->Register_OnRxEMCYCb(MsgHandle,&Cb);		
		
	  NodeState = eNMTStateOffline;
		isLive = false;		
		
		#if(DEBUG_NODE & DEBUG_NMT_Init)
		Serial.print("Node: CONode ");
		Serial.print(NodeId, DEC);
		Serial.println(" initialized - offline");
		#endif
	}
	else
	{
		ConfigState = eCO_NodeError;	
		RequestState = eCO_NodeError;
		
    #if (DEBUG_NODE & DEBUG_NMT_ERROR)
    Serial.print("Node: Could not get an handle for node ");
		Serial.println(NodeId);
		#endif
	}
}
	

/*-------------------------------------------------------------------
 * void CONode::SetNodeId(uint8_t ThisNodeId)
 * Set the NodeId for this instance. Needs to be called before the
 * Msghandler can be registered
 * 
 * 2020-11-21 AW Done
 * -----------------------------------------------------------------*/

void CONode::SetNodeId(uint8_t ThisNodeId)
{
	NodeId = (int16_t)ThisNodeId;
}

/*-------------------------------------------------------------------
 * int16_t CONode::GetNodeId()
 * Resturns the actual configured NodeId
 * 
 * 2025-06-21 AW Done
 * -----------------------------------------------------------------*/

int16_t CONode::GetNodeId()
{
	return (int16_t)NodeId;
}

/*------------------------------------------------------------------
 * void CONode::ResetSDOState()
 * Reset the ComState of the embedded SDOHandler and reset its reflected
 * local state too
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/
 
void CONode::ResetSDOState()
{
	RWSDO.ResetComState();
	//NodeRxTxState = eCO_NodeIdle;	
}

/*--------------------------------------------------------------------
 * void CONode::ResetComState()
 * Reset the ComState of the MCNode
 * Is the only way to recover from an eCWError state.
 * Will reset all Com related states and will unlock an still locked
 * MsgHandler.
 * Does the reset of the embedded SDOHandler too.
 * 
 * 2020-11-21 AW Done
 * ------------------------------------------------------------------*/


void CONode::ResetComState()
{
	ConfigState = eCO_NodeIdle;
	RequestState = eCO_NodeIdle;
	
	TORetryCounter = 0;
	BusyRetryCounter = 0;
	ConfigStep = 0;
	ResetSDOState();
	
}

/*--------------------------------------------------------------------
 * void CONode::RestartNode()
 * Reset the ComState of the MCNode
 * Is the only way to recover from an eCWError state.
 * Will reset all Com related states and will unlock an still locked
 * MsgHandler.
 * Does the reset of the embedded SDOHandler too.
 * 
 * 2020-11-21 AW Done
 * ------------------------------------------------------------------*/

void CONode::RestartNode()
{
	NodeState = eNMTStateOffline;
	ResetComState();
}

/*------------------------------------------------------------------
 * void SetTORetryMax(uint8_t value)
 * Configre a different number of TO from which the Node could  try to 
 * recover autoamtically until the eCWError state is reached.
 * Default is within the class definition.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

void CONode::SetTORetryMax(uint8_t value)
{
	TORetryMax = value;
}

/*------------------------------------------------------------------
 * void SetBusyRetryMax(uint8_t value)
 * When the Msghandler or embedded SDO handler is busy an attemted 
 * access ends up busy and will retry automatically. Define a different
 * number of retrys until the access will fail.
 * Default is within the class definition.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

void CONode::SetBusyRetryMax(uint8_t value)
{
	BusyRetryMax = value;
}


/*------------------------------------------------------------------
 * bool IsLive()
 * Check whether a boot Msg of the drive has been received
 * Please note: in net-mode of multiple drives no boot messages
 * will be sent at all.
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

bool CONode::IsLive()
{
	return isLive;
}

/*----------------------------------------------------------
 * Register_OnNodeStateChangeCb(function_holder *cb)
 * store the function and object pointer for the callback
 * called in case of an unexpected change of the node state
 * 
 * 2025-01-19 AW
 * 
 * --------------------------------------------------------*/

void CONode::Register_OnNodeStateChangeCb(pfunction_holder *Cb)
{
	OnNodeStateChangeCb.callback = Cb->callback;
	OnNodeStateChangeCb.op = Cb->op;
}

/*------------------------------------------------------------------
 * NMTNodeState CONode::InitRemoteNode(uint32_t actTime)
 * 
 * download the configuration to the remote node via SDO
 * return value is the actual node state
 * INitRemoteNode is finished when eNMTStatePreOp is reached
 *
 * 2025-07-26 AW extracted from update
 *-----------------------------------------------------------------*/

NMTNodeState CONode::InitRemoteNode(uint32_t Time)
{
 	actTime = Time;
	RWSDO.SetActTime(Time);
	
	switch(NodeState)
  {
    case eNMTStateOffline:
			if((actTime - RequestTime) > SDORequestTimeout)
		  {
			  COSDOCommStates requestComState;
			  //here we need to send an SDO upload request
			  requestComState = RWSDO.ReadSDO(&ODRemoteNodeType);
				
			  switch(requestComState)
			  {
				  case eCO_SDOTimeout:
					  //time-out
					  //no break;
				  case eCO_SDOError:
					  //we reset the SDOHandler and thus restart
					  RWSDO.ResetComState();
					  //let's take it again from here
					  RequestTime = actTime;
					  break;
				  case eCO_SDODone:
				 	  //here we got an answer
				    //as we end up here being in eNMTStateOffline
				    //we can set the accessStep = 0 and the state to being in reset - still partly unconfigured
					        		
					  #if(DEBUG_NODE & DEBUG_NMT_Init)
					  Serial.println("Node: Node found --> request reset");
		        #endif

					  //send rest to explicitely wait for boot msg
					  if(SendResetNode() == eCO_NodeDone)
						{					
					    RequestTime = actTime;
						}							
					  break;
				  default:
					  //no action 
					  break;
			  }
		  }  //end of check wheter time expired - no action when not
			break;
		case eNMTWaitForBoot:
    case eNMTBootMsgReceived:
			//wait for a boot msg to be received
			if(isLive)
			{
			  NodeState = eNMTBooting;
				#if(DEBUG_NODE & DEBUG_NMT_BOOTING)
				Serial.print("Node: ");
				Serial.print(NodeId);
				Serial.println(" Boot Msg received");
				#endif
			}
			else
				//todo: count the cycles and switch back to Offline if no answer
			  ;
      break;			
		//one extra cyle to be able to flag this externally
		case eNMTBooting:
			NodeState = eNMTStateReset;
      break;			
		case eNMTStateReset:
		{
			CONodeCommStates configResult = eCO_NodeIdle;
			
			//here we do configure Guarding or HB
		  //determine which one based on the the time settings
			//parameters are given
			//need to configure the drive
			if(GuardTime > 0)
		  {
				//configure node guarding
		    configResult = ActivateGuarding();
			}  // end of configuration for Guarding
			else if(HeartbeatProducerTime > 0)
			{
				//configure heartbeat
				configResult = ActivateHeartbeat();
			}
			else
			{
				//don't need to configure anything right now
				configResult = eCO_NodeDone;
			}
			
			//in any cases we are done when nodeRxTx is eCO_NodeDone
			if(configResult == eCO_NodeDone)
			{
			  NodeState = eNMTStatePreOp;
        ConfigState = eCO_NodeIdle;
				if(GuardingState == eCO_GuardingConfigured)
					GuardingState = eCO_GuardingExpected;
				
				#if(DEBUG_NODE & DEBUG_NMT_Init)
				Serial.println("Node: Node configured --> pre-op");
		    #endif

			}
		}
		  break;
		default:
			break;
	}
	return NodeState;
}

/*------------------------------------------------------------------
 * void update(uint32_t actTime)
 * set the local time
 * when in state offline send SDO requests for 0bject 1000
 * and trigger Guarding and / or intial node identification
 *
 * 2025-01-12 AW frame
 * 2ß25-07-26 AW handle pre-op and op only - InitRemoteNode to be executed first
 *-----------------------------------------------------------------*/

NMTNodeState CONode::Update(uint32_t Time)
{
 	actTime = Time;
	RWSDO.SetActTime(Time);
	
	switch(NodeState)
  {
		//included in Update to handle a reset node
		case eNMTStateOffline:
			if((actTime - RequestTime) > SDORequestTimeout)
		  {
			  COSDOCommStates requestComState;
			  //here we need to send an SDO upload request
			  requestComState = RWSDO.ReadSDO(&ODRemoteNodeType);
			
			  switch(requestComState)
			  {
				  case eCO_SDOTimeout:
					  //time-out
					  //no break;
				  case eCO_SDOError:
					  //we reset the SDOHandler and thus restart
					  RWSDO.ResetComState();
					  //let's take it again from here
					  RequestTime = actTime;
					  break;
				  case eCO_SDODone:
				 	  //here we got an answer
				    //as we end up here being in eNMTStateOffline
				    //we can set the accessStep = 0 and the state to being in reset - still partly unconfigured
					
					  #if(DEBUG_NODE & DEBUG_NMT_Init)
					  Serial.println("Node: Node found --> request reset");
		        #endif
					
						//send reset to explicitely wait for boot msg
					  //will force the node state too
					  if(SendResetNode() == eCO_NodeDone)
						{
					    RequestTime = actTime;
						}							
					  break;
				  default:
					  //no action 
					  break;
			  }
		  }  //end of check wheter time expired - no action when not
			break;
		case eNMTWaitForBoot:
    case eNMTBootMsgReceived:
			//wait for a boot msg to be received
			if(isLive)
			{
			  NodeState = eNMTBooting;
				#if(DEBUG_NODE & DEBUG_NMT_BOOTING)
				Serial.print("Node: ");
				Serial.print(NodeId);
				Serial.println(" Boot Msg received");
				#endif
			}
			else
				//todo: count the cycles and switch back to Offline if no answer
			  ;
      break;			
		//one extra cyle to be able to flag this externally
		case eNMTBooting:
			NodeState = eNMTStateReset;
      break;			
		case eNMTStateReset:
		{
			CONodeCommStates configResult = eCO_NodeIdle;

			//here we do configure Guarding or HB
		  //determine which one based on the the time settings
			//parameters are given
			//need to configure the drive
			if(GuardTime > 0)
		  {
				//configure node guarding
		    configResult = ActivateGuarding();
			}  // end of configuration for Guarding
			else if(HeartbeatProducerTime > 0)
			{
				//configure heartbeat
				configResult = ActivateHeartbeat();
			}
			else
			{
				//don't need to configure anything right now
				configResult = eCO_NodeDone;
			}
			
			//in any cases we are done when nodeRxTx is eCO_NodeDone
			//will force the node state too
			if(configResult == eCO_NodeDone)
			{
				//no need to explicitely send a pre.op command
				//a booting node will end up there automatically
			  NodeState = eNMTStatePreOp;
        ConfigState = eCO_NodeIdle;
				
				if(GuardingState == eCO_GuardingConfigured)
					GuardingState = eCO_GuardingExpected;
				
				#if(DEBUG_NODE & DEBUG_NMT_Init)
				Serial.println("Node: Node configured --> pre-op");
		    #endif

			}
		}
		  break;
		case eNMTStatePreOp:
		case eNMTStateOperational:
			//as soon as we are in PreOp we need to probe
		  //also wenn die letzte empfangene Nachricht eine Weile durch ist,
		  //muss eine Anfrage gesendet werden
		  //dann gehen wir auf waiting
		  //wenn ne Antwort kommt -> received
		  //wenn waiting und Time-out --> missed Zähler hoch und entweder nach Error oder nach Expepected
		  //wenn received und Zeit um -- Expected
		  if(isGuardingActive)
			{
				switch(GuardingState)
        {
          case eCO_GuardingExpected:
					  //send request and
				    //denote the time
					  if(Handler->SendMsg(&GuardingRequest))
						{
							//denote this as successful only if true
					    GuardRequestSentAt = actTime;

					    //and switch to waiting
					    GuardingState = eCO_GuardingWaiting;
						}
					  break;
					case eCO_GuardingWaiting:
						//we do only leave the Waiting state when OnRx has received the correct response
					  //we might swtich to TimeOut
					  if((actTime - GuardRequestSentAt) > GuardTime)
						{
							//request was sent and didn't get an answer - this is TO
						  GuardingState = eCO_GuardingTimeOut;
						  NumGuardRequestsOpen++;
						}
						break;
					case eCO_GuardingReceivedIntime:
					  if((actTime - GuardRequestSentAt) > GuardTime)
						{
							//request was sent and didn't get an answer - this is TO
						  GuardingState = eCO_GuardingExpected;
						  NumGuardRequestsOpen = 0;
							NodeState = ReportedState;
						}
						break;
					case eCO_GuardingTimeOut:
						//check whether we can try again
						if(NumGuardRequestsOpen < LiveTimeFactor)
						{
							GuardingState = eCO_GuardingExpected;
						}
						else
						//is not this is an error
						//return to looking for this node
						{
							GuardingState = eCO_GuardingError;
							Serial.println("Node: Guarding Error");
              NodeState = eNMTStateOffline;													
						}						
						break;
					default: //includes the Error state
						break;
				}					
			}
		  if(isHeartbeatActive)
			{
				//no need to send anything
				//but we need to flag an error if time-out
				//return to looking for this node

				//if((HeatbeatReceivedAt - actTime) > HeartbeatProducerTime)
				if((actTime - HeatbeatReceivedAt) > RemoteHBMissedTime)
				{
		      GuardingState = eCO_GuardingError;
					Serial.print("Node: HB failed @");
					Serial.println(actTime);
					Serial.print("Node: threshold was :");
					Serial.println(RemoteHBMissedTime);
          NodeState = eNMTStateOffline;
			  }
			}	
		  break;
		default:
			break;
	} // end switch NodeState
	return NodeState;
}

/*------------------------------------------------------------------
 * CONodeCommStates ConfigureGuarding(uint16_t Time, uint8_t Factor)
 *
 * download the guarding parameters to the node and start guarding if
 * successfull
 * unless Guarding fails the node is considered to be alive
 * do also unconfigure the heartbeat
 *
 * 2025-01-12 AW frame
 *-----------------------------------------------------------------*/

CONodeCommStates CONode::ConfigureGuarding(uint16_t Time, uint8_t Factor)
{
	CONodeCommStates returnValue = eCO_NodeBusy;
	//depending on the state we either store only or reconfigure directly
	GuardTime = Time;
	LiveTimeFactor = Factor;
	//force Heartbeat to 0 then
	HeartbeatProducerTime = 0;
	HeartbeatConsumerTime = 0;
	RemoteHBMissedTime = 0;

	if(NodeState < eNMTStatePreOp)
	{
		//store the value only
		returnValue = eCO_NodeDone;

		#if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
		Serial.print("Node: Guarding preset for :");
		Serial.print(Time);
		Serial.println(" ms");
		#endif
	}
	else
	{
		//we are live - so configure directly
		returnValue = ActivateGuarding();
	} // end of else
	return returnValue;
}

/*------------------------------------------------------------------
 * CONodeCommStates ConfigureRemoteHeartbeat(uint16_t Time)
 *
 * download the hearbeat producer time to the node and start the time-out if
 * successfull
 * unless a time-out accurs the node is considered to be alive
 * do also unconfigure the guarding
 *
 * 2025-01-12 AW frame
 *-----------------------------------------------------------------*/

CONodeCommStates CONode::ConfigureRemoteHeartbeatProducer(uint16_t Time)
{
	CONodeCommStates returnValue = eCO_NodeBusy;
	//depending on the state we either store only or reconfigure directly
	HeartbeatProducerTime = Time;
	RemoteHBMissedTime = Time + Time/4;
  //disable Guarding in that case
	GuardTime = 0;
	LiveTimeFactor = 0;
	
	if(NodeState < eNMTStatePreOp)
	{
		//store the value only
		returnValue = eCO_NodeDone;
		#if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
		Serial.print("Node: HB producer preset for :");
		Serial.print(Time);
		Serial.println(" ms");
		#endif

	}
	else
	{
		//we are live - so configure directly
		returnValue = ActivateHeartbeat();
	} // end of else
	return returnValue;
}

/*------------------------------------------------------------------
 * CONodeCommStates CONode::ConfigureRemoteHeartbeatConsumer(uint8_t TxNode, uint16_t ThresholdTime)
 *
 * store the timing and nodeid of the remote HB consumer
 * activated only when already pre-op
 *
 * 2025-08-22 AW
 *-----------------------------------------------------------------*/

CONodeCommStates CONode::ConfigureRemoteHeartbeatConsumer(uint8_t TxNode, uint16_t ThresholdTime)
{
	CONodeCommStates returnValue = eCO_NodeBusy;

	//depending on the state we either store only or reconfigure directly
	HeartbeatConsumerTime = (((uint32_t)TxNode)<<16) | ThresholdTime;
  //disable Guarding in that case
	GuardTime = 0;
	LiveTimeFactor = 0;
	
	if(NodeState < eNMTStatePreOp)
	{
		//store the value only
		returnValue = eCO_NodeDone;
		#if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
		Serial.print("Node: HB consumer preset for :");
		Serial.print(ThresholdTime);
		Serial.println(" ms");
		#endif

	}
	else
	{
		//we are live - so configure directly
		returnValue = ActivateHeartbeat();
	} // end of else
	return returnValue;
}


/*--------------------------------------------------------------------
 * void CONode::PresetHBMissedTime(uint16_t ThresholdTime)
 * preset the HB threshold explictly
 * otherwise will be 25% of the HB time itself
 * 
 * 2025-08-22 AW Done
 * ------------------------------------------------------------------*/

void CONode::PresetHBMissedTime(uint16_t ThresholdTime)
{
	RemoteHBMissedTime = ThresholdTime;
}

/*--------------------------------------------------------------------
 * void forceNodeState(uint8_t forcedState)
 * Force the node state of this node to be the one give without any
 * commands being sent.
 * 
 * 2025-01-18 AW Done
 * ------------------------------------------------------------------*/


void CONode::forceNodeState(NMTNodeState forcedState)
{	
	NodeState = forcedState;	
}


//!!!!!!!!!!!!!!!!!! todo: Handling des Request prüfen - braucht das einen globalen Rückgabewert?

/*-------------------------------------------------------------------
 * CONodeCommStates SendResetNode();
 *
 * send a NMT command to reset specifially this node
 * reset the node state to isLive = false
 *
 * 2025-01-11 AW
 * 2025-07-28 AW change the interaction with SendRequest
 *
 *--------------------------------------------------------------------*/
CONodeCommStates CONode::SendResetNode()
{  
CONodeCommStates sendResult = eCO_NodeBusy;
	
	switch(RequestState)
	{
		//only in case of being idele a new message is composed
		case eCO_NodeIdle:
	    NmtCommand.command = NMT_ResetRemoteNode;
		  #if(DEBUG_NODE & DEBUG_NMT_StateChange)
		  Serial.println("Node: Reset Node requested");
		  #endif
		  //no break here
		case eCO_NodeRetry:					
			//send the data
			sendResult = SendRequest((CANMsg *)&NmtCommand);
		
		  if(sendResult == eCO_NodeDone)
			{
				RequestState = eCO_NodeIdle;
			  isLive = false;
	      NodeState = eNMTWaitForBoot;
				//force the two of them to be equal until we get an update
				ReportedState = eNMTWaitForBoot;
  
		    #if(DEBUG_NODE & DEBUG_NMT_StateChange)
			  Serial.println("Node: switch remote state --> reset");
				Serial.println("Node.State == eNMTWaitForBoot");
		    #endif
			}
		  break;
		default:
		  #if(DEBUG_NODE & DEBUG_NMT_StateChange)
		  Serial.println("Reset Node state unexpected");
		  #endif
			break;
	}
	return sendResult;
}

/*-------------------------------------------------------------------
 * CONodeCommStates SendResetCom();
 *
 * send a NMT command to reset the CANopen stack of specifially this node
 * reset the node state to isLive = false
 *
 * 2025-01-12 AW
 * 2025-07-28 AW change the interaction with SendRequest
 *
 *--------------------------------------------------------------------*/
CONodeCommStates CONode::SendResetCom()
{
CONodeCommStates sendResult = eCO_NodeBusy;
	
	switch(RequestState)
	{
		//only in case of being idele a new message is composed
		case eCO_NodeIdle:
	    NmtCommand.command = NMT_ResetComRemoteNode;
		  //no break here
		case eCO_NodeRetry:					
			//send the data
			sendResult = SendRequest((CANMsg *)&NmtCommand);

		  if(sendResult == eCO_NodeDone)
			{
				RequestState = eCO_NodeIdle;
			  isLive = false;
	      NodeState = eNMTWaitForBoot;
				//force the two of them to be equal until we get an update
				ReportedState = eNMTWaitForBoot;
  
		    #if(DEBUG_NODE & DEBUG_NMT_StateChange)
			  Serial.println("Node: switch remote state --> reset com");
				Serial.println("Node.State == eNMTWaitForBoot");
		    #endif
			}
		  break;
		default:
		  #if(DEBUG_NODE & DEBUG_NMT_StateChange)
		  Serial.println("Reset COM state unexpected");
		  #endif
			break;
	}
  return sendResult;
}

/*-------------------------------------------------------------------
 * CONodeCommStates SendStartNode();
 *
 * send a NMT command to switch den CANopen stack of specifially this node
 * to operational
 *
 * 2025-01-12 AW
 * 2025-07-28 AW change the interaction with SendRequest
 *
 *--------------------------------------------------------------------*/
CONodeCommStates CONode::SendStartNode()
{
CONodeCommStates sendResult = eCO_NodeBusy;
	
	switch(RequestState)
	{
		//only in case of being idele a new message is composed
		case eCO_NodeIdle:
	    NmtCommand.command = NMT_StartRemoteNode;
		  //no break here
		case eCO_NodeRetry:					
			//send the data
			sendResult = SendRequest((CANMsg *)&NmtCommand);

		  if(sendResult == eCO_NodeDone)
			{
				RequestState = eCO_NodeIdle;
	      NodeState = eNMTStateOperational;
				//force the two of them to be equal until we get an update
				ReportedState = eNMTStateOperational;
				//reset the HB rx time als it will be checked in Pre-Op or Op only
				HeatbeatReceivedAt = actTime;
  
		    #if(DEBUG_NODE & DEBUG_NMT_StateChange)
			  Serial.print("Node: switch remote state --> start @ ");
				Serial.println(actTime);
				Serial.print("Node.State == eNMTStateOperational: ");
				Serial.println(NodeState);
		    #endif
			}
		  break;
		default:
		  #if(DEBUG_NODE & DEBUG_NMT_StateChange)
		  Serial.println("Start Node state unexpected");
		  #endif
			break;
	}
  return sendResult;
}

/*-------------------------------------------------------------------
 * CONodeCommStates SendStopNode();
 *
 * send a NMT command to switch den CANopen stack of specifially this node
 * to stopped state
 *
 * 2025-01-12 AW
 * 2025-07-28 AW change the interaction with SendRequest
 *
 *--------------------------------------------------------------------*/
CONodeCommStates CONode::SendStopNode()
{
CONodeCommStates sendResult = eCO_NodeBusy;
	
	switch(RequestState)
	{
		//only in case of being idele a new message is composed
		case eCO_NodeIdle:
	    NmtCommand.command = NMT_StopRemoteNode;
		  //no break here
		case eCO_NodeRetry:					
			//send the data
			sendResult = SendRequest((CANMsg *)&NmtCommand);

		  if(sendResult == eCO_NodeDone)
			{
				RequestState = eCO_NodeIdle;
	      NodeState = eNMTStateStopped;
				//force the two of them to be equal until we get an update
				ReportedState = eNMTStateStopped;
  
  		  #if(DEBUG_NODE & DEBUG_NMT_StateChange)
			  Serial.println("Node: switch remote state --> stop");
				Serial.println("Node.State == eNMTStateStopped");
		    #endif
			}
		  break;
		default:
		  #if(DEBUG_NODE & DEBUG_NMT_StateChange)
		  Serial.println("Stop Node state unexpected");
		  #endif
			break;
	}
  return sendResult;
}

/*-------------------------------------------------------------------
 * CONodeCommStates SendPreopNode();
 *
 * send a NMT command to switch den CANopen stack of specifially this node
 * to pre-operational state
 *
 * 2025-01-12 AW
 * 2025-07-28 AW change the interaction with SendRequest
 *
 *--------------------------------------------------------------------*/
CONodeCommStates CONode::SendPreopNode()
{
CONodeCommStates sendResult = eCO_NodeBusy;
	
	switch(RequestState)
	{
		//only in case of being idele a new message is composed
		case eCO_NodeIdle:
	    NmtCommand.command = NMT_EnterPreop;
		  //no break here
		case eCO_NodeRetry:					
			//send the data
			sendResult = SendRequest((CANMsg *)&NmtCommand);

		  if(sendResult == eCO_NodeDone)
			{
				RequestState = eCO_NodeIdle;
	      NodeState = eNMTStatePreOp;
				//force the two of them to be equal until we get an update
				ReportedState = eNMTStatePreOp;
				//reset the HB rx time als it will be checked in Pre-Op or Op only
				HeatbeatReceivedAt = actTime;
  
		    #if(DEBUG_NODE & DEBUG_NMT_StateChange)
			  Serial.print("Node: switch remote state --> pre-op @");
				Serial.println(actTime);
		    #endif
			}
		  break;
		default:
		  #if(DEBUG_NODE & DEBUG_NMT_StateChange)
		  Serial.println("Request PreOp state unexpected");
		  #endif
			break;
	}
	return sendResult;
}

//--------------------------------------------------------------------
// --- private functions ---
//--------------------------------------------------------------------

/*-------------------------------------------------------------------
 * void CONode::OnTimeOut()
 * 
 * whateer needs to be done in case of a time-out
 * 
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

void CONode::OnTimeOut()
{
	;
}

/*-------------------------------------------------------------------
 * bool CONode::SendRequest(CANMsg *Msg)
 * 
 * proess and send a CANMsg
 * enter a retry when momentarily blocked
--- todo: Rückgababewert prüfen!!!
--- muss der global sein?
 * 
 * 25-03-09 AW 
 *
 *-------------------------------------------------------------------*/

CONodeCommStates CONode::SendRequest(CANMsg *Msg)
{
	bool result = Handler->SendMsg(Msg);	
	
	if(result)
	{
		//if we were able to send, the service is done
	  RequestState = eCO_NodeDone;
		BusyRetryCounter = 0;				

		#if(DEBUG_NODE & DEBUG_NMT_TXCS)
		Serial.print("Node: N ");
		Serial.print(Handler->GetNodeId(Channel),DEC);
		Serial.print(" TxReq ok ");
		Serial.println(Msg->Id, HEX);
		#endif
		}
	else
	{
		BusyRetryCounter++;
		if(BusyRetryCounter > BusyRetryMax)
		{
			RequestState = eCO_NodeError;
			isTimerActive = false;

			#if(DEBUG_NODE & DEBUG_NMT_ERROR)
			Serial.print("Node: N ");
			Serial.print(Handler->GetNodeId(Channel),DEC);
			Serial.println(" TxReq failed");
			#endif
		}
		else
		{
			RequestState = eCO_NodeRetry;
			isTimerActive = false;
			
			#if(DEBUG_NODE & DEBUG_NMT_BUSY)
			Serial.print("Node: N ");
			Serial.print(Handler->GetNodeId(Channel),DEC);
			Serial.println(" TxReq busy");
			#endif
		}
	}
	return RequestState;
}	

//--- some LL methods for the still unhandled messages ---------------------------------------------

/*------------------------------------------------------------------
 * void CONode::EmcyHandler(CANMsg *Msg)
 * 
 * get the parts of the Emcy Msg and assing them to the loval variables
 * 
 * 2025-08-21 AW Done
 * ----------------------------------------------------------------*/

void CONode::EmcyHandler(CANMsg *Msg)
{	
  EmcyCode = (((uint16_t)(Msg->payload[1])) << 8) | ((uint16_t)(Msg->payload[0]));
	FAULHABERErrorWord = Msg->payload[2];
	CiA301ErrorWord = (((uint16_t)(Msg->payload[4])) << 8) | ((uint16_t)(Msg->payload[3]));
	
	#if(NODE_PrintEMCY)
	PrintEMCY();
	#endif
}

/*------------------------------------------------------------------
 * void CONode::EmcyHandler(CANMsg *Msg)
 * 
 * get the parts of the Emcy Msg and assing them to the loval variables
 * 
 * 2025-08-21 AW Done
 * ----------------------------------------------------------------*/

void CONode::PrintEMCY()
{
	if(EmcyCode > 0)
	{
	  Serial.print("Node: ");
	  Serial.print(NodeId);
	  Serial.print(" EMCY: ");
	  Serial.print(EmcyCode,HEX);
	  Serial.print(" Error word: ");
	  Serial.print(FAULHABERErrorWord);
		Serial.print(" CiA Error: ");
		Serial.println(CiA301ErrorWord);
	}
	else
	{
	  Serial.print("Node: ");
	  Serial.print(NodeId);
	  Serial.println(" Error cleared");
	}
}

		
/*------------------------------------------------------------------
 * void OnRxHandler(MCMsg *Msg)
 * React to any received SysMsg. This callback will be regsitered at the
 * MsgHandler and will deal with any received NMT-Msg:
 *   - Boot Msg
 *   - Guardnig Response
 *   - HB Indication
 * 
 * 2025-03-01 AW Done
 * ----------------------------------------------------------------*/

void CONode::OnRxHandler(CANMsg *Msg)
{	
  if((Msg->Id == GuardingRequest.Id) && (Msg->len == NMTGuardingFrameLength))	
	{
		if(Msg->payload[0] == 0)
		{
			//this is a boot-msg
			NodeState = eNMTBootMsgReceived;
			//the node is back, but unconfigured
			isLive = true;			
			//here Guarding and HB would need to be configured
			isGuardingActive = false;
			isHeartbeatActive = false;
			ConfigStep = 0;
			
			#if(DEBUG_NODE & DEBUG_NMT_RXMSG)
			Serial.println("Node: Rx Boot");
			#endif
		}
		else
		{
			//no boot-msg
			//reaction depends on the activated mechanisms
			if((isGuardingActive) && (GuardingState < eCO_GuardingError))
			{				
				//detect toggle bit and check vs. expected
				if((Msg->payload[0] & 0x80) == expectedToggleBit)
				{
					ReportedState = (NMTNodeState)(Msg->payload[0] & 0x7F);
					
				  #if(DEBUG_NODE & DEBUG_NMT_RXMSG)
				  Serial.println("Node: Rx Guarding");
				  #endif
          
 					GuardingState = eCO_GuardingReceivedIntime;
					//toggle then expected toggle bit
					if(expectedToggleBit == 0x80)
						expectedToggleBit = 0;
					else
						expectedToggleBit = 0x80;
				}
				else
				{
					#if(DEBUG_NODE & DEBUG_NMT_RXMSG)
				  Serial.println("Node: Rx Guarding - wrong toggle");
				  #else
					;
					#endif
				}	
			}
			else if(isHeartbeatActive)
			{
				NodeState = (NMTNodeState)(Msg->payload[0] & 0x7F);
				HeatbeatReceivedAt = actTime;
				
				#if(DEBUG_NODE & DEBUG_NMT_RXMSG)
        Serial.println("Node: Rx HB");
				#endif
			}
		}  //end of not being a boot-msg
	} //end of checking this msg at all
}

/*------------------------------------------------------------------
 * CONodeCommStates ActivateGuarding()
 * Activate the configured Guarding if time > 0 and factor > 0
 * otherwise deactivate it
 * 
 * 2025-01-25 AW Frame
 * ----------------------------------------------------------------*/

CONodeCommStates CONode::ActivateGuarding()
{
  COSDOCommStates requestComState;	

	switch(ConfigStep)
	{
		case 0:
			//disable Heartbeat first
			requestComState = RWSDO.WriteSDO((ODEntry *)&ODProducerHeartbeatTime);
		
			ConfigState = eCO_NodeBusy;
			isHeartbeatActive = false;
		
			switch(requestComState)
			{
				case eCO_SDODone:
					ConfigStep = 1;

  				#if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
				  Serial.println("Node: Reset Producer HB to 0");
					#endif
					break;
				case eCO_SDOError:		
				case eCO_SDOTimeout:
					ConfigState = eCO_NodeError;
					break;
				default:
					break;
			}  //end of step 0
			break;
		case 1:
			//disable Heartbeat first
  		requestComState = RWSDO.WriteSDO((ODEntry *)&ODConsumerHeartbeatTime);
		
			ConfigState = eCO_NodeBusy;
		
			switch(requestComState)
			{
				case eCO_SDODone:
					ConfigStep = 2;

  				#if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
				  Serial.println("Node: Reset Consumer HB to 0");
					#endif
					break;
				case eCO_SDOError:		
				case eCO_SDOTimeout:
					ConfigState = eCO_NodeError;
					break;
				default:
					break;
			}  //end of step 0
			break;
		case 2:
			//now configure for Guarding
		  requestComState = RWSDO.WriteSDO((ODEntry *)&ODGuardTime);
		
			switch(requestComState)
			{
				case eCO_SDODone:
					ConfigStep = 3;

				  #if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
				  Serial.println("Node: Configure GuardTime");
					#endif
					break;
				case eCO_SDOError:		
				case eCO_SDOTimeout:
					ConfigState = eCO_NodeError;
					break;
				default:
					break;
			} //end of step 1
			break;
		case 3:
			//plus live time factor
			requestComState = RWSDO.WriteSDO((ODEntry *)&ODLiveTimeFactor);
		
			switch(requestComState)
			{
				case eCO_SDODone:
					isGuardingActive = true;
				  NumGuardRequestsOpen = 0;
				  expectedToggleBit = 0;
				
				  GuardingState = eCO_GuardingConfigured;

				  #if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
				  Serial.println("Node: Configure Livetime factor");
					#endif

					//done here
					ConfigState = eCO_NodeDone;
					ConfigStep = 0;
					break;
				case eCO_SDOError:		
				case eCO_SDOTimeout:
					ConfigState = eCO_NodeError;
					break;
				default:
					break;
			} //end of step 2		
      break;		
		default:			
			break;
	}  //end of step sequence	
	return ConfigState;
}

/*------------------------------------------------------------------
 * CONodeCommStates ActivateHeartbeat()
 * Activate the configured Heartbeat if time > 0
 * otherwise deactivate it
 * 
 * 2025-01-25 AW Frame
 * ----------------------------------------------------------------*/
CONodeCommStates CONode::ActivateHeartbeat()
{
	COSDOCommStates requestComState;
	
	switch(ConfigStep)
	{
		case 0:
			//disable Gurading first
			requestComState = RWSDO.WriteSDO((ODEntry *)&ODGuardTime);

		  ConfigState = eCO_NodeBusy;
			isGuardingActive = false;
		
			switch(requestComState)
			{
				case eCO_SDODone:
					ConfigStep = 1;

				  #if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
				  Serial.println("Node: Reset Guard Time to 0");
					#endif
					break;
				case eCO_SDOError:		
				case eCO_SDOTimeout:
					ConfigState = eCO_NodeError;
					break;
				default:
					break;
			}  //end of step 0
			break;
		case 1:
			//reset LiceTime Factor
			requestComState = RWSDO.WriteSDO((ODEntry *)&ODLiveTimeFactor);
		
			switch(requestComState)
			{
				case eCO_SDODone:
					ConfigStep = 2;

				  #if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
				  Serial.println("Node: Reset LiveTimeFactor");
					#endif
					break;
				case eCO_SDOError:		
				case eCO_SDOTimeout:
					ConfigState = eCO_NodeError;
					break;
				default:
					break;
			} //end of step 1
			break;
		case 2:
			//set HeartBeatTime
			requestComState = RWSDO.WriteSDO((ODEntry *)&ODProducerHeartbeatTime);
		
			switch(requestComState)
			{
				case eCO_SDODone:
					ConfigStep = 3;
				
					#if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
				  Serial.println("Node: Configure HB producer");
					#endif

					break;
				case eCO_SDOError:		
				case eCO_SDOTimeout:
					ConfigState = eCO_NodeError;
					break;
				default:
					break;
			} //end of step 2	
      break;
		case 3:
			//set Consumer HeartBeatTime
			requestComState = RWSDO.WriteSDO((ODEntry *)&ODConsumerHeartbeatTime);
		
			switch(requestComState)
			{
				case eCO_SDODone:
					isHeartbeatActive = true;
				  GuardingState = eCO_GuardingConfigured;
					//we reset the time for BH to now for the first round
				  HeatbeatReceivedAt = actTime;

				  #if(DEBUG_NODE & DEBUG_NMT_ConfigGuard)
				  Serial.println("Node: Configure HB consumer");
					#endif

					//done here
					ConfigState = eCO_NodeDone;
					ConfigStep = 0;
					break;
				case eCO_SDOError:		
				case eCO_SDOTimeout:
					ConfigState = eCO_NodeError;
					break;
				default:
					break;
			} //end of step 2	
      break;
		default:
      break;					
	}  //end of step sequence	
	return ConfigState;
}


/*------------------------------------------------------------------
 * SDOCommStates CheckSDOState()
 * Read-acces to the ComState of the built-in SDOHandler
 * 
 * 2020-11-21 AW Done
 * ----------------------------------------------------------------*/

COSDOCommStates CONode::GetSDOState()
{
	//check the SDOState
	return RWSDO.GetComState();
}