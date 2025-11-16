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

/*----------------------------------------------------------------------
 *
 * CO_IONode_A
 *
 * Add Gloval Sync and HB
 * remove LL CAN
 * Add PDOHandler
 * Add PDO-based behavior
 * remove local SDO access
 * add test moves
 * switch to CODrive.Update rather than explicit inits
 * move EMCY handler to Node
 *
 * 2025-11-15 AW
 *
 *------------------------------------------------------------------------*/

//---- includes -------------------------------------------------------------
#include <COMsgHandler.h>
#include <CO401Node.h>
#include <COSyncHandler.h>

//---- local definitions -----------------------------------------------

#define DEBUG_Master_Node 0x0001
#define DEBUG_Master_Sync 0x0002
#define DEBUG_Master_PDO  0x0004

#define DEBUG_Master (DEBUG_Master_Node | DEBUG_Master_Sync | DEBUG_Master_PDO)

//---- select which type of guarding to be used ----------------------------------

#define UseNodeGuarding
//#define UseHeartBeat

//--------------------------------------------------------------------------------------------
//--- Defines and instances for ths COMsghandler ---------------------------------------------

COMsgHandler MsgHandler(R4WiFiTx, R4WiFiRx, CanBitRate::BR_250k);

//--------------------------------------------------------------------------------------------
//--- Define and instances for this master node ----------------------------------------------

typedef enum TestMasterStates {
  eMaster_Unconfigured,
  eMaster_ConfigurePDOs,
  eMaster_StartNodes,
	eMaster_operable,
	eMaster_Busy,
	eMaster_Done,
	eMaster_Error
} TestMasterStates;

typedef enum CommandTypes {
  eCmdNone,
  eCmdSetDigOut,
  eCmdResetDigOut,
  eCmdGetDigIn,
  eCmdReadIdentity,
  eCmdReadObject,
  eCmdWriteObject,
  eCmdWriteAnValue,
  eCmdReadAnValue
} CommandTypes;

TestMasterStates MasterState = eMaster_Unconfigured; 

const uint8_t MasterNodeId = 0x7F;

CommandTypes Command = eCmdNone;
uint8_t AccesStep = 0;

uint8_t TestDigOut = 0x55;
uint8_t DigOutLocal = 0;
uint8_t DriveDigIn = 0;
int16_t localAnIn = 0;
int16_t remoteAnIn = 0;

ODEntry08 OdDigInStatusLocal = {0x6000, 0x01, &(DriveDigIn), 1};
ODEntry08 OdDigOutLocal = {0x6200,0x01,&DigOutLocal,1};

//--------------------------------------------------------------------------------------------
//--- Define and instances for this master - handling SYNC------------------------------------

uint16_t SyncInterval = 1000;
COSyncHandler SyncHandler(MasterNodeId);

//--------------------------------------------------------------------------------------------
//--- Define and instances for the first node ------------------------------------------------

const uint16_t TestNodeNode = 8;
const uint16_t GuardTime = 2000;
const uint8_t LiveTimeFactor = 3;
const uint16_t HBThreshold = 20;

CO401Node Node_A(TestNodeNode);

//--- setup ---------------------------------------------

void setup() {
  // put your setup code here, to run once:

  uint32_t stime = millis();
  uint8_t NodeHandle;

  Serial.begin(115200);
  while (!Serial && (millis() - stime < 5000)) {};

  Serial.println();
  Serial.println("> Arduino UNO R4 CAN I/O Node test sketch A");

  //now register all services of all drives

  //finally open the CAN interface
  MsgHandler.Open();

  //register the MsgHandler at the SyncHandler
  //is used to dirctly sednm Sync and HB
  SyncHandler.init(&MsgHandler);
  SyncHandler.SyncInterval = SyncInterval;
  // force the state of the Sync to be Pre-Op now
  SyncHandler.SetState(eSyncStatePreOp);

  //init the node itself
  Node_A.init(&MsgHandler);

  //this node seems to be a little bit slow
  Node_A.Node.RWSDO.SetBusyRetryMax(30);
  //the BK5120 needs about 30ms for a mapping
  Node_A.PDOHandler.SetPDOConfigTimeout(50);

  #ifdef UseNodeGuarding
  Node_A.Node.ConfigureGuarding(GuardTime,LiveTimeFactor);
  #endif
  #ifdef UseHeartBeat
  Node_A.Node.ConfigureRemoteHeartbeatProducer(GuardTime);
  Node_A.Node.PresetHBMissedTime(GuardTime + HBThreshold);
  Node_A.Node.ConfigureRemoteHeartbeatConsumer(MasterNodeId, GuardTime + HBThreshold);
  SyncHandler.ProducerHBTime = GuardTime;
  #endif

  Node_A.PDOHandler.PresetRxPDOisValid(1,true);
  Node_A.PDOHandler.PresetTxPDOisValid(1,true);

  Node_A.PDOHandler.PresetRxPDOTransmission(0, 255);  //parameters are the PDO# and the transmission type
  Node_A.PDOHandler.PresetRxPDOTransmission(1, 1);  //parameters are the PDO# and the transmission type
	Node_A.PDOHandler.PresetTxPDOTransmission(0, 1);  //parameters are the PDO#, the transmission type
  Node_A.PDOHandler.PresetTxPDOTransmission(1, 1);  //parameters are the PDO#, the transmission type
}
 
void loop() 
{
  // put your main code here, to run repeatedly:
  uint32_t actTime = millis();

  MsgHandler.Update(actTime);
  COSyncState syncState = SyncHandler.Update(actTime);
  NMTNodeState Node_A_NodeState = Node_A.Update(actTime, syncState);

  if(Node_A_NodeState == eNMTStatePreOp)
  {
    if((Node_A.isPDOsConfigured) && (Node_A.autoEnable == false))
    {
    Serial.println("Main: TestNode started");
    Node_A.autoEnable = true;
    SyncHandler.SetState(eSyncStateOperational);
    //SyncHandler.SendStartNodes();
    }
  }
  if(Node_A_NodeState == eNMTStateOperational)
  {
    if(syncState == eSyncSyncSent)
    {
      localAnIn = (analogRead(A0))<<4;
      Command = eCmdWriteAnValue;
    }
    else if (Serial.available()) 
    {
      CANMsg TxMsg;
      char c = Serial.read();

      switch(c)
      {
        case 's':   //--- set outputs ----       
          Command = eCmdSetDigOut;  
          break;
        case 'r':   //--- reset outputs ---       
          Command = eCmdResetDigOut;  
          break;
        case 'g':   //--- get inptus ---       
          Command = eCmdGetDigIn;  
          break;
        case 'o':   //---read object  ---
          Command = eCmdReadObject;
          break;
        case 'i':   //--- identify node ---
          Command = eCmdReadIdentity;
          break;
        case 'w':   //---write DigOut ----
          Command = eCmdWriteObject;
          break;
        case '?': //help
          Serial.println("----------------------------------------");
          Serial.println("Commands are:");
          Serial.println("s: set outputs to default");
          Serial.println("r: reset outputs to 0");
          Serial.println("g: get DigIn Status");
          Serial.println("----------------------------------------");
          Serial.println("--> via SDO <--");
          Serial.println("o: read DigIn object via SDO");
          Serial.println("w: write DigOut object to default valuevia SDO");
          Serial.println("i: read & print device identity");
          Serial.println("----------------------------------------");
          Serial.println("?: print commands");
          break;
        default:
          break;
      }
    }

    switch(Command)
    {
      case eCmdNone:
        break;
      case eCmdSetDigOut:
        if(Node_A.SetDigOut(TestDigOut) == eCO_IODone)
        {
          Command = eCmdNone;
          Serial.println("Main: Default DigOut set");
        }
        break;
      case eCmdResetDigOut:
        if(Node_A.SetDigOut(0x00) == eCO_IODone)
        {
          Command = eCmdNone;
          Serial.println("Main: DigOut reset");
        }
        break;
      case eCmdGetDigIn:
        if(Node_A.GetDigInStatus(&DriveDigIn) == eCO_IODone)
        {
          Command = eCmdNone;
          Serial.print("Main: DigIn: ");
          Serial.println(DriveDigIn, HEX);
        }
        break;
      case eCmdReadIdentity:
        if(Node_A.IdentifyIONode() == eCO_IODone)
        {
          Node_A.PrintIdentityObjects();
          Command = eCmdNone;   
        }
        break;
      case eCmdReadObject:
        if(Node_A.GetNumObject(&OdDigInStatusLocal) == eCO_IODone)
        {
          Serial.print("Main: DigIn Status = ");
          Serial.println(DriveDigIn, HEX);
          Command = eCmdNone;   
        }
        break;
      case eCmdWriteObject:
        if(Node_A.SetNumObject(&OdDigOutLocal, TestDigOut) == eCO_IODone)
        {
          Serial.print("Main: DigOut set to = ");
          Serial.println(TestDigOut, HEX);
          Command = eCmdNone;   
        }
        break;
      case eCmdWriteAnValue:
        if(Node_A.SetRemoteAnOut(0,localAnIn) == eCO_IODone)
        {
          Serial.print("Main: analog local: ");
          Serial.println(localAnIn);

          //now lewt's update the remote input
          Command = eCmdReadAnValue;
        }
        //no break needed here
        //break;
      case eCmdReadAnValue:
        if(Node_A.GetRemoteAnIn(0,&remoteAnIn) == eCO_IODone)
        {
          Serial.print("Main: analog remote: ");
          Serial.println(remoteAnIn);

          Command = eCmdNone;
        }
        break;
      default:
        break;
    }
  }
  delay(1);
}
