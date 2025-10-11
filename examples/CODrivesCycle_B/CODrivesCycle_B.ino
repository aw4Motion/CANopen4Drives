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
 * CODrivesCycle_B
 *
 * use 4 drives and cycle them in a fixed cycle
 *
 * 2025-08-23 AW
 * 2025-09-03 AW reset the cyle if a node fails
 *               test cycle moved into a class
 *
 *------------------------------------------------------------------------*/

//---- includes -----------------------------------------------------------
#include <CO402Drive.h>
#include <COSyncHandler.h>
#include <COTestCycle.h>

//---- local definitions ---------------------------------------------------

#define DEBUG_Master_Node  0x0001
#define DEBUG_Master_Sync  0x0002
#define DEBUG_Master_Init  0x0004
#define DEBUG_Master_Steps 0x0008

#define DEBUG_Master (DEBUG_Master_Node | DEBUG_Master_Sync | DEBUG_Master_Init)

//------ select the type of node guarding -----------------------------

#define UseNodeGuarding
//#define UseHeartBeat

//--------------------------------------------------------------------------------------------
//--- Defines and instances for ths COMsghandler ---------------------------------------------

COMsgHandler MsgHandler(R4WiFiTx, R4WiFiRx, CanBitRate::BR_250k);

//--------------------------------------------------------------------------------------------
//--- Define and instances for this master node ----------------------------------------------

const uint8_t MasterNodeId = 0x7F;
const uint8_t NumNodes = 1;

uint8_t NodeUpdated = 0;
uint8_t SteppedNode = 0;

uint8_t NodeUnderConfig = NumNodes;

uint8_t AllNodesFinished = 0;

uint8_t NodeConfigFlags = 0;
uint8_t StepDoneFlags = 0;

uint8_t ConfigStep = 0;
uint8_t CycleStep = 0;
uint32_t targetStep = 0;

NMTNodeState DriveNodeStates[NumNodes];

//--------------------------------------------------------------------------------------------
//--- Define and instances for this master - handling SYNC------------------------------------

uint16_t SyncInterval = 100;
COSyncHandler SyncHandler(MasterNodeId);

//--------------------------------------------------------------------------------------------
//--- Define and instances for the first node ------------------------------------------------

const uint16_t GuardTime = 1000;
const uint8_t LiveTimeFactor = 3;
const uint16_t HBThreshold = 20;

//--------------------------------------------------------------------------------------------
//--- local instances for the test purpose ----------------------------------------------------

const uint8_t Node_A_RefSwitch = 2;
const uint8_t Node_B_RefSwitch = 3;
const uint8_t Node_C_RefSwitch = 4;
const uint8_t Node_D_RefSwitch = 5;

const int8_t HomingMethod = 4;

CO402Drive Drive_A(1), Drive_B(2), Drive_C(3), Drive_D(4);
CO402Drive *Drives[4] = {&Drive_A, &Drive_B, &Drive_C, &Drive_D};

COTestCycle Cycle_A(HomingMethod, Node_A_RefSwitch);
COTestCycle Cycle_B(HomingMethod, Node_B_RefSwitch);
COTestCycle Cycle_C(HomingMethod, Node_C_RefSwitch); 
COTestCycle Cycle_D(HomingMethod, Node_D_RefSwitch);

COTestCycle *Cycle[4] = {&Cycle_A, &Cycle_B, &Cycle_C, &Cycle_D};

//--------------------------------------------------------------------------------------------
//--- local methods -------------------------------------

//--------------------------------------------------------------------------------------------
//--- setup ---------------------------------------------

void setup() {
  // put your setup code here, to run once:

  uint32_t stime = millis();
  uint8_t NodeHandle;

  Serial.begin(115200);
  while (!Serial && ((millis() - stime) < 5000)) {};

  Serial.println();
  Serial.println("> Arduino UNO R4 CAN test Test-Cycle");

  //now register all services of all drives

  //finally open the CAN interface
  MsgHandler.Open();

  //register the MsgHandler at the SyncHandler
  //is used to dirctly sednm Sync and HB
  SyncHandler.init(&MsgHandler);
  SyncHandler.SyncInterval = SyncInterval;
  #ifdef UseHeartBeat
  SyncHandler.ProducerHBTime = GuardTime;
  #endif
  // force the state of the Sync to be Pre-Op now
  SyncHandler.SetState(eSyncStatePreOp);
  
  //init the node itself
  for(uint8_t iter = 0; iter < NumNodes; iter++)
  {
    Drives[iter]->init(&MsgHandler);
    AllNodesFinished |= (0x01 << iter);

    #ifdef UseNodeGuarding
    Drives[iter]->Node.ConfigureGuarding(GuardTime,LiveTimeFactor);
    #endif
    #ifdef UseHeartBeat
    Drives[iter]->Node.ConfigureRemoteHeartbeatProducer(GuardTime);
    Drives[iter]->Node.PresetHBMissedTime(GuardTime + HBThreshold);
    Drives[iter]->Node.ConfigureRemoteHeartbeatConsumer(MasterNodeId, GuardTime + HBThreshold);
    #endif
  }
  delay(5000);
}
 
void loop() 
{
  // put your main code here, to run repeatedly:
  uint32_t actTime = millis();

  MsgHandler.Update(actTime);
  COSyncState syncState = SyncHandler.Update(actTime);

  if (Serial.available()) 
  {
    char c = Serial.read();

    switch(c)
    {
      case '0':  //--start cycle--
        break;
      case '1':
        break;
      default:
        break;
    }
  }

  //now process all nodes

  //depending on the init state either config the nodes or run them
  //1st: config nodes
  if(NodeConfigFlags < AllNodesFinished)
  {
    DriveNodeStates[NodeUpdated] = Drives[NodeUpdated]->Update(actTime, syncState);

    switch(ConfigStep)
    {
      case 0: // find and condigure nodes
        //starts only if already pre-op
        if(DriveNodeStates[NodeUpdated] == eNMTStatePreOp)
        {
          Drives[NodeUpdated]->autoEnable = false;

          if(Drives[NodeUpdated]->isPDOsConfigured)
          {
            NodeConfigFlags |= (0x01 << NodeUpdated);
            Drives[NodeUpdated]->reConfigPDOs = false;

            if(NodeUpdated == NodeUnderConfig)
            {
              NodeUnderConfig = NumNodes;

              #if(DEBUG_Master & DEBUG_Master_Init)
              Serial.print("PDOs @ Node ");
              Serial.print(NodeUpdated);
              Serial.println(" configured");
              #endif
            }
          }
          else if(NodeUpdated == NumNodes)
          {          
            //if so, there is no code under configration right now
            NodeUnderConfig = NodeUpdated;
            Drives[NodeUpdated]->reConfigPDOs = true;
          }
        }
        else if (DriveNodeStates[NodeUpdated] == eNMTStateOperational)
        {
          //this one seems to be still good
          //send it into pre-op
          Drives[NodeUpdated]->Node.SendPreopNode();
        }
        if(NodeConfigFlags == AllNodesFinished)
        {
          ConfigStep = 1;
          SyncHandler.SetState(eSyncStateOperational); 
          
          #if(DEBUG_Master & DEBUG_Master_Init)
          Serial.println("Main: all at least pre-op");
          #endif

          NodeConfigFlags = 0;
        }
        break;
      case 1: //allow them to start
        if(Drives[NodeUpdated]->autoEnable == false)
        {
          Drives[NodeUpdated]->autoEnable = true;
          NodeConfigFlags |= (0x01 << NodeUpdated);

          #if(DEBUG_Master & DEBUG_Master_Init)
          Serial.print("Main: Node ");
          Serial.print(NodeUpdated);
          Serial.println(" flagged for start");
          #endif
        }
        if(NodeConfigFlags == AllNodesFinished)
        {
          ConfigStep = 2;
          NodeConfigFlags = 0;
          Serial.println("Main: all flagged for start or started");
        }
        break;
      case 2:
        if (DriveNodeStates[NodeUpdated] == eNMTStateOperational)
        {
          NodeConfigFlags |= (0x01 << NodeUpdated);
        }
        if(NodeConfigFlags == AllNodesFinished)
        {
          ConfigStep = 0;
          CycleStep = 0;
          StepDoneFlags = 0;

          Serial.println("Main: config done for all");
        }

        break;
      default:
        Serial.println("Main: unexpected ConfigStep");
        break;
    }
    if(NodeConfigFlags < AllNodesFinished)
    {
      NodeUpdated++;      
      if(NodeUpdated == NumNodes)
        NodeUpdated = 0;
    }
    else
      NodeUpdated = 0;
  }
  else
  {
    //all nodes had reached operational mode
    //update only one node per loop
    DriveNodeStates[NodeUpdated] = Drives[NodeUpdated]->Update(actTime, syncState);
    
    if(DriveNodeStates[NodeUpdated] != eNMTStateOperational)
    {
      NodeConfigFlags = 0;
      
      Serial.print("Main: Node ");
      Serial.print(NodeUpdated);
      Serial.print(" @ ");
      Serial.print(Drives[NodeUpdated]->GetNodeId());
      Serial.println(" restarted");
      Drives[NodeUpdated]->isPDOsConfigured = false;
      SyncHandler.SetState(eSyncStatePreOp); 

      StepDoneFlags = 0;
    }
    else
    {
      switch(CycleStep)
      {
        case 0:
          if(Cycle[SteppedNode]->AlignDrive(Drives[SteppedNode]))
          {
            CycleStep = 1;
            Serial.print("Main: node ");
            Serial.print(SteppedNode);
            Serial.println(" aligned");
          }
          break;
        case 1:
          if(Cycle[SteppedNode]->FirstMove(Drives[SteppedNode]))
          {
            StepDoneFlags |= (0x01 << SteppedNode);

            Serial.print("Main: node ");
            Serial.print(SteppedNode);
            Serial.println(" @ 0");

            SteppedNode++;      
            if(SteppedNode == NumNodes)
              SteppedNode = 0;

            if(StepDoneFlags == AllNodesFinished)
            {
              StepDoneFlags = 0;
              targetStep = 1;
              CycleStep = 2;
              SteppedNode = 0;
            }
            else
              CycleStep = 0;
          }
          break;
        case 2:
          if(Cycle[SteppedNode]->DoCycle(Drives[SteppedNode], targetStep) == targetStep)
            StepDoneFlags |= (0x01 << SteppedNode);

          if(StepDoneFlags == AllNodesFinished)
          {
            targetStep++;
            if(targetStep > Cycle[SteppedNode]->MaxStep)
              targetStep = 1;

            StepDoneFlags = 0;
            
            #if(DEBUG_Master & DEBUG_Master_Steps)
            Serial.print("Main: Step done. Next: ");
            Serial.println(targetStep);
            #endif

          }
          //next one to be updated
          SteppedNode++;      
          if(SteppedNode == NumNodes)
            SteppedNode = 0;

          break;
        default:
          Serial.println("Main: unexpected CycleStep");
          break;
      }
    }
    //cycle the nodes for the NMT-Update
    NodeUpdated++;      
    if(NodeUpdated == NumNodes)
      NodeUpdated = 0;

  }
  delay(1);
}
