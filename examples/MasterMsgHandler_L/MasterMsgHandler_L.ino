#include <COMsgHandler.h>
#include <CO402Drive.h>
#include <COSyncHandler.h>

/*----------------------------------------------------------------------
 *
 * MastermsgHandler_L
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
 * 2025-08-21 AW
 *
 *------------------------------------------------------------------------*/

#define DEBUG_Master_Node 0x0001
#define DEBUG_Master_Sync 0x0002
#define DEBUG_Master_PDO  0x0004

#define DEBUG_Master (DEBUG_Master_Node | DEBUG_Master_Sync | DEBUG_Master_PDO)

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
	eMaster_Error,
} TestMasterStates;

typedef enum TargetState {
  eDrive_SwitchedOff,
  eDrive_ResetError,
  eDrive_SwitchedOn,
  eDrive_Stopped,
  eDRive_Enabled
} TargetState;

typedef enum CommandTypes {
  eCmdMoveIdle,
  eCmdMovePositive,
  eCmdMoveNegative,
  eCmdMoveAbs0,
  eCmdMoveRel,
  eCmdMoveHome,
  eCmdSetProfile,
  eCmdReadIdentity,
  eCmdReadInput,
  eCmdReadObject
} CommandTypes;

TestMasterStates MasterState = eMaster_Unconfigured; 

const uint8_t MasterNodeId = 0x7F;
const uint32_t MaxProfileSpeed = 5000;
const uint32_t MinProfileSpeed = 500;
const uint32_t DeltaProfileSpeed = 500;

TargetState ActDriveState = eDrive_SwitchedOff;
TargetState TargetDriveState = eDrive_SwitchedOff;
CommandTypes Command = eCmdMoveIdle;
uint8_t AccesStep = 0;

int32_t ActTestMove = 20000;
uint32_t ActProfileSpeed = MaxProfileSpeed;
uint32_t ActProfileAcc = 1500;
uint32_t ActProfileDec = 500;

uint8_t DriveDigIn = 0;
const uint8_t Node_A_RefSwitch = 2;

//--------------------------------------------------------------------------------------------
//--- Define and instances for this master - handling SYNC------------------------------------

uint16_t SyncInterval = 100;
COSyncHandler SyncHandler(MasterNodeId);

//--------------------------------------------------------------------------------------------
//--- Define and instances for the first node ------------------------------------------------

const uint16_t TestNodeNode = 1;
const uint16_t GuardTime = 500;
const uint8_t LiveTimeFactor = 3;
const uint16_t HBThreshold = 20;

CO402Drive Drive_A(TestNodeNode);

//--- an example for a local object ---
int16_t MotorTemp;
ODEntry16 OdMotorTemp = {0x2326,0x03,(uint16_t *)&MotorTemp,2};

//--- setup ---------------------------------------------

void setup() {
  // put your setup code here, to run once:

  uint32_t stime = millis();
  uint8_t NodeHandle;

  Serial.begin(115200);
  while (!Serial && (millis() - stime < 5000)) {};

  Serial.println();
  Serial.println("> Arduino UNO R4 CAN test sketch L");

  //configure the test output
  pinMode(Node_A_RefSwitch, OUTPUT);  
  digitalWrite(Node_A_RefSwitch, LOW);

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
  Drive_A.init(&MsgHandler);

  #ifdef UseNodeGuarding
  Drive_A.Node.ConfigureGuarding(GuardTime,LiveTimeFactor);
  #endif
  #ifdef UseHeartBeat
  Drive_A.Node.ConfigureRemoteHeartbeatProducer(GuardTime);
  Drive_A.Node.PresetHBMissedTime(GuardTime + HBThreshold);
  Drive_A.Node.ConfigureRemoteHeartbeatConsumer(MasterNodeId, GuardTime + HBThreshold);
  SyncHandler.ProducerHBTime = GuardTime;
  #endif

  //Drive_A.PDOHandler.PresetRxPDOTransmission(0, 1);  //parameters are the PDO# and the transmission type
	Drive_A.PDOHandler.PresetTxPDOTransmission(0, 1);  //parameters are the PDO#, the transmission type
  Drive_A.PDOHandler.PresetTxPDOTransmission(1, 1);  //parameters are the PDO#, the transmission type
}
 
void loop() 
{
  // put your main code here, to run repeatedly:
  uint32_t actTime = millis();


  MsgHandler.Update(actTime);
  COSyncState syncState = SyncHandler.Update(actTime);
  NMTNodeState DriveANodeState = Drive_A.Update(actTime, syncState);

  if(DriveANodeState == eNMTStatePreOp)
  {
    if((Drive_A.isPDOsConfigured) && (Drive_A.autoEnable == false))
    {
    Serial.println("Main: TestNode started");
    Drive_A.autoEnable = true;
    SyncHandler.SetState(eSyncStateOperational);
    //SyncHandler.SendStartNodes();
    }
  }
  if(DriveANodeState == eNMTStateOperational)
  {
    if (Serial.available()) 
    {
      CANMsg TxMsg;
      char c = Serial.read();

      switch(c)
      {
        case 'e':  //--enable--
          if(ActDriveState < eDRive_Enabled)
          {
            //Serial.print("Main: SW: ");
            //Serial.println(Drive_A.GetStatusWord(), HEX);
            Serial.println("Main: enable drive");
            TargetDriveState = eDRive_Enabled;
          }
          break;
        case 'd':  //--disable--
          if(ActDriveState == eDRive_Enabled)
          {
            //Serial.print("Main: SW: ");
            //Serial.println(Drive_A.GetStatusWord(), HEX);
            Serial.println("Main: disable drive");
            TargetDriveState = eDrive_SwitchedOn;
          }
          break;
        case 's':  //--stop ---
          if(ActDriveState == eDRive_Enabled)
          {
            //Serial.print("Main: SW: ");
            //Serial.println(Drive_A.GetStatusWord(), HEX);
            Serial.println("Main: stop drive");
            TargetDriveState = eDrive_Stopped;
          }
          break;
        case 'o':  //--off ---
          if(ActDriveState > eDrive_SwitchedOff)
          {
            //Serial.print("Main: SW: ");
            //Serial.println(Drive_A.GetStatusWord(), HEX);
            Serial.println("Main: switch drive off");
            TargetDriveState = eDrive_SwitchedOff;
          }
          break;
        case 'r':  // Fault reset
          TargetDriveState = eDrive_ResetError;
          break;
        case 'p':  //--pos move--
          if(ActDriveState == eDRive_Enabled)
          {
            Command = eCmdMovePositive;  
          }          
          break;
        case 'n':  //--neg move--          
          if(ActDriveState == eDRive_Enabled)
          {
            Command = eCmdMoveNegative;  
          }          
          break;
        case '0':  //--moce to 0--          
          if(ActDriveState == eDRive_Enabled)
          {
            Command = eCmdMoveAbs0;  
          }          
          break;
        case 'm':  //--start move--          
          if(ActDriveState == eDRive_Enabled)
          {
            Command = eCmdMoveRel;  
          }          
          break;
        case 'h':  //-- home --          
          if(ActDriveState == eDRive_Enabled)
          {
            Command = eCmdMoveHome;  
          }          
          break;
        case '+':
            if(ActProfileSpeed < MaxProfileSpeed)
            {
              ActProfileSpeed += DeltaProfileSpeed;
              Serial.print("Main: Speed = ");
              Serial.println(ActProfileSpeed);
            }      
            Command = eCmdSetProfile;  
          break;
        case '-':
            if(ActProfileSpeed > MinProfileSpeed)
            {
              ActProfileSpeed -= DeltaProfileSpeed;
              Serial.print("Main: Speed = ");
              Serial.println(ActProfileSpeed);
            }
            Command = eCmdSetProfile;  
          break;
        case 'i':
          Command = eCmdReadInput;
          break;
        case 't':
          Command = eCmdReadObject;
          break;
        case 'x':
          digitalWrite(Node_A_RefSwitch, HIGH);
          Serial.println("Main: set ref-switch");
          break;
        case 'c':
          digitalWrite(Node_A_RefSwitch, LOW);
          Serial.println("Main: clear ref-switch");
          break;
        case '?': //help
          Serial.println("Commands are:");
          Serial.println("e: enable drive control");
          Serial.println("d: shutdown drive");
          Serial.println("s: (quick)stop");
          Serial.println("o: direct switch off");
          Serial.println("r: reset from fault state");
          Serial.println("-------------");
          Serial.println("p: command a positive rel. move");
          Serial.println("n: command a negative rel. move");
          Serial.println("0: command an abs move to 0");
          Serial.println("+: step-up speed");
          Serial.println("-: step-down speed");
          Serial.println("m: no move");
          Serial.println("-------------");
          Serial.println("h: do homing");
          Serial.println("x: set the ref-switch dig-out");
          Serial.println("c: clear the ref-switch dig-out");
          Serial.println("-------------");
          Serial.println("i: read & print the identiy entries");
          Serial.println("t: read & print the motor temeprature");
          Serial.println("-------------");
          Serial.println("?: print commands");
          break;
        default:
          break;
      }
    }
    if(TargetDriveState != ActDriveState)
    {
      switch(TargetDriveState)    
      {
        case eDRive_Enabled:
          if(Drive_A.Enable() == eCO_DriveDone)
          {
            ActDriveState = eDRive_Enabled; 
            //no need for an explicit reaet of the enable state
            Serial.println("Main: drive enabled");
          }       
          break;
        case eDrive_SwitchedOff:
          if(Drive_A.DisableVoltage() == eCO_DriveDone)
          {
            ActDriveState = eDrive_SwitchedOff;
            //no need for an explicit reaet of the enable state
            Serial.println("Main: drive switched off");
          }
          break;
        case eDrive_ResetError:
          ;
          break;
        case eDrive_SwitchedOn:
          if(Drive_A.Disable() == eCO_DriveDone)
          {
            ActDriveState = eDrive_SwitchedOn;
            //no need for an explicit reaet of the enable state
            Serial.println("Main: drive switched off");
          }
          break;
        case eDrive_Stopped:
          if(Drive_A.Stop() == eCO_DriveDone)
          {
            ActDriveState = eDrive_Stopped;
            //no need for an explicit reaet of the enable state
            Serial.println("Main: drive stopped");
          }
          break;
      }
    }
    switch(Command)
    {
      case eCmdMoveIdle:
        break;
      case eCmdMovePositive:
        if(Drive_A.SetTargetPos(ActTestMove) == eCO_DriveDone)
          Command = eCmdMoveIdle;
        break;
      case eCmdMoveNegative:
        if(Drive_A.SetTargetPos(-ActTestMove) == eCO_DriveDone)
          Command = eCmdMoveIdle;
        break;
      case eCmdMoveRel:
        if(Drive_A.StartMoveRel(false) == eCO_DriveDone)
          Command = eCmdMoveIdle;
        break;
      case eCmdMoveAbs0:
        switch(AccesStep)
        {
          case 0:
            if(Drive_A.SetTargetPos(0) == eCO_DriveDone)
              AccesStep = 1;
            break;
          case 1:
            if(Drive_A.StartMoveAbs(false) == eCO_DriveDone)
            {
              Command = eCmdMoveIdle;
              AccesStep = 0;
            }
            break;
          default:
            break;
        }
        break;
      case eCmdMoveHome:
        if(Drive_A.DoHoming() == eCO_DriveDone)
          Command = eCmdMoveIdle;
        break;
      case eCmdSetProfile: 
        if(Drive_A.UpdateProfile(ActProfileAcc, ActProfileSpeed, ActProfileDec) == eCO_DriveDone)
          Command = eCmdMoveIdle;
        break;
      case eCmdReadIdentity:
        if(Drive_A.IdentifyDrive() == eCO_DriveDone)
        {
          Drive_A.PrintIdentityObjects();
          Command = eCmdMoveIdle;   
        }
        break;
      case eCmdReadInput:
        if(Drive_A.GetDigInStatus(&DriveDigIn) == eCO_DriveDone)
        {
          Serial.print("Main: DigIn = ");
          Serial.println(DriveDigIn,HEX);
          Command = eCmdMoveIdle;   
        }
        break;
      case eCmdReadObject:
        if(Drive_A.GetNumObject(&OdMotorTemp) == eCO_DriveDone)
        {
          Serial.print("Main: Motor Temp = ");
          Serial.print(MotorTemp);
          Serial.println("°C");
          Command = eCmdMoveIdle;   
        }
        break;
      default:
        break;
    }
  }
  delay(1);
}
