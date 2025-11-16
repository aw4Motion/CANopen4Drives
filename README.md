# Arduino library for a CANopen central device on the UNO R4

The libray implements CAN in Automation (CiA) 301 services for classic CANopen.
- per node:
  - SDO handling (expedited or segmented)
  - NMT using either Node Guarding or Heartbeat
  - basic reception of EMCY messages per node
  - PDO handling
- global service
  - SYNC generation
  
All of these register at the single MsgHandler which calls the upper layers vis call-back.

                               examples
                                  |
                      --CO402Drive--CO401Node--
    ------              |         |        |
    |                   |  OPDOHandler     |
    |                   |  |    |          |
    |              ---CONode    |          |
    |              |       |    |          |
    |       COSDOHandler   |    |     COSyncHandler
    |       |              |    |         |
    |       ---------->COMsgHandler<-------   
    |----                      |
                            UNOR4CAN

So far there is no LSS service implemented. Node-id and baud rate need to be preset.

NMT and PDOs are configured on boot-up using the SDO service. PDO mappings and transmission types
can and will be configured by this central device.
  
On top of this CiA 301 stack a handler for a CiA 402 servo drive is implemented which uses the 
per node services to enable/diable the drive (behavior implemented) and move in the different OpModes.

No other device profiles have been provided so far but would be straight forward in case of a CiA 401 I/O node.

A sample implementation consists of
- a single instance of the COMsgHandler
- if using the CO402Drive it's an instance of this class per remote drive
- a single instance of the COSyncHandler

## Hardware

UNO R4 Minima and WiFi, likely NANO R4

For an R4 Wifi it's D10: CANTX0 --> MCP2551::pin 1
                    D13: CANRX0 <-- MCP2551::Pin 4
				   
For an R4 Mini it's D04: CANTX0 --> MCP2551::pin 1
                    D05: CANRX0 <-- MCP2551::Pin 4				   


## Documentation

See the included example sketches
If controlling multiple remote nodes take care to configure them one by one as the non-buffered Tx might
otherwise be overloaded. 

The complete library is implemented in an non-blocking pattern, where the calls on all levels will return directls and
the return code will tell whether they already finished. Therefore it's simple to run them "in parallel" to whatever activity
from the loop.

## Limitations

The low-level Rx/Tx is handled by a slightly modified version of the UNOR4CAN.
In this library there is no explicit queuing of the messages to be transmitted. An unsuccessful
transmission is reported back and the different services of the CANopen 301 library will re-transmit.

## Testing

Tested using a with an MCP2551 CAN bus transceiver chip at 250kbps 
against 1 ... 4 FAULHABER MC 3603 S RS/CO MotionController.

