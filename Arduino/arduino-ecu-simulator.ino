#include <mcp_can.h>
#include <SPI.h>
#include "pids.h"

//Using Cory J Fowler's fork of the Seeed Studio CAN BUS Shield library:
//https://github.com/coryjfowler/MCP_CAN_lib


#define CAN0_INT 2                                     // Set INT to pin 2
MCP_CAN CAN0(10);                                      // Set CS to pin 10 for the ElecFreaks CAN-BUS Shield v1.2

// This will store the ID of the incoming can message
long unsigned int canId = 0x000;
// this is the length of the incoming message
unsigned char len = 0;
// This the eight byte buffer of the incoming message data payload
unsigned char buf[8];
String canMessageRead="";

byte uintMSB(unsigned int value)
{
  return (byte)((value & 0xFF00) >> 8);
}

byte uintLSB(unsigned int value)
{
  return (byte)(value & 0x00FF);
}

void setup() {
  Serial.begin(115200);
  
START_INIT:

  // Try and open the CAN controller:
  // https://github.com/coryjfowler/MCP_CAN_lib/blob/master/mcp_can_dfs.h
  // MCP_ANY = Disables Masks and Filters and accepts any message on the CAN Bus (MCP_STDEXT,MCP_STD,MCP_EXT,MCP_ANY)
  // CAN_500KBPS specifies a baud rate of 500 Kbps
  // MCP_16MHZ indicates a 16MHz oscillator (crystal) is used as a clock for the MCP2515 chip
  //           you need to check your MCP2515 circuit for the right frequency (MCP_8MHZ, MCP_16MHZ, MCP_20MHZ)
  if(CAN_OK == CAN0.begin(MCP_ANY,CAN_500KBPS, MCP_8MHZ))                   
  {
      Serial.println("CAN BUS Shield init ok!");
      CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.
      pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

      delay(1000);

      //attachInterrupt(0, pin_ISR, CHANGE);
  }
  else
  {
      Serial.println("CAN BUS Shield init fail");
      Serial.println("Init CAN BUS Shield again");
      delay(100);
      goto START_INIT;
  }
}

void loop()
{
    unsigned int ect [8]        = {30,35,40,45,20,15,70,85};
    unsigned int rpm [8]= {1030,2045,5600,8000,8400,9100,12000,7000};
    unsigned int tp [8]= {25,24,20,16,15,34,50,45};
    // Define the set of PIDs you wish you ECU to support.  For more information, see:
    // https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00
    // For this sample, we are only supporting the following PIDs
    // PID (HEX)  PID (DEC)  DESCRIPTION
    // ---------  ---------  --------------------------
    //      0x05         05  Engine Coolant Temperature
    //      0x0C         12  Engine RPM
    //      0x11         17  Throttle position

    // As per the information on bitwise encoded PIDs (https://en.wikipedia.org/wiki/OBD-II_PIDs#Mode_1_PID_00)
    // Our supported PID value is: 
    //     PID 0x05 (05) - Engine Coolant Temperature
    //     |      PID 0x0C (12) - Engine RPM
    //     |      |   PID 0x10 (16) - MAF Air Flow Rate
    //     |      |   |
    //     V      V   V
    // 00001000000100010000000000000000
    // Converted to hex, that is the following four byte value
    // 0x08110000

    
    // Of course, if you want your ECU simulator to be able to respond to any PID From 0x00 to 0x20, you can just use the following PID Bit mask
    // 11111111111111111111111111111111
    // Or 
    // 0xFFFFFFFF

    // Next, we'll create the bytearray that will be the Supported PID query response data payload using the four bye supported pi hex value
    // we determined above (0x08110000):
    
    //                      0x06 - additional meaningful bytes after this one (1 byte Service Mode, 1 byte PID we are sending, and the four by Supported PID value)
    //                       |    0x41 - This is a response (0x40) to a service mode 1 (0x01) query.  0x40 + 0x01 = 0x41
    //                       |     |    0x00 - The response is for PID 0x00 (Supported PIDS 1-20)
    //                       |     |     |    0x08 - The first of four bytes of the Supported PIDS value
    //                       |     |     |     |    0x11 - The second of four bytes of the Supported PIDS value
    //                       |     |     |     |     |    0x00 - The third of four bytes of the Supported PIDS value
    //                       |     |     |     |     |      |   0x00 - The fourth of four bytes of the Supported PIDS value
    //                       |     |     |     |     |      |    |    0x00 - OPTIONAL - Just extra zeros to fill up the 8 byte CAN message data payload)
    //                       |     |     |     |     |      |    |     |
    //                       V     V     V     V     V      V    V     V 
    byte SupportedPID[8] = {0x06, 0x41, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    
    //VIRTUAL SENSORS (SIMULATED)
    byte ectSensor[8] = {3, 65, 0x05, ect[0]};
    byte rpmSensor[8] = {4, 65, 0x0C, uintMSB(rpm[0]), uintLSB(rpm[0])};
    byte tpSensor[8] = {4, 65, 0x11, uintMSB(tp[0]), uintLSB(tp[0])};

    if(!digitalRead(CAN0_INT))
    {
      
      CAN0.readMsgBuf(&canId, &len, buf); 
        // FROM: https://en.wikipedia.org/wiki/OBD-II_PIDs#CAN_(11-bit)_bus_format
        // A CANId value of 0x7DF indicates a query from a diagnostic reader, which acts as a broadcast address and accepts
        // responses from any ID in the range 0x7E8 to 0x7EF.  ECUs that can respond to OBD queries listen both to the functional 
        // broadcast ID of 0x7DF and one assigned in the range 0x7E0 to 0x&7E7.  Their response has an ID of their assigned ID
        // plus 8 (e.g. 0x7E8 through 0x7EF).  Typically, the main ECU responds on 0x7E8.
        Serial.print("<<");Serial.print(canId,HEX);Serial.print(",");

        for(int i = 0; i<len; i++)
        {  
          canMessageRead = canMessageRead + buf[i] + ",";
        }
        Serial.println(canMessageRead);
        
        //Check which message was received.
        if(canMessageRead=="2,1,0,0,0,0,0,0,") {CAN0.sendMsgBuf(0x7E8, 0, 8, SupportedPID);}

        int return_value;
        int responseId = canId+8;
        
        delay(1500);
        //SEND SENSOR STATUSES
        if(canMessageRead=="2,1,5,0,0,0,0,0,")  {return_value = CAN0.sendMsgBuf(responseId, 0, 8, ectSensor);}
        if(canMessageRead=="2,1,12,0,0,0,0,0,"){return_value = CAN0.sendMsgBuf(responseId, 0, 8, rpmSensor);}
        if(canMessageRead=="2,1,17,0,0,0,0,0,"){return_value = CAN0.sendMsgBuf(responseId, 0, 8, tpSensor);}

        Serial.println(return_value);
        Serial.print("Id de resposta:");
        Serial.println(responseId,HEX);
        
        canMessageRead="";

        delay(100);
    }
}

