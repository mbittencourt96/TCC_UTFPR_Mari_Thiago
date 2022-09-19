#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
//#include "utils/uartstdio.h"
//#include "utils/uartstdio.c"

volatile bool rxFlag = 0; // msg received flag
volatile bool txFlag = 0; // msg received flag
volatile bool errFlag = 0; // error flag
int sendRequestAgain = 0;
uint32_t g_ui32SysClock;
int firstByte = 0;
int secondByte = 0;
tCANMsgObject msgRx; // the CAN msg Rx object
tCANMsgObject msgTx; // the CAN msg Tx object
unsigned int msgDataTx; // the message data is four bytes long which we can allocate as an int32
unsigned char *msgDataTxPtr = (unsigned char *)&msgDataTx; // make a pointer to msgDataTx so we can access individual bytes
unsigned char msgDataRx[8]; // 8 byte buffer for rx message data

// CAN interrupt handler
void CANIntHandler(void) {

	unsigned long status = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE); // read interrupt status

	if(status == CAN_INT_INTID_STATUS) { // controller status interrupt
		status = CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);
		errFlag = 1;
	} else if(status == 2) { // msg object 2
		CANIntClear(CAN1_BASE, 2); // clear interrupt
		rxFlag = 1; // set rx flag
		errFlag = 0; // clear any error flags
	}
        else if(status == 1)
        {
          CANIntClear(CAN1_BASE, 1); // clear interrupt
          txFlag = 1;
          errFlag = 0; // clear any error flags
        }
        else { // should never happen
		printf("Unexpected CAN bus interrupt\n");
	}
}

void delay(unsigned int milliseconds) {
	SysCtlDelay((g_ui32SysClock / 3) * (milliseconds / 1000.0f));
}

void requestPID(int pid)

{
          //Request PID
          
          msgDataTxPtr[0] = 2;
          msgDataTxPtr[1] = 1;
          msgDataTxPtr[2] = pid;
          
          printf("Sending request for PID  %d", pid);
          
          CANMessageSet(CAN1_BASE, 1, &msgTx, MSG_OBJ_TYPE_TX);
          
 
}

int main(void) {
	g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 16000000); 

	// Set up debugging UART
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        
       
         //while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA) & !SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
          //{
          //}
        
	//GPIOPinConfigure(GPIO_PA0_U0RX);
	//GPIOPinConfigure(GPIO_PA1_U0TX);
	//GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//UARTStdioConfig(0, 115200, g_ui32SysClock);

	// Set up CAN1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        
           while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
          {
          }
        
	GPIOPinConfigure(GPIO_PB0_CAN1RX);
	GPIOPinConfigure(GPIO_PB1_CAN1TX);
        //GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_LOW_LEVEL);
        //GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_LOW_LEVEL);
	GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
        
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN1))
          {
          }
        
	CANInit(CAN1_BASE);
	CANBitRateSet(CAN1_BASE,g_ui32SysClock, 500000);
        CANIntRegister(CAN1_BASE, CANIntHandler); // use dynamic vector table allocation
        IntMasterEnable();
	CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
	IntEnable(INT_CAN1);
     
	CANEnable(CAN1_BASE);
        
	// Use ID and mask 0 to recieved messages with any CAN ID
	msgRx.ui32MsgID = 0x7E7;
	//msgRx.ui32MsgIDMask = 0;
	msgRx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
	msgRx.ui32MsgLen = 8; // allow up to 8 bytes
        msgRx.pui8MsgData = msgDataRx;
        
          
        msgDataTx = 0;
	msgTx.ui32MsgID = 0x7DF;  //Request ID
	msgTx.ui32MsgIDMask = 0;
	msgTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
        msgTx.ui32MsgLen = sizeof(msgDataTxPtr); // allow up to 8 bytes
        msgTx.pui8MsgData = msgDataTxPtr;
        
         // Load msg into CAN peripheral message object 1 so it can trigger interrupts on any matched rx messages
         CANMessageSet(CAN1_BASE, 2, &msgRx, MSG_OBJ_TYPE_RX);

        requestPID(12); //request engine RPM
        
	while(1) {
           
           if (rxFlag)         
           {
          
                  msgRx.pui8MsgData = msgDataRx; // set pointer to rx buffer
                  CANMessageGet(CAN1_BASE, 2, &msgRx, 0); // read CAN message object 2 from CAN peripheral

                  rxFlag = 0; // clear rx flaG

                  if(msgRx.ui32Flags & MSG_OBJ_DATA_LOST) { // check msg flags for any lost messages
                          printf("CAN message loss detected\n");
                  }

                  // read in colour data from rx buffer (scale from 0-255 to 0-0xFFFF for LED driver)
                  firstByte = msgDataRx[3];
                  secondByte = msgDataRx[4];

                  // write to UART for debugging
                  printf("Received msg\t: %d\t %d\t\n", msgDataRx[3], msgDataRx[4]);
                  
                  sendRequestAgain = 1;
            }
           
           else
           {
             if (sendRequestAgain)
             {
                requestPID(12); //request engine RPM  
             }
           }
           
        }          
     return 0;
}