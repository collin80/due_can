// Arduino Due - CAN Sample 1
// Brief CAN example for Arduino Due
// Test the transmission from CAN0 Mailbox 0 to CAN1 Mailbox 0
// By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013

// Required libraries
#include "variant.h"
#include <due_can.h>

#define TEST1_CAN_COMM_MB_IDX    0
#define TEST1_CAN_TRANSFER_ID    0x07
#define TEST1_CAN0_TX_PRIO       15
#define CAN_MSG_DUMMY_DATA       0x55AAEE22

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

// Message variable to be send
uint32_t CAN_MSG_1 = 0;

//Leave defined if you use native port, comment if using programming port
#define Serial SerialUSB

void setup()
{
  CAN_FRAME output;

  // start serial port at 9600 bps:
  Serial.begin(9600);
  Serial.println("Type CAN message to send");
  while (Serial.available() == 0);
}
void loop(){

  CAN_FRAME output;
  while (Serial.available() > 0) {
    CAN_MSG_1 = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.print("Sent value= ");
      Serial.println(CAN_MSG_1);
    }
  }

  // Initialize CAN0 and CAN1, baudrate is 250kb/s
  CAN.init(CAN_BPS_250K);
  CAN2.init(CAN_BPS_250K);

  //Set a filter for CAN2 such that only frames that exactly match 
  //TEST1_CAN_TRANSFER_ID can get through
  CAN2.setRXFilter(0, TEST1_CAN_TRANSFER_ID, 0x7FF, false);

  // Prepare transmit ID, data and data length in CAN0 mailbox 0
  output.id = TEST1_CAN_TRANSFER_ID;
  output.length = MAX_CAN_FRAME_DATA_LEN;
  //Set first four bytes (32 bits) all at once
  output.data.low = CAN_MSG_1;
  //Set last four bytes (32 bits) all at once
  output.data.high = CAN_MSG_DUMMY_DATA;
  //Send out the frame on whichever mailbox is free or queue it for
  //sending when there is an opening.
  CAN.sendFrame(output);

  // Wait for CAN1 mailbox 0 to receive the data
  while (!CAN2.rx_avail()) {
  }

  // Read the received data from CAN1 mailbox 0
  CAN_FRAME incoming;
  CAN2.get_rx_buff(incoming);
  
  Serial.print("CAN message received= ");
  Serial.print(incoming.data.low, HEX);
  Serial.print(incoming.data.high, HEX);
  
  // Disable CAN0 Controller
  CAN.disable();

  // Disable CAN1 Controller
  CAN2.disable();

  Serial.print("\nEnd of test");

  while (1) {
  }
}

