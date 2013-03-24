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
#define CAN_MSG_DUMMY_DATA       0x55AAAA55

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

// Message variable to be send
uint32_t CAN_MSG_1 = 0;

void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(9600);
  Serial.println("Type CAN message to send");
  while (Serial.available() == 0);
}
void loop(){

  while (Serial.available() > 0) {
    CAN_MSG_1 = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.print("Sent value= ");
      Serial.println(CAN_MSG_1);
    }
  }

  // Initialize CAN0 and CAN1, baudrate is 1Mb/s
  CAN.init(SystemCoreClock, CAN_BPS_1000K);
  CAN2.init(SystemCoreClock, CAN_BPS_1000K);

  // Initialize CAN1 mailbox 0 as receiver, frame ID is 0x07
  CAN2.mailbox_init(0);
  CAN2.mailbox_set_mode(0, CAN_MB_RX_MODE);
  CAN2.mailbox_set_accept_mask(0, 0x7FF, false);
  CAN2.mailbox_set_id(0, TEST1_CAN_TRANSFER_ID, false);

  // Initialize CAN0 mailbox 0 as transmitter, transmit priority is 15
  CAN.mailbox_init(0);
  CAN.mailbox_set_mode(0, CAN_MB_TX_MODE);
  CAN.mailbox_set_priority(0, TEST1_CAN0_TX_PRIO);
  CAN.mailbox_set_accept_mask(0, 0, false);
  // Prepare transmit ID, data and data length in CAN0 mailbox 0
  CAN.mailbox_set_id(0, TEST1_CAN_TRANSFER_ID, false);
  CAN.mailbox_set_datal(0, CAN_MSG_1);
  CAN.mailbox_set_datah(0, CAN_MSG_DUMMY_DATA);
  CAN.mailbox_set_datalen(0, MAX_CAN_FRAME_DATA_LEN);

  // Send out the information in the mailbox
  CAN.global_send_transfer_cmd(CAN_TCR_MB0);

  // Wait for CAN1 mailbox 0 to receive the data
  while (!(CAN2.mailbox_get_status(0) & CAN_MSR_MRDY)) {
  }

  RX_CAN_FRAME incoming;
  // Read the received data from CAN1 mailbox 0
  CAN2.mailbox_read(0, &incoming);
  Serial.print("CAN message received= ");
  Serial.print(incoming.data[0]);
  Serial.print(incoming.data[1]);
  Serial.print(incoming.data[2]);
  Serial.print(incoming.data[3]);
  Serial.print(incoming.data[4]);
  Serial.print(incoming.data[5]);
  Serial.print(incoming.data[6]);
  Serial.println(incoming.data[7]);
  
  // Disable CAN0 Controller
  CAN.disable();

  // Disable CAN1 Controller
  CAN2.disable();

  Serial.print("End of test");

  while (1) {
  }
}

