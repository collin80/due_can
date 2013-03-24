// Arduino Due - CAN Sample 2
// Brief CAN example for Arduino Due
// Test the transmission from CAN0 Mailbox 0 to CAN1 Mailbox 0 using interruption
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

//Message variable to be send
uint32_t CAN_MSG_1 = 0;

void setup() {

// start serial port at 9600 bps: 
  Serial.begin(9600);
  Serial.println("Type CAN message to send");
  while (Serial.available() == 0);  
}

// Test the transmission from CAN0 Mailbox 0 to CAN1 Mailbox 0
static void test_1(void)
{
  //Reset all CAN0 and CAN1 mailboxes:
  CAN.reset_all_mailbox();
  CAN2.reset_all_mailbox();

  CAN.mailbox_set_mode(0, CAN_MB_RX_MODE); 
  CAN.mailbox_set_accept_mask(0, 0, false);
  CAN.mailbox_set_id(0, TEST1_CAN_TRANSFER_ID, false);
  
  CAN2.mailbox_set_mode(0, CAN_MB_TX_MODE);
  CAN2.mailbox_set_priority(0, TEST1_CAN0_TX_PRIO);
  CAN2.mailbox_set_accept_mask(0, 0x7FF, false);
  CAN2.mailbox_set_id(0, TEST1_CAN_TRANSFER_ID, false);
  CAN2.mailbox_set_datalen(0, MAX_CAN_FRAME_DATA_LEN);
  CAN2.mailbox_set_datal(0, CAN_MSG_1);
  CAN2.mailbox_set_datah(0, CAN_MSG_DUMMY_DATA);

  // Enable interrupt for mailbox 0 on first CAN
  CAN.enable_interrupt(CAN_IER_MB0);

  // Send out the information in the mailbox
  CAN2.global_send_transfer_cmd(CAN_TCR_MB0);
  // Wait for the communication to be completed.
  while (!CAN.rx_avail()) { //while no frame is received
  }
  RX_CAN_FRAME inFrame;
  CAN.get_rx_buff(&inFrame);
  Serial.print("CAN message received= ");
  Serial.print(inFrame.data[0]);
  Serial.print(inFrame.data[1]);
  Serial.print(inFrame.data[2]);
  Serial.print(inFrame.data[3]);
  Serial.print(inFrame.data[4]);
  Serial.print(inFrame.data[5]);
  Serial.print(inFrame.data[6]);
  Serial.println(inFrame.data[7]);
  Serial.println("End of test");
}

// can_example application entry point
void loop()
{
  while (Serial.available() > 0) {
     CAN_MSG_1 = Serial.parseInt();
      if (Serial.read() == '\n') {      
      Serial.print("Sent value= ");
      Serial.println(CAN_MSG_1);
      delay(1000);
    }
  }

// Verify CAN0 and CAN1 initialization, baudrate is 1Mb/s:
if (CAN.init(SystemCoreClock, CAN_BPS_1000K) &&
CAN2.init(SystemCoreClock, CAN_BPS_1000K)) {

// Disable all CAN0 & CAN1 interrupts
CAN.disable_interrupt(CAN_DISABLE_ALL_INTERRUPT_MASK);
CAN2.disable_interrupt(CAN_DISABLE_ALL_INTERRUPT_MASK);

// Configure and enable interrupt of CAN1, as the tests will use receiver interrupt
NVIC_EnableIRQ(CAN0_IRQn);

// Run test
test_1();

// Disable CAN0 Controller
CAN.disable();
// Disable CAN1 Controller
CAN2.disable();
} else {
Serial.println("CAN initialization (sync) ERROR");
}

while (1) {
	}
}
