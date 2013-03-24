// Arduino Due - CAN Sample 3
// A more in-depth example with multiple receive and transmit mailboxes and
// continuous sending. Because of the way this program is set up the receive
//counter should settle to four less than the transmit counter. If the numbers
//begin to skew then there is a problem with canbus transmission and/or reception
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

uint32_t sentFrames, receivedFrames;

void setup() {

// start serial port at 115200 bps: 
  Serial.begin(115200);
  
  // Verify CAN0 and CAN1 initialization, baudrate is 1Mb/s:
  if (CAN.init(SystemCoreClock, CAN_BPS_1000K) &&
  CAN2.init(SystemCoreClock, CAN_BPS_1000K)) {

    // Disable all CAN0 & CAN1 interrupts
    CAN.disable_interrupt(CAN_DISABLE_ALL_INTERRUPT_MASK);
    CAN2.disable_interrupt(CAN_DISABLE_ALL_INTERRUPT_MASK);

    // Configure and enable interrupt of CAN1, as the tests will use receiver interrupt
    NVIC_EnableIRQ(CAN0_IRQn);
  }
  else {
    Serial.println("CAN initialization (sync) ERROR");
  }

}

// Test the transmission from CAN0 Mailbox 0 to CAN1 Mailbox 0
static void test_1(void)
{
  //Reset all CAN0 and CAN1 mailboxes:
  CAN.reset_all_mailbox();
  CAN2.reset_all_mailbox();
  
  //setup four receive mailboxes that each accept a different frame id
  for (uint8_t count = 0; count < 4; count++)
  {
    CAN.mailbox_set_mode(count, CAN_MB_RX_MODE); 
    CAN.mailbox_set_accept_mask(count, 0x7FF, false); //only accept the exact ID
    CAN.mailbox_set_id(count, TEST1_CAN_TRANSFER_ID + count * 4, false);
  }
  
  for (uint8_t count = 0; count < 4; count++)
  {
    CAN2.mailbox_set_mode(count, CAN_MB_TX_MODE);
    CAN2.mailbox_set_priority(count, TEST1_CAN0_TX_PRIO);
    CAN2.mailbox_set_accept_mask(count, 0x7FF, false);
    CAN2.mailbox_set_id(count, TEST1_CAN_TRANSFER_ID + count * 4, false);
    CAN2.mailbox_set_datalen(count, MAX_CAN_FRAME_DATA_LEN);
    CAN2.mailbox_set_datal(count, CAN_MSG_1 + count * 16);
    CAN2.mailbox_set_datah(count, CAN_MSG_DUMMY_DATA);
  }

  // Enable interrupt for mailboxes 0-3 on first canbus
  CAN.enable_interrupt(CAN_IER_MB0 | CAN_IER_MB1 | CAN_IER_MB2 | CAN_IER_MB3);

  // Send out the information in the mailboxes
  CAN2.global_send_transfer_cmd(CAN_TCR_MB0 | CAN_TCR_MB1 | CAN_TCR_MB2 | CAN_TCR_MB3);
  sentFrames += 4;
  // Wait for the communication to be completed.
  while (!CAN.rx_avail()) { //while no frame is received
  }
  
  RX_CAN_FRAME inFrame;
  
  while (CAN.rx_avail()) {
      CAN.get_rx_buff(&inFrame);
      receivedFrames++;
  }
}

// can_example application entry point
void loop()
{

// Run test
while (Serial.available() == 0)
{
  test_1();
  delay(8);
  Serial.print("S: ");
  Serial.print(sentFrames);
  Serial.print(" R: ");
  Serial.println(receivedFrames);
}

// Disable CAN0 Controller
CAN.disable();
// Disable CAN1 Controller
CAN2.disable();

while (1) {
	}
}
