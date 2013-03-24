// Arduino Due - CAN Sample 3
// Ping/Pong torture test with extended frames.
// This example sets up a receive and transmit mailbox on both canbus devices.
// First CAN0 sends to CAN1. When CAN1 receives it sends to CAN0. PING/PONG forever
// and as quickly as possible.
// By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013

// Required libraries
#include "variant.h"
#include <due_can.h>

#define TEST1_CAN_TRANSFER_ID    0x11AE756A //random 29 bits
#define TEST1_CAN0_TX_PRIO       15
#define CAN_MSG_DUMMY_DATA       0x55AAAA55

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

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

    NVIC_EnableIRQ(CAN0_IRQn);
    NVIC_EnableIRQ(CAN1_IRQn);
  }
  else {
    Serial.println("CAN initialization (sync) ERROR");
  }
  test_1();
}

// Test rapid fire ping/pong of extended frames
static void test_1(void)
{
  
  RX_CAN_FRAME inFrame;
  uint32_t counter = 0;
  
  //Reset all CAN0 and CAN1 mailboxes:
  CAN.reset_all_mailbox();
  CAN2.reset_all_mailbox();
  
  CAN.mailbox_set_mode(0, CAN_MB_RX_MODE);
  CAN2.mailbox_set_mode(0, CAN_MB_RX_MODE);
  
  CAN.mailbox_set_accept_mask(0, 0x1FFFFFFF, true);
  CAN2.mailbox_set_accept_mask(0, 0x1FFFFFFF, true);
  
  CAN.mailbox_set_id(0, TEST1_CAN_TRANSFER_ID, true);
  CAN2.mailbox_set_id(0, TEST1_CAN_TRANSFER_ID + 0x200, true);
  

  CAN.mailbox_set_mode(1, CAN_MB_TX_MODE);
  CAN2.mailbox_set_mode(1, CAN_MB_TX_MODE);
  
  CAN.mailbox_set_priority(1, TEST1_CAN0_TX_PRIO);
  CAN2.mailbox_set_priority(1, TEST1_CAN0_TX_PRIO);  
  
  CAN.mailbox_set_accept_mask(1, 0x1FFFFFFF, true);
  CAN2.mailbox_set_accept_mask(1, 0x1FFFFFFF, true);

  CAN.mailbox_set_id(1, TEST1_CAN_TRANSFER_ID + 0x200, true);
  CAN2.mailbox_set_id(1, TEST1_CAN_TRANSFER_ID, true);
  
  CAN.mailbox_set_datalen(1, MAX_CAN_FRAME_DATA_LEN);
  CAN2.mailbox_set_datalen(1, MAX_CAN_FRAME_DATA_LEN);
  
  CAN.mailbox_set_datal(1, 0x20103040);
  CAN.mailbox_set_datah(1, CAN_MSG_DUMMY_DATA);
  
  CAN2.mailbox_set_datal(1, 0xB8C8A8E8);
  CAN2.mailbox_set_datah(1, 0x01020304);
  
  CAN.enable_interrupt(CAN_IER_MB0);
  CAN2.enable_interrupt(CAN_IER_MB0);  

  // Send out the first frame
  CAN.global_send_transfer_cmd(CAN_TCR_MB1);
  sentFrames++;

  while (1==1) {
    if (CAN.rx_avail()) {
      CAN.global_send_transfer_cmd(CAN_TCR_MB1);
      delayMicroseconds(100);
      sentFrames++;
      receivedFrames++;
      counter++;
    }
    if (CAN2.rx_avail()) {
      CAN2.global_send_transfer_cmd(CAN_TCR_MB1);      
      delayMicroseconds(100);
      sentFrames++;
      receivedFrames++;
      counter++;
    }
    if (counter > 5000) {
       counter = 0;
       Serial.print("S: ");
       Serial.print(sentFrames);
       Serial.print(" R: ");
       Serial.println(receivedFrames);
    }
  }
}

// can_example application entry point
void loop()
{
}
