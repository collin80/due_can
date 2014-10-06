// Arduino Due - CANbus Library - Extended Frames with Ping/Pong sending
// Ping/Pong torture test with extended frames.
// This example sets up a receive and transmit mailbox on both canbus devices.
// First CAN0 sends to CAN1. When CAN1 receives it sends to CAN0. PING/PONGs forever
// and as quickly as possible - This will saturate the bus so don't have anything important connected.
// By Thibaut Viard/Wilfredo Molina/Collin Kidder 2014

// Required libraries
#include "variant.h"
#include <due_can.h>

#define TEST1_CAN_TRANSFER_ID    0x11AE756A //random 29 bits
#define TEST1_CAN0_TX_PRIO       15
#define CAN_MSG_DUMMY_DATA       0x11BFFA4E

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

uint32_t sentFrames, receivedFrames;

//Leave this defined if you use the native port or comment it out if you use the programming port
#define Serial SerialUSB

CAN_FRAME frame1, frame2, incoming;

void setup() {

// start serial port at 115200 bps: 
  Serial.begin(115200);
  
  // Verify CAN0 and CAN1 initialization, baudrate is 1Mb/s:
  if (Can0.begin(CAN_BPS_1000K) &&
	  Can1.begin(CAN_BPS_1000K)) {
  }
  else {
    Serial.println("CAN initialization (sync) ERROR");
  }
  
  //Initialize the definitions for the frames we'll be sending.
  //This can be done here because the frame never changes
  frame1.id = TEST1_CAN_TRANSFER_ID;
  frame1.length = MAX_CAN_FRAME_DATA_LEN;
  //Below we set the 8 data bytes in 32 bit (4 byte) chunks
  //Bytes can be set individually with frame1.data.bytes[which] = something
  frame1.data.low = 0x20103040;
  frame1.data.high = CAN_MSG_DUMMY_DATA;
  //We are using extended frames so mark that here. Otherwise it will just use
  //the first 11 bits of the ID set
  frame1.extended = 1;
  
  frame2.id = TEST1_CAN_TRANSFER_ID + 0x200;
  frame2.length = MAX_CAN_FRAME_DATA_LEN;
  frame2.data.low = 0xB8C8A8E8;
  frame2.data.high = 0x01020304;
  frame2.extended = 1;
  
  //Both of these lines create a filter on the corresponding CAN device that allows
  //just the one ID we're interested in to get through.
  //The syntax is (mailbox #, ID, mask, extended)
  //You can also leave off the mailbox number: (ID, mask, extended)
  Can1.watchFor(TEST1_CAN_TRANSFER_ID + 0x200);
  Can0.watchFor(TEST1_CAN_TRANSFER_ID);
  
  test_1();
}

// Test rapid fire ping/pong of extended frames
static void test_1(void)
{
  
  CAN_FRAME inFrame;
  uint32_t counter = 0;
        
  // Send out the first frame
  Can0.sendFrame(frame2);
  sentFrames++;

  while (1==1) {
    if (Can0.available() > 0) {
      Can0.read(incoming);
      Can0.sendFrame(frame2);
      delayMicroseconds(100);
      sentFrames++;
      receivedFrames++;
      counter++;
    }
    if (Can1.available() > 0) {
      Can1.read(incoming);
      Can1.sendFrame(frame1);
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
