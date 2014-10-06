/*
Arduino Due - Displays frames that fall between a range of addresses.
Modified from some of the other examples - notably CAN_SnooperCallBack
The difference here is that we only allow a range of ids. This demonstrates
the new watchForRange function as well as per-mailbox and general callback
functionality

By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013-2014
*/

// Required libraries
#include "variant.h"
#include <due_can.h>

//Leave defined if you use native port, comment if using programming port
//This sketch could provide a lot of traffic so it might be best to use the
//native port
#define Serial SerialUSB

void printFrame(CAN_FRAME *frame, int filter) {
   Serial.print("Fltr: ");
   if (filter > -1) Serial.print(filter);
   else Serial.print("???");
   Serial.print(" ID: 0x");
   Serial.print(frame->id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame->length);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame->length; count++) {
       Serial.print(frame->data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}

void gotFrameMB0(CAN_FRAME *frame) 
{
  printFrame(frame, 0);
}

void gotFrameMB1(CAN_FRAME *frame) 
{
  printFrame(frame, 1);
}

void gotFrameMB3(CAN_FRAME *frame) 
{
  printFrame(frame, 3);
}

void gotFrameMB4(CAN_FRAME *frame) 
{
  printFrame(frame, 4);
}

void gotFrameMB5(CAN_FRAME *frame) 
{
  printFrame(frame, 5);
}

void gotFrame(CAN_FRAME *frame) 
{
  printFrame(frame, -1);
}

void setup()
{

  Serial.begin(115200);
  
  // Initialize CAN0, Set the proper baud rates here
  Can0.init(CAN_BPS_250K);
  
  //By default there are 7 RX mailboxes for each device
  //extended
  //syntax is mailbox, ID, mask, extended
  Can0.watchForRange(0x10000000, 0x10004000);
  Can0.watchForRange(0x10050000, 0x100500FF);
  Can0.setRXFilter(0, 0, true); //catch all mailbox
  
  //standard  
  Can0.watchForRange(0x400, 0x470);
  Can0.watchForRange(0x500, 0x5FF);
  Can0.watchForRange(0x200, 0x220);
  Can0.setRXFilter(0, 0, false); //catch all mailbox
  
  //now register all of the callback functions.
  Can0.attachCANInterrupt(0, gotFrameMB0);
  Can0.attachCANInterrupt(1, gotFrameMB1);
  Can0.attachCANInterrupt(3, gotFrameMB3);
  Can0.attachCANInterrupt(4, gotFrameMB4);
  Can0.attachCANInterrupt(5, gotFrameMB5);
  //this function will get a callback for any mailbox that doesn't have a registered callback from above -> 2 and 6
  Can0.attachCANInterrupt(gotFrame);
  
}

void loop(){ //note the empty loop here. All work is done via callback as frames come in - no need to poll for them
}


