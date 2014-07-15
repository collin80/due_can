// Arduino Due - Displays all traffic found on either canbus port
//Modified version of the SnooperCallback sketch. This one illustrates
//how to fix up the issue of not being able to do a callback to a class method function

// By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013-2014

// Required libraries
#include "variant.h"
#include <due_can.h>

//Leave defined if you use native port, comment if using programming port
//This sketch could provide a lot of traffic so it might be best to use the
//native port
#define Serial SerialUSB

class ExampleClass
{
  public:
  void printFrame(CAN_FRAME *frame);
};

//Prints everything out in decimal instead of hex to show it is different
void ExampleClass::printFrame(CAN_FRAME *frame)
{
   Serial.print("Special OOP print frame:   ");
   Serial.print(" ID: ");
   Serial.print(frame->id);
   Serial.print(" Len: ");
   Serial.print(frame->length);
   Serial.print(" Data: ");
   for (int count = 0; count < frame->length; count++) {
       Serial.print(frame->data.bytes[count]);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}

ExampleClass myClass; //initialize the class global so the reference to it can be picked up anywhere

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

/*This is a trampoline function essentially. What that means is that it fixes up the calling convention
  and then passes (bounces) to the proper function. Since the object was registered global above it
  is possible to reference it here. Feel free to use more advanced schemes (such as a static reference and getReference) in actual code.
*/
void gotFrameClass(CAN_FRAME *frame) 
{
  myClass.printFrame(frame); //bounce!
}

void setup()
{

  Serial.begin(115200);
  
  // Initialize CAN0, Set the proper baud rates here
  CAN.init(CAN_BPS_250K);
  
  //By default there are 7 RX mailboxes for each device
  //extended
  //syntax is mailbox, ID, mask, extended
  CAN.setRXFilter(0, 0x2FF00, 0x1FF2FF00, true);
  CAN.setRXFilter(1, 0x1F0000, 0x1F1F0000, true);
  CAN.setRXFilter(2, 0, 0, true); //catch all mailbox
  
  //standard  
  CAN.setRXFilter(3, 0x40F, 0x7FF, false);
  CAN.setRXFilter(4, 0x310, 0x7F0, false);
  CAN.setRXFilter(5, 0x200, 0x700, false);
  CAN.setRXFilter(6, 0, 0, false); //catch all mailbox
  
  //now register all of the callback functions.
  CAN.setCallback(0, gotFrameMB0);
  CAN.setCallback(1, gotFrameMB1);
  CAN.setCallback(3, gotFrameMB3);
  CAN.setCallback(4, gotFrameMB4);
  CAN.setCallback(5, gotFrameMB5);
  
  //this function will get a callback for any mailbox that doesn't have a registered callback from above -> 2 and 6
  //CAN.setGeneralCallback(myClass.printFrame); //NO!! You can't do C-style callbacks to a member function
  CAN.setGeneralCallback(gotFrameClass); //Better! You can call a C function that bounces to the C++ member function
}

void loop(){ //note the empty loop here. All work is done via callback as frames come in - no need to poll for them
}


