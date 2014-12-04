//Reads all traffic on CAN0 and forwards it to CAN1 (and in the reverse direction) but modifies some frames first.
// Required libraries
#include "variant.h"
#include <due_can.h>

//Leave defined if you use native port, comment if using programming port
#define Serial SerialUSB

void setup()
{

  Serial.begin(115200);
  
  // Initialize CAN0 and CAN1, Set the proper baud rates here
  Can0.begin(CAN_BPS_250K);
  Can1.begin(CAN_BPS_250K);
  
  //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames
  int filter;
  //extended
  for (filter = 0; filter < 3; filter++) {
	Can0.setRXFilter(filter, 0, 0, true);
	Can1.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
	Can0.setRXFilter(filter, 0, 0, false);
	Can1.setRXFilter(filter, 0, 0, false);
  }  
  
}

void printFrame(CAN_FRAME &frame) {
   Serial.print("ID: 0x");
   Serial.print(frame.id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame.length);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame.length; count++) {
       Serial.print(frame.data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}


//If an incoming frame ID matches one of the case statements then we'll modify it. Otherwise it is sent as-is
void modifyFrame(CAN_FRAME &frame) {
	switch (frame.id)
	{
	case 0x200:
		frame.data.byte[0] += 0x10;
		frame.data.byte[3] = 0xDE;
		break;
	case 0x300:
		frame.data.byte[0] = frame.data.byte[1] + frame.data.byte[2];
		break;
	case 0x320:
		frame.data.byte[7] = 0;
		frame.data.byte[5] = 0xFF;
		break;
		
	}
}

void loop(){
  CAN_FRAME incoming;

  if (Can0.available() > 0) {
	Can0.read(incoming);
	modifyFrame(incoming);
	Can1.sendFrame(incoming);
	//printFrame(incoming);  //uncomment line to print frames that are going out
   }
  if (Can1.available() > 0) {
	Can1.read(incoming);
	modifyFrame(incoming);
	Can0.sendFrame(incoming);
	//printFrame(incoming);
  }
}


