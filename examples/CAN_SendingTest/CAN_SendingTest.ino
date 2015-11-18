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
  
  Can0.watchFor();  
}

void sendData()
{
	CAN_FRAME outgoing;
	outgoing.id = 0x400;
	outgoing.extended = false;
	outgoing.priority = 4; //0-15 lower is higher priority
	
	outgoing.data.s0 = 0xFEED;
    outgoing.data.byte[2] = 0xDD;
	outgoing.data.byte[3] = 0x55;
	outgoing.data.high = 0xDEADBEEF;
	Can0.sendFrame(outgoing);
}

void loop(){
  CAN_FRAME incoming;
  static unsigned long lastTime = 0;

  if (Can0.available() > 0) {
	Can0.read(incoming);
	Can1.sendFrame(incoming);
   }
  if (Can1.available() > 0) {
	Can1.read(incoming);
	Can0.sendFrame(incoming);
  }

  if ((millis() - lastTime) > 1000) 
  {
     lastTime = millis();
     sendData();    
  }
}


