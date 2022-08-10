due_can
=======

Object oriented canbus library for Arduino Due compatible boards

Implements both CAN buses exposed by Due hardware.

This library requires the can_common library now. That library
is a common base that other libraries can be built off of to allow
a more universal API for CAN. 

The needed can_common library is found here:
https://github.com/collin80/can_common

Note of Antonio Previtali:

Collin released version 2.01 in 2015 then worked on the master branch until 2019 without releasing any releases.
I am trying the master.
my attempt is to modify the master to add the possibility of knowing if all the frames of which I made the SendFrame have been sent and if they have not been sent to be able to remove from the transmission buffer the frames that have not yet been sent and insert new ones.
I added 2 methods:

isAllFrameSend (); // return true if all frame is sended

ClearTxBuff (void); // clear all frame not already sended, but not abort frame current in sending.

maybe it is possible to get this thing without modifying the library but I have not found a way and so I have modified.
