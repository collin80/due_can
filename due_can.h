/*
  Copyright (c) 2013 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _CAN_LIBRARY_
#define _CAN_LIBRARY_

#include "arduino.h"

//add some extra stuff that is needed for Arduino 1.5.2
#ifndef PINS_CAN0
	static const uint8_t CAN1RX = 88;
	static const uint8_t CAN1TX = 89;

	// CAN0
	#define PINS_CAN0            (90u)
	// CAN1
	#define PINS_CAN1            (91u)
	#define ARDUINO152
#endif

#define CAN		Can0
#define CAN2	Can1

#define CAN0_EN  62
#define CAN1_EN  65

/** Define the Mailbox mask for eight mailboxes. */
#define GLOBAL_MAILBOX_MASK           0x000000ff

/** Disable all interrupt mask */
#define CAN_DISABLE_ALL_INTERRUPT_MASK 0xffffffff

/** Define the typical baudrate for CAN communication in KHz. */
#define CAN_BPS_1000K                 1000000
#define CAN_BPS_800K                  800000
#define CAN_BPS_500K                  500000
#define CAN_BPS_250K                  250000
#define CAN_BPS_125K                  125000
#define CAN_BPS_50K                   50000
#define CAN_BPS_33333				  33333
#define CAN_BPS_25K                   25000
#define CAN_BPS_10K                   10000
#define CAN_BPS_5K                    5000

#define CAN_DEFAULT_BAUD	CAN_BPS_250K

/** Define the mailbox mode. */
#define CAN_MB_DISABLE_MODE           0
#define CAN_MB_RX_MODE                1
#define CAN_MB_RX_OVER_WR_MODE        2
#define CAN_MB_TX_MODE                3
#define CAN_MB_CONSUMER_MODE          4
#define CAN_MB_PRODUCER_MODE          5

/** Define CAN mailbox transfer status code. */
#define CAN_MAILBOX_TRANSFER_OK       0     //! Read from or write into mailbox successfully.
#define CAN_MAILBOX_NOT_READY         0x01  //! Receiver is empty or transmitter is busy.
#define CAN_MAILBOX_RX_OVER           0x02  //! Message overwriting happens or there're messages lost in different receive modes.
#define CAN_MAILBOX_RX_NEED_RD_AGAIN  0x04  //! Application needs to re-read the data register in Receive with Overwrite mode.

#define SIZE_RX_BUFFER	32 //RX incoming ring buffer is this big
#define SIZE_TX_BUFFER	16 //TX ring buffer is this big

	/** Define the timemark mask. */
#define TIMEMARK_MASK              0x0000ffff

/* CAN timeout for synchronization. */
#define CAN_TIMEOUT                100000

/** The max value for CAN baudrate prescale. */
#define CAN_BAUDRATE_MAX_DIV       128

/** Define the scope for TQ. */
#define CAN_MIN_TQ_NUM             8
#define CAN_MAX_TQ_NUM             25

/** Define the fixed bit time value. */
#define CAN_BIT_SYNC               1
#define CAN_BIT_IPT                2

typedef struct {
	uint8_t uc_tq;      //! CAN_BIT_SYNC + uc_prog + uc_phase1 + uc_phase2 = uc_tq, 8 <= uc_tq <= 25.
	uint8_t uc_prog;    //! Propagation segment, (3-bits + 1), 1~8;
	uint8_t uc_phase1;  //! Phase segment 1, (3-bits + 1), 1~8;
	uint8_t uc_phase2;  //! Phase segment 2, (3-bits + 1), 1~8, CAN_BIT_IPT <= uc_phase2;
	uint8_t uc_sjw;     //! Resynchronization jump width, (2-bits + 1), min(uc_phase1, 4);
	uint8_t uc_sp;      //! Sample point value, 0~100 in percent.
} can_bit_timing_t;


/** Values of bit time register for different baudrates, Sample point = ((1 + uc_prog + uc_phase1) / uc_tq) * 100%. */
const can_bit_timing_t can_bit_time[] = {
	{8,   (2 + 1), (1 + 1), (1 + 1), (2 + 1), 75},
	{9,   (1 + 1), (2 + 1), (2 + 1), (1 + 1), 67},
	{10,  (2 + 1), (2 + 1), (2 + 1), (2 + 1), 70},
	{11,  (3 + 1), (2 + 1), (2 + 1), (3 + 1), 72},
	{12,  (2 + 1), (3 + 1), (3 + 1), (3 + 1), 67},
	{13,  (3 + 1), (3 + 1), (3 + 1), (3 + 1), 77},
	{14,  (3 + 1), (3 + 1), (4 + 1), (3 + 1), 64},
	{15,  (3 + 1), (4 + 1), (4 + 1), (3 + 1), 67},
	{16,  (4 + 1), (4 + 1), (4 + 1), (3 + 1), 69},
	{17,  (5 + 1), (4 + 1), (4 + 1), (3 + 1), 71},
	{18,  (4 + 1), (5 + 1), (5 + 1), (3 + 1), 67},
	{19,  (5 + 1), (5 + 1), (5 + 1), (3 + 1), 68},
	{20,  (6 + 1), (5 + 1), (5 + 1), (3 + 1), 70},
	{21,  (7 + 1), (5 + 1), (5 + 1), (3 + 1), 71},
	{22,  (6 + 1), (6 + 1), (6 + 1), (3 + 1), 68},
	{23,  (7 + 1), (7 + 1), (6 + 1), (3 + 1), 70},
	{24,  (6 + 1), (7 + 1), (7 + 1), (3 + 1), 67},
	{25,  (7 + 1), (7 + 1), (7 + 1), (3 + 1), 68}
};

//This is architecture specific. DO NOT USE THIS UNION ON ANYTHING OTHER THAN THE CORTEX M3 / Arduino Due
//UNLESS YOU DOUBLE CHECK THINGS!
typedef union {
    uint64_t value;
	struct {
		uint32_t low;
		uint32_t high;
	};
	struct {
        uint16_t s0;
		uint16_t s1;
		uint16_t s2;
		uint16_t s3;
    };
	uint8_t bytes[8];
	uint8_t byte[8]; //alternate name so you can omit the s if you feel it makes more sense
} BytesUnion;

typedef struct
{
	uint32_t id;		// EID if ide set, SID otherwise
	uint32_t fid;		// family ID
	uint8_t rtr;		// Remote Transmission Request
	uint8_t priority;	// Priority but only important for TX frames and then only for special uses.
	uint8_t extended;	// Extended ID flag
	uint8_t length;		// Number of data bytes
	BytesUnion data;	// 64 bits - lots of ways to access it.
} CAN_FRAME;

class CANRaw
{
  protected:
	int numTXBoxes; //There are 8 mailboxes, anything not TX will be set RX

  private:
	/* CAN peripheral, set by constructor */
	Can* m_pCan ;

	volatile CAN_FRAME rx_frame_buff[SIZE_RX_BUFFER];
	volatile CAN_FRAME tx_frame_buff[SIZE_TX_BUFFER];

	volatile uint16_t rx_buffer_head, rx_buffer_tail;
	volatile uint16_t tx_buffer_head, tx_buffer_tail;
	void mailbox_int_handler(uint8_t mb, uint32_t ul_status);

	uint8_t enablePin;
	uint32_t write_id; //public storage for an id. Will be used by the write function to set which ID to send to.
	bool bigEndian;

	void (*cbCANFrame[9])(CAN_FRAME *); //8 mailboxes plus an optional catch all

  public:

    // Constructor
    CANRaw( Can* pCan, uint32_t En);

	int setRXFilter(uint32_t id, uint32_t mask, bool extended);
	int setRXFilter(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
	int watchFor(); //allow anything through
	int watchFor(uint32_t id); //allow just this ID through (automatic determination of extended status)
	int watchFor(uint32_t id, uint32_t mask); //allow a range of ids through
	int watchForRange(uint32_t id1, uint32_t id2); //try to allow the range from id1 to id2 - automatically determine base ID and mask

	void setNumTXBoxes(int txboxes);
	int findFreeRXMailbox();
	uint8_t mailbox_get_mode(uint8_t uc_index);
	uint32_t mailbox_get_id(uint8_t uc_index);
	uint32_t getMailboxIer(int8_t mailbox);
	uint32_t set_baudrate(uint32_t ul_baudrate);

	uint32_t init(uint32_t ul_baudrate);
	uint32_t begin();
	uint32_t begin(uint32_t baudrate);
	uint32_t begin(uint32_t baudrate, uint8_t enablePin);

	void enable();
	void disable();
	void disable_low_power_mode();
	void enable_low_power_mode();
	void disable_autobaud_listen_mode();
	void enable_autobaud_listen_mode();
	void disable_overload_frame();
	void enable_overload_frame();
	void set_timestamp_capture_point(uint32_t ul_flag);
	void disable_time_triggered_mode();
	void enable_time_triggered_mode();
	void disable_timer_freeze();
	void enable_timer_freeze();
	void disable_tx_repeat();
	void enable_tx_repeat();
	void set_rx_sync_stage(uint32_t ul_stage);
	void enable_interrupt(uint32_t dw_mask);
	void disable_interrupt(uint32_t dw_mask);
	uint32_t get_interrupt_mask();
	uint32_t get_status();
	uint32_t get_internal_timer_value();
	uint32_t get_timestamp_value();
	uint8_t get_tx_error_cnt();
	uint8_t get_rx_error_cnt();
	void reset_internal_timer();
	void global_send_transfer_cmd(uint8_t uc_mask);
	void global_send_abort_cmd(uint8_t uc_mask);
	void mailbox_set_timemark(uint8_t uc_index, uint16_t us_cnt);
	uint32_t mailbox_get_status(uint8_t uc_index);
	void mailbox_send_transfer_cmd(uint8_t uc_index);
	void mailbox_send_abort_cmd(uint8_t uc_index);
	void mailbox_init(uint8_t uc_index);
	uint32_t mailbox_read(uint8_t uc_index, volatile CAN_FRAME *rxframe);
	uint32_t mailbox_tx_frame(uint8_t uc_index);
	void mailbox_set_id(uint8_t uc_index, uint32_t id, bool extended);
	void mailbox_set_priority(uint8_t uc_index, uint8_t pri);
	void mailbox_set_accept_mask(uint8_t uc_index, uint32_t mask, bool ext);
	void mailbox_set_mode(uint8_t uc_index, uint8_t mode);
	void mailbox_set_databyte(uint8_t uc_index, uint8_t bytepos, uint8_t val);
	void mailbox_set_datalen(uint8_t uc_index, uint8_t dlen);
	void mailbox_set_datal(uint8_t uc_index, uint32_t val);
	void mailbox_set_datah(uint8_t uc_index, uint32_t val);

	void sendFrame(CAN_FRAME& txFrame);
	void setWriteID(uint32_t id);
	template <typename t> void write(t inputValue); //write a variable # of bytes out in a frame. Uses id as the ID.
	void setBigEndian(bool);

	void setCallback(int mailbox, void (*cb)(CAN_FRAME *));
	void setGeneralCallback(void (*cb)(CAN_FRAME *));
	//note that these below versions still use mailbox number. There isn't a good way around this. 
	void attachCANInterrupt(void (*cb)(CAN_FRAME *)); //alternative callname for setGeneralCallback
	void attachCANInterrupt(uint8_t mailBox, void (*cb)(CAN_FRAME *));
	void detachCANInterrupt(uint8_t mailBox);

	void reset_all_mailbox();
	void interruptHandler();
	bool rx_avail();
	int available(); //like rx_avail but returns the number of waiting frames

	uint32_t get_rx_buff(CAN_FRAME &);
	uint32_t read(CAN_FRAME &);
};

extern CANRaw Can0;
extern CANRaw Can1;

#endif // _CAN_LIBRARY_
