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

#include <Arduino.h>
#include <can_common.h>

#define DUE_CAN_MAILBOX_TX_BUFFER_SUPPORT  // helper definition for handling different FlexCAN revisions
#define DUE_CAN_DYNAMIC_BUFFER_SUPPORT  // helper definition for handling different FlexCAN revisions

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

#define CAN0_EN  50 //these enable pins match most all recent EVTV boards (EVTVDue, CAN Due 2.0)
#define CAN1_EN  48 //they're only defaults, you can set whichever pin you need when calling begin()

/** Define the Mailbox mask for eight mailboxes. */
#define GLOBAL_MAILBOX_MASK           0x000000ff

/** Disable all interrupt mask */
#define CAN_DISABLE_ALL_INTERRUPT_MASK 0xffffffff

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
   //TQ     PROG     PH1      PH2      SJW    SAMP
	{8,   (2 + 1), (1 + 1), (1 + 1), (2 + 1), 75},
	{9,   (1 + 1), (2 + 1), (2 + 1), (1 + 1), 67},
	{10,  (2 + 1), (2 + 1), (2 + 1), (2 + 1), 70},
	{11,  (3 + 1), (2 + 1), (2 + 1), (2 + 1), 72},
	{12,  (2 + 1), (3 + 1), (3 + 1), (2 + 1), 67},
	{13,  (3 + 1), (3 + 1), (3 + 1), (2 + 1), 77},
	{14,  (3 + 1), (3 + 1), (4 + 1), (2 + 1), 64},
	{15,  (3 + 1), (4 + 1), (4 + 1), (2 + 1), 67},
	{16,  (4 + 1), (4 + 1), (4 + 1), (2 + 1), 69},
	{17,  (5 + 1), (4 + 1), (4 + 1), (2 + 1), 71},
	{18,  (4 + 1), (5 + 1), (5 + 1), (2 + 1), 67},
	{19,  (5 + 1), (5 + 1), (5 + 1), (2 + 1), 68},
	{20,  (6 + 1), (5 + 1), (5 + 1), (2 + 1), 70},
	{21,  (7 + 1), (5 + 1), (5 + 1), (2 + 1), 71},
	{22,  (6 + 1), (6 + 1), (6 + 1), (2 + 1), 68},
	{23,  (7 + 1), (7 + 1), (6 + 1), (2 + 1), 70},
	{24,  (6 + 1), (7 + 1), (7 + 1), (2 + 1), 67},
	{25,  (7 + 1), (7 + 1), (7 + 1), (2 + 1), 68}
};

class CANRaw: public CAN_COMMON
{
public:
    // Constructor
    CANRaw( Can* pCan, uint32_t En);

    // Before begin, you can define rx buffer size. Default is SIZE_RX_BUFFER. This does not have effect after begin.
    void setRxBufferSize(uint16_t size) { if (!isInitialized() ) sizeRxBuffer=size; }

    // Before begin, you can define global tx buffer size. Default is SIZE_TX_BUFFER. This does not have effect after begin.
    void setTxBufferSize(uint16_t size) { if (!isInitialized() ) sizeTxBuffer=size; }

    // You can define mailbox specific tx buffer size. This can be defined only once per mailbox.
    // As default prioritized messages will not be buffered. If you define buffer size for mail box, the messages will be
    // buffered to own buffer, if necessary.
    void setMailBoxTxBufferSize(uint8_t mbox, uint16_t size);

    int setNumTXBoxes(int txboxes);
    inline uint8_t getFirstTxBox() { return getNumMailBoxes()-numTXBoxes; }
    inline uint8_t getLastTxBox() { return getNumMailBoxes()-1; }
    inline uint8_t getNumMailBoxes() { return CANMB_NUMBER; }
    inline uint8_t getNumRxBoxes() { return getNumMailBoxes()-numTXBoxes; }
  
	int findFreeRXMailbox();
	uint8_t mailbox_get_mode(uint8_t uc_index);
	uint32_t mailbox_get_id(uint8_t uc_index);
	uint32_t getMailboxIer(int8_t mailbox);

	bool sendFrame(CAN_FRAME& txFrame, uint8_t mbox);
	void setWriteID(uint32_t id);
	template <typename t> void write(t inputValue); //write a variable # of bytes out in a frame. Uses id as the ID.
	void setBigEndian(bool);

	void reset_all_mailbox();
	void interruptHandler();

    //declaration of all the functions we need to override from CAN_COMMON
	int _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);
    int _setFilter(uint32_t id, uint32_t mask, bool extended);
	uint32_t init(uint32_t ul_baudrate);
    uint32_t beginAutoSpeed();
    uint32_t set_baudrate(uint32_t ul_baudrate);
    void setListenOnlyMode(bool state);
	void enable();
	void disable();
	bool sendFrame(CAN_FRAME& txFrame);
	bool rx_avail();
	uint16_t available(); //like rx_avail but returns the number of waiting frames
	uint32_t get_rx_buff(CAN_FRAME &msg);
	
	//misc old cruft kept around just in case anyone actually used any of it in older code.
	//some are used within the functions above. Unless you really know of a good reason to use
	//any of these you probably should steer clear of them.
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
	uint16_t get_internal_timer_value();
	uint16_t get_timestamp_value();
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
	void  mailbox_set_rtr (uint8_t mbox,  uint8_t rtr) ;

protected:
  struct ringbuffer_t {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t size;
    volatile CAN_FRAME *buffer;
  };
 
  int numTXBoxes; //There are 8 mailboxes, anything not TX will be set RX
  uint16_t sizeRxBuffer;
  uint16_t sizeTxBuffer;

  void initRingBuffer (ringbuffer_t &ring, volatile CAN_FRAME *buffer, uint16_t size);
  bool addToRingBuffer (ringbuffer_t &ring, const CAN_FRAME &msg);
  bool removeFromRingBuffer (ringbuffer_t &ring, CAN_FRAME &msg);
  inline bool isRingBufferEmpty (ringbuffer_t &ring) { return (ring.head == ring.tail); }
  uint16_t ringBufferCount (ringbuffer_t &ring);

  void irqLock() { NVIC_DisableIRQ(nIRQ); }
  void irqRelease() { NVIC_EnableIRQ(nIRQ); }

  void initializeBuffers();
  bool isInitialized() { return tx_frame_buff!=0; }
    
  bool usesGlobalTxRing(uint8_t mbox) { return (mbox<getNumMailBoxes()?txRings[mbox]==0:true); }
  bool isTxBox(uint8_t mbox) { return (mbox>=getFirstTxBox() && mbox<getNumMailBoxes() ); }
  
private:
	/* CAN peripheral, set by constructor */
	Can* m_pCan;
    IRQn_Type nIRQ;

	volatile CAN_FRAME *rx_frame_buff; //[SIZE_RX_BUFFER];
	volatile CAN_FRAME *tx_frame_buff; //[SIZE_TX_BUFFER];
    ringbuffer_t txRing;
    ringbuffer_t rxRing;
    ringbuffer_t * txRings[CANMB_NUMBER];
	uint8_t filterSet[CANMB_NUMBER];

    void writeTxRegisters(const CAN_FRAME &txFrame, uint8_t mb);
	void setModeBit(uint32_t bit);
	void unsetModeBit(uint32_t bit);
	void mailbox_int_handler(uint8_t mb, uint32_t ul_status);
	
	uint32_t write_id; //storage for an id. Will be used by the write function to set which ID to send to.
	bool bigEndian;
    
    uint32_t numBusErrors;
    uint32_t numRxFrames;
};

extern CANRaw Can0;
extern CANRaw Can1;

#endif // _CAN_LIBRARY_
