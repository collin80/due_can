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

/* Reference for how this struct is defined:
This is from can.h from the libsam files
typedef struct {
	uint32_t ul_mb_idx;
	uint8_t uc_obj_type;  //! Mailbox object type, one of the six different objects.
	uint8_t uc_id_ver;    //! 0 stands for standard frame, 1 stands for extended frame.
	uint8_t uc_length;    //! Received data length or transmitted data length.
	uint8_t uc_tx_prio;   //! Mailbox priority, no effect in receive mode.
	uint32_t ul_status;   //! Mailbox status register value.
	uint32_t ul_id_msk;   //! No effect in transmit mode.
	uint32_t ul_id;       //! Received frame ID or the frame ID to be transmitted.
	uint32_t ul_fid;      //! Family ID.
	uint32_t ul_datal;
	uint32_t ul_datah;
} can_mb_conf_t;
These are from component_can.h in the CMSIS files
typedef struct {
  RwReg  CAN_MMR;       // (CanMb Offset: 0x0) Mailbox Mode Register
  RwReg  CAN_MAM;       // (CanMb Offset: 0x4) Mailbox Acceptance Mask Register
  RwReg  CAN_MID;       /**< \brief (CanMb Offset: 0x8) Mailbox ID Register 
  RwReg  CAN_MFID;      /**< \brief (CanMb Offset: 0xC) Mailbox Family ID Register
  RwReg  CAN_MSR;       /**< \brief (CanMb Offset: 0x10) Mailbox Status Register 
  RwReg  CAN_MDL;       /**< \brief (CanMb Offset: 0x14) Mailbox Data Low Register 
  RwReg  CAN_MDH;       /**< \brief (CanMb Offset: 0x18) Mailbox Data High Register 
  RwReg  CAN_MCR;       /**< \brief (CanMb Offset: 0x1C) Mailbox Control Register 
} CanMb;
/** \brief Can hardware registers
#define CANMB_NUMBER 8
typedef struct {
  RwReg  CAN_MR;        /**< \brief (Can Offset: 0x0000) Mode Register 
  WoReg  CAN_IER;       /**< \brief (Can Offset: 0x0004) Interrupt Enable Register 
  WoReg  CAN_IDR;       /**< \brief (Can Offset: 0x0008) Interrupt Disable Register 
  RoReg  CAN_IMR;       /**< \brief (Can Offset: 0x000C) Interrupt Mask Register 
  RoReg  CAN_SR;        /**< \brief (Can Offset: 0x0010) Status Register 
  RwReg  CAN_BR;        /**< \brief (Can Offset: 0x0014) Baudrate Register 
  RoReg  CAN_TIM;       /**< \brief (Can Offset: 0x0018) Timer Register 
  RoReg  CAN_TIMESTP;   /**< \brief (Can Offset: 0x001C) Timestamp Register 
  RoReg  CAN_ECR;       /**< \brief (Can Offset: 0x0020) Error Counter Register 
  WoReg  CAN_TCR;       /**< \brief (Can Offset: 0x0024) Transfer Command Register 
  WoReg  CAN_ACR;       /**< \brief (Can Offset: 0x0028) Abort Command Register 
  RoReg  Reserved1[46];
  RwReg  CAN_WPMR;      /**< \brief (Can Offset: 0x00E4) Write Protect Mode Register 
  RoReg  CAN_WPSR;      /**< \brief (Can Offset: 0x00E8) Write Protect Status Register 
  RoReg  Reserved2[69];
  CanMb  CAN_MB[CANMB_NUMBER]; /**< \brief (Can Offset: 0x200) MB = 0 .. 7 
} Can;
*/



#ifndef _CAN_LIBRARY_
#define _CAN_LIBRARY_

#include "sn65hvd234.h"

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

	  

#define CAN0_RS  61
#define CAN0_EN  62
#define CAN1_RS  63
#define CAN1_EN  64

/** Define the Mailbox mask for eight mailboxes. */
#define GLOBAL_MAILBOX_MASK           0x000000ff

/** Disable all interrupt mask */
#define CAN_DISABLE_ALL_INTERRUPT_MASK 0xffffffff

/** Define the typical baudrate for CAN communication in KHz. */
#define CAN_BPS_1000K                 1000
#define CAN_BPS_800K                  800
#define CAN_BPS_500K                  500
#define CAN_BPS_250K                  250
#define CAN_BPS_125K                  125
#define CAN_BPS_50K                   50
#define CAN_BPS_25K                   25
#define CAN_BPS_10K                   10
#define CAN_BPS_5K                    5

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

typedef struct
{
	uint32_t id;		// EID if ide set, SID otherwise
	uint32_t fid;		// family ID
	uint8_t rtr;			// Remote Transmission Request
	uint8_t ide;			// Extended ID flag
	uint8_t dlc;			// Number of data bytes
	uint8_t data[8];		// Data bytes
} RX_CAN_FRAME;

typedef struct
{
	uint32_t id;		// EID if ide set, SID otherwise
	uint8_t rtr;			// Remote Transmission Request
	uint8_t ide;			// Extended ID flag
	uint8_t dlc;			// Number of data bytes
	uint8_t priority;		//transmit priority for this message
	uint8_t data[8];		// Data bytes
} TX_CAN_FRAME;

class CANRaw
{
  protected:
    /* CAN peripheral, set by constructor */
    Can* m_pCan ;

    /* CAN Transceiver */
    SSN65HVD234* m_Transceiver;

	volatile RX_CAN_FRAME rx_frame_buff[SIZE_RX_BUFFER];
	volatile TX_CAN_FRAME tx_frame_buff[SIZE_TX_BUFFER];

	volatile uint16_t rx_buffer_head, rx_buffer_tail;
	volatile uint16_t tx_buffer_head, tx_buffer_tail;
	void mailbox_int_handler(uint8_t mb, uint32_t ul_status);

  private:

  public:
    // Constructor
    CANRaw( Can* pCan , uint32_t Rs, uint32_t En);

    /**
 * \defgroup sam_driver_can_group Controller Area Network (CAN) Driver
 *
 * See \ref sam_can_quickstart.
 *
 * \par Purpose
 *
 * The CAN controller provides all the features required to implement
 * the serial communication protocol CAN defined by Robert Bosch GmbH,
 * the CAN specification. This is a driver for configuration, enabling,
 * disabling and use of the CAN peripheral.
 *
 * @{
 */

uint32_t set_baudrate(uint32_t ul_mck, uint32_t ul_baudrate);
uint32_t init(uint32_t ul_mck, uint32_t ul_baudrate);
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
uint32_t mailbox_read(uint8_t uc_index, volatile RX_CAN_FRAME *rxframe);
uint32_t mailbox_tx_frame(uint8_t uc_index);
void mailbox_set_id(uint8_t uc_index, uint32_t id, bool extended);
void mailbox_set_priority(uint8_t uc_index, uint8_t pri);
void mailbox_set_accept_mask(uint8_t uc_index, uint32_t mask, bool ext);
void mailbox_set_mode(uint8_t uc_index, uint8_t mode);
void mailbox_set_databyte(uint8_t uc_index, uint8_t bytepos, uint8_t val);
void mailbox_set_datalen(uint8_t uc_index, uint8_t dlen);
void mailbox_set_datal(uint8_t uc_index, uint32_t val);
void mailbox_set_datah(uint8_t uc_index, uint32_t val);

void reset_all_mailbox();
void interruptHandler();
bool rx_avail();
uint32_t get_rx_buff(RX_CAN_FRAME *);
};

extern CANRaw CAN;
extern CANRaw CAN2;

#endif // _CAN_LIBRARY_
