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


#include "due_can.h"


/**
* \brief constructor for the class
*
* \param pCan Which canbus hardware to use (CAN0 or CAN1)
* \param Rs pin to use for transceiver Rs control
* \param En pin to use for transceiver enable
*/
CANRaw::CANRaw(Can* pCan, uint32_t En ) {
	m_pCan = pCan;
	enablePin = En;
	bigEndian = false;
	busSpeed = 0;
	
	for (int i = 0; i < 4; i++) listener[i] = NULL;
}

/**
 * \brief Configure CAN baudrate.
 *
 * \param ul_baudrate Baudrate value (kB/s), allowed values:
 *                    1000, 800, 500, 250, 125, 50, 25, 10, 5.
 *
 * \retval Set the baudrate successfully or not.
 */
uint32_t CANRaw::set_baudrate(uint32_t ul_baudrate)
{
	uint8_t uc_tq;
	uint8_t uc_prescale;
	uint32_t ul_mod;
	uint32_t ul_cur_mod;
	can_bit_timing_t *p_bit_time;

	static uint32_t ul_mck = SystemCoreClock;

	/* Check whether the baudrate prescale will be greater than the max divide value. */
	if (((ul_mck + (ul_baudrate * CAN_MAX_TQ_NUM - 1)) /
		(ul_baudrate * CAN_MAX_TQ_NUM)) > CAN_BAUDRATE_MAX_DIV) {
		return 0;
	}

	/* Check whether the input MCK is too small. */
	if (ul_mck  < ul_baudrate * CAN_MIN_TQ_NUM) {
		return 0;
	}

	/* Initialize it as the minimum Time Quantum. */
	uc_tq = CAN_MIN_TQ_NUM;

	/* Initialize the remainder as the max value. When the remainder is 0, get the right TQ number. */
	ul_mod = 0xffffffff;
	/* Find out the approximate Time Quantum according to the baudrate. */
	for (uint8_t i = CAN_MIN_TQ_NUM; i <= CAN_MAX_TQ_NUM; i++) {
		if ((ul_mck / (ul_baudrate * i)) <= CAN_BAUDRATE_MAX_DIV) {
			ul_cur_mod = ul_mck % (ul_baudrate * i);
			if (ul_cur_mod < ul_mod){
				ul_mod = ul_cur_mod;
				uc_tq = i;
				if (!ul_mod) {
					break;
				}
			}
		}
	}

	/* Calculate the baudrate prescale value. */
	uc_prescale = ul_mck / (ul_baudrate * uc_tq);

	/* Get the right CAN BIT Timing group. */
	p_bit_time = (can_bit_timing_t *)&can_bit_time[uc_tq - CAN_MIN_TQ_NUM];

	/* Before modifying the CANBR register, disable the CAN controller. */
	//can_disable(m_pCan);
    m_pCan->CAN_MR &= ~CAN_MR_CANEN;

	/* Write into the CAN baudrate register. */
	m_pCan->CAN_BR = CAN_BR_PHASE2(p_bit_time->uc_phase2 - 1) |
					CAN_BR_PHASE1(p_bit_time->uc_phase1 - 1) |
					CAN_BR_PROPAG(p_bit_time->uc_prog - 1) |
					CAN_BR_SJW(p_bit_time->uc_sjw - 1) |
					CAN_BR_BRP(uc_prescale - 1);
	return 1;
}


uint32_t CANRaw::begin()
{
	return init(CAN_DEFAULT_BAUD);
}

uint32_t CANRaw::begin(uint32_t baudrate) 
{
	return init(baudrate);
}

uint32_t CANRaw::begin(uint32_t baudrate, uint8_t enablePin) 
{
	this->enablePin = enablePin;
	return init(baudrate);
}

uint32_t CANRaw::getBusSpeed()
{
	return busSpeed;
}

/**
 * \brief Initialize CAN controller.
 *
 * \param ul_mck CAN module input clock.
 * \param ul_baudrate CAN communication baudrate in kbs.
 *
 * \retval 0 If failed to initialize the CAN module; otherwise successful.
 *
 * \note PMC clock for CAN peripheral should be enabled before calling this function.
 */
uint32_t CANRaw::init(uint32_t ul_baudrate)
{
	uint32_t ul_flag;
	uint32_t ul_tick;
	
	//there used to be code here to cause this function to not run if the hardware is already initialized. But,
	//it can be helpful to reinitialize. For instance, if the bus rate was set improperly you might need to
	//go back through this code to reset the hardware.

	//initialize all function pointers to null
	for (int i = 0; i < 9; i++) cbCANFrame[i] = 0;

//arduino 1.5.2 doesn't init canbus so make sure to do it here. 
#ifdef ARDUINO152
	PIO_Configure(PIOA,PIO_PERIPH_A, PIO_PA1A_CANRX0|PIO_PA0A_CANTX0, PIO_DEFAULT);
	PIO_Configure(PIOB,PIO_PERIPH_A, PIO_PB15A_CANRX1|PIO_PB14A_CANTX1, PIO_DEFAULT);
#endif
	
	if (m_pCan == CAN0) pmc_enable_periph_clk(ID_CAN0);
	if (m_pCan == CAN1) pmc_enable_periph_clk(ID_CAN1);

	if (enablePin != 255) {
		pinMode(enablePin, OUTPUT);
		digitalWrite(enablePin, HIGH);
	}

	/* Initialize the baudrate for CAN module. */
	ul_flag = set_baudrate(ul_baudrate);
	if (ul_flag == 0) {
		return 0;
	}

	/* Reset the CAN eight message mailbox. */
	reset_all_mailbox();

	//Also disable all interrupts by default
	disable_interrupt(CAN_DISABLE_ALL_INTERRUPT_MASK);

	//By default use one mailbox for TX 
	setNumTXBoxes(1);

	/* Enable the CAN controller. */
	enable();

	/* Wait until the CAN is synchronized with the bus activity. */
	ul_flag = 0;
	ul_tick = 0;
	while (!(ul_flag & CAN_SR_WAKEUP) && (ul_tick < CAN_TIMEOUT)) {
		ul_flag = m_pCan->CAN_SR;
		ul_tick++;
	}

	//set a fairly low priority so almost anything can preempt.
	//this has the effect that most anything can interrupt our interrupt handler
	//that's a good thing because the interrupt handler is long and complicated
	//and can send callbacks into user code which could also be long and complicated.
	//But, keep in mind that user code in callbacks runs in interrupt context
	//but can still be preempted at any time.
	NVIC_SetPriority(m_pCan == CAN0 ? CAN0_IRQn : CAN1_IRQn, 12); 
	
	NVIC_EnableIRQ(m_pCan == CAN0 ? CAN0_IRQn : CAN1_IRQn); //tell the nested interrupt controller to turn on our interrupt

	/* Timeout or the CAN module has been synchronized with the bus. */
	if (CAN_TIMEOUT == ul_tick) {
		return 0;
	} else {
		busSpeed = ul_baudrate;
		return busSpeed;
	}
}

 /* \brief Initializes mailboxes to the requested mix of RX and TX boxes
 *
 * \param txboxes How many of the 8 boxes should be used for TX
 *
 */
void CANRaw::setNumTXBoxes(int txboxes) {
	int c;

	if (txboxes > 8) txboxes = 8;
	if (txboxes < 0) txboxes = 0;
	numTXBoxes = txboxes;

	//Inialize RX boxen
	for (c = 0; c < 8 - numTXBoxes; c++) {
		mailbox_set_mode(c, CAN_MB_RX_MODE);
		mailbox_set_id(c, 0x0, false);
		mailbox_set_accept_mask(c, 0x7FF, false);
	}

	//Initialize TX boxen
	for (c = 8 - numTXBoxes; c < 8; c++) {
		mailbox_set_mode(c, CAN_MB_TX_MODE);
		mailbox_set_priority(c, 10);
		mailbox_set_accept_mask(c, 0x7FF, false);
	}
}

/**
 * \brief Set up a callback function for given mailbox
 *
 * \param mailbox Which mailbox (0-7) to assign callback to.
 * \param cb A function pointer to a function with prototype "void functionname(CAN_FRAME *frame);"
 *
 */
void CANRaw::setCallback(int mailbox, void (*cb)(CAN_FRAME *))
{
	if ((mailbox < 0) || (mailbox > 7)) return;
	cbCANFrame[mailbox] = cb;
}

/**
 * \brief Set up a general callback that will be used if no callback was registered for receiving mailbox
 *
 * \param cb A function pointer to a function with prototype "void functionname(CAN_FRAME *frame);"
 *
 * \note If this function is used to set up a callback then no buffering of frames will ever take place.
 */
void CANRaw::setGeneralCallback(void (*cb)(CAN_FRAME *))
{
	cbCANFrame[8] = cb;
}

void CANRaw::attachCANInterrupt(void (*cb)(CAN_FRAME *)) 
{
	setGeneralCallback(cb);
}

void CANRaw::attachCANInterrupt(uint8_t mailBox, void (*cb)(CAN_FRAME *)) 
{
	setCallback(mailBox, cb);
}

void CANRaw::detachCANInterrupt(uint8_t mailBox)
{
	if ((mailBox < 0) || (mailBox > 7)) return;
	cbCANFrame[mailBox] = 0;
}

boolean CANRaw::attachObj(CANListener *listener)
{
	for (int i = 0; i < SIZE_LISTENERS; i++)
	{
		if (this->listener[i] == NULL)
		{
			this->listener[i] = listener;
			listener->callbacksActive = 0;
			return true;			
		}
	}
	return false;
}

boolean CANRaw::detachObj(CANListener *listener)
{
	for (int i = 0; i < SIZE_LISTENERS; i++)
	{
		if (this->listener[i] == listener)
		{
			this->listener[i] = NULL;			
			return true;			
		}
	}
	return false;  
}

/**
 * \brief Enable CAN Controller.
 *
 */
void CANRaw::enable()
{
	m_pCan->CAN_MR |= CAN_MR_CANEN;
	if (enablePin != 255) digitalWrite(enablePin, HIGH);
}

/**
 * \brief Disable CAN Controller.
 *
 */
void CANRaw::disable()
{
	m_pCan->CAN_MR &= ~CAN_MR_CANEN;
	if (enablePin != 255) digitalWrite(enablePin, LOW);
}

/**
 * \brief Disable CAN Controller low power mode.
 *
 */
void CANRaw::disable_low_power_mode()
{
	m_pCan->CAN_MR &= ~CAN_MR_LPM;
}

/**
 * \brief Enable CAN Controller low power mode.
 *
 */
void CANRaw::enable_low_power_mode()
{
	m_pCan->CAN_MR |= CAN_MR_LPM;
}

/**
 * \brief Disable CAN Controller autobaud/listen mode.
 *
 */
void CANRaw::disable_autobaud_listen_mode()
{
	m_pCan->CAN_MR &= ~CAN_MR_ABM;
}

/**
 * \brief Enable CAN Controller autobaud/listen mode.
 *
 */
void CANRaw::enable_autobaud_listen_mode()
{
	m_pCan->CAN_MR |= CAN_MR_ABM;
}

/**
 * \brief CAN Controller won't generate overload frame.
 *
 */
void CANRaw::disable_overload_frame()
{
	m_pCan->CAN_MR &= ~CAN_MR_OVL;
}

/**
 * \brief CAN Controller will generate an overload frame after each successful
 * reception for mailboxes configured in Receive mode, Producer and Consumer.
 *
 */
void CANRaw::enable_overload_frame()
{
	m_pCan->CAN_MR |= CAN_MR_OVL;
}

/**
 * \brief Configure the timestamp capture point, at the start or the end of frame.
 *
 * \param m_pCan   Pointer to a CAN peripheral instance.
 * \param ul_flag 0: Timestamp is captured at each start of frame;
 *                1: Timestamp is captured at each end of frame.
 */
void CANRaw::set_timestamp_capture_point(uint32_t ul_flag)
{
	if (ul_flag) {
		m_pCan->CAN_MR |= CAN_MR_TEOF;
	} else {
		m_pCan->CAN_MR &= ~CAN_MR_TEOF;
	}
}

/**
 * \brief Disable CAN Controller time triggered mode.
 *
 */
void CANRaw::disable_time_triggered_mode()
{
	m_pCan->CAN_MR &= ~CAN_MR_TTM;
}

/**
 * \brief Enable CAN Controller time triggered mode.
 *
 */
void CANRaw::enable_time_triggered_mode()
{
	m_pCan->CAN_MR |= CAN_MR_TTM;
}

/**
 * \brief Disable CAN Controller timer freeze.
 *
 */
void CANRaw::disable_timer_freeze()
{
	m_pCan->CAN_MR &= ~CAN_MR_TIMFRZ;
}

/**
 * \brief Enable CAN Controller timer freeze.
 *
 */
void CANRaw::enable_timer_freeze()
{
	m_pCan->CAN_MR |= CAN_MR_TIMFRZ;
}

/**
 * \brief Disable CAN Controller transmit repeat function.
 *
 */
void CANRaw::disable_tx_repeat()
{
	m_pCan->CAN_MR |= CAN_MR_DRPT;
}

/**
 * \brief Enable CAN Controller transmit repeat function.
 *
 */
void CANRaw::enable_tx_repeat()
{
	m_pCan->CAN_MR &= ~CAN_MR_DRPT;
}

/**
 * \brief Configure CAN Controller reception synchronization stage.
 *
 * \param ul_stage The reception stage to be configured.
 *
 * \note This is just for debug purpose only.
 */
void CANRaw::set_rx_sync_stage(uint32_t ul_stage)
{
	m_pCan->CAN_MR = (m_pCan->CAN_MR & ~CAN_MR_RXSYNC_Msk) | ul_stage;
}

/**
 * \brief Enable CAN interrupt.
 *
 * \param dw_mask Interrupt to be enabled.
 */
void CANRaw::enable_interrupt(uint32_t dw_mask)
{
	m_pCan->CAN_IER = dw_mask;
}

/**
 * \brief Disable CAN interrupt.
 *
 * \param dw_mask Interrupt to be disabled.
 */
void CANRaw::disable_interrupt(uint32_t dw_mask)
{
	m_pCan->CAN_IDR = dw_mask;
}

/**
 * \brief Get CAN Interrupt Mask.
 *
 *
 * \retval CAN interrupt mask.
 */
uint32_t CANRaw::get_interrupt_mask()
{
	return (m_pCan->CAN_IMR);
}

/**
 * \brief Get CAN status.
 *
 *
 * \retval CAN status.
 */
uint32_t CANRaw::get_status()
{
	return (m_pCan->CAN_SR);
}

/**
 * \brief Get the 16-bit free-running internal timer count.
 *
 *
 * \retval The internal CAN free-running timer counter.
 */
uint32_t CANRaw::get_internal_timer_value()
{
	return (m_pCan->CAN_TIM);
}

/**
 * \brief Get CAN timestamp register value.
 *
 *
 * \retval The timestamp value.
 */
uint32_t CANRaw::get_timestamp_value()
{
	return (m_pCan->CAN_TIMESTP);
}

/**
 * \brief Get CAN transmit error counter.
 *
 *
 * \retval Transmit error counter.
 */
uint8_t CANRaw::get_tx_error_cnt()
{
	return (uint8_t) (m_pCan->CAN_ECR >> CAN_ECR_TEC_Pos);
}

/**
 * \brief Get CAN receive error counter.
 *
 *
 * \retval Receive error counter.
 */
uint8_t CANRaw::get_rx_error_cnt()
{
	return (uint8_t) (m_pCan->CAN_ECR >> CAN_ECR_REC_Pos);
}

/**
 * \brief Reset the internal free-running 16-bit timer.
 *
 *
 * \note If the internal timer counter is frozen, this function automatically
 * re-enables it.
 */
void CANRaw::reset_internal_timer()
{
	m_pCan->CAN_TCR |= CAN_TCR_TIMRST;
}

/**
 * \brief Send global transfer request.
 *
 * \param uc_mask Mask for mailboxes that are requested to transfer.
 */
void CANRaw::global_send_transfer_cmd(uint8_t uc_mask)
{
	m_pCan->CAN_TCR = uc_mask & GLOBAL_MAILBOX_MASK;
}

/**
 * \brief Send global abort request.
 *
 * \param uc_mask Mask for mailboxes that are requested to abort.
 */
void CANRaw::global_send_abort_cmd(uint8_t uc_mask)
{
	uint32_t ul_reg;
	
	ul_reg = m_pCan->CAN_ACR & ((uint32_t)~GLOBAL_MAILBOX_MASK);
	m_pCan->CAN_ACR = ul_reg | uc_mask;
}

/**
 * \brief Configure the timemark for the mailbox.
 *
 * \param uc_index Indicate which mailbox is to be configured.
 * \param us_cnt   The timemark to be set.
 *
 * \note The timemark is active in Time Triggered mode only.
 */
void CANRaw::mailbox_set_timemark(uint8_t uc_index, uint16_t us_cnt)
{
	uint32_t ul_reg;
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	ul_reg = m_pCan->CAN_MB[uc_index].CAN_MMR & ((uint32_t)~TIMEMARK_MASK);
	m_pCan->CAN_MB[uc_index].CAN_MMR = ul_reg | us_cnt;
}

/**
 * \brief Get status of the mailbox.
 *
 * \param uc_index Indicate which mailbox is to be read.
 *
 * \retval The mailbox status.
 */
uint32_t CANRaw::mailbox_get_status(uint8_t uc_index)
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	return (m_pCan->CAN_MB[uc_index].CAN_MSR);
}

/**
 * \brief Send single mailbox transfer request.
 *
 * \param uc_index Indicate which mailbox is to be configured.
 */
void CANRaw::mailbox_send_transfer_cmd(uint8_t uc_index)
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MTCR;
}

/**
 * \brief Send single mailbox abort request.
 *
 * \param uc_index Indicate which mailbox is to be configured.
 */
void CANRaw::mailbox_send_abort_cmd(uint8_t uc_index)
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MACR;
}

/**
 * \brief Initialize the mailbox to a default, known state.
 *
 * \param p_mailbox Pointer to a CAN mailbox instance.
 */
void CANRaw::mailbox_init(uint8_t uc_index)
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	m_pCan->CAN_MB[uc_index].CAN_MMR = 0;
	m_pCan->CAN_MB[uc_index].CAN_MAM = 0;
	m_pCan->CAN_MB[uc_index].CAN_MID = 0;
	m_pCan->CAN_MB[uc_index].CAN_MDL = 0;
	m_pCan->CAN_MB[uc_index].CAN_MDH = 0;
	m_pCan->CAN_MB[uc_index].CAN_MCR = 0;
}

/**
 * \brief Reset the eight mailboxes.
 *
 * \param m_pCan Pointer to a CAN peripheral instance.
 */
void CANRaw::reset_all_mailbox()
{
	for (uint8_t i = 0; i < CANMB_NUMBER; i++) {		
		mailbox_init(i);
	}
}

void CANRaw::setBigEndian(bool end)  
{
	bigEndian = end;
}

void CANRaw::setWriteID(uint32_t id)
{
	write_id = id;
}

template <typename t> void CANRaw::write(t inputValue)
{
	CAN_FRAME tempFrame;
	uint8_t *buff = (uint8_t *)inputValue;
	int thisSize = sizeof(t);
	if (thisSize > 8) thisSize = 8;
	if (!bigEndian) {
		for (int i = 0; i < thisSize; i++) 
		{
			tempFrame.data.bytes[i] = buff[i];
		}
	}
	else //reverse byte order. The M3 is in little endian so this causes big endian order
	{
		for (int i = 0; i < thisSize; i++) 
		{
			tempFrame.data.bytes[i] = buff[thisSize - i - 1];
		}
	}

	tempFrame.id = this->write_id;
	tempFrame.length = thisSize;
	if (this->write_id > 0x7FF) tempFrame.extended = true;
	else tempFrame.extended = false;
	sendFrame(tempFrame);
}

/**
 * \brief Send a frame out of this canbus port
 *
 * \param txFrame The filled out frame structure to use for sending
 *
 * \note Will do one of two things - 1. Send the given frame out of the first available mailbox
 * or 2. queue the frame for sending later via interrupt. Automatically turns on TX interrupt
 * if necessary.
 * 
 * Returns whether sending/queueing succeeded. Will not smash the queue if it gets full.
 */
bool CANRaw::sendFrame(CAN_FRAME& txFrame) 
{
	for (int i = 0; i < 8; i++) {
		if (((m_pCan->CAN_MB[i].CAN_MMR >> 24) & 7) == CAN_MB_TX_MODE)
		{//is this mailbox set up as a TX box?
			if (m_pCan->CAN_MB[i].CAN_MSR & CAN_MSR_MRDY) 
			{//is it also available (not sending anything?)
				mailbox_set_id(i, txFrame.id, txFrame.extended);
				mailbox_set_datalen(i, txFrame.length);
				mailbox_set_priority(i, txFrame.priority);
				for (uint8_t cnt = 0; cnt < 8; cnt++)
				{    
					mailbox_set_databyte(i, cnt, txFrame.data.bytes[cnt]);
				}       
				enable_interrupt(0x01u << i); //enable the TX interrupt for this box
				global_send_transfer_cmd((0x1u << i));
				return true; //we've sent it. mission accomplished.
			}
		}
    }
	
    //if execution got to this point then no free mailbox was found above
    //so, queue the frame if possible. But, don't increment the 
	//tail if it would smash into the head and kill the queue.
	uint8_t temp;
	temp = (tx_buffer_tail + 1) % SIZE_TX_BUFFER;
	if (temp == tx_buffer_head) return false;
    tx_frame_buff[tx_buffer_tail].id = txFrame.id;
    tx_frame_buff[tx_buffer_tail].extended = txFrame.extended;
    tx_frame_buff[tx_buffer_tail].length = txFrame.length;
    tx_frame_buff[tx_buffer_tail].data.value = txFrame.data.value;
    tx_buffer_tail = temp;
	return true;
}


/**
 * \brief Read a frame from out of the mailbox and into a software buffer
 *
 * \param uc_index which mailbox to read
 * \param rxframe Pointer to a receive frame structure which we'll fill out
 *
 * \retval Different CAN mailbox transfer status.
 *
 */
uint32_t CANRaw::mailbox_read(uint8_t uc_index, volatile CAN_FRAME *rxframe)
{
	uint32_t ul_status;
	uint32_t ul_retval;
	uint32_t ul_id;
	uint32_t ul_datal, ul_datah;

	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;

	ul_retval = 0;
	ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;

	/* Check whether there is overwriting happening in Receive with Overwrite mode,
	   or there're messages lost in Receive mode. */
	if ((ul_status & CAN_MSR_MRDY) && (ul_status & CAN_MSR_MMI)) {
		ul_retval = CAN_MAILBOX_RX_OVER;
	}

	ul_id = m_pCan->CAN_MB[uc_index].CAN_MID;
	if ((ul_id & CAN_MID_MIDE) == CAN_MID_MIDE) { //extended id
		rxframe->id = ul_id & 0x1FFFFFFFu;
		rxframe->extended = true;
	}
	else { //standard ID
		rxframe->id = (ul_id  >> CAN_MID_MIDvA_Pos) & 0x7ffu;
		rxframe->extended = false;
	}
	rxframe->fid = m_pCan->CAN_MB[uc_index].CAN_MFID;
	rxframe->length = (ul_status & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;
	ul_datal = m_pCan->CAN_MB[uc_index].CAN_MDL;
	ul_datah = m_pCan->CAN_MB[uc_index].CAN_MDH;

	rxframe->data.high = ul_datah;
	rxframe->data.low = ul_datal;

	/* Read the mailbox status again to check whether the software needs to re-read mailbox data register. */
	ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;	
	if (ul_status & CAN_MSR_MMI) {
		ul_retval |= CAN_MAILBOX_RX_NEED_RD_AGAIN;
	} else {
		ul_retval |= CAN_MAILBOX_TRANSFER_OK;
	}

	/* Enable next receive process. */
	mailbox_send_transfer_cmd(uc_index);

	return ul_retval;
}

/**
 * \brief Sets the ID portion of the given mailbox
 *
 * \param uc_index The mailbox to set (0-7)
 * \param id The ID to set (11 or 29 bit)
 * \param extended Boolean indicating if this ID should be designated as extended
 *
 */
void CANRaw::mailbox_set_id(uint8_t uc_index, uint32_t id, bool extended) 
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	if (extended) {
		m_pCan->CAN_MB[uc_index].CAN_MID = id | CAN_MID_MIDE;
	}
	else {
		m_pCan->CAN_MB[uc_index].CAN_MID = CAN_MID_MIDvA(id);
	}
}

/**
 * \brief Get ID currently associated with a given mailbox
 *
 * \param uc_index The mailbox to get the ID from (0-7)
 *
 * \retval The ID associated with the mailbox
 *
 */
uint32_t CANRaw::mailbox_get_id(uint8_t uc_index) {
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	if (m_pCan->CAN_MB[uc_index].CAN_MID & CAN_MID_MIDE) {
		return m_pCan->CAN_MB[uc_index].CAN_MID;
	}
	else {
		return (m_pCan->CAN_MB[uc_index].CAN_MID >> CAN_MID_MIDvA_Pos) & 0x7ffu;
	}
}

/**
 * \brief Set the transmission priority for given mailbox
 *
 * \param uc_index The mailbox to use
 * \param pri The priority to set (0-15 in descending priority)
 *
 */
void CANRaw::mailbox_set_priority(uint8_t uc_index, uint8_t pri) 
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	m_pCan->CAN_MB[uc_index].CAN_MMR = (m_pCan->CAN_MB[uc_index].CAN_MMR & ~CAN_MMR_PRIOR_Msk) | (pri << CAN_MMR_PRIOR_Pos);
}

/**
 * \brief Set mask for RX on the given mailbox
 *
 * \param uc_index The mailbox to use
 * \param mask The mask to set
 * \param ext Whether this should be an extended mask or not
 *
 */
void CANRaw::mailbox_set_accept_mask(uint8_t uc_index, uint32_t mask, bool ext)
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	if (ext) {
		m_pCan->CAN_MB[uc_index].CAN_MAM = mask | CAN_MAM_MIDE;
		m_pCan->CAN_MB[uc_index].CAN_MID |= CAN_MAM_MIDE;
	} else {
		m_pCan->CAN_MB[uc_index].CAN_MAM = CAN_MAM_MIDvA(mask);
		m_pCan->CAN_MB[uc_index].CAN_MID &= ~CAN_MAM_MIDE;
	}
}

/**
 * \brief Set the mode of the given mailbox
 *
 * \param uc_index Which mailbox to set (0-7)
 * \param mode The mode to set mailbox to 
 *
 * \Note Modes: 0 = Disabled, 1 = RX, 2 = RX with overwrite
 * 3 = TX, 4 = consumer 5 = producer
 */
void CANRaw::mailbox_set_mode(uint8_t uc_index, uint8_t mode) {
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	if (mode > 5) mode = 0; //set disabled on invalid mode
	m_pCan->CAN_MB[uc_index].CAN_MMR = (m_pCan->CAN_MB[uc_index].CAN_MMR &
		~CAN_MMR_MOT_Msk) | (mode << CAN_MMR_MOT_Pos);
}

/**
 * \brief Get current mode of given mailbox
 *
 * \param uc_index Which mailbox to retrieve mode from (0-7)
 *
 * \retval Mode of mailbox
 *
 */
uint8_t CANRaw::mailbox_get_mode(uint8_t uc_index) {
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	return (uint8_t)(m_pCan->CAN_MB[uc_index].CAN_MMR >> CAN_MMR_MOT_Pos) & 0x7;
}

/**
 * \brief Set value of one byte of data for mailbox
 *
 * \param uc_index Which mailbox (0-7)
 * \param bytepos Which byte to set (0-7)
 * \param val The byte value to set
 *
 */
void CANRaw::mailbox_set_databyte(uint8_t uc_index, uint8_t bytepos, uint8_t val)
{
	uint8_t shift; //how many bits to shift
	uint32_t working;  //working copy of the relevant data int
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	if (bytepos > 7) bytepos = 7;
	shift = 8 * (bytepos & 3); //how many bytes to shift up into position
	if (bytepos < 4) { //low data block
		working = m_pCan->CAN_MB[uc_index].CAN_MDL & ~(255 << shift); //mask out where we have to be
		working |= (val << shift);
		m_pCan->CAN_MB[uc_index].CAN_MDL = working;
	}
	else { //high data block
		working = m_pCan->CAN_MB[uc_index].CAN_MDH & ~(255 << shift); //mask out where we have to be
		working |= (val << shift);
		m_pCan->CAN_MB[uc_index].CAN_MDH = working;
	}
}

/**
 * \brief Set the lower 32 bits of the mailbox's data in one shot
 *
 * \param uc_index Which mailbox? (0-7)
 * \param val The 32 bit value to use
 *
 */
void CANRaw::mailbox_set_datal(uint8_t uc_index, uint32_t val)
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	m_pCan->CAN_MB[uc_index].CAN_MDL = val;
}

/**
 * \brief Set the upper 32 bits of the mailbox's data in one shot
 *
 * \param uc_index Which mailbox? (0-7)
 * \param val The 32 bit value to use
 *
 */
void CANRaw::mailbox_set_datah(uint8_t uc_index, uint32_t val)
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	m_pCan->CAN_MB[uc_index].CAN_MDH = val;
}

/**
 * \brief Set # of data bytes for given mailbox
 *
 * \param uc_index Which mailbox? (0-7)
 * \param dlen The number of data bytes to use (0-8)
 *
 */
void CANRaw::mailbox_set_datalen(uint8_t uc_index, uint8_t dlen)
{
	if (uc_index > CANMB_NUMBER-1) uc_index = CANMB_NUMBER-1;
	if (dlen > 8) dlen = 8;
	m_pCan->CAN_MB[uc_index].CAN_MCR = (m_pCan->CAN_MB[uc_index].CAN_MCR &
			~CAN_MCR_MDLC_Msk) | CAN_MCR_MDLC(dlen);
}

/**
 * \brief Command a mailbox to send the frame stored in it
 *
 * \param uc_index which mailbox to send frame. Load it up first
 *
 * \retval CAN_MAILBOX_NOT_READY: Failed because mailbox isn't ready for transmitting message.
 *       CAN_MAILBOX_TRANSFER_OK: Successfully send out a frame.
 */
uint32_t CANRaw::mailbox_tx_frame(uint8_t uc_index)
{
	uint32_t ul_status;
	
	/* Read the mailbox status firstly to check whether the mailbox is ready or not. */
	ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;	
	if (!(ul_status & CAN_MSR_MRDY)) {
		return CAN_MAILBOX_NOT_READY;
	}
	
	/* Set the MBx bit in the Transfer Command Register to send out the remote frame. */
	global_send_transfer_cmd((1 << uc_index));

	return CAN_MAILBOX_TRANSFER_OK;
}

int CANRaw::available()
{
	int val;
	if (rx_avail()) 
	{
		val = rx_buffer_head - rx_buffer_tail;
		//Now, because this is a cyclic buffer it is possible that the ordering was reversed
		//So, handle that case
		if (val < 0) val += SIZE_RX_BUFFER;
	}
	else return 0;
}


/**
* \brief Check whether there are received canbus frames in the buffer
*
* \retval true if there are frames waiting in buffer, otherwise false
*/
bool CANRaw::rx_avail() {
	return (rx_buffer_head != rx_buffer_tail)?true:false;
}

//wrapper for syntactic sugar reasons - could have just been a #define
uint32_t CANRaw::read(CAN_FRAME & buffer) 
{
	return get_rx_buff(buffer);
}

/**
 * \brief Retrieve a frame from the RX buffer
 *
 * \param buffer Reference to the frame structure to fill out
 *
 * \retval 0 no frames waiting to be received, 1 if a frame was returned
 */
uint32_t CANRaw::get_rx_buff(CAN_FRAME& buffer) {
	if (rx_buffer_head == rx_buffer_tail) return 0;
	buffer.id = rx_frame_buff[rx_buffer_tail].id;
	buffer.extended = rx_frame_buff[rx_buffer_tail].extended;
	buffer.length = rx_frame_buff[rx_buffer_tail].length;
	buffer.data.value = rx_frame_buff[rx_buffer_tail].data.value;
	rx_buffer_tail = (rx_buffer_tail + 1) % SIZE_RX_BUFFER;
	return 1;
}

/**
* \brief Handle all interrupt reasons
*/
void CANRaw::interruptHandler() {

	uint32_t ul_status = m_pCan->CAN_SR; //get status of interrupts

	if (ul_status & CAN_SR_MB0) { //mailbox 0 event
		mailbox_int_handler(0, ul_status);
	}
	if (ul_status & CAN_SR_MB1) { //mailbox 1 event
		mailbox_int_handler(1, ul_status);
	}
	if (ul_status & CAN_SR_MB2) { //mailbox 2 event
		mailbox_int_handler(2, ul_status);
	}
	if (ul_status & CAN_SR_MB3) { //mailbox 3 event
		mailbox_int_handler(3, ul_status);
	}
	if (ul_status & CAN_SR_MB4) { //mailbox 4 event
		mailbox_int_handler(4, ul_status);
	}
	if (ul_status & CAN_SR_MB5) { //mailbox 5 event
		mailbox_int_handler(5, ul_status);
	}
	if (ul_status & CAN_SR_MB6) { //mailbox 6 event
		mailbox_int_handler(6, ul_status);
	}
	if (ul_status & CAN_SR_MB7) { //mailbox 7 event
		mailbox_int_handler(7, ul_status);
	}
	if (ul_status & CAN_SR_ERRA) { //error active
	}
	if (ul_status & CAN_SR_WARN) { //warning limit
	}
	if (ul_status & CAN_SR_ERRP) { //error passive
	}
	if (ul_status & CAN_SR_BOFF) { //bus off
	}
	if (ul_status & CAN_SR_SLEEP) { //controller in sleep mode
	}
	if (ul_status & CAN_SR_WAKEUP) { //controller woke up
	}
	if (ul_status & CAN_SR_TOVF) { //timer overflow
	}
	if (ul_status & CAN_SR_TSTP) { //timestamp - start or end of frame
	}
	if (ul_status & CAN_SR_CERR) { //CRC error in mailbox
	}
	if (ul_status & CAN_SR_SERR) { //stuffing error in mailbox
	}
	if (ul_status & CAN_SR_AERR) { //ack error
	}
	if (ul_status & CAN_SR_FERR) { //form error
	} 
	if (ul_status & CAN_SR_BERR) { //bit error
	}  
}

/**
* \brief Find unused RX mailbox and return its number
*/
int CANRaw::findFreeRXMailbox() {
	for (int c = 0; c < 8; c++) {
		if (mailbox_get_mode(c) == CAN_MB_RX_MODE) {
			if (mailbox_get_id(c) == 0) {
				return c;
			}
		}
	}
	return -1;
}

/**
* \brief Set up an RX mailbox (first free) for the given parameters.
*
* \param id - the post mask ID to match against
* \param mask - the mask to use for this filter
* \param extended - whether to use 29 bit filter
*
* \ret number of mailbox we just used (or -1 if there are no free boxes to use)
*/
int CANRaw::setRXFilter(uint32_t id, uint32_t mask, bool extended) {
	int c = findFreeRXMailbox();
	if (c < 0) return -1;

	mailbox_set_accept_mask(c, mask, extended);
	mailbox_set_id(c, id, extended);
	enable_interrupt(getMailboxIer(c));

	return c;
}

/**
* \brief Set up an RX mailbox (given MB number) filter
*

* \param mailbox Which mailbox to use (0-7)
* \param id The ID to match against
* \param mask The mask to apply before ID matching
* \param extended Whether this should be extended mask or not
*
* \retval Mailbox number if successful or -1 on failure
*/
int CANRaw::setRXFilter(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended) {
	if (mailbox > 7) return -1;

	mailbox_set_accept_mask(mailbox, mask, extended);
	mailbox_set_id(mailbox, id, extended);
	enable_interrupt(getMailboxIer(mailbox));
	return mailbox;
}

//set up to allow everything through. Sets up two mailboxes and returns the first one
//Not a terribly good idea to call because it totally ignores most everything and just
//quickly sets things up for all in. But, it's perfect for the novice getting started.
int CANRaw::watchFor() 
{
	int retVal = setRXFilter(0, 0, 0, false);
	enable_interrupt(getMailboxIer(0));
	setRXFilter(1, 0, 0, true);
	enable_interrupt(getMailboxIer(1));

	return retVal;
}

//Let a single frame ID through. Automatic determination of extended. Also automatically sets mask
int CANRaw::watchFor(uint32_t id)
{
	if (id > 0x7FF) return setRXFilter(id, 0x1FFFFFFF, true);
	else return setRXFilter(id, 0x7FF, false);
}

//Allow a range of IDs through based on mask. Still auto determines extended.
int CANRaw::watchFor(uint32_t id, uint32_t mask)
{
	if (id > 0x7FF) return setRXFilter(id, mask, true);
	else return setRXFilter(id, mask, false);
}

//A bit more complicated. Makes sure that the range from id1 to id2 is let through. This might open
//the floodgates if you aren't careful.
//There are undoubtedly better ways to calculate the proper values for the filter but this way seems to work.
//It'll be kind of slow if you try to let a huge span through though.
int CANRaw::watchForRange(uint32_t id1, uint32_t id2)
{
	int id = 0;
	int mask = 0;
	int temp;

	if (id1 > id2) 
	{   //looks funny I know. In place swap with no temporary storage. Neato!
		id1 = id1 ^ id2;
		id2 = id1 ^ id2; //note difference here.
		id1 = id1 ^ id2;
	}

	id = id1;

	if (id2 <= 0x7FF) mask = 0x7FF;
	else mask = 0x1FFFFFFF;

	/* Here is a quick overview of the theory behind these calculations.
	   We start with mask set to 11 or 29 set bits (all 1's)
	   and id set to the lowest ID in the range.
	   From there we go through every single ID possible in the range. For each ID
	   we AND with the current ID. At the end only bits that never changed and were 1's
	   will still be 1's. This yields the ID we can match against to let these frames through
	   The mask is calculated by finding the bitfield difference between the lowest ID and
	   the current ID. This calculation will be 1 anywhere the bits were different. We invert
	   this so that it is 1 anywhere the bits where the same. Then we AND with the current Mask.
	   At the end the mask will be 1 anywhere the bits never changed. This is the perfect mask.
	*/
	for (int c = id1; c <= id2; c++)
	{
		id &= c;
		temp = (~(id1 ^ c)) & 0x1FFFFFFF;
		mask &= temp;
	}
	//output of the above crazy loop is actually the end result.
	if (id > 0x7FF) return setRXFilter(id, mask, true);
	else return setRXFilter(id, mask, false);
}


/*
 * Get the IER (interrupt mask) for the specified mailbox index.
 *
 * \param mailbox - the index of the mailbox to get the IER for
 * \retval the IER of the specified mailbox
 */
uint32_t CANRaw::getMailboxIer(int8_t mailbox) {
	switch (mailbox) {
	case 0:
		return CAN_IER_MB0;
	case 1:
		return CAN_IER_MB1;
	case 2:
		return CAN_IER_MB2;
	case 3:
		return CAN_IER_MB3;
	case 4:
		return CAN_IER_MB4;
	case 5:
		return CAN_IER_MB5;
	case 6:
		return CAN_IER_MB6;
	case 7:
		return CAN_IER_MB7;
	}
	return 0;
}

/**
* \brief Handle a mailbox interrupt event
* \param mb which mailbox generated this event
* \param ul_status The status register of the canbus hardware
* 
*/
void CANRaw::mailbox_int_handler(uint8_t mb, uint32_t ul_status) {
	CAN_FRAME tempFrame;
	boolean caughtFrame = false;
	CANListener *thisListener;
	if (mb > 7) mb = 7;
	if (m_pCan->CAN_MB[mb].CAN_MSR & CAN_MSR_MRDY) { //mailbox signals it is ready
		switch(((m_pCan->CAN_MB[mb].CAN_MMR >> 24) & 7)) { //what sort of mailbox is it?
		case 1: //receive
		case 2: //receive w/ overwrite
		case 4: //consumer - technically still a receive buffer
			mailbox_read(mb, &tempFrame);
			//First, try to send a callback. If no callback registered then buffer the frame.
			if (cbCANFrame[mb]) 
			{
				caughtFrame = true;
				(*cbCANFrame[mb])(&tempFrame);
			}
			else if (cbCANFrame[8]) 
			{
				caughtFrame = true;
				(*cbCANFrame[8])(&tempFrame);
			}
			else
			{
				for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++)
				{
					thisListener = listener[listenerPos];
					if (thisListener != NULL)
					{
						if (thisListener->callbacksActive & (1 << mb)) 
						{
							caughtFrame = true;
							thisListener->gotFrame(&tempFrame, mb);
						}
						else if (thisListener->callbacksActive & 256) 
						{
							caughtFrame = true;
							thisListener->gotFrame(&tempFrame, -1);
						}
					}
				}
			}
			if (!caughtFrame) //if none of the callback types caught this frame then queue it in the buffer
			{
				uint8_t temp = (rx_buffer_head + 1) % SIZE_RX_BUFFER;
				if (temp != rx_buffer_tail) 
				{
					memcpy((void *)&rx_frame_buff[rx_buffer_head], &tempFrame, sizeof(CAN_FRAME));
					rx_buffer_head = temp;
				}
			}
			break;
		case 3: //transmit
			if (tx_buffer_head != tx_buffer_tail) 
			{ //if there is a frame in the queue to send
				mailbox_set_id(mb, tx_frame_buff[tx_buffer_head].id, tx_frame_buff[tx_buffer_head].extended);
				mailbox_set_datalen(mb, tx_frame_buff[tx_buffer_head].length);
				mailbox_set_priority(mb, tx_frame_buff[tx_buffer_head].priority);
				for (uint8_t cnt = 0; cnt < 8; cnt++)
					mailbox_set_databyte(mb, cnt, tx_frame_buff[tx_buffer_head].data.bytes[cnt]);
				global_send_transfer_cmd((0x1u << mb));
				tx_buffer_head = (tx_buffer_head + 1) % SIZE_TX_BUFFER;
			}
			else {
				disable_interrupt(0x01 << mb);
			}
			break;
		case 5: //producer - technically still a transmit buffer
			break;
		}
	}
}

CANListener::CANListener()
{
	callbacksActive = 0; //none. Bitfield were bits 0-7 are the mailboxes and bit 8 is the general callback
}

//an empty version so that the linker doesn't complain that no implementation exists.
void CANListener::gotFrame(CAN_FRAME *frame, int mailbox)
{
  
}

void CANListener::attachMBHandler(uint8_t mailBox)
{
	if (mailBox >= 0 && mailBox < 8)
	{
		callbacksActive |= (1<<mailBox);
	}
}

void CANListener::detachMBHandler(uint8_t mailBox)
{
	if (mailBox >= 0 && mailBox < 8)
	{
		callbacksActive &= ~(1<<mailBox);
	}  
}

void CANListener::attachGeneralHandler()
{
	callbacksActive |= 256;
}

void CANListener::detachGeneralHandler()
{
	callbacksActive &= ~256;
}


/**
 * \brief Interrupt dispatchers - Never directly call these
 *
 * \note These two functions needed because interrupt handlers cannot be part of a class
 */
void CAN0_Handler(void)
{
	Can0.interruptHandler();
}
void CAN1_Handler(void)
{
	Can1.interruptHandler();
}

/// instantiate the two canbus adapters
CANRaw Can0(CAN0, CAN0_EN);
CANRaw Can1(CAN1, CAN1_EN);

