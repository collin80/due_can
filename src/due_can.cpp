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

//Set the debugging interface for autobaud. You can switch serial ports or cause the calls to be removed entirely
//#define AUTOBAUD_DEBUG(x)  SerialUSB.print(x);
#define AUTOBAUD_DEBUG(x)

/**
* \brief constructor for the class
*
* \param pCan Which canbus hardware to use (CAN0 or CAN1)
* \param Rs pin to use for transceiver Rs control
* \param En pin to use for transceiver enable
*/
CANRaw::CANRaw(Can* pCan, uint32_t En ) : CAN_COMMON(8){
	m_pCan = pCan;
  nIRQ=(m_pCan == CAN0 ? CAN0_IRQn : CAN1_IRQn);
	enablePin = En;
	bigEndian = false;
	busSpeed = 0;
	
	for (int i = 0; i < SIZE_LISTENERS; i++) listener[i] = NULL;
  
  rx_frame_buff=0;
  tx_frame_buff=0;
  numTXBoxes=1;
  
  sizeRxBuffer=SIZE_RX_BUFFER;
  sizeTxBuffer=SIZE_TX_BUFFER;

  disable_autobaud_listen_mode();
  m_pCan->CAN_MR &= ~CAN_MR_CANEN;
  
  // Initialize all message box spesific ring buffers to 0.
  for (uint8_t i=0; i<getNumMailBoxes(); i++) {
    txRings[i]=0;
	filterSet[i] = 0;
  }
}

/**
 * \brief Configure CAN baudrate.
 *
 * \param ul_baudrate Baudrate value in bits per second
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

	uint32_t ul_status = m_pCan->CAN_SR; //read the status register just to be sure it gets cleared out

	/* Calculate the baudrate prescale value. */
	uc_prescale = ul_mck / (ul_baudrate * uc_tq);

	/* Get the right CAN BIT Timing group. */
	p_bit_time = (can_bit_timing_t *)&can_bit_time[uc_tq - CAN_MIN_TQ_NUM];

	/* Before modifying the CANBR register, disable the CAN controller. */
	//can_disable(m_pCan);
	uint32_t oldCANMR = m_pCan->CAN_MR;
    m_pCan->CAN_MR &= ~CAN_MR_CANEN;

	/* Write into the CAN baudrate register. */
	m_pCan->CAN_BR = CAN_BR_PHASE2(p_bit_time->uc_phase2 - 1) |
					CAN_BR_PHASE1(p_bit_time->uc_phase1 - 1) |
					CAN_BR_PROPAG(p_bit_time->uc_prog - 1) |
					CAN_BR_SJW(p_bit_time->uc_sjw - 1) |
					CAN_BR_BRP(uc_prescale - 1);

	m_pCan->CAN_MR = oldCANMR; //restore mode register which might re-enable CAN

    ul_status = m_pCan->CAN_SR; //read the status register just to be sure it gets cleared out
	(void)ul_status;  // avoid compiler warnings	

	numBusErrors = 0;
    numRxFrames = 0;
	busSpeed = ul_baudrate;	

	return 1;
}

uint32_t CANRaw::beginAutoSpeed()
{
	//set a list of speeds to check here. Terminate that list with 0 or you'll have a bad time.
	uint32_t speeds[] = {250000ul, 500000ul, 1000000ul, 125000ul, 33333ul, 50000ul, 800000ul, 0};
	int speedCounter = 0;
	uint32_t ret;

	enable_autobaud_listen_mode(); //go into listen only mode so we don't clobber the bus with a wrong speed setting
	AUTOBAUD_DEBUG("\n");

	while (speeds[speedCounter] != 0)
	{
		AUTOBAUD_DEBUG("\nTrying CAN rate: ");
		AUTOBAUD_DEBUG(speeds[speedCounter]);
		ret = init(speeds[speedCounter]);
		if (ret == 0)
		{
			AUTOBAUD_DEBUG("\nCould not init bus at requested speed!\n");
		}
		for (int filter = 0; filter < 3; filter++) {
			setRXFilter(filter, 0, 0, true);
		}
		//standard
		for (int filter = 3; filter < 7; filter++) {
			setRXFilter(filter, 0, 0, false);
		}
        for (int waiting = 0; waiting < 100; waiting++)
		{
			CAN_FRAME thisFrame;
			if (rx_avail() > 0) {
				read(thisFrame);
				break;
			}
			delay(6);
		}
		if (numRxFrames > 0) 
		{
			AUTOBAUD_DEBUG(" SUCCESS!\n\n");
			disable_autobaud_listen_mode(); //the default is to not be in listen only
			reset_all_mailbox(); //return mailboxes to default state which is to let nothing through yet
			init(ret);
			return ret; //return the speed that succeeded
		}
		else 
		{ 
			AUTOBAUD_DEBUG(" FAILURE!\n"); 
		}
		speedCounter++;
	}
	AUTOBAUD_DEBUG("\nNo speeds worked! Are you sure you're connected to a CAN bus?!\n");
	disable();
	return 0; 
}

/*
 * \brief 
 *
 * \param 
 *
 * \retval None.
 *
 */
void CANRaw::setMailBoxTxBufferSize(uint8_t mbox, uint16_t size) 
{
  if ( mbox>=getNumMailBoxes() || txRings[mbox]!=0 ) return;
    
  volatile CAN_FRAME *buf=new CAN_FRAME[size];
  txRings[mbox]=new ringbuffer_t;
  initRingBuffer (*(txRings[mbox]), buf, size);
}

/*
 * \brief Initializes dynamically sized buffers.
 *
 * \param mask - default filter mask
 *
 * \retval None.
 *
 */

void CANRaw::initializeBuffers() 
{
    if (isInitialized())  return ;
  
  Serial.println("Initialize buffers");
	// set up the transmit and receive ring buffers
	if (tx_frame_buff==0) tx_frame_buff=new CAN_FRAME[sizeTxBuffer];
	if (rx_frame_buff==0) rx_frame_buff=new CAN_FRAME[sizeRxBuffer];
	
	initRingBuffer (txRing, tx_frame_buff, sizeTxBuffer);
	initRingBuffer (rxRing, rx_frame_buff, sizeRxBuffer);
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
  
    initializeBuffers();
    m_pCan->CAN_MR &= ~CAN_MR_CANEN; //immediately disable the CAN hardware if it had previously been enabled

	uint32_t ul_status = m_pCan->CAN_SR; //read the status register just to be sure it gets cleared out
	(void)ul_status;  // avoid compiler warnings
	
	numBusErrors = 0;
    numRxFrames = 0;

	//initialize all function pointers to null
	for (int i = 0; i < getNumMailBoxes()+1; i++) cbCANFrame[i] = 0;

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
	setNumTXBoxes(numTXBoxes);

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
	NVIC_SetPriority(nIRQ, 12); 
	
	NVIC_EnableIRQ(nIRQ); //tell the nested interrupt controller to turn on our interrupt

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
 * \retval number of tx boxes set.
 *
 */
int CANRaw::setNumTXBoxes(int txboxes) 
{
	int c;

	if (txboxes > getNumMailBoxes())  txboxes = getNumMailBoxes() ;
	if (txboxes < 0)                  txboxes = 0 ;
	numTXBoxes = txboxes;

	//Inialize RX boxen
	for (c = 0; c < getNumRxBoxes(); c++) {
		mailbox_set_mode(c, CAN_MB_RX_MODE);
		mailbox_set_id(c, 0x0, false);
		mailbox_set_accept_mask(c, 0x7FF, false);
	}

	//Initialize TX boxen
	for (c = getFirstTxBox(); c < getNumMailBoxes(); c++) {
		mailbox_set_mode(c, CAN_MB_TX_MODE);
		mailbox_set_priority(c, 10);
		mailbox_set_accept_mask(c, 0x7FF, false);
	}
	
	return (numTXBoxes);
}

/*
 * \brief Initialize the specified ring buffer.
 *
 * \param ring - ring buffer to initialize.
 * \param buffer - buffer to use for storage.
 * \param size - size of the buffer in bytes.
 *
 * \retval None.
 *
 */

void CANRaw::initRingBuffer (ringbuffer_t &ring, volatile CAN_FRAME *buffer, uint16_t size)
{
    ring.buffer = buffer;
    ring.size   = size;
    ring.head   = 0;
    ring.tail   = 0;
}

/*
 * \brief Add a CAN message to the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 * \param msg - message structure to add.
 *
 * \retval true if added, false if the ring is full.
 *
 */

bool CANRaw::addToRingBuffer (ringbuffer_t &ring, const CAN_FRAME &msg)
{
    uint16_t nextEntry;

    nextEntry = (ring.head + 1) % ring.size;

    /* check if the ring buffer is full */
    if (nextEntry == ring.tail)  return false ;

    /* add the element to the ring */
    memcpy ((void *)&ring.buffer[ring.head], (void *)&msg, sizeof (CAN_FRAME));

    /* bump the head to point to the next free entry */
    ring.head = nextEntry;

    return (true);
}

/*
 * \brief Remove a CAN message from the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 * \param msg - message structure to fill in.
 *
 * \retval true if a message was removed, false if the ring is empty.
 *
 */

bool CANRaw::removeFromRingBuffer (ringbuffer_t &ring, CAN_FRAME &msg)
{

    /* check if the ring buffer has data available */
    if (isRingBufferEmpty (ring) == true)  return false ;

    /* copy the message */
    memcpy ((void *)&msg, (void *)&ring.buffer[ring.tail], sizeof (CAN_FRAME));

    /* bump the tail pointer */
    ring.tail = (ring.tail + 1) % ring.size;

    return (true);
}

/*
 * \brief Count the number of entries in the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 *
 * \retval a count of the number of elements in the ring buffer.
 *
 */

uint16_t CANRaw::ringBufferCount (ringbuffer_t &ring)
{
    if (ring.tail == ring.head)  return 0 ;
    if (ring.tail < ring.head)   return ring.head - ring.tail ;
	else                         return ring.size - ring.tail + ring.head;
}

void CANRaw::setListenOnlyMode(bool state) 
{
    if (state) enable_autobaud_listen_mode() ; 
	else       disable_autobaud_listen_mode() ;
}

/**
 * \brief Enable CAN Controller.
 *
 */
void CANRaw::enable()
{
	m_pCan->CAN_MR |= CAN_MR_CANEN;
	
	if (enablePin != 255)  digitalWrite(enablePin, HIGH) ;
}

/**
 * \brief Disable CAN Controller.
 *
 */
void CANRaw::disable()
{
	m_pCan->CAN_MR &= ~CAN_MR_CANEN;
	
	if (enablePin != 255)  digitalWrite(enablePin, LOW) ;
}

/**
 * \brief Set a CAN controller mode bit (disabling/reenabling if necessary)
 *
 */
void CANRaw::setModeBit(uint32_t bit)
{
	uint32_t savedMR = m_pCan->CAN_MR;
	
	m_pCan->CAN_MR &= ~CAN_MR_CANEN;
	m_pCan->CAN_MR |= bit;
	
	if (savedMR & CAN_MR_CANEN) {
		savedMR |= bit;
		m_pCan->CAN_MR = savedMR;
	}
}

/**
 * \brief Unset a CAN controller mode bit (disabling/reenabling if necessary)
 *
 */
void CANRaw::unsetModeBit(uint32_t bit)
{
	uint32_t savedMR = m_pCan->CAN_MR;
	
	m_pCan->CAN_MR &= ~CAN_MR_CANEN;
	m_pCan->CAN_MR &= ~bit;
	
	if (savedMR & CAN_MR_CANEN) {
		savedMR &= ~bit;
		m_pCan->CAN_MR = savedMR;
	}
}

/**
 * \brief Disable CAN Controller low power mode.
 *
 */
void CANRaw::disable_low_power_mode()
{
	unsetModeBit(CAN_MR_LPM);
}

/**
 * \brief Enable CAN Controller low power mode.
 *
 */
void CANRaw::enable_low_power_mode()
{
	setModeBit(CAN_MR_LPM);
}

/**
 * \brief Disable CAN Controller autobaud/listen mode.
 *
 */
void CANRaw::disable_autobaud_listen_mode()
{
	unsetModeBit(CAN_MR_ABM);
}

/**
 * \brief Enable CAN Controller autobaud/listen mode.
 *
 */
void CANRaw::enable_autobaud_listen_mode()
{
	setModeBit(CAN_MR_ABM);
}

/**
 * \brief CAN Controller won't generate overload frame.
 *
 */
void CANRaw::disable_overload_frame()
{
	unsetModeBit(CAN_MR_OVL);
}

/**
 * \brief CAN Controller will generate an overload frame after each successful
 * reception for mailboxes configured in Receive mode, Producer and Consumer.
 *
 */
void CANRaw::enable_overload_frame()
{
	setModeBit(CAN_MR_OVL);
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
	if (ul_flag)  setModeBit(CAN_MR_TEOF) ;
	else          unsetModeBit(CAN_MR_TEOF) ;
}

/**
 * \brief Disable CAN Controller time triggered mode.
 *
 */
void CANRaw::disable_time_triggered_mode()
{
	unsetModeBit(CAN_MR_TTM);
}

/**
 * \brief Enable CAN Controller time triggered mode.
 *
 */
void CANRaw::enable_time_triggered_mode()
{
	setModeBit(CAN_MR_TTM);
}

/**
 * \brief Disable CAN Controller timer freeze.
 *
 */
void CANRaw::disable_timer_freeze()
{
	unsetModeBit(CAN_MR_TIMFRZ);
}

/**
 * \brief Enable CAN Controller timer freeze.
 *
 */
void CANRaw::enable_timer_freeze()
{
	setModeBit(CAN_MR_TIMFRZ);
}

/**
 * \brief Disable CAN Controller transmit repeat function.
 *
 */
void CANRaw::disable_tx_repeat()
{
	setModeBit(CAN_MR_DRPT);
}

/**
 * \brief Enable CAN Controller transmit repeat function.
 *
 */
void CANRaw::enable_tx_repeat()
{
	unsetModeBit(CAN_MR_DRPT);
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
	return m_pCan->CAN_IMR;
}

/**
 * \brief Get CAN status.
 *
 *
 * \retval CAN status.
 */
uint32_t CANRaw::get_status()
{
	return m_pCan->CAN_SR;
}

/**
 * \brief Get the 16-bit free-running internal timer count.
 *
 *
 * \retval The internal CAN free-running timer counter.
 */
uint16_t CANRaw::get_internal_timer_value()
{
	return m_pCan->CAN_TIM;
}

/**
 * \brief Get CAN timestamp register value.
 *
 *
 * \retval The timestamp value.
 */
uint16_t CANRaw::get_timestamp_value()
{
	return m_pCan->CAN_TIMESTP;
}

/**
 * \brief Get CAN transmit error counter.
 *
 *
 * \retval Transmit error counter.
 */
uint8_t CANRaw::get_tx_error_cnt()
{
	return (uint8_t)(m_pCan->CAN_ECR >> CAN_ECR_TEC_Pos);
}

/**
 * \brief Get CAN receive error counter.
 *
 *
 * \retval Receive error counter.
 */
uint8_t CANRaw::get_rx_error_cnt()
{
	return (uint8_t)(m_pCan->CAN_ECR >> CAN_ECR_REC_Pos);
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
	
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
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
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
	m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MTCR;
}

/**
 * \brief Send single mailbox abort request.
 *
 * \param uc_index Indicate which mailbox is to be configured.
 */
void CANRaw::mailbox_send_abort_cmd(uint8_t uc_index)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
	m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MACR;
}

/**
 * \brief Initialize the mailbox to a default, known state.
 *
 * \param p_mailbox Pointer to a CAN mailbox instance.
 */
void CANRaw::mailbox_init(uint8_t uc_index)
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
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
	for (uint8_t  i = 0;  i < CANMB_NUMBER;  i++) 
	{
		mailbox_init(i);
		filterSet[i] = 0;
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
	CAN_FRAME  tempFrame;
	uint8_t*   buff     = (uint8_t *)inputValue;
	int        thisSize = sizeof(t) > 8 ? 8 : sizeof(t) ;
	
	//if (thisSize > 8) thisSize = 8;
	
	if (!bigEndian) {
		for (int i = 0; i < thisSize; i++) 
			tempFrame.data.bytes[i] = buff[i];
		
	} else {  //reverse byte order. The M3 is in little endian so this causes big endian order
		for (int i = 0; i < thisSize; i++) 
			tempFrame.data.bytes[i] = buff[thisSize - i - 1];
	}

	tempFrame.id = this->write_id;
	tempFrame.length = thisSize;
	if (this->write_id > 0x7FF)  tempFrame.extended = true ;
	else                         tempFrame.extended = false ;
	sendFrame(tempFrame);
}

void CANRaw::writeTxRegisters(const CAN_FRAME &txFrame, uint8_t mb)
{
	mailbox_set_id      (mb, txFrame.id, txFrame.extended);
	mailbox_set_datalen (mb, txFrame.length);
	mailbox_set_rtr(mb, txFrame.rtr);
	mailbox_set_priority(mb, txFrame.priority);
	
	for (uint8_t cnt = 0; cnt < 8; cnt++)
		mailbox_set_databyte(mb, cnt, txFrame.data.bytes[cnt]);
	
	global_send_transfer_cmd((0x1u << mb));
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
   bool result=false;

   irqLock();
   {
	   if ( isRingBufferEmpty(txRing) ) { // If there is nothing buffered, find free mailbox
	      for (uint8_t mbox = 0; mbox < 8; mbox++) {
	         if (((m_pCan->CAN_MB[mbox].CAN_MMR >> 24) & 7) == CAN_MB_TX_MODE)
	         {//is this mailbox set up as a TX box?
	            if ( usesGlobalTxRing(mbox) && (m_pCan->CAN_MB[mbox].CAN_MSR & CAN_MSR_MRDY) ) {
	               //is it also available (not sending anything?)
	               writeTxRegisters(txFrame,mbox);
	               enable_interrupt(0x01u << mbox); //enable the TX interrupt for this box
	               result = true; //we've sent it. mission accomplished.
	               break; //no need to keep going. We sent our message
	            }
	         }
	      }
	   }
	  
	   if ( !result) {
	      //no free mailbox was found above
	      //so, queue the frame if possible. But, don't increment the 
	      //tail if it would smash into the head and kill the queue.
	      result = addToRingBuffer(txRing, txFrame);
	   }
   }
   irqRelease();

   return result;
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
bool CANRaw::sendFrame(CAN_FRAME& txFrame, uint8_t mbox) 
{
	bool  result = false;
	
	if (!isTxBox(mbox))  return result ;
	
	irqLock();
	{
		if ( (!txRings[mbox] || isRingBufferEmpty(*txRings[mbox]))
		     && (m_pCan->CAN_MB[mbox].CAN_MSR & CAN_MSR_MRDY) ) {
			writeTxRegisters(txFrame,mbox);
			enable_interrupt(0x01u << mbox); //enable the TX interrupt for this box
			result = true; //we've sent it. mission accomplished.
		}
		
		if (!result && txRings[mbox] && addToRingBuffer(*txRings[mbox], txFrame))
			result=true;
	}
	irqRelease();
    
	return result;
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

	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;

	ul_retval = 0;
	ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;

	/* Check whether there is overwriting happening in Receive with Overwrite mode,
	   or there're messages lost in Receive mode. */
	if ((ul_status & CAN_MSR_MRDY) && (ul_status & CAN_MSR_MMI))
		ul_retval = CAN_MAILBOX_RX_OVER;

	ul_id = m_pCan->CAN_MB[uc_index].CAN_MID;
	if ((ul_id & CAN_MID_MIDE) == CAN_MID_MIDE) { //extended id
		rxframe->id = ul_id & 0x1FFFFFFFu;
		rxframe->extended = true;
		
	} else { //standard ID
		rxframe->id = (ul_id  >> CAN_MID_MIDvA_Pos) & 0x7ffu;
		rxframe->extended = false;
	}
	
	rxframe->fid    = m_pCan->CAN_MB[uc_index].CAN_MFID;
	rxframe->length = (ul_status & CAN_MSR_MDLC_Msk) >> CAN_MSR_MDLC_Pos;
    rxframe->timestamp = (ul_status & CAN_MSR_MTIMESTAMP_Msk);
	rxframe->rtr    = (m_pCan->CAN_MB[uc_index].CAN_MSR & CAN_MSR_MRTR) ? 1 : 0 ;
    ul_datal        = m_pCan->CAN_MB[uc_index].CAN_MDL;
	ul_datah        = m_pCan->CAN_MB[uc_index].CAN_MDH;

	rxframe->data.high = ul_datah;
	rxframe->data.low = ul_datal;

	/* Read the mailbox status again to check whether the software needs to re-read mailbox data register. */
	ul_status = m_pCan->CAN_MB[uc_index].CAN_MSR;	
	if (ul_status & CAN_MSR_MMI)  ul_retval |= CAN_MAILBOX_RX_NEED_RD_AGAIN ;
	else                          ul_retval |= CAN_MAILBOX_TRANSFER_OK ;

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
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
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
uint32_t CANRaw::mailbox_get_id(uint8_t uc_index) 
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
	if (m_pCan->CAN_MB[uc_index].CAN_MID & CAN_MID_MIDE)
		return m_pCan->CAN_MB[uc_index].CAN_MID;
	else
		return (m_pCan->CAN_MB[uc_index].CAN_MID >> CAN_MID_MIDvA_Pos) & 0x7ffu;
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
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
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
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
	if (ext) {
		m_pCan->CAN_MB[uc_index].CAN_MAM  = mask | CAN_MAM_MIDE;
		m_pCan->CAN_MB[uc_index].CAN_MID |= CAN_MAM_MIDE;
		
	} else {
		m_pCan->CAN_MB[uc_index].CAN_MAM  = CAN_MAM_MIDvA(mask);
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
void CANRaw::mailbox_set_mode(uint8_t uc_index, uint8_t mode) 
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
	if (mode > 5)  mode = 0 ;  //set disabled on invalid mode
	
	m_pCan->CAN_MB[uc_index].CAN_MMR = 
		(m_pCan->CAN_MB[uc_index].CAN_MMR & ~CAN_MMR_MOT_Msk) | (mode << CAN_MMR_MOT_Pos);
}

/**
 * \brief Get current mode of given mailbox
 *
 * \param uc_index Which mailbox to retrieve mode from (0-7)
 *
 * \retval Mode of mailbox
 *
 */
uint8_t CANRaw::mailbox_get_mode(uint8_t uc_index) 
{
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
	return (uint8_t)((m_pCan->CAN_MB[uc_index].CAN_MMR >> CAN_MMR_MOT_Pos) & 0x07);
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
	uint8_t  shift;    // how many bits to shift
	uint32_t working;  // working copy of the relevant data int
	
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	
	if (bytepos > 7)  bytepos = 7 ;
	shift = 8 * (bytepos & 0x03); //how many bits to shift up into position
	
	if (bytepos < 4) { //low data block
		working  = m_pCan->CAN_MB[uc_index].CAN_MDL & ~(0xFF << shift); //mask out where we have to be
		working |= (val << shift);
		m_pCan->CAN_MB[uc_index].CAN_MDL = working;
		
	} else { //high data block
		working  = m_pCan->CAN_MB[uc_index].CAN_MDH & ~(0xFF << shift); //mask out where we have to be
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
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
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
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	m_pCan->CAN_MB[uc_index].CAN_MDH = val;
}

/**
 * \brief Set RTR for given mailbox
 *
 * \param mbox Which mailbox? (0-7)
 * \param rtr value of RTR {0,1}
 *
 */
void  CANRaw::mailbox_set_rtr (uint8_t mbox,  uint8_t rtr)
{
	if (mbox >= CANMB_NUMBER)  mbox = CANMB_NUMBER - 1;

	if (rtr)  m_pCan->CAN_MB[mbox].CAN_MSR |=  CAN_MSR_MRTR;
	else      m_pCan->CAN_MB[mbox].CAN_MSR &= ~CAN_MSR_MRTR;
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
	if (uc_index >= CANMB_NUMBER)  uc_index = CANMB_NUMBER - 1 ;
	if (dlen > 8)  dlen = 8 ;
	m_pCan->CAN_MB[uc_index].CAN_MCR = 
		(m_pCan->CAN_MB[uc_index].CAN_MCR & ~CAN_MCR_MDLC_Msk) | CAN_MCR_MDLC(dlen);
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
	/* Read the mailbox status firstly to check whether the mailbox is ready or not. */
	if (!(m_pCan->CAN_MB[uc_index].CAN_MSR & CAN_MSR_MRDY))  return CAN_MAILBOX_NOT_READY ;
	
	/* Set the MBx bit in the Transfer Command Register to send out the remote frame. */
	global_send_transfer_cmd(1 << uc_index);

	return CAN_MAILBOX_TRANSFER_OK;
}

uint16_t CANRaw::available()
{
	uint16_t val;
  
	irqLock();
	val = ringBufferCount(rxRing);
	irqRelease();
	
	return val;
}


/**
* \brief Check whether there are received canbus frames in the buffer
*
* \retval true if there are frames waiting in buffer, otherwise false
*/
bool CANRaw::rx_avail() 
{
	bool result;
	
	irqLock();
	result = !isRingBufferEmpty(rxRing);
	irqRelease();

	return result;
}

/**
 * \brief Retrieve a frame from the RX buffer
 *
 * \param buffer Reference to the frame structure to fill out
 *
 * \retval 0 no frames waiting to be received, 1 if a frame was returned
 */
uint32_t CANRaw::get_rx_buff(CAN_FRAME& msg) 
{
	uint32_t result;
	
	irqLock();
	{
		result = removeFromRingBuffer(rxRing,msg) ? 1 : 0;
	}
	irqRelease();

	return result;
}

/**
* \brief Handle all interrupt reasons
*/
void CANRaw::interruptHandler() 
{
	uint32_t  ul_status = m_pCan->CAN_SR ;  // Get interrupt Status Register
	
	if (ul_status & CAN_SR_MB0)     mailbox_int_handler(0, ul_status) ;
	if (ul_status & CAN_SR_MB1)     mailbox_int_handler(1, ul_status) ;
	if (ul_status & CAN_SR_MB2)     mailbox_int_handler(2, ul_status) ;
	if (ul_status & CAN_SR_MB3)     mailbox_int_handler(3, ul_status) ;
	if (ul_status & CAN_SR_MB4)     mailbox_int_handler(4, ul_status) ;
	if (ul_status & CAN_SR_MB5)     mailbox_int_handler(5, ul_status) ;
	if (ul_status & CAN_SR_MB6)     mailbox_int_handler(6, ul_status) ;
	if (ul_status & CAN_SR_MB7)     mailbox_int_handler(7, ul_status) ;
	
	if (ul_status & CAN_SR_ERRA)    numBusErrors++ ;  // error active
	if (ul_status & CAN_SR_WARN)    {} ;              // warning limit
	if (ul_status & CAN_SR_ERRP)    {} ;              // error passive
	if (ul_status & CAN_SR_BOFF)    numBusErrors++ ;  // bus off
	if (ul_status & CAN_SR_SLEEP)   {} ;              // controller in sleep mode
	if (ul_status & CAN_SR_WAKEUP)  {} ;              // controller woke up
	if (ul_status & CAN_SR_TOVF)    {} ;              // timer overflow
	if (ul_status & CAN_SR_TSTP)    {} ;              // timestamp - start or end of frame
	if (ul_status & CAN_SR_CERR)    {} ;              // CRC error in mailbox
	if (ul_status & CAN_SR_SERR)    numBusErrors++ ;  // stuffing error in mailbox
	if (ul_status & CAN_SR_AERR)    numBusErrors++ ;  // ack error
	if (ul_status & CAN_SR_FERR)    numBusErrors++ ;  // form error
	if (ul_status & CAN_SR_BERR)    numBusErrors++ ;  // bit error
}

/**
* \brief Find unused RX mailbox and return its number
*/
int  CANRaw::findFreeRXMailbox () 
{
	for (int  c = 0;  c < getNumMailBoxes();  c++)
		if (filterSet[c] == 0) return c ;
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
int CANRaw::_setFilter(uint32_t id, uint32_t mask, bool extended) 
{
	int  c = findFreeRXMailbox();
	
	if (c < 0)  return -1 ;

	mailbox_set_accept_mask(c, mask, extended);
	mailbox_set_id(c, id, extended);
	enable_interrupt(getMailboxIer(c));
	filterSet[c] = 1;

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
int CANRaw::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended) 
{
	if (mailbox >= getNumMailBoxes())  return -1 ;

	mailbox_set_accept_mask(mailbox, mask, extended);
	mailbox_set_id(mailbox, id, extended);
	enable_interrupt(getMailboxIer(mailbox));
	filterSet[mailbox] = 1;
	return mailbox;
}

/*
 * Get the IER (interrupt mask) for the specified mailbox index.
 *
 * \param mailbox - the index of the mailbox to get the IER for
 * \retval the IER of the specified mailbox
 */
uint32_t  CANRaw::getMailboxIer (int8_t mailbox) 
{
	switch (mailbox) {
		case 0  :  return CAN_IER_MB0;
		case 1  :  return CAN_IER_MB1;
		case 2  :  return CAN_IER_MB2;
		case 3  :  return CAN_IER_MB3;
		case 4  :  return CAN_IER_MB4;
		case 5  :  return CAN_IER_MB5;
		case 6  :  return CAN_IER_MB6;
		case 7  :  return CAN_IER_MB7;
		default :  return 0;
	}
	return 0;
}

/**
* \brief Handle a mailbox interrupt event
* \param mb which mailbox generated this event
* \param ul_status The status register of the canbus hardware
* 
*/
void CANRaw::mailbox_int_handler(uint8_t mb,  uint32_t /*ul_status*/)
{
	CAN_FRAME      tempFrame;
	boolean        caughtFrame = false;
	CANListener*   thisListener;
	ringbuffer_t*  pRing;
  
	if (mb >= getNumMailBoxes())  mb = getNumMailBoxes() - 1 ;
	
	if (m_pCan->CAN_MB[mb].CAN_MSR & CAN_MSR_MRDY) { //mailbox signals it is ready
		switch(((m_pCan->CAN_MB[mb].CAN_MMR >> 24) & 7)) { //what sort of mailbox is it?
			case 1: //receive
			case 2: //receive w/ overwrite
			case 4: //consumer - technically still a receive buffer
				mailbox_read(mb, &tempFrame);
				numRxFrames++;
	
				//First, try to send a callback. If no callback registered then buffer the frame.
				if (cbCANFrame[mb]) {
					caughtFrame = true;
					(*cbCANFrame[mb])(&tempFrame);
					
				} else if (cbGeneral) {
					caughtFrame = true;
					(*cbGeneral)(&tempFrame);
					
				} else {
					for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++) {
						if ((thisListener = listener[listenerPos])) {
							if (thisListener->isCallbackActive(mb))  {
								caughtFrame = true;
								thisListener->gotFrame(&tempFrame, mb);
								
							} else if (thisListener->isCallbackActive(8)) {  //global catch-all 
								caughtFrame = true;
								thisListener->gotFrame(&tempFrame, -1);
							}
						}
					}
				}
				// if none of the callback types caught this frame then queue it in the buffer
				if (!caughtFrame)  addToRingBuffer(rxRing, tempFrame) ;
				break;
				
			case 3: //transmit
				pRing = usesGlobalTxRing(mb) ? &txRing : txRings[mb];
				if (removeFromRingBuffer(*pRing, tempFrame)) //if there is a frame in the queue to send
					writeTxRegisters(tempFrame,mb);
				else
					disable_interrupt(0x01 << mb);
				break;
				
			case 5: // producer - technically still a transmit buffer
			break;
		}
	}
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

