/*   ==================================================================

     >>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
     ------------------------------------------------------------------
     Copyright (c) 2019-2023 by Lattice Semiconductor Corporation
     ALL RIGHTS RESERVED
     ------------------------------------------------------------------

       IMPORTANT: THIS FILE IS USED BY OR GENERATED BY the LATTICE PROPEL™
       DEVELOPMENT SUITE, WHICH INCLUDES PROPEL BUILDER AND PROPEL SDK.

       Lattice grants permission to use this code pursuant to the
       terms of the Lattice Propel License Agreement.

     DISCLAIMER:

    LATTICE MAKES NO WARRANTIES ON THIS FILE OR ITS CONTENTS,
    WHETHER EXPRESSED, IMPLIED, STATUTORY,
    OR IN ANY PROVISION OF THE LATTICE PROPEL LICENSE AGREEMENT OR
    COMMUNICATION WITH LICENSEE,
    AND LATTICE SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTY OF
    MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
    LATTICE DOES NOT WARRANT THAT THE FUNCTIONS CONTAINED HEREIN WILL MEET
    LICENSEE 'S REQUIREMENTS, OR THAT LICENSEE' S OPERATION OF ANY DEVICE,
    SOFTWARE OR SYSTEM USING THIS FILE OR ITS CONTENTS WILL BE
    UNINTERRUPTED OR ERROR FREE,
    OR THAT DEFECTS HEREIN WILL BE CORRECTED.
    LICENSEE ASSUMES RESPONSIBILITY FOR SELECTION OF MATERIALS TO ACHIEVE
    ITS INTENDED RESULTS, AND FOR THE PROPER INSTALLATION, USE,
    AND RESULTS OBTAINED THEREFROM.
    LICENSEE ASSUMES THE ENTIRE RISK OF THE FILE AND ITS CONTENTS PROVING
    DEFECTIVE OR FAILING TO PERFORM PROPERLY AND IN SUCH EVENT,
    LICENSEE SHALL ASSUME THE ENTIRE COST AND RISK OF ANY REPAIR, SERVICE,
    CORRECTION,
    OR ANY OTHER LIABILITIES OR DAMAGES CAUSED BY OR ASSOCIATED WITH THE
    SOFTWARE.IN NO EVENT SHALL LATTICE BE LIABLE TO ANY PARTY FOR DIRECT,
    INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
    INCLUDING LOST PROFITS,
    ARISING OUT OF THE USE OF THIS FILE OR ITS CONTENTS,
    EVEN IF LATTICE HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
    LATTICE 'S SOLE LIABILITY, AND LICENSEE' S SOLE REMEDY,
    IS SET FORTH ABOVE.
    LATTICE DOES NOT WARRANT OR REPRESENT THAT THIS FILE,
    ITS CONTENTS OR USE THEREOF DOES NOT INFRINGE ON THIRD PARTIES'
    INTELLECTUAL PROPERTY RIGHTS, INCLUDING ANY PATENT. IT IS THE USER' S
    RESPONSIBILITY TO VERIFY THE USER SOFTWARE DESIGN FOR CONSISTENCY AND
    FUNCTIONALITY THROUGH THE USE OF FORMAL SOFTWARE VALIDATION METHODS.
     ------------------------------------------------------------------

     ================================================================== */
#include "uart.h"
#include "hal.h"

/*
 ***************************************************************
 * UART ISR: Automatically optimized-out (it's a "static") if
 * _UART_NO_INTERRUPTS_ is defined as part of preprocessor
 * settings.  Note this preprocessor setting affects ALL
 * UART instances.
 ***************************************************************
*/

#if _UART_ENABLE_INTERRUPTS_
void uart_isr(void *ctx)
{
	struct uart_instance *uart;
	volatile struct uart_dev *dev;
	unsigned char iir;
	unsigned char ier;

	uart = (struct uart_instance *) ctx;
	dev = (volatile struct uart_dev *) (uart->base);

	/* interrogate interrupt-cause */
	iir = dev->iir;
	ier = dev->ier;

	/* see if the interrupt is due to rx-data */
	switch (iir) {
	case UART_IIR_RXRDY:
		{
			/*
			 * read lsr to check for errors associated with current word.
			 * reading lsr clears error bits
			 */
			unsigned char lsr = dev->lsr;

			/* read the data into the buffer */
			uart->rxBuffer[uart->rxWriteLoc] = dev->rxtx;

			/* check for error(s) with the current word */
			if ((lsr & (UART_LSR_PE_MASK | UART_LSR_FE_MASK))
			    == 0) {
				/* no error detected, adjust rx buffer pointers */
				uart->rxWriteLoc++;
				if (uart->rxWriteLoc >= uart->rxBufferSize) {
					uart->rxWriteLoc = 0;
				}
				uart->rxDataBytes++;

				/* if buffer's full, disable interrupts */
				if (uart->rxDataBytes ==
				    uart->rxBufferSize) {
					uart->ier &=
					    (~UART_IER_RX_INT_MASK);
					dev->ier = uart->ier;
				}
			}
		}
		break;

	case UART_IIR_TXRDY:
		{
			unsigned int count;
			if (uart->fifoenable) {
				/* see if there's stuff to transmit */
				count =
				    (uart->txDataBytes >
				     16) ? 16 : uart->txDataBytes;
				//count = 16;
			} else {
				count = (uart->txDataBytes == 0) ? 0 : 1;
				//count = 1;
			}
			/* put 16 data words to the TX FIFO of
			 * the UART if data is available
			 */
			while (count != 0) {
				if (uart->txDataBytes == 1) {
					/* disable tx interrupts as this is the last data to send */
					uart->ier &=
					    (~UART_IER_TX_INT_MASK);
					dev->ier = uart->ier;
				}
				dev->rxtx =
				    uart->txBuffer[uart->txReadLoc];
				uart->txReadLoc++;
				if (uart->txReadLoc >= uart->txBufferSize) {
					uart->txReadLoc = 0;
				}
				--uart->txDataBytes;
				--count;
#if 1
				if (uart->txDataBytes == 0) {
					break;
				}
#endif
			}
		}
		break;

	default:
		{
			/*
			 * This will never happen unless someone's reading rx
			 * in parallel.  If using interrupts, isr is the only
			 * code that should be reading rx
			 */
		}
		break;
	}

	/* all done */
	return;
}
#endif

unsigned char uart_init(struct uart_instance *this_uart,
			unsigned int base_addr,
			unsigned int sys_clk,
			unsigned int baud_rate,
			unsigned char stop_bits, unsigned char data_width)
{
	volatile struct uart_dev *dev;
	if (NULL == this_uart) {
		return 1;
	}
	this_uart->base = base_addr;
	dev = (volatile struct uart_dev *) (this_uart->base);

	/*initialize the instance data */
	this_uart->base = base_addr;
	this_uart->sys_clk = sys_clk;
	this_uart->baudrate = baud_rate;
	this_uart->databits = data_width;
	this_uart->stopbits = stop_bits;

	/* set the data-configuration */
	//uart_config(this_uart, 8, 0, 0, 1);

	this_uart->blockingTx = 1;
#if _UART_ENABLE_INTERRUPTS_
	this_uart->intrAvail = true;

	/* If interrupts are available use interrupt-mode */
	if (this_uart->intrAvail) {

		/* initialize buffer-related parameters */
		this_uart->txDataBytes = 0;
		this_uart->rxDataBytes = 0;
		this_uart->txReadLoc = 0;
		this_uart->txWriteLoc = 0;
		this_uart->rxReadLoc = 0;
		this_uart->rxWriteLoc = 0;
		this_uart->rxBufferSize = UART_BUFFER_SIZE;
		this_uart->txBufferSize = UART_BUFFER_SIZE;
		/*
		 * Don't enable transmit interrupt as the write-function takes care of
		 * doing that
		 */
		this_uart->ier = UART_IER_RX_INT_MASK;
		dev->ier = this_uart->ier;

		/* Register interrupt-handler for this uart's interrupt */
		//pic_isr_register(this_uart->intrLevel, uart_isr, this_uart);	//uart_isr have two parameter!!!!!
	}
#endif

	/* all done! */
	return 0;
}

/*
 ***************************************************************
 * Retrieves a character from the UART
 * Returns 0 if no error.
 * If interrupts are enabled, go through the buffer.
 * Else, interact directly with the uart.
 ***************************************************************
*/
unsigned char uart_getc(struct uart_instance *this_uart,
			unsigned char *pucChar)
{
	volatile unsigned char uiValue;
	volatile struct uart_dev *dev;
	if (NULL == this_uart) {
		return 1;
	}
	if (pucChar == 0)
		return (UART_ERR_INVALID_ARGUMENT);

	dev = (volatile struct uart_dev *) (this_uart->base);

	do {
#if _UART_ENABLE_INTERRUPTS_
		if (this_uart->intrAvail) {

			/* using interrupts */
			if (this_uart->rxDataBytes > 0) {
				*pucChar =
				    this_uart->
				    rxBuffer[this_uart->rxReadLoc];
				this_uart->rxReadLoc++;
				if (this_uart->rxReadLoc >=
				    this_uart->rxBufferSize)
					this_uart->rxReadLoc = 0;
				pic_int_disable(this_uart->intrLevel);
				this_uart->rxDataBytes--;
				this_uart->ier |= UART_IER_RX_INT_MASK;
				dev->ier = this_uart->ier;
				pic_int_enable(this_uart->intrLevel);
				return 0;
			}
		} else
#endif
		{
			/* not using interrupts */
			uiValue = dev->lsr;
			if (uiValue & UART_LSR_RX_RDY_MASK) {
				*pucChar = dev->rxtx;
				return 0;
			}
		}

		/* if rx is non-blocking, return immediately */
		if (this_uart->blockingRx == 0)
			return (UART_ERR_WOULD_BLOCK);

	} while (1);

	/* this "return" should never happen */
	return (255);
}

/*
 ***************************************************************
 * Sends a character over the UART
 * Returns 0 if no error
 ***************************************************************
*/
unsigned char uart_putc(struct uart_instance *this_uart,
			unsigned char ucChar)
{
	volatile unsigned char uiValue;
	volatile struct uart_dev *dev;
	if (NULL == this_uart) {
		return 1;
	}
	dev = (volatile struct uart_dev *) (this_uart->base);

#if _UART_ENABLE_INTERRUPTS_
	if (this_uart->intrAvail) {
		/* if tx-buffer's full, wait for it to get empty */
		while (this_uart->txDataBytes == this_uart->txBufferSize) {

			/* if non-blocking tx, return immediately */
			if (this_uart->blockingTx == 0)
				return (UART_ERR_WOULD_BLOCK);
		}

		/* stuff data into the write-buffer */
		this_uart->txBuffer[this_uart->txWriteLoc] = ucChar;
		this_uart->txWriteLoc++;
		if (this_uart->txWriteLoc >= this_uart->txBufferSize)
			this_uart->txWriteLoc = 0;

		/* increment tx-databytes and enable transmit interrupt */
		pic_int_disable(this_uart->intrLevel);
		this_uart->txDataBytes++;
		this_uart->ier |= UART_IER_TX_INT_MASK;
		dev->ier = this_uart->ier;
		pic_int_enable(this_uart->intrLevel);

		/* all done */
		return (0);
	} else
#endif
	{
		do {
			/* if uart's ready to accept character, send immediately */
			uiValue = dev->lsr;
			if (uiValue & UART_LSR_TX_RDY_MASK) {
				dev->rxtx = ucChar;
				return (0);
			}

			/* if non-blocking tx, return immediately */
			if (this_uart->blockingTx == 0)
				return (UART_ERR_WOULD_BLOCK);

		} while (1);
	}

	/* all done */
	return 0;
}

/*
 ***************************************************************
 * Changes datawidth, parity-selection and stop-bit selection
 *
 * Argument:
 *
 * uart_instance *ctx: context to uart
 * unsigned int dwidth: data width in bits(5, 6, 7, 8)
 * unsigned int parity_en: 0 => no pareity, 1 => parity enabled
 * unsigned int even_odd (parity): - 1 => even, 0 => odd
 * unsigned int stopbits: stop bits (1,2)
 *
 ***************************************************************
 */
unsigned char uart_config(struct uart_instance *this_uart,
			  unsigned int dwidth,
			  unsigned char parity_en,
			  unsigned char even_odd, unsigned int stopbits)
{
	volatile struct uart_dev *dev;
	unsigned char lcr;
	if (NULL == this_uart) {
		return 1;
	}
	dev = (volatile struct uart_dev *) (this_uart->base);

	/* check data-width value */
	if (dwidth > 8)
		return (UART_ERR_INVALID_ARGUMENT);
	else if (dwidth < 5)
		return (UART_ERR_INVALID_ARGUMENT);

	/* check stopbit value */
	if ((stopbits != 1) && (stopbits != 2))
		return (UART_ERR_INVALID_ARGUMENT);

	/* update context */
	this_uart->databits = dwidth;
	this_uart->stopbits = stopbits;

	/* configure UART */
	switch (dwidth) {
	case 5:{
			lcr = (stopbits == 2) ? 0x4 : 0x0;
			break;
		}
	case 6:{
			lcr = (stopbits == 2) ? 0x5 : 0x1;
			break;
		}
	case 7:{
			lcr = (stopbits == 2) ? 0x6 : 0x2;
			break;
		}
	default:{		/* 8 databits */
			lcr = (stopbits == 2) ? 0x7 : 0x3;
			this_uart->databits = 8;
			break;
		}
	}

	/* parity control */
	if (parity_en == 1) {
		if (even_odd == 1)
			lcr |= 0x18;
		else
			lcr |= 0x8;
	}
	dev->lcr = lcr;

	/* all done */
	return 0;
}

/*
 ***************************************************************
 * Changes baudrate.
 * NOTE: User must make sure the baudrate passed is correct.
 ***************************************************************
*/
unsigned char uart_set_rate(struct uart_instance *this_uart,
			    unsigned int baudrate)
{
	unsigned int divisor;
	if (NULL == this_uart) {
		return 1;
	}
	volatile struct uart_dev *dev;
	dev = (volatile struct uart_dev *) (this_uart->base);

	/* it would be nice to have uart stop-receiver/stop-transmitter functionality! */
	/* the only check we do is for zero (to avoid divide-by-zero) */
	if (baudrate == 0)
		return (UART_ERR_INVALID_ARGUMENT);

#if _UART_ENABLE_INTERRUPTS_
	/* disable interrupt for the UART */
	pic_int_disable(this_uart->intrLevel);
#endif

	/* set to new baudrate */
	this_uart->baudrate = baudrate;

	/* Calculate clock-divisor */
	divisor = (this_uart->sys_clk) / baudrate;
	/*
	 * Setup uart:
	 * - divisor
	 */
	unsigned int *address = (unsigned int *) &(dev->dlr_lsb);
	*address = (unsigned char) (divisor);
	address++;
	*address = (unsigned char) (divisor >> 8);

#if _UART_ENABLE_INTERRUPTS_
	/* re-enable interrupt for the UART */
	pic_int_enable(this_uart->intrLevel);
#endif

	return 0;
}
