/***************************************************************************//**
 *   @file   stm32/stm32_uart_stdio.c
 *   @brief  Implementation file of stm32 UART driver stdout/stdin redirection.
 *   @author Darius Berghe (darius.berghe@analog.com)
 *           Credit to Carmine Noviello for stdio redirection in this file
 * 	     that was inspired from his "Mastering STM32" book.
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include "no-os/uart.h"
#include "stm32_uart_stdio.h"
#include "stm32_hal.h"

#if !defined(OS_USE_SEMIHOSTING)

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

static struct uart_desc *guart = NULL;

void stm32_uart_stdio(struct uart_desc *desc)
{
	if(!desc || !desc->extra)
		return;

	guart = desc;

	/* Disable I/O buffering for STDOUT stream, so that
	* chars are sent out as soon as they are printed. */
	setvbuf(stdout, NULL, _IONBF, 0);
}

int _isatty(int fd)
{
	if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
		return 1;

	errno = EBADF;
	return 0;
}

int _write(int fd, char* ptr, int len)
{
	int ret;

	if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
		ret = uart_write(guart, (uint8_t *)ptr, len);
		if (ret < 0) {
			errno = -ret;
			return -1;
		}

		return len;
	}
	errno = EBADF;
	return -1;
}

int _close(int fd)
{
	if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
		return 0;

	errno = EBADF;
	return -1;
}

int _lseek(int fd, int ptr, int dir)
{
	(void) fd;
	(void) ptr;
	(void) dir;

	errno = EBADF;
	return -1;
}

int _read(int fd, char* ptr, int len)
{
	int ret;

	if (fd == STDIN_FILENO) {
		ret = uart_read(guart, (uint8_t *)ptr, 1);
		if (ret < 0) {
			errno = -ret;
			return -1;
		}

		return ret;
	}
	errno = EBADF;
	return -1;
}

int _fstat(int fd, struct stat* st)
{
	if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
		st->st_mode = S_IFCHR;
		return 0;
	}

	errno = EBADF;
	return 0;
}

#endif //#if !defined(OS_USE_SEMIHOSTING)
