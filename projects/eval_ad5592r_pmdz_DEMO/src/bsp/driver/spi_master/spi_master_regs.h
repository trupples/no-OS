/*   ==================================================================
     >>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
     ------------------------------------------------------------------
     Copyright (c) 2006-2023 by Lattice Semiconductor Corporation
     ALL RIGHTS RESERVED
     ------------------------------------------------------------------

     IMPORTANT: THIS FILE IS AUTO-GENERATED BY LATTICE RADIANT Software.

     Permission:

        Lattice grants permission to use this code pursuant to the
        terms of the Lattice Corporation Open Source License Agreement.

     Disclaimer:

        Lattice provides no warranty regarding the use or functionality
        of this code. It is the user's responsibility to verify the
        user Software design for consistency and functionality through
        the use of formal Software validation methods.

     ------------------------------------------------------------------

     Lattice Semiconductor Corporation
     111 SW Fifth Avenue, Suite 700
     Portland, OR 97204
     U.S.A

     Email: techsupport@latticesemi.com
     Web: http://www.latticesemi.com/Home/Support/SubmitSupportTicket.aspx
     ================================================================== */

#ifndef SPI_MASTER_REGS_H
#define SPI_MASTER_REGS_H


/*
*  spi master registers definition
*/


#define SPI_MASTER_WR_DATA                        (0x00)

#define SPI_MASTER_RD_DATA                        (0x00)

#define SPI_MASTER_SLAVE_SEL                      (0x01 * 4)

#define SPI_MASTER_SPI_CFG                        (0x02 * 4)
        #define SPI_CFG_SPI_EN_MASK        (0x80)
        #define SPI_CFG_WR_ONLY            (0x20)
        #define SPI_CFG_DATA_WIDTH_MASK    (0x18)
        #define SPI_CFG_SSNP_MASK          (0x04)
        #define SPI_CFG_CPOL_MASK          (0x02)
        #define SPI_CFG_CPHA_MASK          (0x01)

#define SPI_MASTER_CLK_PRESCALER_LOW               (0x03 * 4)
#define SPI_MASTER_CLK_PRESCALER_HIGH              (0x04 * 4)


#define SPI_MASTER_INT_STATUS                      (0x05 * 4)
#define SPI_MASTER_INT_ENABLE                      (0x06 * 4)
#define SPI_MASTER_INT_SET                         (0x07 * 4)
        #define SPI_INT_TRAN_CMP           (0x80)
		#define SPI_INT_TX_BUFFER_FULL     (0x20)
        #define SPI_INT_TX_BUFFER_AEMPT    (0x10)
       // #define SPI_INT_WORD_CMP         (0x10)
        #define SPI_INT_TX_BUFFER_EMPTY    (0x08)
        #define SPI_INT_RX_BUFFER_FULL     (0x04)
		#define SPI_INT_RX_BUFFER_AFULL    (0x02)
 	 	#define SPI_INT_RX_BUFFER_READY    (0x01)

#define SPI_MASTER_BYTE_COUNT                      (0x08 * 4)
#define SPI_MASTER_BYTE_COUNT_RST                  (0x09 * 4)
#define SPI_MASTER_TARGET_WORD_COUNT               (0x0A * 4)
#define SPI_MASTER_FIFO_RST                        (0x0B * 4)
        #define SPI_MASTER_TX_FIFO_RST     (0x02)
        #define SPI_MASTER_RX_FIFO_RST     (0x01)
#define SPI_MASTER_POL_SEL                         (0x0C * 4)
#define SPI_MASTER_FIFO_STATUS                     (0x0D * 4)
#define SPI_MASTER_SPI_ENABLE_REG                  (0x0E * 4)
		#define SPI_MASTER_RX_FIFO_EMPTY   (0x01)

#endif  /*spi Master Registers Header File*/
