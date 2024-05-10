/**
 * This file is automatically generated - DO NOT EDIT
 *
 * Copyright 2015 - 2022 Analog Devices Inc.
 * Released under the ADRV904X API license, for more information
 * see the "LICENSE.pdf" file in this zip file.
 */

#ifndef _ADRV904X_REG_ADDR_MACROS_H
#define _ADRV904X_REG_ADDR_MACROS_H

#define ADRV904X_BF_ENCODE(value, mask, shift) (((value) << (shift)) & (mask))
#define ADRV904X_BF_DECODE(value, mask, shift) (((value) & (mask)) >> (shift))
#define ADRV904X_BF_CLEAR(result, mask)  (result = ((result) & ~(mask)))
#define ADRV904X_BF_SET(result, mask) (result = (result) | (mask))
#define ADRV904X_BF_UPDATE(result, value, mask, shift)  (result = ((result) & ~(mask)) | ( ((value) << (shift)) & (mask)))
#define ADRV904X_BF_EQUAL(value, mask) ((mask) == ((value) & (mask)))


/* ETM Offset registers */
#define ADRV904X_ETM_MAX                           0x4U
#define ADRV904X_ETM_TRCCONFIGR_OFFSET             0x10U
#define ADRV904X_ETM_TRCTRACEIDR_OFFSET            0x40U
#define ADRV904X_ETM_TRCPRGCTLR_OFFSET             0x4U
#define ADRV904X_ETM_TRCIDR0_OFFSET                0x1E0U
#define ADRV904X_ETM_TRCIDR1_OFFSET                0x1E4U
#define ADRV904X_ETM_TRCIDR2_OFFSET                0x1E8U
#define ADRV904X_ETF_RSZ_OFFSET                    0x4U
#define ADRV904X_ETF_RRD_OFFSET                    0x10U

/* Bit positions */
#define ADRV904X_CPU_MASK_CTL1_M3_RUN              0x01
#define ADRV904X_STREAM_RESET_BIT                  0
#define ADRV904X_RX_STREAM_RESET_BIT               16
#define ADRV904X_STREAM_BASE_BYTE0_BIT             3
#define ADRV904X_STREAM_BASE_BYTE1_BIT             11
#define ADRV904X_LAST_STREAM_NUMBER_BIT            19
#define ADRV904X_RX_STREAM_BASE_BYTE0_BIT          0
#define ADRV904X_RX_STREAM_BASE_BYTE1_BIT          8
#define ADRV904X_RX_LAST_STREAM_NUMBER_BIT         16

/* Core stream register offsets */
#define ADRV904X_STREAM_BASE_BYTE0_REG_OFFSET      1
#define ADRV904X_STREAM_BASE_BYTE1_REG_OFFSET      2
#define ADRV904X_LAST_STREAM_NUMBER_REG_OFFSET     3

/* Registers that require using 32-bit SPI transactions */
/* ORx RAM, Tx DPD RAM, and Tx/Rx Gain Tables */
#define ADRV904X_AHB_ADDR_STRIDE                   0x100000U
#define ADRV904X_ORX_RAM_ADDR_BASE                 0x61000000U
#define ADRV904X_ORX_RAM_REG_LEN                   0xC000U
#define ADRV904X_ORX_ANALOG_ADC_ADDR_BASE          0x61070000U
#define ADRV904X_ORX_ANALOG_ADC_REG_LEN            0xD800U
#define ADRV904X_RX_GAIN_TABLE_ADDR_BASE           0x600A0000U
#define ADRV904X_RX_GAIN_TABLE_REG_LEN             0x800U
#define ADRV904X_TX_DPD_RAM_ADDR_BASE              0x60800000U
#define ADRV904X_TX_DPD_RAM_REG_LEN                0x10000U
#define ADRV904X_TX_ATTEN_TABLE_ADDR_BASE          0x60810000U
#define ADRV904X_TX_ATTEN_TABLE_REG_LEN            0x2F00U
#define ADRV904X_UART_ADDR_BASE                    0x46100000U
#define ADRV904X_UART_REG_LEN                      0x100000U
#define ADRV904X_SPI_MASTER_ADDR_BASE              0x46200000U
#define ADRV904X_SPI_MASTER_REG_LEN                0x100000U
#define ADRV904X_INTR_TSMTR_ADDR_BASE              0x46800000U
#define ADRV904X_INTR_TSMTR_REG_LEN                0x10000U
#define ADRV904X_TELEMETRY_ADDR_BASE               0x46400000U
#define ADRV904X_TELEMETRY_REG_LEN                 0x100000U

/* 32 Hardware semaphore addresses */
#define ADRV904X_SEMAPHORE_ADDR_BASE       0x46500000U
#define ADRV904X_SEMAPHORE_ADDR_STRIDE     0x00010000U

#define ADRV904X_CORE_0_INTR_AGG_ADDR_BASE       0x41200000U
#define ADRV904X_CORE_0_INTR_AGG_REG_LEN         0x100000U
#define ADRV904X_CORE_1_INTR_AGG_ADDR_BASE       0x44200000U
#define ADRV904X_CORE_1_INTR_AGG_REG_LEN         0x100000U
#define ADRV904X_TX_DPD_ACT_LUT_ADDR_BASE        0x608E4000U
#define ADRV904X_TX_DPD_ACT_LUT_REG_LEN          0x4000U
#define ADRV904X_TX_ANA_LB_ADC_32_ADDR_BASE      0x60890000U
#define ADRV904X_TX_ANA_LB_ADC_32_REG_LEN        0x4000U
#define ADRV904X_TX_SEMAPHORE_ADDR_BASE          0x60824000U
#define ADRV904X_TX_SEMAPHORE_REG_LEN            0x8000U
#define ADRV904X_TX_CFR_PULSE_RAM_ADDR_BASE      0x608D2000U
#define ADRV904X_TX_CFR_PULSE_RAM_REG_LEN        0xC000U
#define ADRV904X_RX_SEMAPHORE_ADDR_BASE          0x60024000U
#define ADRV904X_RX_SEMAPHORE_REG_LEN            0x8000U
#define ADRV904X_ORX_SEMAPHORE_ADDR_BASE         0x61024000U
#define ADRV904X_ORX_SEMAPHORE_REG_LEN           0x8000U
#define ADRV904X_RS_GLOBAL_CFG_1_ADDR_BASE       0x4D200008U
#define ADRV904X_RS_GLOBAL_CFG_1_REG_LEN         0x0004U
#define ADRV904X_RS_CONFIG_0_ADDR_BASE           0x4D200020U
#define ADRV904X_RS_CONFIG_0_REG_LEN             0x003CU
#define ADRV904X_A55_INTR_AGG_ADDR_BASE          0xB2300000U
#define ADRV904X_A55_INTR_AGG_REG_LEN            0x100000U
#define ADRV904X_A55_INTR_TSMTR_ADDR_BASE        0xB2400000U
#define ADRV904X_A55_INTR_TSMTR_REG_LEN          0x100000U
#define ADRV904X_A55_UART0_ADDR_BASE             0xB1200000U
#define ADRV904X_A55_UART0_REG_LEN               0x100000U
#define ADRV904X_A55_UART1_ADDR_BASE             0xB1400000U
#define ADRV904X_A55_UART1_REG_LEN               0x100000U
#define ADRV904X_A55_TRU_ADDR_BASE               0xB1000000U
#define ADRV904X_A55_TRU_REG_LEN                 0x100000U
#define ADRV904X_A55_GIC_ADDR_BASE               0xB8100000U
#define ADRV904X_A55_GIC_REG_LEN                 0x100000U
#define ADRV904X_A55_MDMA_ADDR_BASE              0xB0000000U
#define ADRV904X_A55_MDMA_REG_LEN                0x400000U
#define ADRV904X_A55_L4CFG_ADDR_BASE             0xB0800000U
#define ADRV904X_A55_L4CFG_REG_LEN               0x400000U
#define ADRV904X_A55_CAPBUFDDE_ADDR_BASE         0xB0C00000U
#define ADRV904X_A55_CAPBUFDDE_REG_LEN           0x200000U
#define ADRV904X_A55_TSGEN_CTRL_ADDR_BASE        0xB0F00000U
#define ADRV904X_A55_TSGEN_CTRL_REG_LEN          0x100000U
#define ADRV904X_A55_MMR_FABRIC_ADDR_BASE        0xB1800000U
#define ADRV904X_A55_MMR_FABRIC_REG_LEN          0x100000U
#define ADRV904X_A55_SYSTEM_CTI_ADDR_BASE        0xB6001000U
#define ADRV904X_A55_SYSTEM_CTI_REG_LEN          0x1000U
#define ADRV904X_A55_ATB_FUNNEL_ADDR_BASE        0xB6002000U
#define ADRV904X_A55_ATB_FUNNEL_REG_LEN          0x1000U
#define ADRV904X_A55_TRACE_FIFO_ADDR_BASE        0xB6003000U
#define ADRV904X_A55_TRACE_FIFO_REG_LEN          0x1000U
#define ADRV904X_A55_TPIU_ADDR_BASE              0xB6004000U
#define ADRV904X_A55_TPIU_REG_LEN                0x1000U
#define ADRV904X_A55_PE_DBGVIEW_ADDR_BASE        0xB7010000U
#define ADRV904X_A55_PE_DBGVIEW_REG_LEN          0x400000U
#define ADRV904X_A55_PE_SYSVIEW_ADDR_BASE        0xB7810000U
#define ADRV904X_A55_PE_SYSVIEW_REG_LEN          0x400000U
#define ADRV904X_A55_FABRIC_ADDR_BASE            0xB8000000U
#define ADRV904X_A55_FABRIC_REG_LEN              0x100000U
#define ADRV904X_TX_VSWR_PLAYBACK_RAM_ADDR_BASE  0x608F9000U
#define ADRV904X_TX_VSWR_PLAYBACK_RAM_REG_LEN    0x1000U
#define ADRV904X_TX_CAPTURE_RAM_ADDR_BASE        0x60800000U
#define ADRV904X_TX_CAPTURE_RAM_REG_LEN          0x1000U

/* Register addresses
 * To add new register macros append the [API macro name]    [YODA Header name]
 * pair in c_src\devices\adrv904x\private\include\adrv904x_yoda_reg_list.txt file.
*/
#define ADRV904X_ADDR_SPIFIFO_MODE                         (0x10U)
#define ADRV904X_CPU_0_ADDR_CTL_1                          (0x22U)
#define ADRV904X_CPU_0_ADDR_BOOT_ADDR_BYTE0                (0x24U)
#define ADRV904X_CPU_0_ADDR_BOOT_ADDR_BYTE1                (0x25U)
#define ADRV904X_CPU_0_ADDR_BOOT_ADDR_BYTE2                (0x26U)
#define ADRV904X_CPU_0_ADDR_BOOT_ADDR_BYTE3                (0x27U)
#define ADRV904X_CPU_0_ADDR_STACK_PTR_BYTE0                (0x28U)
#define ADRV904X_CPU_0_ADDR_STACK_PTR_BYTE1                (0x29U)
#define ADRV904X_CPU_0_ADDR_STACK_PTR_BYTE2                (0x2AU)
#define ADRV904X_CPU_0_ADDR_STACK_PTR_BYTE3                (0x2BU)
#define ADRV904X_CPU_0_ADDR_MEM_BANK_CTRL                  (0x4DU)
#define ADRV904X_ADDR_SPIDMA0_CTL                          (0x4FU)
#define ADRV904X_ADDR_SPIDMA0_ADDR3                        (0x50U)
#define ADRV904X_ADDR_SPIDMA0_ADDR2                        (0x51U)
#define ADRV904X_ADDR_SPIDMA0_ADDR1                        (0x52U)
#define ADRV904X_ADDR_SPIDMA0_ADDR0                        (0x53U)
#define ADRV904X_ADDR_SPIDMA0_DATA3                        (0x54U)
#define ADRV904X_ADDR_SPIDMA0_DATA2                        (0x55U)
#define ADRV904X_ADDR_SPIDMA0_DATA1                        (0x56U)
#define ADRV904X_ADDR_SPIDMA0_DATA0                        (0x57U)
#define ADRV904X_CPU_0_ADDR_COMMAND                        (0x63U)
#define ADRV904X_CPU_0_ADDR_EXT_CMD_BYTE_1                 (0x64U)
#define ADRV904X_CPU_0_ADDR_CMD_STATUS_0                   (0x6BU)
#define ADRV904X_ADDR_MAIN_STREAM_CTL                      (0x95U)
#define ADRV904X_ADDR_KFA_STREAM_CTL                       (0x95U)
#define ADRV904X_CPU_1_ADDR_CTL_1                          (0xC6U)
#define ADRV904X_CPU_1_ADDR_BOOT_ADDR_BYTE0                (0xC8U)
#define ADRV904X_CPU_1_ADDR_BOOT_ADDR_BYTE1                (0xC9U)
#define ADRV904X_CPU_1_ADDR_BOOT_ADDR_BYTE2                (0xCAU)
#define ADRV904X_CPU_1_ADDR_BOOT_ADDR_BYTE3                (0xCBU)
#define ADRV904X_CPU_1_ADDR_STACK_PTR_BYTE0                (0xCCU)
#define ADRV904X_CPU_1_ADDR_STACK_PTR_BYTE1                (0xCDU)
#define ADRV904X_CPU_1_ADDR_STACK_PTR_BYTE2                (0xCEU)
#define ADRV904X_CPU_1_ADDR_STACK_PTR_BYTE3                (0xCFU)
#define ADRV904X_CPU_1_ADDR_MEM_BANK_CTRL                  (0xF2U)
#define ADRV904X_CPU_1_ADDR_COMMAND                        (0xF4U)
#define ADRV904X_CPU_1_ADDR_EXT_CMD_BYTE_1                 (0xF5U)
#define ADRV904X_CPU_1_ADDR_CMD_STATUS_0                   (0xFCU)
#define ADRV904X_ADDR_SPI0_PAGE_31TO24                     (0x126U)
#define ADRV904X_ADDR_SPI0_PAGE_23TO16                     (0x127U)
#define ADRV904X_ADDR_SPI0_PAGE_15TO8                      (0x128U)
#define ADRV904X_ADDR_SPI0_PAGE_7TO0                       (0x129U)
#define ADRV904X_ADDR_SPI0_PAGING_CONTROL                  (0x12AU)
#define ADRV904X_ADDR_SPI0_MASK_7TO0                       (0x132U)
#define ADRV904X_ADDR_GPINT_MASK_PIN1_BYTE1                (0x147U)
#define ADRV904X_ADDR_GPINT_MASK_PIN1_BYTE9                (0x14FU)
#define ADRV904X_ADDR_GPINT_MASK_PIN0_BYTE1                (0x153U)
#define ADRV904X_ADDR_GPINT_MASK_PIN0_BYTE9                (0x15BU)
#define ADRV904X_ADDR_GPINT_STATUS_BYTE1                   (0x16BU)
#define ADRV904X_ADDR_EFUSE_READ_CTRL                      (0x18BU)
#define ADRV904X_ADDR_EFUSE_READ_ADDR                      (0x192U)
#define ADRV904X_ADDR_CORE_STREAM_SCRATCH204               (0x2CCU)
#define ADRV904X_ADDR_CORE_STREAM_SCRATCH212               (0x2D4U)
#define ADRV904X_ADDR_CORE_STREAM_SCRATCH213               (0x2D5U)
#define ADRV904X_ADDR_CORE_STREAM_SCRATCH214               (0x2D6U)
#define ADRV904X_ADDR_CORE_STREAM_SCRATCH215               (0x2D7U)
#define ADRV904X_ADDR_CORE_STREAM_SCRATCH216               (0x2D8U)
#define ADRV904X_ADDR_CORE_STREAM_SCRATCH481               (0x3E1U)
#define ADRV904X_ADDR_RFLO0_PLL_MISC_PD                    (0x47300001U)
#define ADRV904X_ADDR_RFLO1_PLL_MISC_PD                    (0x47400001U)
#define ADRV904X_ADDR_SERDES_PLL_MISC_PD                   (0x48060001U)
#define ADRV904X_ADDR_SERDES_PLL_OUTPUT_DIVIDER_CTL        (0x480600AEU)
#define ADRV904X_ADDR_CLK_PLL_MISC_PD                      (0x48090001U)
#define ADRV904X_ADDR_DESER_PHY0_SPI_INTFCONFA             (0x48080000U)
#define ADRV904X_ADDR_SERDES_RXDIG_PHY_PD_0                (0x48080002U)
#define ADRV904X_ADDR_DESER_PHY0_PD_REG_0                  (0x48080002U)
#define ADRV904X_ADDR_DESER_PHY0_PD_REG_1                  (0x48080003U)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_TEST              (0x4808000EU)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL0        (0x480800C0U)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL18       (0x480800D2U)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL28       (0x480800DCU)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL31       (0x480800E2U)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL41       (0x480800ECU)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL43       (0x480800EFU)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL45       (0x480800F2U)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL49       (0x480800F7U)
#define ADRV904X_ADDR_DESER_PHY0_CORE1P2_DECFE_CTL54       (0x480800FCU)
#define ADRV904X_ADDR_DESER_PHY1_SPI_INTFCONFA             (0x48080800U)
#define ADRV904X_ADDR_DESER_PHY1_PD_REG_0                  (0x48080802U)
#define ADRV904X_ADDR_DESER_PHY1_PD_REG_1                  (0x48080803U)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_TEST              (0x4808080EU)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL0        (0x480808C0U)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL18       (0x480808D2U)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL28       (0x480808DCU)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL31       (0x480808E2U)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL41       (0x480808ECU)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL43       (0x480808EFU)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL45       (0x480808F2U)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL49       (0x480808F7U)
#define ADRV904X_ADDR_DESER_PHY1_CORE1P2_DECFE_CTL54       (0x480808FCU)
#define ADRV904X_ADDR_DESER_PHY2_SPI_INTFCONFA             (0x48081000U)
#define ADRV904X_ADDR_DESER_PHY2_PD_REG_0                  (0x48081002U)
#define ADRV904X_ADDR_DESER_PHY2_PD_REG_1                  (0x48081003U)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_TEST              (0x4808100EU)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL0        (0x480810C0U)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL18       (0x480810D2U)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL28       (0x480810DCU)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL31       (0x480810E2U)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL41       (0x480810ECU)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL43       (0x480810EFU)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL45       (0x480810F2U)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL49       (0x480810F7U)
#define ADRV904X_ADDR_DESER_PHY2_CORE1P2_DECFE_CTL54       (0x480810FCU)
#define ADRV904X_ADDR_DESER_PHY3_SPI_INTFCONFA             (0x48081800U)
#define ADRV904X_ADDR_DESER_PHY3_PD_REG_0                  (0x48081802U)
#define ADRV904X_ADDR_DESER_PHY3_PD_REG_1                  (0x48081803U)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_TEST              (0x4808180EU)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL0        (0x480818C0U)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL18       (0x480818D2U)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL28       (0x480818DCU)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL31       (0x480818E2U)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL41       (0x480818ECU)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL43       (0x480818EFU)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL45       (0x480818F2U)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL49       (0x480818F7U)
#define ADRV904X_ADDR_DESER_PHY3_CORE1P2_DECFE_CTL54       (0x480818FCU)
#define ADRV904X_ADDR_DESER_PHY4_SPI_INTFCONFA             (0x48082000U)
#define ADRV904X_ADDR_DESER_PHY4_PD_REG_0                  (0x48082002U)
#define ADRV904X_ADDR_DESER_PHY4_PD_REG_1                  (0x48082003U)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_TEST              (0x4808200EU)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL0        (0x480820C0U)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL18       (0x480820D2U)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL28       (0x480820DCU)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL31       (0x480820E2U)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL41       (0x480820ECU)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL43       (0x480820EFU)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL45       (0x480820F2U)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL49       (0x480820F7U)
#define ADRV904X_ADDR_DESER_PHY4_CORE1P2_DECFE_CTL54       (0x480820FCU)
#define ADRV904X_ADDR_DESER_PHY5_SPI_INTFCONFA             (0x48082800U)
#define ADRV904X_ADDR_DESER_PHY5_PD_REG_0                  (0x48082802U)
#define ADRV904X_ADDR_DESER_PHY5_PD_REG_1                  (0x48082803U)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_TEST              (0x4808280EU)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL0        (0x480828C0U)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL18       (0x480828D2U)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL28       (0x480828DCU)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL31       (0x480828E2U)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL41       (0x480828ECU)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL43       (0x480828EFU)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL45       (0x480828F2U)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL49       (0x480828F7U)
#define ADRV904X_ADDR_DESER_PHY5_CORE1P2_DECFE_CTL54       (0x480828FCU)
#define ADRV904X_ADDR_DESER_PHY6_SPI_INTFCONFA             (0x48083000U)
#define ADRV904X_ADDR_DESER_PHY6_PD_REG_0                  (0x48083002U)
#define ADRV904X_ADDR_DESER_PHY6_PD_REG_1                  (0x48083003U)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_TEST              (0x4808300EU)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL0        (0x480830C0U)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL18       (0x480830D2U)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL28       (0x480830DCU)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL31       (0x480830E2U)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL41       (0x480830ECU)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL43       (0x480830EFU)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL45       (0x480830F2U)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL49       (0x480830F7U)
#define ADRV904X_ADDR_DESER_PHY6_CORE1P2_DECFE_CTL54       (0x480830FCU)
#define ADRV904X_ADDR_DESER_PHY7_SPI_INTFCONFA             (0x48083800U)
#define ADRV904X_ADDR_DESER_PHY7_PD_REG_0                  (0x48083802U)
#define ADRV904X_ADDR_DESER_PHY7_PD_REG_1                  (0x48083803U)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_TEST              (0x4808380EU)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL0        (0x480838C0U)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL18       (0x480838D2U)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL28       (0x480838DCU)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL31       (0x480838E2U)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL41       (0x480838ECU)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL43       (0x480838EFU)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL45       (0x480838F2U)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL49       (0x480838F7U)
#define ADRV904X_ADDR_DESER_PHY7_CORE1P2_DECFE_CTL54       (0x480838FCU)
#define ADRV904X_ADDR_RX0_STREAM_CTL                       (0x60023000U)
#define ADRV904X_ADDR_RX0_STREAM_SCRATCH0                  (0x60023014U)
#define ADRV904X_ADDR_RX0_STREAM_SCRATCH1                  (0x60023018U)
#define ADRV904X_ADDR_RX0_STREAM_SCRATCH2                  (0x6002301CU)
#define ADRV904X_ADDR_RX0_STREAM_SCRATCH3                  (0x60023020U)
#define ADRV904X_ADDR_RX0_STREAM_SCRATCH4                  (0x60023024U)
#define ADRV904X_ADDR_RX0_STREAM_SCRATCH5                  (0x60023028U)
#define ADRV904X_ADDR_RX0_STREAM_SCRATCH6                  (0x6002302CU)
#define ADRV904X_ADDR_RX0_STREAM_SCRATCH7                  (0x60023030U)
#define ADRV904X_ADDR_RX1_STREAM_CTL                       (0x60123000U)
#define ADRV904X_ADDR_RX1_STREAM_SCRATCH0                  (0x60123014U)
#define ADRV904X_ADDR_RX1_STREAM_SCRATCH1                  (0x60123018U)
#define ADRV904X_ADDR_RX1_STREAM_SCRATCH2                  (0x6012301CU)
#define ADRV904X_ADDR_RX1_STREAM_SCRATCH3                  (0x60123020U)
#define ADRV904X_ADDR_RX1_STREAM_SCRATCH4                  (0x60123024U)
#define ADRV904X_ADDR_RX1_STREAM_SCRATCH5                  (0x60123028U)
#define ADRV904X_ADDR_RX1_STREAM_SCRATCH6                  (0x6012302CU)
#define ADRV904X_ADDR_RX1_STREAM_SCRATCH7                  (0x60123030U)
#define ADRV904X_ADDR_RX2_STREAM_CTL                       (0x60223000U)
#define ADRV904X_ADDR_RX2_STREAM_SCRATCH0                  (0x60223014U)
#define ADRV904X_ADDR_RX2_STREAM_SCRATCH1                  (0x60223018U)
#define ADRV904X_ADDR_RX2_STREAM_SCRATCH2                  (0x6022301CU)
#define ADRV904X_ADDR_RX2_STREAM_SCRATCH3                  (0x60223020U)
#define ADRV904X_ADDR_RX2_STREAM_SCRATCH4                  (0x60223024U)
#define ADRV904X_ADDR_RX2_STREAM_SCRATCH5                  (0x60223028U)
#define ADRV904X_ADDR_RX2_STREAM_SCRATCH6                  (0x6022302CU)
#define ADRV904X_ADDR_RX2_STREAM_SCRATCH7                  (0x60223030U)
#define ADRV904X_ADDR_RX3_STREAM_CTL                       (0x60323000U)
#define ADRV904X_ADDR_RX3_STREAM_SCRATCH0                  (0x60323014U)
#define ADRV904X_ADDR_RX3_STREAM_SCRATCH1                  (0x60323018U)
#define ADRV904X_ADDR_RX3_STREAM_SCRATCH2                  (0x6032301CU)
#define ADRV904X_ADDR_RX3_STREAM_SCRATCH3                  (0x60323020U)
#define ADRV904X_ADDR_RX3_STREAM_SCRATCH4                  (0x60323024U)
#define ADRV904X_ADDR_RX3_STREAM_SCRATCH5                  (0x60323028U)
#define ADRV904X_ADDR_RX3_STREAM_SCRATCH6                  (0x6032302CU)
#define ADRV904X_ADDR_RX3_STREAM_SCRATCH7                  (0x60323030U)
#define ADRV904X_ADDR_RX4_STREAM_CTL                       (0x60423000U)
#define ADRV904X_ADDR_RX4_STREAM_SCRATCH0                  (0x60423014U)
#define ADRV904X_ADDR_RX4_STREAM_SCRATCH1                  (0x60423018U)
#define ADRV904X_ADDR_RX4_STREAM_SCRATCH2                  (0x6042301CU)
#define ADRV904X_ADDR_RX4_STREAM_SCRATCH3                  (0x60423020U)
#define ADRV904X_ADDR_RX4_STREAM_SCRATCH4                  (0x60423024U)
#define ADRV904X_ADDR_RX4_STREAM_SCRATCH5                  (0x60423028U)
#define ADRV904X_ADDR_RX4_STREAM_SCRATCH6                  (0x6042302CU)
#define ADRV904X_ADDR_RX4_STREAM_SCRATCH7                  (0x60423030U)
#define ADRV904X_ADDR_RX5_STREAM_CTL                       (0x60523000U)
#define ADRV904X_ADDR_RX5_STREAM_SCRATCH0                  (0x60523014U)
#define ADRV904X_ADDR_RX5_STREAM_SCRATCH1                  (0x60523018U)
#define ADRV904X_ADDR_RX5_STREAM_SCRATCH2                  (0x6052301CU)
#define ADRV904X_ADDR_RX5_STREAM_SCRATCH3                  (0x60523020U)
#define ADRV904X_ADDR_RX5_STREAM_SCRATCH4                  (0x60523024U)
#define ADRV904X_ADDR_RX5_STREAM_SCRATCH5                  (0x60523028U)
#define ADRV904X_ADDR_RX5_STREAM_SCRATCH6                  (0x6052302CU)
#define ADRV904X_ADDR_RX5_STREAM_SCRATCH7                  (0x60523030U)
#define ADRV904X_ADDR_RX6_STREAM_CTL                       (0x60623000U)
#define ADRV904X_ADDR_RX6_STREAM_SCRATCH0                  (0x60623014U)
#define ADRV904X_ADDR_RX6_STREAM_SCRATCH1                  (0x60623018U)
#define ADRV904X_ADDR_RX6_STREAM_SCRATCH2                  (0x6062301CU)
#define ADRV904X_ADDR_RX6_STREAM_SCRATCH3                  (0x60623020U)
#define ADRV904X_ADDR_RX6_STREAM_SCRATCH4                  (0x60623024U)
#define ADRV904X_ADDR_RX6_STREAM_SCRATCH5                  (0x60623028U)
#define ADRV904X_ADDR_RX6_STREAM_SCRATCH6                  (0x6062302CU)
#define ADRV904X_ADDR_RX6_STREAM_SCRATCH7                  (0x60623030U)
#define ADRV904X_ADDR_RX7_STREAM_CTL                       (0x60723000U)
#define ADRV904X_ADDR_RX7_STREAM_SCRATCH0                  (0x60723014U)
#define ADRV904X_ADDR_RX7_STREAM_SCRATCH1                  (0x60723018U)
#define ADRV904X_ADDR_RX7_STREAM_SCRATCH2                  (0x6072301CU)
#define ADRV904X_ADDR_RX7_STREAM_SCRATCH3                  (0x60723020U)
#define ADRV904X_ADDR_RX7_STREAM_SCRATCH4                  (0x60723024U)
#define ADRV904X_ADDR_RX7_STREAM_SCRATCH5                  (0x60723028U)
#define ADRV904X_ADDR_RX7_STREAM_SCRATCH6                  (0x6072302CU)
#define ADRV904X_ADDR_RX7_STREAM_SCRATCH7                  (0x60723030U)
#define ADRV904X_ADDR_TX0_STREAM_CTL                       (0x60823000U)
#define ADRV904X_ADDR_TX0_STREAM_SCRATCH0                  (0x60823014U)
#define ADRV904X_ADDR_TX0_STREAM_SCRATCH1                  (0x60823018U)
#define ADRV904X_ADDR_TX0_STREAM_SCRATCH2                  (0x6082301CU)
#define ADRV904X_ADDR_TX0_STREAM_SCRATCH5                  (0x60823028U)
#define ADRV904X_ADDR_TX0_STREAM_SCRATCH6                  (0x6082302CU)
#define ADRV904X_ADDR_TX1_STREAM_CTL                       (0x60923000U)
#define ADRV904X_ADDR_TX1_STREAM_SCRATCH0                  (0x60923014U)
#define ADRV904X_ADDR_TX1_STREAM_SCRATCH1                  (0x60923018U)
#define ADRV904X_ADDR_TX1_STREAM_SCRATCH2                  (0x6092301CU)
#define ADRV904X_ADDR_TX1_STREAM_SCRATCH5                  (0x60923028U)
#define ADRV904X_ADDR_TX1_STREAM_SCRATCH6                  (0x6092302CU)
#define ADRV904X_ADDR_TX2_STREAM_CTL                       (0x60A23000U)
#define ADRV904X_ADDR_TX2_STREAM_SCRATCH0                  (0x60A23014U)
#define ADRV904X_ADDR_TX2_STREAM_SCRATCH1                  (0x60A23018U)
#define ADRV904X_ADDR_TX2_STREAM_SCRATCH2                  (0x60A2301CU)
#define ADRV904X_ADDR_TX2_STREAM_SCRATCH5                  (0x60A23028U)
#define ADRV904X_ADDR_TX2_STREAM_SCRATCH6                  (0x60A2302CU)
#define ADRV904X_ADDR_TX3_STREAM_CTL                       (0x60B23000U)
#define ADRV904X_ADDR_TX3_STREAM_SCRATCH0                  (0x60B23014U)
#define ADRV904X_ADDR_TX3_STREAM_SCRATCH1                  (0x60B23018U)
#define ADRV904X_ADDR_TX3_STREAM_SCRATCH2                  (0x60B2301CU)
#define ADRV904X_ADDR_TX3_STREAM_SCRATCH5                  (0x60B23028U)
#define ADRV904X_ADDR_TX3_STREAM_SCRATCH6                  (0x60B2302CU)
#define ADRV904X_ADDR_TX4_STREAM_CTL                       (0x60C23000U)
#define ADRV904X_ADDR_TX4_STREAM_SCRATCH0                  (0x60C23014U)
#define ADRV904X_ADDR_TX4_STREAM_SCRATCH1                  (0x60C23018U)
#define ADRV904X_ADDR_TX4_STREAM_SCRATCH2                  (0x60C2301CU)
#define ADRV904X_ADDR_TX4_STREAM_SCRATCH5                  (0x60C23028U)
#define ADRV904X_ADDR_TX4_STREAM_SCRATCH6                  (0x60C2302CU)
#define ADRV904X_ADDR_TX5_STREAM_CTL                       (0x60D23000U)
#define ADRV904X_ADDR_TX5_STREAM_SCRATCH0                  (0x60D23014U)
#define ADRV904X_ADDR_TX5_STREAM_SCRATCH1                  (0x60D23018U)
#define ADRV904X_ADDR_TX5_STREAM_SCRATCH2                  (0x60D2301CU)
#define ADRV904X_ADDR_TX5_STREAM_SCRATCH5                  (0x60D23028U)
#define ADRV904X_ADDR_TX5_STREAM_SCRATCH6                  (0x60D2302CU)
#define ADRV904X_ADDR_TX6_STREAM_CTL                       (0x60E23000U)
#define ADRV904X_ADDR_TX6_STREAM_SCRATCH0                  (0x60E23014U)
#define ADRV904X_ADDR_TX6_STREAM_SCRATCH1                  (0x60E23018U)
#define ADRV904X_ADDR_TX6_STREAM_SCRATCH2                  (0x60E2301CU)
#define ADRV904X_ADDR_TX6_STREAM_SCRATCH5                  (0x60E23028U)
#define ADRV904X_ADDR_TX6_STREAM_SCRATCH6                  (0x60E2302CU)
#define ADRV904X_ADDR_TX7_STREAM_CTL                       (0x60F23000U)
#define ADRV904X_ADDR_TX7_STREAM_SCRATCH0                  (0x60F23014U)
#define ADRV904X_ADDR_TX7_STREAM_SCRATCH1                  (0x60F23018U)
#define ADRV904X_ADDR_TX7_STREAM_SCRATCH2                  (0x60F2301CU)
#define ADRV904X_ADDR_TX7_STREAM_SCRATCH5                  (0x60F23028U)
#define ADRV904X_ADDR_TX7_STREAM_SCRATCH6                  (0x60F2302CU)
#define ADRV904X_ADDR_ORX0_STREAM_CTL                      (0x61023000U)
#define ADRV904X_ADDR_ORX0_STREAM_SCRATCH1                 (0x61023018U)
#define ADRV904X_ADDR_ORX1_STREAM_CTL                      (0x61123000U)
#define ADRV904X_ADDR_ORX1_STREAM_SCRATCH1                 (0x61123018U)
#define ADRV904X_ADDR_RX0_GAIN_COMP_CONFIG0_DDC0           (0x60060020U)
#define ADRV904X_ADDR_RX0_DEC_PWR_CONFIG1                  (0x60060084U)
#define ADRV904X_ADDR_RX0_GAIN_COMP_CONFIG0_DDC1           (0x60070020U)
#define ADRV904X_ADDR_RX1_GAIN_COMP_CONFIG0_DDC0           (0x60160020U)
#define ADRV904X_ADDR_RX1_DEC_PWR_CONFIG1                  (0x60160084U)
#define ADRV904X_ADDR_RX1_GAIN_COMP_CONFIG0_DDC1           (0x60170020U)
#define ADRV904X_ADDR_RX2_GAIN_COMP_CONFIG0_DDC0           (0x60260020U)
#define ADRV904X_ADDR_RX2_DEC_PWR_CONFIG1                  (0x60260084U)
#define ADRV904X_ADDR_RX2_GAIN_COMP_CONFIG0_DDC1           (0x60270020U)
#define ADRV904X_ADDR_RX3_GAIN_COMP_CONFIG0_DDC0           (0x60360020U)
#define ADRV904X_ADDR_RX3_DEC_PWR_CONFIG1                  (0x60360084U)
#define ADRV904X_ADDR_RX3_GAIN_COMP_CONFIG0_DDC1           (0x60370020U)
#define ADRV904X_ADDR_RX4_GAIN_COMP_CONFIG0_DDC0           (0x60460020U)
#define ADRV904X_ADDR_RX4_DEC_PWR_CONFIG1                  (0x60460084U)
#define ADRV904X_ADDR_RX4_GAIN_COMP_CONFIG0_DDC1           (0x60470020U)
#define ADRV904X_ADDR_RX5_GAIN_COMP_CONFIG0_DDC0           (0x60560020U)
#define ADRV904X_ADDR_RX5_DEC_PWR_CONFIG1                  (0x60560084U)
#define ADRV904X_ADDR_RX5_GAIN_COMP_CONFIG0_DDC1           (0x60570020U)
#define ADRV904X_ADDR_RX6_GAIN_COMP_CONFIG0_DDC0           (0x60660020U)
#define ADRV904X_ADDR_RX6_DEC_PWR_CONFIG1                  (0x60660084U)
#define ADRV904X_ADDR_RX6_GAIN_COMP_CONFIG0_DDC1           (0x60670020U)
#define ADRV904X_ADDR_RX7_GAIN_COMP_CONFIG0_DDC0           (0x60760020U)
#define ADRV904X_ADDR_RX7_DEC_PWR_CONFIG1                  (0x60760084U)
#define ADRV904X_ADDR_RX7_GAIN_COMP_CONFIG0_DDC1           (0x60770020U)
#define ADRV904X_ADDR_RX0_ADC_REGMAP0_CTRL_FD_PD           (0x600C000CU)
#define ADRV904X_ADDR_RX0_ADC_REGMAP0_CTRL_FL_PD           (0x600C0020U)
#define ADRV904X_ADDR_RX1_ADC_REGMAP0_CTRL_FD_PD           (0x601C000CU)
#define ADRV904X_ADDR_RX1_ADC_REGMAP0_CTRL_FL_PD           (0x601C0020U)
#define ADRV904X_ADDR_RX2_ADC_REGMAP0_CTRL_FD_PD           (0x602C000CU)
#define ADRV904X_ADDR_RX2_ADC_REGMAP0_CTRL_FL_PD           (0x602C0020U)
#define ADRV904X_ADDR_RX3_ADC_REGMAP0_CTRL_FD_PD           (0x603C000CU)
#define ADRV904X_ADDR_RX3_ADC_REGMAP0_CTRL_FL_PD           (0x603C0020U)
#define ADRV904X_ADDR_RX4_ADC_REGMAP0_CTRL_FD_PD           (0x604C000CU)
#define ADRV904X_ADDR_RX4_ADC_REGMAP0_CTRL_FL_PD           (0x604C0020U)
#define ADRV904X_ADDR_RX5_ADC_REGMAP0_CTRL_FD_PD           (0x605C000CU)
#define ADRV904X_ADDR_RX5_ADC_REGMAP0_CTRL_FL_PD           (0x605C0020U)
#define ADRV904X_ADDR_RX6_ADC_REGMAP0_CTRL_FD_PD           (0x606C000CU)
#define ADRV904X_ADDR_RX6_ADC_REGMAP0_CTRL_FL_PD           (0x606C0020U)
#define ADRV904X_ADDR_RX7_ADC_REGMAP0_CTRL_FD_PD           (0x607C000CU)
#define ADRV904X_ADDR_RX7_ADC_REGMAP0_CTRL_FL_PD           (0x607C0020U)
#define ADRV904X_ADDR_RX0_INTERLEAVER_SLOT_TABLE_0         (0x600E0D00U)
#define ADRV904X_ADDR_RX0_CDDC_DELAY_0                     (0x600E0D28U)
#define ADRV904X_ADDR_TX0_CPT_CONFIG                       (0x6083005CU)
#define ADRV904X_ADDR_TX0_CPT_STATUS                       (0x60830060U)
#define ADRV904X_ADDR_TX0_CPT_TIMESTAMP0                   (0x60830064U)
#define ADRV904X_ADDR_TX0_CPT_TIMESTAMP1                   (0x60830068U)
#define ADRV904X_ADDR_TX1_CPT_CONFIG                       (0x6093005CU)
#define ADRV904X_ADDR_TX1_CPT_STATUS                       (0x60930060U)
#define ADRV904X_ADDR_TX1_CPT_TIMESTAMP0                   (0x60930064U)
#define ADRV904X_ADDR_TX1_CPT_TIMESTAMP1                   (0x60930068U)
#define ADRV904X_ADDR_TX2_CPT_CONFIG                       (0x60A3005CU)
#define ADRV904X_ADDR_TX2_CPT_STATUS                       (0x60A30060U)
#define ADRV904X_ADDR_TX2_CPT_TIMESTAMP0                   (0x60A30064U)
#define ADRV904X_ADDR_TX2_CPT_TIMESTAMP1                   (0x60A30068U)
#define ADRV904X_ADDR_TX3_CPT_CONFIG                       (0x60B3005CU)
#define ADRV904X_ADDR_TX3_CPT_STATUS                       (0x60B30060U)
#define ADRV904X_ADDR_TX3_CPT_TIMESTAMP0                   (0x60B30064U)
#define ADRV904X_ADDR_TX3_CPT_TIMESTAMP1                   (0x60B30068U)
#define ADRV904X_ADDR_TX4_CPT_CONFIG                       (0x60C3005CU)
#define ADRV904X_ADDR_TX4_CPT_STATUS                       (0x60C30060U)
#define ADRV904X_ADDR_TX4_CPT_TIMESTAMP0                   (0x60C30064U)
#define ADRV904X_ADDR_TX4_CPT_TIMESTAMP1                   (0x60C30068U)
#define ADRV904X_ADDR_TX5_CPT_CONFIG                       (0x60D3005CU)
#define ADRV904X_ADDR_TX5_CPT_STATUS                       (0x60D30060U)
#define ADRV904X_ADDR_TX5_CPT_TIMESTAMP0                   (0x60D30064U)
#define ADRV904X_ADDR_TX5_CPT_TIMESTAMP1                   (0x60D30068U)
#define ADRV904X_ADDR_TX6_CPT_CONFIG                       (0x60E3005CU)
#define ADRV904X_ADDR_TX6_CPT_STATUS                       (0x60E30060U)
#define ADRV904X_ADDR_TX6_CPT_TIMESTAMP0                   (0x60E30064U)
#define ADRV904X_ADDR_TX6_CPT_TIMESTAMP1                   (0x60E30068U)
#define ADRV904X_ADDR_TX7_CPT_CONFIG                       (0x60F3005CU)
#define ADRV904X_ADDR_TX7_CPT_STATUS                       (0x60F30060U)
#define ADRV904X_ADDR_TX7_CPT_TIMESTAMP0                   (0x60F30064U)
#define ADRV904X_ADDR_TX7_CPT_TIMESTAMP1                   (0x60F30068U)
#define ADRV904X_ADDR_TX0_DAC_PWD_DOWN_I                   (0x6088003EU)
#define ADRV904X_ADDR_TX1_DAC_PWD_DOWN_I                   (0x6098003EU)
#define ADRV904X_ADDR_TX2_DAC_PWD_DOWN_I                   (0x60A8003EU)
#define ADRV904X_ADDR_TX3_DAC_PWD_DOWN_I                   (0x60B8003EU)
#define ADRV904X_ADDR_TX4_DAC_PWD_DOWN_I                   (0x60C8003EU)
#define ADRV904X_ADDR_TX5_DAC_PWD_DOWN_I                   (0x60D8003EU)
#define ADRV904X_ADDR_TX6_DAC_PWD_DOWN_I                   (0x60E8003EU)
#define ADRV904X_ADDR_TX7_DAC_PWD_DOWN_I                   (0x60F8003EU)
#define ADRV904X_ADDR_TX0_LB_ADC_WEST_TRM_PDN_CTRL         (0x60896446U)
#define ADRV904X_ADDR_TX1_LB_ADC_WEST_TRM_PDN_CTRL         (0x60996446U)
#define ADRV904X_ADDR_TX2_LB_ADC_WEST_TRM_PDN_CTRL         (0x60A96446U)
#define ADRV904X_ADDR_TX3_LB_ADC_WEST_TRM_PDN_CTRL         (0x60B96446U)
#define ADRV904X_ADDR_TX4_LB_ADC_WEST_TRM_PDN_CTRL         (0x60C96446U)
#define ADRV904X_ADDR_TX5_LB_ADC_WEST_TRM_PDN_CTRL         (0x60D96446U)
#define ADRV904X_ADDR_TX6_LB_ADC_WEST_TRM_PDN_CTRL         (0x60E96446U)
#define ADRV904X_ADDR_TX7_LB_ADC_WEST_TRM_PDN_CTRL         (0x60F96446U)
#define ADRV904X_ADDR_TX0_LB_ADC_EAST_TRM_PDN_CTRL         (0x608968FDU)
#define ADRV904X_ADDR_TX1_LB_ADC_EAST_TRM_PDN_CTRL         (0x609968FDU)
#define ADRV904X_ADDR_TX2_LB_ADC_EAST_TRM_PDN_CTRL         (0x60A968FDU)
#define ADRV904X_ADDR_TX3_LB_ADC_EAST_TRM_PDN_CTRL         (0x60B968FDU)
#define ADRV904X_ADDR_TX4_LB_ADC_EAST_TRM_PDN_CTRL         (0x60C968FDU)
#define ADRV904X_ADDR_TX5_LB_ADC_EAST_TRM_PDN_CTRL         (0x60D968FDU)
#define ADRV904X_ADDR_TX6_LB_ADC_EAST_TRM_PDN_CTRL         (0x60E968FDU)
#define ADRV904X_ADDR_TX7_LB_ADC_EAST_TRM_PDN_CTRL         (0x60F968FDU)
#define ADRV904X_ADDR_TX0_DEINTERLEAVER_SLOT_TABLE_0       (0x608B0D20U)
#define ADRV904X_ADDR_TX0_CDDC_DELAY_0                     (0x608B0D48U)
#define ADRV904X_ADDR_ORX0_CPT_CONFIG                      (0x610500D0U)
#define ADRV904X_ADDR_ORX0_CPT_STATUS0                     (0x610500D4U)
#define ADRV904X_ADDR_ORX1_CPT_CONFIG                      (0x611500D0U)
#define ADRV904X_ADDR_ORX1_CPT_STATUS0                     (0x611500D4U)

#include <stdint.h>

typedef struct adrv904x_CpuMemDump
{
    uint32_t cpuMemAddr;
    uint32_t cpuMemSize;
    uint8_t  cpuMemWidth;
    uint8_t  cpuMemEndianness;
} adrv904x_CpuMemDump_t;
typedef struct adrv904x_CpuRamMemDump
{
    uint32_t cpuMemAddr;
    uint32_t cpuMemSize;
    uint16_t ramId;
    uint8_t  cpuMemEndianness;
} adrv904x_CpuRamMemDump_t;

extern const adrv904x_CpuRamMemDump_t adrv904x_CpuRamMemDumpTable[24];
extern const adrv904x_CpuMemDump_t adrv904x_TelemetryMemDumpTable[1];
extern const adrv904x_CpuMemDump_t adrv904x_CpuMemDumpTable[2855];
extern const adrv904x_CpuRamMemDump_t adrv904x_DfeMemDumpTable[5];

typedef struct adrv904x_EtmDumpValues
{
    uint16_t format;
    uint32_t baseAddr;
    uint8_t  dataSize;
    uint8_t  configSize;
} adrv904x_EtmDumpValues_t;

extern const adrv904x_EtmDumpValues_t adrv904x_DfeEtmMemDumpTable[4];
#endif
