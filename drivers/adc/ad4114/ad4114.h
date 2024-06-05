#ifndef __AD4114_H__
#define __AD4114_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* Total Number of Setups */
#define AD4114_MAX_SETUPS   8
/* Maximum number of channels */
#define AD4114_MAX_CHANNELS 16

/* AD4114 Register Map */
#define AD4114_COMMS_REG        0x00
#define AD4114_STATUS_REG       0x00
#define AD4114_ADCMODE_REG      0x01
#define AD4114_IFMODE_REG       0x02
#define AD4114_REGCHECK_REG     0x03
#define AD4114_DATA_REG         0x04
#define AD4114_GPIOCON_REG      0x06
#define AD4114_ID_REG           0x07
#define AD4114_CH0_REG          0x10
#define AD4114_CH1_REG          0x11
#define AD4114_CH2_REG          0x12
#define AD4114_CH3_REG          0x13
#define AD4114_CH4_REG          0x14
#define AD4114_CH5_REG          0x15
#define AD4114_CH6_REG          0x16
#define AD4114_CH7_REG          0x17
#define AD4114_CH8_REG          0x18
#define AD4114_CH9_REG          0x19
#define AD4114_CH10_REG         0x1A
#define AD4114_CH11_REG         0x1B
#define AD4114_CH12_REG         0x1C
#define AD4114_CH13_REG         0x1D
#define AD4114_CH14_REG         0x1E
#define AD4114_CH15_REG         0x1F
#define AD4114_SETUPCON0_REG    0x20
#define AD4114_SETUPCON1_REG    0x21
#define AD4114_SETUPCON2_REG    0x22
#define AD4114_SETUPCON3_REG    0x23
#define AD4114_SETUPCON4_REG    0x24
#define AD4114_SETUPCON5_REG    0x25
#define AD4114_SETUPCON6_REG    0x26
#define AD4114_SETUPCON7_REG    0x27
#define AD4114_FILTCON0_REG     0x28
#define AD4114_FILTCON1_REG     0x29
#define AD4114_FILTCON2_REG     0x2A
#define AD4114_FILTCON3_REG     0x2B
#define AD4114_FILTCON4_REG     0x2C
#define AD4114_FILTCON5_REG     0x2D
#define AD4114_FILTCON6_REG     0x2E
#define AD4114_FILTCON7_REG     0x2F
#define AD4114_OFFSET0_REG      0x30
#define AD4114_OFFSET1_REG      0x31
#define AD4114_OFFSET2_REG      0x32
#define AD4114_OFFSET3_REG      0x33
#define AD4114_OFFSET4_REG      0x34
#define AD4114_OFFSET5_REG      0x35
#define AD4114_OFFSET6_REG      0x36
#define AD4114_OFFSET7_REG      0x37
#define AD4114_GAIN0_REG        0x38
#define AD4114_GAIN1_REG        0x39
#define AD4114_GAIN2_REG        0x3A
#define AD4114_GAIN3_REG        0x3B
#define AD4114_GAIN4_REG        0x3C
#define AD4114_GAIN5_REG        0x3D
#define AD4114_GAIN6_REG        0x3E
#define AD4114_GAIN7_REG        0x3F

/* Communication Register bits */
#define AD4114_COMMS_REG_WEN   (0 << 7)
#define AD4114_COMMS_REG_WR    (0 << 6)
#define AD4114_COMMS_REG_RD    (1 << 6)
#define AD4114_COMMS_REG_RA(x) ((x) & 0b1111)

/* Status Register bits */
#define AD4114_STATUS_REG_RDY        (1 << 7)
#define AD4114_STATUS_REG_ADC_ERROR  (1 << 6)
#define AD4114_STATUS_REG_CRC_ERROR  (1 << 5)
#define AD4114_STATUS_REG_REG_ERROR  (1 << 4)
#define AD4114_STATUS_REG_CHANNEL(x) ((x) & 0b1111)

/* ADCMODE Register bits */
#define AD4114_ADCMODE_REG_REF_EN      (1 << 15)
#define AD4114_ADCMODE_REG_SING_CYC    (1 << 13)
#define AD4114_ADCMODE_REG_DELAY(x)    (((x) & 0b111) << 8)
#define AD4114_ADCMODE_REG_MODE(x)     (((x) & 0b111) << 4)
#define AD4114_ADCMODE_REG_CLOCKSEL(x) (((x) & 0b11) << 2)

/* IFMODE Register bits */
#define AD4114_IFMODE_REG_ALT_SYNC   (1 << 12)
#define AD4114_IFMODE_REG_IOSTRENGTH (1 << 11)
#define AD4114_IFMODE_REG_DOUT_RESET (1 << 8)
#define AD4114_IFMODE_REG_CONTREAD   (1 << 7)
#define AD4114_IFMODE_REG_DATA_STAT  (1 << 6)
#define AD4114_IFMODE_REG_REG_CHECK  (1 << 5)
#define AD4114_IFMODE_REG_CRC_EN(x)  (((x) & 0b11) << 2)
#define AD4114_IFMODE_REG_WL16       (1 << 0)

/* GPIOCON Register bits */
#define AD4114_GPIOCON_REG_OP_EN2_3  (1 << 13)
#define AD4114_GPIOCON_REG_MUX_IO    (1 << 12)
#define AD4114_GPIOCON_REG_SYNC_EN   (1 << 11)
#define AD4114_GPIOCON_REG_ERR_EN(x) (((x) & 0b11) << 9)
#define AD4114_GPIOCON_REG_ERR_DAT   (1 << 8)
#define AD4114_GPIOCON_REG_GP_DATA3  (1 << 7)
#define AD4114_GPIOCON_REG_GP_DATA2  (1 << 6)
#define AD4114_GPIOCON_REG_IP_EN1    (1 << 5)
#define AD4114_GPIOCON_REG_IP_EN0    (1 << 4)
#define AD4114_GPIOCON_REG_OP_EN1    (1 << 3)
#define AD4114_GPIOCON_REG_OP_EN0    (1 << 2)
#define AD4114_GPIOCON_REG_GP_DATA1  (1 << 1)
#define AD4114_GPIOCON_REG_GP_DATA0  (1 << 0)

/* CH0-15 Register bits */
#define AD4114_CH_REG_EN           (1 << 15)
#define AD4114_CH_REG_SETUP_SEL(x) (((x) & 0b111) << 12)
#define AD4114_CH_REG_INPUT(x)     (((x) & 0b1111111111) << 0)

/* SETUPCON0-7 Register bits */
#define AD4114_SETUPCON_REG_BI_UNIPOLAR (1 << 12)
#define AD4114_SETUPCON_REG_REFBUFP     (1 << 11)
#define AD4114_SETUPCON_REG_REFBUFM     (1 << 10)
#define AD4114_SETUPCON_REG_INBUF(x)    (((x) & 0b11) << 8)
#define AD4114_SETUPCON_REG_REF_SEL(x)  (((x) & 0b11) << 4)

/* FILTCON0-7 Register bits */
#define AD4114_FILTCON_REG_SINC3_MAP  (1 << 15)
#define AD4114_FILTCON_REG_ENHFILTEN  (1 << 11)
#define AD4114_FILTCON_REG_ENHFILT(x) (((x) & 0b111) << 8)
#define AD4114_FILTCON_REG_ORDER(x)   (((x) & 0b11) << 5)
#define AD4114_FILTCON_REG_ODR(x)     (((x) & 0b11111) << 0)

#define AD4114_CRC8_POLYNOMIAL_REPRESENTATION 0x07 /* x8 + x2 + x + 1 */
// TODO(ioan): AD4114_..._REG_..._MSK NO_OS_GENMASK(hi_inclusive, lo_inclusive)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @enum ad4114_mode
 * @brief ADC Modes of Operation
*/
enum ad4114_mode {
    AD4114_CONTINUOUS,
    AD4114_SINGLE,
    AD4114_STANDBY,
    AD4114_POWER_DOWN,
    AD4114_INTERNAL_OFFSET_CALIBRATION,
    AD4114_INTERNAL_GAIN_CALIBRATION,
    AD4114_SYSTEM_OFFSET_CALIBRATION,
    AD4114_SYSTEM_GAIN_CALIBRATION,
    ADC_MAX_MODES
};

/**
 * @enum ad4114_input
 * @brief ADC input sources for each channel.
*/
enum ad4114_input {
    AD4114_VIN0_VIN1    = 0b0000000001,
    AD4114_VIN0_VINCOM  = 0b0000010000,
    AD4114_VIN1_VIN0    = 0b0000100000,
    AD4114_VIN1_VINCOM  = 0b0000110000,
    AD4114_VIN2_VIN3    = 0b0001000011,
    AD4114_VIN2_VINCOM  = 0b0001010000,
    AD4114_VIN3_VIN2    = 0b0001100010,
    AD4114_VIN3_VINCOM  = 0b0001110000,
    AD4114_VIN4_VIN5    = 0b0010000101,
    AD4114_VIN4_VINCOM  = 0b0010010000,
    AD4114_VIN5_VIN4    = 0b0010100100,
    AD4114_VIN5_VINCOM  = 0b0010110000,
    AD4114_VIN6_VIN7    = 0b0011000111,
    AD4114_VIN6_VINCOM  = 0b0011010000,
    AD4114_VIN7_VIN6    = 0b0011100110,
    AD4114_VIN7_VINCOM  = 0b0011110000,
    AD4114_VIN8_VIN9    = 0b0100001001,
    AD4114_VIN8_VINCOM  = 0b0100010000,
    AD4114_VIN9_VIN8    = 0b0100101000,
    AD4114_VIN9_VINCOM  = 0b0100110000,
    AD4114_VIN10_VIN11  = 0b0101001011,
    AD4114_VIN10_VINCOM = 0b0101010000,
    AD4114_VIN11_VIN10  = 0b0101101010,
    AD4114_VIN11_VINCOM = 0b0101110000,
    AD4114_VIN12_VIN13  = 0b0110001101,
    AD4114_VIN12_VINCOM = 0b0110010000,
    AD4114_VIN13_VIN12  = 0b0110101100,
    AD4114_VIN13_VINCOM = 0b0110110000,
    AD4114_VIN14_VIN15  = 0b0111001111,
    AD4114_VIN14_VINCOM = 0b0111010000,
    AD4114_VIN15_VIN14  = 0b0111101110,
    AD4114_VIN15_VINCOM = 0b0111110000,
    AD4114_REMPERATURE  = 0b1000110010,
    AD4114_REFERENCE    = 0b1010110110
};

/* Device register info */
struct ad4114_st_reg {
    uint8_t addr;
    uint8_t size;
    int32_t value;
};

/* AD4114 registers list */
enum ad4114_registers {
    AD4114_Status,
    AD4114_ADCMODE,
    AD4114_IFMODE,
    AD4114_REGCHECK,
    AD4114_Data,
    AD4114_GPIOCON,
    AD4114_ID,
    AD4114_CH0,
    AD4114_CH1,
    AD4114_CH2,
    AD4114_CH3,
    AD4114_CH4,
    AD4114_CH5,
    AD4114_CH6,
    AD4114_CH7,
    AD4114_CH8,
    AD4114_CH9,
    AD4114_CH10,
    AD4114_CH11,
    AD4114_CH12,
    AD4114_CH13,
    AD4114_CH14,
    AD4114_CH15,
    AD4114_SETUPCON0,
    AD4114_SETUPCON1,
    AD4114_SETUPCON2,
    AD4114_SETUPCON3,
    AD4114_SETUPCON4,
    AD4114_SETUPCON5,
    AD4114_SETUPCON6,
    AD4114_SETUPCON7,
    AD4114_FILTCON0,
    AD4114_FILTCON1,
    AD4114_FILTCON2,
    AD4114_FILTCON3,
    AD4114_FILTCON4,
    AD4114_FILTCON5,
    AD4114_FILTCON6,
    AD4114_FILTCON7,
    AD4114_OFFSET0,
    AD4114_OFFSET1,
    AD4114_OFFSET2,
    AD4114_OFFSET3,
    AD4114_OFFSET4,
    AD4114_OFFSET5,
    AD4114_OFFSET6,
    AD4114_OFFSET7,
    AD4114_GAIN0,
    AD4114_GAIN1,
    AD4114_GAIN2,
    AD4114_GAIN3,
    AD4114_GAIN4,
    AD4114_GAIN5,
    AD4114_GAIN6,
    AD4114_GAIN7,
    AD4114_REG_NO
};

struct ad4114_dev {
    /* SPI */
    struct no_os_spi_desc *spi_desc;
};

struct ad4114_init_param {
    /* SPI */
    struct no_os_spi_init_param *spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

int32_t ad4114_read_reg(struct ad4114_dev *dev, struct ad4114_st_reg *reg);

/* Resets the device. */
int32_t ad4114_reset(struct ad4114_dev *dev);

/* Initializes the AD4114. */
int32_t ad4114_setup(struct ad4114_dev **dev,
		     struct ad4114_init_param *init_param);

/* Free the resources allocated by ad4114_setup(). */
int32_t ad4114_remove(struct ad4114_dev *dev);

#endif /* __AD4114_H__ */
