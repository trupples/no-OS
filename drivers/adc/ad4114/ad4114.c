#include <errno.h>
#include "ad4114.h"

// TODO(ioan): proper function annotations

int32_t ad4114_read_reg(struct ad4114_dev *dev, struct ad4114_st_reg *reg)
{
    if(!dev || !reg)
        return -EINVAL;
    
    uint8_t buf[5] = { 0 };

    buf[0] = AD4114_COMMS_REG_WEN | AD4114_COMMS_REG_RD | AD4114_COMMS_REG_RA(reg->addr);

    // TODO(ioan): DATA_STAT
    // TODO(ioan): CRC
    int transfer_len = 1 + reg->size; //+ (reg->addr == AD4114_DATA_REG && dev->data_stat ? 1 : 0) + (dev->crc_en != AD4114_CRC_DISABLED ? 1 : 0);

    int ret = no_os_spi_write_and_read(dev->spi_desc, buf, transfer_len);
    if(ret)
        return ret;

    reg->value = 0;
    for(int i = 1; i <= reg->size; i++)
    {
        reg->value = (reg->value << 8) | buf[i];
    }

    return 0;
}

/* Resets the device. */
int32_t ad4114_reset(struct ad4114_dev *dev)
{
    // 64 "1" bits
    uint8_t wr_buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    if(!dev)
        return -EINVAL;
    
    int ret = no_os_spi_write_and_read(dev->spi_desc, wr_buf, 8);
    if(ret)
        return ret;

    // TODO(ioan): reset software registers to default values
    
    no_os_udelay(500);

    return 0;
}

/* Initializes the AD4114. */
int32_t ad4114_setup(struct ad4114_dev **ad4114_dev, struct ad4114_init_param *init_param)
{
    struct ad4114_dev *dev;
    dev = (struct ad4114_dev *) no_os_malloc(sizeof(*dev));
    if(!dev)
        return -ENOMEM;
    
    int ret = no_os_spi_init(&dev->spi_desc, init_param->spi_init);
    if(ret)
        goto error_dev;

    ret = ad4114_reset(dev);
    if(ret)
        goto error_spi;

    // TODO(ioan): set a fully known config

error_spi:
    no_os_spi_remove(dev->spi_desc);
error_dev:
    free(dev);

    return ret;    
}

/* Free the resources allocated by ad4114_setup(). */
int32_t ad4114_remove(struct ad4114_dev *dev)
{
    int ret = no_os_spi_remove(dev->spi_desc);
    if(ret)
        return ret;

    free(dev);

    return 0;
}
