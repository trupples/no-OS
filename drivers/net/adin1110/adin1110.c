#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "no_os_spi.h"
#include "adin1110.h"
#include "no_os_util.h"
#include "no_os_delay.h"

int adin1110_format_frame(struct adin1110_sk_buff *sk_buff, uint8_t *frame)
{

}

int adin1110_reg_write(struct adin1110_desc *desc, uint16_t addr, uint32_t data)
{
	struct no_os_spi_msg xfer = {
		.tx_buff = desc->tx_buff,
		.bytes_number = 6,
		.cs_change = 1,
	};

	/** The address is 13 bit wide */
	addr &= NO_OS_GENMASK(12, 0);
	addr |= ADIN1110_CD_MASK;
	addr |= ADIN1110_RW_MASK;
	no_os_put_unaligned_be16(addr, desc->tx_buff);
	no_os_put_unaligned_be32(data, &desc->tx_buff[2]);

	return no_os_spi_transfer(desc->comm_desc, &xfer, 1);
}

int adin1110_reg_read(struct adin1110_desc *desc, uint16_t addr, uint32_t *data)
{
	struct no_os_spi_msg xfer = {
		.tx_buff = desc->tx_buff,
		.rx_buff = desc->rx_buff,
		.bytes_number = ADIN1110_CTRL_FRAME_SIZE,
		.cs_change = 1,
	};
	int ret;

	memset(desc->tx_buff, 0, ADIN1110_CTRL_FRAME_SIZE);
	no_os_put_unaligned_be16(addr, &desc->tx_buff[0]);
	desc->tx_buff[0] |= NO_OS_BIT(7);
	desc->tx_buff[2] = 0x00;

	ret = no_os_spi_transfer(desc->comm_desc, &xfer, 1);
	if (ret)
		return ret;

	*data = no_os_get_unaligned_be32(&desc->rx_buff[3]);

	return 0;
}

int adin1110_reg_update(struct adin1110_desc *desc, uint16_t addr,
			uint32_t mask, uint32_t data)
{
	uint32_t val;
	int ret;

	ret = adin1110_reg_read(desc, addr, &val);
	if (ret)
		return ret;

	val &= ~mask;
	val |= mask & data;

	return adin1110_reg_write(desc, addr, val);
}

int adin1110_write_fifo(struct adin1110_desc *desc, struct adin1110_sk_buff *sk_buff)
{
	int padding = 0;
	int frame_len;
	int ret;

	struct no_os_spi_msg xfer = {
		.tx_buff = desc->data,
		.cs_change = 1,
	};

	if (sk_buff->buff_len + ADIN1110_FCS_LEN < 64)
		padding = 64 - (sk_buff->buff_len + ADIN1110_FCS_LEN);

	frame_len = sk_buff->buff_len + padding + ADIN1110_FRAME_HEADER_LEN;
	
	/** Align the frame length to 4 bytes */
	frame_len = no_os_round_up(frame_len, 4);

	ret = adin1110_reg_write(desc, ADIN1110_TX_FSIZE, frame_len);
	if (ret)
		return ret;

	memset(desc->data, 0, frame_len + ADIN1110_WR_HEADER_LEN);

	no_os_put_unaligned_be16(ADIN1110_TX, &desc->data[0]);
	desc->data[0] |= ADIN1110_SPI_CD | ADIN1110_SPI_RW;
	xfer.bytes_number = frame_len + ADIN1110_WR_HEADER_LEN;

	//memcpy(desc->data[3], )

	return no_os_spi_transfer(desc->comm_desc, &xfer, 1);
}

int adin1110_read_fifo(struct adin1110_desc *desc, struct adin1110_sk_buff *sk_buff)
{
	uint32_t frame_size;
	int ret;

	ret = adin1110_reg_read(desc, ADIN1110_RX_FSIZE, &frame_size);
	if (ret)
		return ret;

	ret = adin1110_reg_read(desc, 0x0D, &frame_size);
	ret = adin1110_reg_read(desc, ADIN1110_RX_FRM_CNT, &frame_size);
	ret = adin1110_reg_read(desc, 0x08, &frame_size);
	ret = adin1110_reg_read(desc, 0x09, &frame_size);
	ret = adin1110_reg_read(desc, ADIN1110_RX_FRM_CNT, &frame_size);
	ret = adin1110_reg_read(desc, ADIN1110_RX_FRM_CNT, &frame_size);
	ret = adin1110_reg_read(desc, ADIN1110_RX_FRM_CNT, &frame_size);
	ret = adin1110_reg_read(desc, ADIN1110_RX_FRM_CNT, &frame_size);

	ret = adin1110_reg_read(desc, ADIN1110_RX_CRC_ERR_CNT, &frame_size);
	ret = adin1110_reg_read(desc, ADIN1110_RX_PHY_ERR_CNT, &frame_size);
	ret = adin1110_reg_read(desc, ADIN1110_RX_ALGN_ERR_CNT, &frame_size);
	ret = adin1110_reg_read(desc, ADIN1110_RX_LS_ERR_CNT, &frame_size);
}

static int adin1110_mac_reset(struct adin1110_desc *desc)
{
	uint32_t val;
	int ret;

	ret = adin1110_reg_write(desc, ADIN1110_SOFT_RST_REG, ADIN1110_SWRESET_KEY1);
	if (ret)
		return ret;

	ret = adin1110_reg_write(desc, ADIN1110_SOFT_RST_REG, ADIN1110_SWRESET_KEY2);
	if (ret)
		return ret;

	no_os_udelay(1);

	ret = adin1110_reg_write(desc, ADIN1110_SOFT_RST_REG, ADIN1110_SWRELEASE_KEY1);
	if (ret)
		return ret;

	ret = adin1110_reg_write(desc, ADIN1110_SOFT_RST_REG, ADIN1110_SWRELEASE_KEY2);
	if (ret)
		return ret;

	ret = adin1110_reg_read(desc, ADIN1110_MAC_RST_STATUS_REG, &val);
	if (ret)
		return ret;

	if (!val)
		return -EBUSY;

	return 0;
}

static int adin1110_check_reset(struct adin1110_desc *desc)
{
	uint32_t reg_val;
	int ret;

	ret = adin1110_reg_read(desc, ADIN1110_STATUS0, &reg_val);
	if (ret)
		return ret;

	reg_val = no_os_field_get(ADIN1110_RESETC_MASK, reg_val);
	if (!reg_val)
		return -EBUSY;

	ret = adin1110_reg_read(desc, ADIN1110_CONFIG1, &reg_val);
	if (ret)
		return ret;

	/** 
	 * Signal that the MAC configuration is complete and the link
	 * autonegotiation may start.
	 */
	ret = adin1110_reg_update(desc, ADIN1110_CONFIG1,
			    	  ADIN1110_CONFIG1_SYNC, ADIN1110_CONFIG1_SYNC);
	if (ret)
		return ret;

	return 0;
}

static int adin1110_phy_reset(struct adin1110_desc *desc)
{
	uint32_t phy_id;
	int ret;

	ret = no_os_gpio_set_value(desc->reset_gpio, 0);
	if (ret)
		return ret;
	
	no_os_mdelay(10);

	ret = no_os_gpio_set_value(desc->reset_gpio, 1);
	if (ret)
		return ret;
	
	no_os_mdelay(200);

	ret = adin1110_reg_read(desc, ADIN1110_PHY_ID_REG, &phy_id);
	if (ret)
		return ret;

	if (phy_id != ADIN1110_PHY_ID)
		return -EINVAL;

	return 0;
}

int adin1110_init(struct adin1110_desc **desc,
                  struct adin1110_init_param *param)
{
	struct adin1110_desc *descriptor;
	uint32_t reg_val;
	uint32_t phy_id;
	int ret;

	descriptor = calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = no_os_spi_init(&descriptor->comm_desc, &param->comm_param);
	if (ret)
		goto err;

	descriptor->spi_oa = param->spi_oa;
	strncpy(descriptor->mac_address, param->mac_address, ADIN1110_MAC_LEN);

	ret = no_os_gpio_get(&descriptor->reset_gpio, &param->reset_param);
	if (ret)
		return ret;

	// ret = adin1110_reg_update(descriptor, 0x03, 1, 1);
	// if (ret)
	// 	return ret;

	ret = adin1110_phy_reset(descriptor);
	if (ret)
		return ret;

	ret = adin1110_check_reset(descriptor);
	if (ret)
		return ret;

	ret = adin1110_reg_read(descriptor, ADIN1110_STATUS1, &reg_val);
	if (ret)
		return ret;

	reg_val = no_os_field_get(NO_OS_BIT(0), reg_val);

	ret = adin1110_reg_read(descriptor, 0x32, &reg_val);
	if (ret)
		return ret;

	*desc = descriptor;

	return 0;

err:
	free(descriptor);

	return ret;
}

int adin1110_remove(struct adin1110_desc *desc)
{
	int ret;

	if (!desc)
		return -EINVAL;

	ret = no_os_spi_remove(desc->comm_desc);
	if (ret)
		return ret;

	free(desc);

	return 0;
}