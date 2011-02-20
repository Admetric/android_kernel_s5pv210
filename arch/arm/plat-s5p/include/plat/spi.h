/* linux/arch/arm/plat-s3c/include/plat/spi.h
 *
 * Copyright (C) 2009 Samsung Electronics Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __S3C64XX_PLAT_SPI_H
#define __S3C64XX_PLAT_SPI_H __FILE__

/**
 * struct s3c64xx_spi_csinfo - ChipSelect description
 * @fb_delay: Slave specific feedback delay.
 * @set_level: CS line control.
 */
struct s3c64xx_spi_csinfo {
	u8 fb_delay;
	void (*set_level)(int lvl);
};

/**
 * struct s3c64xx_spi_cntrlr_info - SPI Controller defining structure
 * @src_clk_nr: Clock source index for the CLK_CFG[SPI_CLKSEL] field.
 * @src_clk_name: Platform name of the corresponding clock.
 * @src_clk: Pointer to the source clock.
 * @num_cs: Number of CS this controller emulates.
 * @cs: Array describing each CS.
 * @cfg_gpio: Configure pins for this SPI controller.
 * @fifo_lvl_mask: All tx fifo_lvl fields start at offset-6
 * @rx_lvl_offset: Depends on tx fifo_lvl field and bus number
 * @high_speed: If the controller supports HIGH_SPEED_EN bit
 */
struct s3c64xx_spi_cntrlr_info {
	int src_clk_nr;
	char *src_clk_name;
	struct clk *src_clk;

	int num_cs;
	struct s3c64xx_spi_csinfo *cs;

	int (*cfg_gpio)(struct platform_device *pdev);

	/* Following two fields are for future compatibility */
	int fifo_lvl_mask;
	int rx_lvl_offset;
	int high_speed;
};

/**
 * s3c64xx_spi_set_info - SPI Controller configure callback by the board
 *				initialization code.
 * @cntrlr: SPI controller number the configuration is for.
 * @src_clk_nr: Clock the SPI controller is to use to generate SPI clocks.
 * @cs: Pointer to the array of CS descriptions.
 * @num_cs: Number of elements in the 'cs' array.
 */
extern void s3c64xx_spi_set_info(int cntrlr, int src_clk_nr, int num_cs);

#endif /* __S3C64XX_PLAT_SPI_H */
