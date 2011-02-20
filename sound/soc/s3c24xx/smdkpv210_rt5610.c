/*
 * smdkpv100_ac97.c  --  SoC audio for smdkpv210
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/io.h>

#include "../codecs/rt5610.h"
#include "s3c-pcm.h"
#include "s3c-ac97.h"
#include "s3c-dma.h"

static struct snd_soc_dai_link s3c_dai_link[] = {
	{
		.name = "AC97",
		.stream_name = "AC97 HiFi",
		.cpu_dai = &s3c_ac97_dai[0],
		.codec_dai = &rt5610_dai[0],
	},
};

static struct snd_soc_card smdkpv210_rt5610_card = {
	.name = "smdkpv210 rt5610",
	.dai_link = s3c_dai_link,
	.num_links = ARRAY_SIZE(s3c_dai_link),
	.platform = &s3c24xx_soc_platform,
};

static struct snd_soc_device smdkpv210_rt5610_snd_soc_device = {
	.card = &smdkpv210_rt5610_card, 
	.codec_dev = &soc_codec_dev_rt5610,
};

static struct platform_device *smdkpv210_snd_ac97_device;

static int __init smdkpv210_rt5610_init(void)
{
	int ret;

	snd_soc_register_dai(&s3c_ac97_dai[0]);
	smdkpv210_snd_ac97_device = platform_device_alloc("soc-audio", -1);
	if (!smdkpv210_snd_ac97_device)
		return -ENOMEM;

	platform_set_drvdata(smdkpv210_snd_ac97_device,
				&smdkpv210_rt5610_snd_soc_device);
	smdkpv210_rt5610_snd_soc_device.dev = &smdkpv210_snd_ac97_device->dev;
	ret = platform_device_add(smdkpv210_snd_ac97_device);

	if (ret) {
		printk("%s: platform_device_add failed\n", __func__);
		platform_device_put(smdkpv210_snd_ac97_device);
	}

	return ret;
}

static void __exit smdkpv210_rt5610_exit(void)
{
	platform_device_unregister(smdkpv210_snd_ac97_device);
}

module_init(smdkpv210_rt5610_init);
module_exit(smdkpv210_rt5610_exit);

/* Module information */
MODULE_AUTHOR("nickmit.zheng@gmail.com");
MODULE_DESCRIPTION("ALSA SoC AC97 SMDKPV210");
MODULE_LICENSE("GPL");
