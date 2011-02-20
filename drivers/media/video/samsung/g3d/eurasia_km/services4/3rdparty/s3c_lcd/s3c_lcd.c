/*-------------------------------------------------------------
 * Filename: s3c_lcd.c
 *
 * Contents: Implemention of display kernel module bridging to LCD driver
 *
 * Abbreviations:
 *
 * Person Involved: chun gil lee
 *
 * Notes: 
 *
 * History
 *  - First created by chun gil lee, 20090205
 *
 * Copyright (c) 2009 SAMSUNG Electronics.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 --------------------------------------------------------------*/

#if defined(SUPPORT_DRI_DRM)
#include <drm/drmP.h>
#else
#include <linux/module.h>
#endif

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
//#include <asm/hardware.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"

#include "s3c_lcd.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("s3c_lcd display driver");
MODULE_SUPPORTED_DEVICE("s3c_lcd");

#define	DRVNAME	MAKESTRING(DISPLAY_CONTROLLER)

#define MAKENAME_HELPER(x, y) x ## y
#define	MAKENAME2(x, y) MAKENAME_HELPER(x, y)
#define	MAKENAME(x) MAKENAME2(DISPLAY_CONTROLLER, x)

#if !defined(SUPPORT_DRI_DRM)
MODULE_SUPPORTED_DEVICE(DRVNAME);
#endif

#if defined(SUPPORT_DRI_DRM)
int MAKENAME(_Init)(struct drm_device unref__ *dev)
#else
static int __init S3cLcdBridgeInit(void)
#endif
{
	if(init())return -1;
	return 0;
}

#if defined(SUPPORT_DRI_DRM)
void MAKENAME(_Cleanup)(struct drm_device unref__ *dev)
#else
static void __exit S3cLcdBridgeExit (void)
#endif
{
	deInit();
}

/*
 These macro calls define the initialisation and removal functions of the
 driver.  Although they are prefixed `module_', they apply when compiling
 statically as well; in both cases they define the function the kernel will
 run to start/stop the driver.
*/
#if !defined(SUPPORT_DRI_DRM)
module_init(S3cLcdBridgeInit);
module_exit(S3cLcdBridgeExit);
#endif

