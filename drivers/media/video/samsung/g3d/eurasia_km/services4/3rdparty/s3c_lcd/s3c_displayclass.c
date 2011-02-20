/*-------------------------------------------------------------
 * Filename: s3c_displayclass.c
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <asm/hardirq.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/memory.h>
//#include <asm/hardware.h>
#include <plat/regs-fb.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"

#include "s3c_lcd.h"


#define USE_IOREMAP
#define S3C_MAX_BACKBUFFERRS 	2
#define USE_VSYNC_INTERRUPT

//take default window config from kernel config
#ifdef CONFIG_FB_S3C_DEFAULT_WINDOW
#define LCD_IS_PRESENT
#define FB_NUM CONFIG_FB_S3C_DEFAULT_WINDOW
#else
#define FB_NUM 0
#endif

#ifdef LCD_IS_PRESENT
extern int s3cfb_direct_ioctl(int id, unsigned int cmd, unsigned long arg);
#else
extern int s5ptvfb_direct_ioctl(int id, unsigned int cmd, unsigned long arg);
#endif

#ifdef USE_IOREMAP

static volatile unsigned int regs;
static volatile unsigned int * LCDControllerBase = NULL;

#else

#define S3CFB_SET_VSYNC_INT		_IOW ('F', 206, u32)
#define S3CFB_SET_WIN_ADDR		_IOW ('F', 308, unsigned long)
#define S3CFB_GET_VSYNC_INT_STATUS	_IOR ('F', 207, u32)

#endif

#define S3C_DISPLAY_FORMAT_NUM 1
#define S3C_DISPLAY_DIM_NUM 1

#define VSYCN_IRQ 0x61


#define S3C_NUM_TOTAL_BUFFER (S3C_MAX_BACKBUFFERRS+1)

#define DC_S3C_LCD_COMMAND_COUNT 1

#ifndef UNREFERENCED_PARAMETER
#define	UNREFERENCED_PARAMETER(param) (param) = (param)
#endif

#define STRIDE_ALIGNMENT 32
#define STRIDE_ALIGN(n) ((n + (STRIDE_ALIGNMENT - 1)) & (~(STRIDE_ALIGNMENT - 1)))

typedef struct S3C_FRAME_BUFFER_TAG
{
	IMG_CPU_VIRTADDR bufferVAddr;
	IMG_SYS_PHYADDR bufferPAddr;
	IMG_UINT32 byteSize;
	IMG_UINT32 byteSizeLCDDisplay;
}S3C_FRAME_BUFFER;

typedef void *		 S3C_HANDLE;

typedef enum tag_s3c_bool
{
	S3C_FALSE = 0,
	S3C_TRUE  = 1,
	
} S3C_BOOL, *S3C_PBOOL;

typedef struct S3C_SWAPCHAIN_TAG
{
	/* number of buffers in swapchain */
	unsigned long   ulBufferCount;
	/* list of buffers in the swapchain */
	S3C_FRAME_BUFFER  	*psBuffer;
	
}S3C_SWAPCHAIN;

/* flip item structure used for queuing of flips */
typedef struct S3C_VSYNC_FLIP_ITEM_TAG
{
	/*
		command complete cookie to be passed to services
		command complete callback function
	*/
	S3C_HANDLE		  hCmdComplete;

	S3C_FRAME_BUFFER	*psFb;
	/* swap interval between flips */
	unsigned long	  ulSwapInterval;
	/* is this item valid? */
	S3C_BOOL		  bValid;
	/* has this item been flipped? */
	S3C_BOOL		  bFlipped;
	/* has the flip cmd completed? */
	S3C_BOOL		  bCmdCompleted;

} S3C_VSYNC_FLIP_ITEM;

typedef struct S3C_LCD_DEVINFO_TAG
{
	IMG_UINT32 						ui32DisplayID;
	DISPLAY_INFO 					sDisplayInfo;

	// sys surface info
	S3C_FRAME_BUFFER				sSysBuffer;

	// number of supported format
	IMG_UINT32 						ui32NumFormats;

	// list of supported display format
	DISPLAY_FORMAT 					asDisplayForamtList[S3C_DISPLAY_FORMAT_NUM];

	IMG_UINT32 						ui32NumDims;
	DISPLAY_DIMS					asDisplayDimList[S3C_DISPLAY_DIM_NUM];

	// jump table into pvr services
	PVRSRV_DC_DISP2SRV_KMJTABLE 	sPVRJTable;

	// jump table into DC
	PVRSRV_DC_SRV2DISP_KMJTABLE 	sDCJTable;

	// backbuffer info
	S3C_FRAME_BUFFER				asBackBuffers[S3C_MAX_BACKBUFFERRS];


	S3C_SWAPCHAIN					*psSwapChain;

	/* set of vsync flip items - enough for 1 outstanding flip per back buffer */
	S3C_VSYNC_FLIP_ITEM				asVSyncFlips[S3C_NUM_TOTAL_BUFFER];
	/* insert index for the internal queue of flip items */
	unsigned long					ulInsertIndex;
	
	/* remove index for the internal queue of flip items */
	unsigned long					ulRemoveIndex;
	S3C_BOOL						bFlushCommands;
	unsigned long					uPreviousVsyncInterruptStatus;


}S3C_LCD_DEVINFO;

static S3C_LCD_DEVINFO *g_psLCDInfo = NULL;

extern IMG_BOOL IMG_IMPORT PVRGetDisplayClassJTable(PVRSRV_DC_DISP2SRV_KMJTABLE *psJTable);

static void AdvanceFlipIndex(S3C_LCD_DEVINFO *psDevInfo,
							 unsigned long	*pulIndex)
{
	unsigned long	ulMaxFlipIndex;

	ulMaxFlipIndex = psDevInfo->psSwapChain->ulBufferCount - 1;
	if (ulMaxFlipIndex >= S3C_NUM_TOTAL_BUFFER)
	{
		ulMaxFlipIndex = S3C_NUM_TOTAL_BUFFER-1;
	}

	(*pulIndex)++;

	if (*pulIndex > ulMaxFlipIndex )
	{
		*pulIndex = 0;
	}
}
static IMG_VOID ResetVSyncFlipItems(S3C_LCD_DEVINFO* psDevInfo)
{
	unsigned long i;

	psDevInfo->ulInsertIndex = 0;
	psDevInfo->ulRemoveIndex = 0;

	for(i=0; i < S3C_NUM_TOTAL_BUFFER; i++)
	{
		psDevInfo->asVSyncFlips[i].bValid = S3C_FALSE;
		psDevInfo->asVSyncFlips[i].bFlipped = S3C_FALSE;
		psDevInfo->asVSyncFlips[i].bCmdCompleted = S3C_FALSE;
	}
}


static IMG_VOID S3C_Clear_interrupt(void)
{
	u32 cfg = 0;

	cfg = readl(regs + S3C_VIDINTCON1);

	if (cfg & S3C_VIDINTCON1_INTFIFOPEND)
		printk("fifo underrun occur\n");

	cfg |= (S3C_VIDINTCON1_INTVPPEND | S3C_VIDINTCON1_INTI80PEND |
		S3C_VIDINTCON1_INTFRMPEND | S3C_VIDINTCON1_INTFIFOPEND);

	writel(cfg, regs + S3C_VIDINTCON1);
}


static IMG_VOID S3C_DisableVsyncInterrupt(void)
{
	/* Disable Vsync ISR */
#ifdef USE_IOREMAP
	
	unsigned int cfg = 0;
	
	cfg = readl(regs + S3C_VIDINTCON0);
	cfg &= ~S3C_VIDINTCON0_FRAMESEL0_MASK;
		
	cfg &= ~S3C_VIDINTCON0_FRAMESEL0_VSYNC;

	writel(cfg, regs + S3C_VIDINTCON0);

#else	
#ifdef LCD_IS_PRESENT
	s3cfb_direct_ioctl(FB_NUM, S3CFB_SET_VSYNC_INT, 0);
#endif
#endif
}

static IMG_VOID S3C_EnableVsyncInterrupt(void)
{
	/* Enable Vsync ISR */
#ifdef USE_IOREMAP

	unsigned int cfg = 0;
	
	cfg = readl(regs + S3C_VIDINTCON0);
	cfg &= ~(S3C_VIDINTCON0_INTFRMEN_ENABLE | S3C_VIDINTCON0_INT_ENABLE);

	cfg |= (S3C_VIDINTCON0_INTFRMEN_ENABLE |
		S3C_VIDINTCON0_INT_ENABLE);
	
	writel(cfg, regs + S3C_VIDINTCON0);
#else
#ifdef LCD_IS_PRESENT
	s3cfb_direct_ioctl(FB_NUM, S3CFB_SET_VSYNC_INT, 1);
#endif
#endif
}

static IMG_VOID S3C_Flip(S3C_LCD_DEVINFO  *psDevInfo,
					   S3C_FRAME_BUFFER *fb)
{
#ifdef USE_IOREMAP

	LCDControllerBase[0xC/4] =(1<<11);

#if (FB_NUM == 0)
	LCDControllerBase[0xA0/4] = fb->bufferPAddr.uiAddr;
	LCDControllerBase[0xD0/4] = (fb->bufferPAddr.uiAddr+fb->byteSizeLCDDisplay)&0xffffffff;
#elif (FB_NUM ==1)
	LCDControllerBase[0xA8/4] = fb->bufferPAddr.uiAddr;
	LCDControllerBase[0xD8/4] = (fb->bufferPAddr.uiAddr+fb->byteSizeLCDDisplay)&0xffffffff;
#elif (FB_NUM ==2)
	LCDControllerBase[0xB0/4] = fb->bufferPAddr.uiAddr;
	LCDControllerBase[0xE0/4] = (fb->bufferPAddr.uiAddr+fb->byteSizeLCDDisplay)&0xffffffff;
#elif (FB_NUM ==3)
	LCDControllerBase[0xB8/4] = fb->bufferPAddr.uiAddr;
	LCDControllerBase[0xE8/4] = (fb->bufferPAddr.uiAddr+fb->byteSizeLCDDisplay)&0xffffffff;
#endif

	LCDControllerBase[0xC/4] = 0;
	//printk(KERN_ERR"S3C_Flip: 0x%x\n", (unsigned int)fb->bufferPAddr.uiAddr);
#else
#ifdef LCD_IS_PRESENT
	s3cfb_direct_ioctl(FB_NUM, S3CFB_SET_WIN_ADDR, (unsigned long)fb->bufferPAddr.uiAddr);
#endif
#endif
}
static void FlushInternalVSyncQueue(S3C_LCD_DEVINFO*psDevInfo)
{
	S3C_VSYNC_FLIP_ITEM*  psFlipItem;


	/* Disable interrupts while we remove the internal vsync flip queue */
	S3C_DisableVsyncInterrupt();

	/* Need to flush any flips now pending in Internal queue */
	psFlipItem = &psDevInfo->asVSyncFlips[psDevInfo->ulRemoveIndex];

	while(psFlipItem->bValid)
	{
		if(psFlipItem->bFlipped ==S3C_FALSE)
		{
			/* flip to new surface - flip latches on next interrupt */
			S3C_Flip (psDevInfo, psFlipItem->psFb);
		}

		/* command complete handler - allows dependencies for outstanding flips to be updated -
		   doesn't matter that vsync interrupts have been disabled.
		*/
		if(psFlipItem->bCmdCompleted == S3C_FALSE)
		{
			/*
				2nd arg == IMG_FALSE - don't schedule the MISR as we're
				just emptying the internal VsyncQueue
			*/
			psDevInfo->sPVRJTable.pfnPVRSRVCmdComplete((IMG_HANDLE)psFlipItem->hCmdComplete, IMG_FALSE);
		}

		/* advance remove index */
		AdvanceFlipIndex(psDevInfo, &psDevInfo->ulRemoveIndex);

		/* clear item state */
		psFlipItem->bFlipped = S3C_FALSE;
		psFlipItem->bCmdCompleted = S3C_FALSE;
		psFlipItem->bValid = S3C_FALSE;

		/* update to next flip item */
		psFlipItem = &psDevInfo->asVSyncFlips[psDevInfo->ulRemoveIndex];
	}

	psDevInfo->ulInsertIndex = 0;
	psDevInfo->ulRemoveIndex = 0;

	S3C_EnableVsyncInterrupt();
}
#ifdef USE_VSYNC_INTERRUPT
static irqreturn_t S3C_VSyncISR(int irq, void *dev_id)
{

	S3C_LCD_DEVINFO *psDevInfo = g_psLCDInfo;
	S3C_VSYNC_FLIP_ITEM *psFlipItem;

	// it assumes that kernel installs its  vsync ISR and clears interrupt in its handler
	S3C_Clear_interrupt();

	if(psDevInfo == NULL)
		goto Handled;

	S3C_DisableVsyncInterrupt();
	if(!psDevInfo->psSwapChain)
	{
		S3C_EnableVsyncInterrupt();
		goto Handled;
	}

	psFlipItem = &psDevInfo->asVSyncFlips[psDevInfo->ulRemoveIndex];
	
	while(psFlipItem->bValid)
	{
		/* have we already flipped BEFORE this interrupt */
		if(psFlipItem->bFlipped)
		{
			/* have we already 'Cmd Completed'? */
			if(!psFlipItem->bCmdCompleted)
			{
				IMG_BOOL bScheduleMISR;
				/* only schedule the MISR if the display vsync is on its own LISR */
#if 1
				bScheduleMISR = IMG_TRUE;
#else
				bScheduleMISR = IMG_FALSE;
#endif
				/* command complete the flip */
				psDevInfo->sPVRJTable.pfnPVRSRVCmdComplete((IMG_HANDLE)psFlipItem->hCmdComplete, bScheduleMISR);
				/* signal we've done the cmd complete */
				psFlipItem->bCmdCompleted = S3C_TRUE;
			}

			/* we've cmd completed so decrement the swap interval */
			psFlipItem->ulSwapInterval--;

			/* can we remove the flip item? */
			if(psFlipItem->ulSwapInterval == 0)
			{
				/* advance remove index */
				AdvanceFlipIndex(psDevInfo, &psDevInfo->ulRemoveIndex);

				/* clear item state */
				psFlipItem->bCmdCompleted = S3C_FALSE;
				psFlipItem->bFlipped = S3C_FALSE;

				/* only mark as invalid once item data is finished with */
				psFlipItem->bValid = S3C_FALSE;
			}
			else
			{
				/*	we're waiting for the last flip to finish displaying
				 *	so the remove index hasn't been updated to block any
				 *	new flips occuring. Nothing more to do on interrupt
				 */
				break;
			}
		}
		else
		{
			/* flip to new surface - flip latches on next interrupt */
			S3C_Flip (psDevInfo, psFlipItem->psFb);

			/* signal we've issued the flip to the HW */
			psFlipItem->bFlipped = S3C_TRUE;

			/* nothing more to do on interrupt */
			break;
		}

		/* update to next flip item */
		psFlipItem = &psDevInfo->asVSyncFlips[psDevInfo->ulRemoveIndex];
	}


Handled:
	S3C_EnableVsyncInterrupt();

	return IRQ_HANDLED;
}
#endif 

static IMG_VOID S3C_InstallVsyncISR(void)
{	
#ifdef USE_VSYNC_INTERRUPT
	if(request_irq(VSYCN_IRQ, S3C_VSyncISR, IRQF_SHARED , "s3cfb", g_psLCDInfo))
	{
		printk("S3C_InstallVsyncISR: Couldn't install system LISR on IRQ %d", VSYCN_IRQ);
		return;
	}
	//printk("[s3c_lcd] S3C_InstallVsyn -\n");
#endif
}
static IMG_VOID S3C_UninstallVsyncISR(void)
{	
	free_irq(VSYCN_IRQ, g_psLCDInfo);
}

static PVRSRV_ERROR OpenDCDevice(IMG_UINT32 ui32DeviceID,
								 IMG_HANDLE *phDevice,
								 PVRSRV_SYNC_DATA* psSystemBufferSyncData)
{
	PVR_UNREFERENCED_PARAMETER(ui32DeviceID);

	*phDevice =  (IMG_HANDLE)g_psLCDInfo;

	return PVRSRV_OK;
}

static PVRSRV_ERROR CloseDCDevice(IMG_HANDLE hDevice)
{
	S3C_LCD_DEVINFO *psLCDInfo = (S3C_LCD_DEVINFO*)hDevice;

	PVR_UNREFERENCED_PARAMETER(hDevice);


	if(psLCDInfo == g_psLCDInfo)
	{}
	
	return PVRSRV_OK;
}

static PVRSRV_ERROR EnumDCFormats(IMG_HANDLE		hDevice,
								  IMG_UINT32		*pui32NumFormats,
								  DISPLAY_FORMAT	*psFormat)
{
	S3C_LCD_DEVINFO *psLCDInfo = (S3C_LCD_DEVINFO*)hDevice;
	int i;
	
	if(!hDevice || !pui32NumFormats)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	*pui32NumFormats = S3C_DISPLAY_FORMAT_NUM;

	if(psFormat)
	{
		for (i = 0 ; i < S3C_DISPLAY_FORMAT_NUM ; i++)
			psFormat[i] = psLCDInfo->asDisplayForamtList[i];
	}
	
	return PVRSRV_OK;
}

static PVRSRV_ERROR EnumDCDims(IMG_HANDLE		hDevice,
							   DISPLAY_FORMAT	*psFormat,
							   IMG_UINT32		*pui32NumDims,
							   DISPLAY_DIMS		*psDim)
{
	int i;

	
	S3C_LCD_DEVINFO *psLCDInfo = (S3C_LCD_DEVINFO*)hDevice;
	
	if(!hDevice || !psFormat || !pui32NumDims)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	
	*pui32NumDims = S3C_DISPLAY_DIM_NUM;
	
	if(psDim)
	{
		for (i = 0 ; i < S3C_DISPLAY_DIM_NUM ; i++)
			psDim[i] = psLCDInfo->asDisplayDimList[i];
	
	}
	
	return PVRSRV_OK;
}

static PVRSRV_ERROR GetDCSystemBuffer(IMG_HANDLE hDevice, IMG_HANDLE *phBuffer)
{
	S3C_LCD_DEVINFO *psLCDInfo = (S3C_LCD_DEVINFO*)hDevice;

	if(!hDevice || !phBuffer)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	
	*phBuffer=(IMG_HANDLE)(&(psLCDInfo->sSysBuffer));
	return PVRSRV_OK;
}

static PVRSRV_ERROR GetDCInfo(IMG_HANDLE hDevice, DISPLAY_INFO *psDCInfo)
{
	S3C_LCD_DEVINFO *psLCDInfo = (S3C_LCD_DEVINFO*)hDevice;

	if(!hDevice || !psDCInfo)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	
	*psDCInfo = psLCDInfo->sDisplayInfo;
	
	return PVRSRV_OK;
}

static PVRSRV_ERROR GetDCBufferAddr(IMG_HANDLE		hDevice,
									IMG_HANDLE		hBuffer,
									IMG_SYS_PHYADDR	**ppsSysAddr,
									IMG_UINT32		*pui32ByteSize, 
									IMG_VOID		**ppvCpuVAddr,
									IMG_HANDLE		*phOSMapInfo,
									IMG_BOOL		*pbIsContiguous)
{
	S3C_FRAME_BUFFER *buf = (S3C_FRAME_BUFFER *)hBuffer;
	S3C_LCD_DEVINFO *psLCDInfo = (S3C_LCD_DEVINFO*)hDevice;
	PVR_UNREFERENCED_PARAMETER(psLCDInfo);
	
	//printk("GetDCBufferAddr+++++ hBuffer=%x\n",(int)hBuffer);
	
	if(!hDevice || !hBuffer || !ppsSysAddr || !pui32ByteSize)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	

	*phOSMapInfo = IMG_NULL;
	*pbIsContiguous = IMG_TRUE;

	*ppvCpuVAddr = (IMG_VOID *)buf->bufferVAddr;
	*ppsSysAddr = &(buf->bufferPAddr);
	*pui32ByteSize = buf->byteSize;
	
//	printk("GetDCBufferAddr:cpuVAddr=%p,sysAddr=%p\n",(*ppvCpuVAddr), (void*)(unsigned int)(**ppsSysAddr));
	
	return PVRSRV_OK;
}

static PVRSRV_ERROR CreateDCSwapChain(IMG_HANDLE hDevice,
									  IMG_UINT32 ui32Flags,
									  DISPLAY_SURF_ATTRIBUTES *psDstSurfAttrib,
									  DISPLAY_SURF_ATTRIBUTES *psSrcSurfAttrib,
									  IMG_UINT32 ui32BufferCount,
									  PVRSRV_SYNC_DATA **ppsSyncData,
									  IMG_UINT32 ui32OEMFlags,
									  IMG_HANDLE *phSwapChain,
									  IMG_UINT32 *pui32SwapChainID)
{
	IMG_UINT32 i;

	S3C_FRAME_BUFFER *psBuffer;
	S3C_SWAPCHAIN *psSwapChain;
	S3C_LCD_DEVINFO *psDevInfo = (S3C_LCD_DEVINFO*)hDevice;
	
	//printk("CreateDCSwapChain:ui32BufferCount=%d\n",(int)ui32BufferCount);

	PVR_UNREFERENCED_PARAMETER(ui32OEMFlags);
	PVR_UNREFERENCED_PARAMETER(pui32SwapChainID);

	/* check parameters */
	if(!hDevice
	|| !psDstSurfAttrib
	|| !psSrcSurfAttrib
	|| !ppsSyncData
	|| !phSwapChain)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	
	/* check the buffer count */
	if(ui32BufferCount > S3C_NUM_TOTAL_BUFFER)
	{
		return PVRSRV_ERROR_TOOMANYBUFFERS;
	}
	
	/* the pdp only supports a single swapchain */
	if(psDevInfo->psSwapChain)
	{
		return (PVRSRV_ERROR_FLIP_CHAIN_EXISTS);
	}

	psSwapChain = (S3C_SWAPCHAIN *)kmalloc(sizeof(S3C_SWAPCHAIN),GFP_KERNEL);
	psBuffer = (S3C_FRAME_BUFFER*)kmalloc(sizeof(S3C_FRAME_BUFFER) * ui32BufferCount, GFP_KERNEL);
	
	if(!psBuffer)
	{
		kfree(psSwapChain);
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}
	
	psSwapChain->ulBufferCount = (unsigned long)ui32BufferCount;
	psSwapChain->psBuffer = psBuffer;
	
	/* populate the buffers */
	//buffer 0 - sSysBuffer
	//buffer 1 -asBackBuffers[0]
	//buffer 2 -asBackBuffers[1]
	
	psBuffer[0].bufferPAddr = psDevInfo->sSysBuffer.bufferPAddr;
	psBuffer[0].bufferVAddr = psDevInfo->sSysBuffer.bufferVAddr;
	psBuffer[0].byteSize = psDevInfo->sSysBuffer.byteSize;

	for (i=1; i<ui32BufferCount; i++)
	{
		psBuffer[i].bufferPAddr = psDevInfo->asBackBuffers[i-1].bufferPAddr;
		psBuffer[i].bufferVAddr = psDevInfo->asBackBuffers[i-1].bufferVAddr;
		psBuffer[i].byteSize = psDevInfo->asBackBuffers[i-1].byteSize;
	}
	
	//printk("CreateDCSwapChain:swapchain.buffercount=%d,sc=%p\n",(int)psSwapChain->ulBufferCount, psSwapChain);

	*phSwapChain = (IMG_HANDLE)psSwapChain;
	*pui32SwapChainID =(IMG_UINT32)psSwapChain;	
	
	/* mark swapchain's existence */
	psDevInfo->psSwapChain = psSwapChain;
    
	S3C_DisableVsyncInterrupt();
    ResetVSyncFlipItems(psDevInfo);
	S3C_InstallVsyncISR();
#ifndef USE_IOREMAP
#ifdef LCD_IS_PRESENT
	g_psLCDInfo->uPreviousVsyncInterruptStatus = s3cfb_direct_ioctl(FB_NUM, S3CFB_GET_VSYNC_INT_STATUS, 0);
#endif
#endif
	S3C_EnableVsyncInterrupt();

	return PVRSRV_OK;
}


static PVRSRV_ERROR DestroyDCSwapChain(IMG_HANDLE hDevice,
									   IMG_HANDLE hSwapChain)
{
	S3C_SWAPCHAIN *sc = (S3C_SWAPCHAIN *)hSwapChain;
	S3C_LCD_DEVINFO *psLCDInfo = (S3C_LCD_DEVINFO*)hDevice;
	/* check parameters */
	if(!hDevice 
	|| !hSwapChain)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Flush the vsync flip queue */
	FlushInternalVSyncQueue(psLCDInfo);

	/* swap to primary */
	S3C_Flip(psLCDInfo, &psLCDInfo->sSysBuffer);
	
	kfree(sc->psBuffer);
	kfree(sc);

	if (psLCDInfo->psSwapChain == sc)
		psLCDInfo->psSwapChain = NULL;

	ResetVSyncFlipItems(psLCDInfo);

	S3C_DisableVsyncInterrupt();
	S3C_UninstallVsyncISR();
#ifndef USE_IOREMAP
#ifdef LCD_IS_PRESENT
	s3cfb_direct_ioctl(FB_NUM, S3CFB_SET_VSYNC_INT, g_psLCDInfo->uPreviousVsyncInterruptStatus);
#endif
#endif
	return PVRSRV_OK;
}

static PVRSRV_ERROR SetDCDstRect(IMG_HANDLE	hDevice,
								 IMG_HANDLE	hSwapChain,
								 IMG_RECT	*psRect)
{

	
	PVR_UNREFERENCED_PARAMETER(hDevice);
	PVR_UNREFERENCED_PARAMETER(hSwapChain);
	PVR_UNREFERENCED_PARAMETER(psRect);

	return PVRSRV_ERROR_NOT_SUPPORTED;
}


static PVRSRV_ERROR SetDCSrcRect(IMG_HANDLE	hDevice,
								 IMG_HANDLE	hSwapChain,
								 IMG_RECT	*psRect)
{


	PVR_UNREFERENCED_PARAMETER(hDevice);
	PVR_UNREFERENCED_PARAMETER(hSwapChain);
	PVR_UNREFERENCED_PARAMETER(psRect);

	return PVRSRV_ERROR_NOT_SUPPORTED;
}


static PVRSRV_ERROR SetDCDstColourKey(IMG_HANDLE	hDevice,
									  IMG_HANDLE	hSwapChain,
									  IMG_UINT32	ui32CKColour)
{
	PVR_UNREFERENCED_PARAMETER(hDevice);
	PVR_UNREFERENCED_PARAMETER(hSwapChain);
	PVR_UNREFERENCED_PARAMETER(ui32CKColour);

	return PVRSRV_ERROR_NOT_SUPPORTED;
}

static PVRSRV_ERROR SetDCSrcColourKey(IMG_HANDLE	hDevice,
									  IMG_HANDLE	hSwapChain,
									  IMG_UINT32	ui32CKColour)
{
	PVR_UNREFERENCED_PARAMETER(hDevice);
	PVR_UNREFERENCED_PARAMETER(hSwapChain);
	PVR_UNREFERENCED_PARAMETER(ui32CKColour);



	return PVRSRV_ERROR_NOT_SUPPORTED;
}

IMG_VOID S3CSetState(IMG_HANDLE hDevice, IMG_UINT32 ui32State)
{
	S3C_LCD_DEVINFO	*psDevInfo;

	psDevInfo = (S3C_LCD_DEVINFO*)hDevice;

	if (ui32State == DC_STATE_FLUSH_COMMANDS)
	{
		if (psDevInfo->psSwapChain != 0)
		{
			FlushInternalVSyncQueue(psDevInfo);
		}

		psDevInfo->bFlushCommands =S3C_TRUE;
	}
	else if (ui32State == DC_STATE_NO_FLUSH_COMMANDS)
	{
		psDevInfo->bFlushCommands = S3C_FALSE;
	}
}

static PVRSRV_ERROR GetDCBuffers(IMG_HANDLE hDevice,
								 IMG_HANDLE hSwapChain,
								 IMG_UINT32 *pui32BufferCount,
								 IMG_HANDLE *phBuffer)
{
	S3C_LCD_DEVINFO *psLCDInfo = (S3C_LCD_DEVINFO*)hDevice;
	int	i;
	
	/* check parameters */
	if(!hDevice
	|| !hSwapChain
	|| !pui32BufferCount
	|| !phBuffer)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	
	printk("GetDCBuffers:hSwapChain=%p ui32BufferCount=%d\n",(void*)hSwapChain,(int)*pui32BufferCount);

	*pui32BufferCount = S3C_MAX_BACKBUFFERRS + 1;
	phBuffer[0] = (IMG_HANDLE)(&(psLCDInfo->sSysBuffer));
	for (i=0; i<S3C_MAX_BACKBUFFERRS; i++)
	{
		phBuffer[i+1] = (IMG_HANDLE)(&(psLCDInfo->asBackBuffers[i]));
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR SwapToDCBuffer(IMG_HANDLE	hDevice,
								   IMG_HANDLE	hBuffer,
								   IMG_UINT32	ui32SwapInterval,
								   IMG_HANDLE	hPrivateTag,
								   IMG_UINT32	ui32ClipRectCount,
								   IMG_RECT		*psClipRect)
{

	//printk("SwapToDCBuffer+++\n");

	PVR_UNREFERENCED_PARAMETER(ui32SwapInterval);
	PVR_UNREFERENCED_PARAMETER(hPrivateTag);
	PVR_UNREFERENCED_PARAMETER(psClipRect);

	if(!hDevice
	|| !hBuffer
	|| (ui32ClipRectCount != 0))
	{
		return PVRSRV_ERROR_INVALID_PARAMS;	
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR SwapToDCSystem(IMG_HANDLE hDevice,
								   IMG_HANDLE hSwapChain)
{
	//printk("SwapToDCSystem++++++++++++++\n");

	if(!hDevice 
	|| !hSwapChain)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;	
	}
	
	
	/* FIXME: nothing to do here - deprecate API? */
	//printk(__FUNCTION__);

	return PVRSRV_OK;
}

#if 0
static int AllocLinearMemory(IMG_UINT32 size,IMG_UINT32 *vaddr,IMG_UINT32 *paddr)
{
	dma_addr_t dma;
	IMG_VOID *pvLinAddr;
	
	pvLinAddr = dma_alloc_coherent(NULL, size, &dma, GFP_KERNEL);

	if(pvLinAddr == NULL)
	{
		return 1;
	}


	*paddr = (IMG_UINT32)dma;
	*vaddr = (IMG_UINT32)pvLinAddr;

	return 0;
}
#else
static int AllocLinearMemory(IMG_UINT32 size, IMG_UINT32 * vaddr, IMG_UINT32 * paddr,
			     IMG_UINT32 offset)
{
	struct fb_fix_screeninfo fix;
	IMG_VOID *pvLinAddr;

#ifdef LCD_IS_PRESENT
	s3cfb_direct_ioctl(FB_NUM, FBIOGET_FSCREENINFO, (unsigned long) &fix);
#else
	s5ptvfb_direct_ioctl(FB_NUM, FBIOGET_FSCREENINFO, (unsigned long) &fix);
#endif
	pvLinAddr = ioremap_wc(fix.smem_start + offset, size);

	if (pvLinAddr == NULL) {
		return 1;
	}

	*paddr = (IMG_UINT32) (fix.smem_start + offset);
	*vaddr = (IMG_UINT32) pvLinAddr;

	return 0;
}
#endif

#if 0
static void FreeLinearMemory(  IMG_UINT32 size,
			   IMG_UINT32 *vaddr, 
			    IMG_UINT32 *paddr)
{
	dma_free_coherent(NULL, size, vaddr, (dma_addr_t)paddr);
}
#else
static void FreeLinearMemory(IMG_UINT32 size, IMG_UINT32 * vaddr, IMG_UINT32 * paddr)
{
	*vaddr = 0;
	*paddr = 0;
}
#endif


static IMG_BOOL ProcessFlip(IMG_HANDLE	hCmdCookie,
							IMG_UINT32	ui32DataSize,
							IMG_VOID	*pvData)
{
	DISPLAYCLASS_FLIP_COMMAND *psFlipCmd;
	S3C_LCD_DEVINFO *psDevInfo;
	S3C_FRAME_BUFFER *fb;
	S3C_VSYNC_FLIP_ITEM* psFlipItem;

	/* check parameters */
	if(!hCmdCookie || !pvData)
	{
		return IMG_FALSE;
	}

	/* validate data packet */
	psFlipCmd = (DISPLAYCLASS_FLIP_COMMAND*)pvData;
	if (psFlipCmd == IMG_NULL || sizeof(DISPLAYCLASS_FLIP_COMMAND) != ui32DataSize)
	{
		return IMG_FALSE;
	}
	
	/* setup some useful pointers */
	psDevInfo = (S3C_LCD_DEVINFO*)psFlipCmd->hExtDevice;
	fb = (S3C_FRAME_BUFFER*)psFlipCmd->hExtBuffer; /* This is the buffer we are flipping to */
	
	if (psDevInfo->bFlushCommands)
	{
		psDevInfo->sPVRJTable.pfnPVRSRVCmdComplete(hCmdCookie, IMG_FALSE);
		return IMG_TRUE;
	}
	/*
		Support for vsync "unlocked" flipping - not real support as this is a latched display,
		we just complete immediately.
	*/
#ifdef USE_VSYNC_INTERRUPT
	if(psFlipCmd->ui32SwapInterval == 0)
#else
	if(1)
#endif
	{
		/*
			The 'baseaddr' register can be updated outside the vertical blanking region.
			The 'baseaddr' update only takes effect on the vfetch event and the baseaddr register
			update is double-buffered. Hence page flipping is 'latched'.
		*/
	
		/* flip to new surface */
		S3C_Flip(psDevInfo, fb);
	
		/*
			call command complete Callback
			Also don't schedule the MISR as we're already in the MISR
		*/
		psDevInfo->sPVRJTable.pfnPVRSRVCmdComplete(hCmdCookie, IMG_FALSE);
	
		return IMG_TRUE;

	}

	psFlipItem = &psDevInfo->asVSyncFlips[psDevInfo->ulInsertIndex];
	
	/* try to insert command into list */
	if(!psFlipItem->bValid)
	{
		if(psDevInfo->ulInsertIndex == psDevInfo->ulRemoveIndex)
		{
			/* flip to new surface */
			S3C_Flip(psDevInfo, fb);

			psFlipItem->bFlipped = S3C_TRUE;
		}
		else
		{
			psFlipItem->bFlipped = S3C_FALSE;
		}

		psFlipItem->hCmdComplete = hCmdCookie;
		psFlipItem->psFb= fb;
		psFlipItem->ulSwapInterval = (unsigned long)psFlipCmd->ui32SwapInterval;
		printk("[ProcessFlip] ulSwapInterval=%d\n", (int)psFlipItem->ulSwapInterval);
		psFlipItem->bValid = S3C_TRUE;

		AdvanceFlipIndex(psDevInfo, &psDevInfo->ulInsertIndex);

		return IMG_TRUE;

	}

	return IMG_FALSE;
}
	
int init()
{
	IMG_UINT32 screen_w, screen_h;
	IMG_UINT32 pa_fb, va_fb;
	IMG_UINT32 byteSize;
	IMG_UINT32 byteSizeLCDDisplay;
	int	i;

	int rgb_format, bytes_per_pixel;
	
	struct fb_fix_screeninfo fix;
	struct fb_var_screeninfo var;

#ifdef LCD_IS_PRESENT
	s3cfb_direct_ioctl(FB_NUM, FBIOGET_FSCREENINFO, (unsigned long)&fix);
	s3cfb_direct_ioctl(FB_NUM, FBIOGET_VSCREENINFO, (unsigned long)&var);
#else
	s5ptvfb_direct_ioctl(FB_NUM, FBIOGET_FSCREENINFO, (unsigned long)&fix);
	s5ptvfb_direct_ioctl(FB_NUM, FBIOGET_VSCREENINFO, (unsigned long)&var);
#endif

	screen_w = var.xres;
	screen_h = var.yres;
	pa_fb = fix.smem_start;
	printk("FB_NUM=%d PA FB = 0x%X, bits per pixel = %d\n",FB_NUM,(unsigned int)fix.smem_start,(unsigned int)var.bits_per_pixel);
	va_fb = (unsigned long)phys_to_virt(pa_fb);

	printk("screen width=%d height=%d va=0x%x pa=0x%x\n", (int)screen_w, (int)screen_h, (unsigned int)va_fb, (unsigned int)pa_fb);

#ifdef USE_IOREMAP
	regs = (volatile unsigned int)ioremap(0xF8000000, 0x00100000);
#endif

	if (g_psLCDInfo == NULL)
	{
		PFN_CMD_PROC	pfnCmdProcList[DC_S3C_LCD_COMMAND_COUNT];
		IMG_UINT32	aui32SyncCountList[DC_S3C_LCD_COMMAND_COUNT][2];

		g_psLCDInfo = (S3C_LCD_DEVINFO*)kmalloc(sizeof(S3C_LCD_DEVINFO),GFP_KERNEL);
		

		g_psLCDInfo->ui32NumFormats = S3C_DISPLAY_FORMAT_NUM;
		switch (var.bits_per_pixel)
		{
		case 16:
			rgb_format = PVRSRV_PIXEL_FORMAT_RGB565;
			bytes_per_pixel = 2;
			break;
		case 32:
			rgb_format = PVRSRV_PIXEL_FORMAT_ARGB8888;
			bytes_per_pixel = 4;
			break;
		default:
			rgb_format = PVRSRV_PIXEL_FORMAT_ARGB8888;
			bytes_per_pixel = 4;
			break;
		}

		g_psLCDInfo->asDisplayForamtList[0].pixelformat = rgb_format;
		g_psLCDInfo->ui32NumDims = S3C_DISPLAY_DIM_NUM;
		//g_psLCDInfo->asDisplayDimList[0].ui32ByteStride = (bytes_per_pixel) * screen_w;
		//g_psLCDInfo->asDisplayDimList[0].ui32ByteStride = STRIDE_ALIGN(screen_w) * bytes_per_pixel;
		g_psLCDInfo->asDisplayDimList[0].ui32ByteStride = STRIDE_ALIGN(var.xres_virtual) * (bytes_per_pixel);
		g_psLCDInfo->asDisplayDimList[0].ui32Height = screen_h;
		g_psLCDInfo->asDisplayDimList[0].ui32Width = screen_w;

		g_psLCDInfo->sSysBuffer.bufferPAddr.uiAddr = pa_fb;
		g_psLCDInfo->sSysBuffer.bufferVAddr = (IMG_CPU_VIRTADDR)va_fb;
		//byteSize = screen_w * screen_h * bytes_per_pixel;
		//byteSize = STRIDE_ALIGN(screen_w) * screen_h * bytes_per_pixel;
		byteSize = STRIDE_ALIGN(var.xres_virtual) * var.yres_virtual * bytes_per_pixel;
		byteSizeLCDDisplay = STRIDE_ALIGN(var.xres_virtual) * var.yres * bytes_per_pixel;
		g_psLCDInfo->sSysBuffer.byteSize = (IMG_UINT32)byteSize;
		g_psLCDInfo->sSysBuffer.byteSizeLCDDisplay = (IMG_UINT32)byteSizeLCDDisplay;

		for (i=0; i<S3C_MAX_BACKBUFFERRS; i++)
		{
			g_psLCDInfo->asBackBuffers[i].byteSize = g_psLCDInfo->sSysBuffer.byteSize;
			g_psLCDInfo->asBackBuffers[i].byteSizeLCDDisplay = g_psLCDInfo->sSysBuffer.byteSizeLCDDisplay;

#if 0
			if(AllocLinearMemory(
				g_psLCDInfo->asBackBuffers[i].byteSize,
				(IMG_UINT32*)&(g_psLCDInfo->asBackBuffers[i].bufferVAddr),
				&(g_psLCDInfo->asBackBuffers[i].bufferPAddr.uiAddr)))
#else
			if(AllocLinearMemory(
				g_psLCDInfo->asBackBuffers[i].byteSize,
				(IMG_UINT32*)&(g_psLCDInfo->asBackBuffers[i].bufferVAddr),
				&(g_psLCDInfo->asBackBuffers[i].bufferPAddr.uiAddr), byteSize << i))
#endif
			return 1;

			printk("Back frameBuffer[%d].VAddr=%p PAddr=%p size=%d\n",
				i, 
				(void*)g_psLCDInfo->asBackBuffers[i].bufferVAddr,
				(void*)g_psLCDInfo->asBackBuffers[i].bufferPAddr.uiAddr,
				(int)g_psLCDInfo->asBackBuffers[i].byteSize);
		}

		g_psLCDInfo->bFlushCommands = S3C_FALSE;
		g_psLCDInfo->psSwapChain = NULL;

		PVRGetDisplayClassJTable(&(g_psLCDInfo->sPVRJTable));

		g_psLCDInfo->sDCJTable.ui32TableSize = sizeof(PVRSRV_DC_SRV2DISP_KMJTABLE);
		g_psLCDInfo->sDCJTable.pfnOpenDCDevice = OpenDCDevice;
		g_psLCDInfo->sDCJTable.pfnCloseDCDevice = CloseDCDevice;
		g_psLCDInfo->sDCJTable.pfnEnumDCFormats = EnumDCFormats;
		g_psLCDInfo->sDCJTable.pfnEnumDCDims = EnumDCDims;
		g_psLCDInfo->sDCJTable.pfnGetDCSystemBuffer = GetDCSystemBuffer;
		g_psLCDInfo->sDCJTable.pfnGetDCInfo = GetDCInfo;
		g_psLCDInfo->sDCJTable.pfnGetBufferAddr = GetDCBufferAddr;
		g_psLCDInfo->sDCJTable.pfnCreateDCSwapChain = CreateDCSwapChain;
		g_psLCDInfo->sDCJTable.pfnDestroyDCSwapChain = DestroyDCSwapChain;
		g_psLCDInfo->sDCJTable.pfnSetDCDstRect = SetDCDstRect;
		g_psLCDInfo->sDCJTable.pfnSetDCSrcRect = SetDCSrcRect;
		g_psLCDInfo->sDCJTable.pfnSetDCDstColourKey = SetDCDstColourKey;
		g_psLCDInfo->sDCJTable.pfnSetDCSrcColourKey = SetDCSrcColourKey;
		g_psLCDInfo->sDCJTable.pfnGetDCBuffers = GetDCBuffers;
		g_psLCDInfo->sDCJTable.pfnSwapToDCBuffer = SwapToDCBuffer;
		g_psLCDInfo->sDCJTable.pfnSwapToDCSystem = SwapToDCSystem;
		g_psLCDInfo->sDCJTable.pfnSetDCState = S3CSetState;

		g_psLCDInfo->sDisplayInfo.ui32MinSwapInterval=0;
		g_psLCDInfo->sDisplayInfo.ui32MaxSwapInterval=0;
		g_psLCDInfo->sDisplayInfo.ui32MaxSwapChains=1;
		g_psLCDInfo->sDisplayInfo.ui32MaxSwapChainBuffers=S3C_NUM_TOTAL_BUFFER;
		g_psLCDInfo->sDisplayInfo.ui32PhysicalWidthmm=var.width;	// width of lcd in mm 
		g_psLCDInfo->sDisplayInfo.ui32PhysicalHeightmm=var.height;	// height of lcd in mm 

		strncpy(g_psLCDInfo->sDisplayInfo.szDisplayName, "s3c_lcd", MAX_DISPLAY_NAME_SIZE);



		if(g_psLCDInfo->sPVRJTable.pfnPVRSRVRegisterDCDevice	(&(g_psLCDInfo->sDCJTable),
			(IMG_UINT32 *)(&(g_psLCDInfo->ui32DisplayID))) != PVRSRV_OK)
		{
			return 1;
		}

		//printk("deviceID:%d\n",(int)g_psLCDInfo->ui32DisplayID);

		// register flip command
		pfnCmdProcList[DC_FLIP_COMMAND] = ProcessFlip;
		aui32SyncCountList[DC_FLIP_COMMAND][0] = 0;/* no writes */
		aui32SyncCountList[DC_FLIP_COMMAND][1] = 2;/* 2 reads: To / From */

		if (g_psLCDInfo->sPVRJTable.pfnPVRSRVRegisterCmdProcList(g_psLCDInfo->ui32DisplayID,
			&pfnCmdProcList[0], aui32SyncCountList, DC_S3C_LCD_COMMAND_COUNT)
			!= PVRSRV_OK)
		{
			printk("failing register commmand proc list   deviceID:%d\n",(int)g_psLCDInfo->ui32DisplayID);
			return PVRSRV_ERROR_CANT_REGISTER_CALLBACK;
		}
#ifdef USE_IOREMAP
		LCDControllerBase = (volatile unsigned int *)ioremap(0xf8000000,1024);
#endif
	}

	return 0;
	
}


void deInit()
{
	int i;
	//printk("s3c_displayclass deinit++\n");
	
	g_psLCDInfo->sPVRJTable.pfnPVRSRVRemoveCmdProcList ((IMG_UINT32)g_psLCDInfo->ui32DisplayID,
														DC_S3C_LCD_COMMAND_COUNT);

	g_psLCDInfo->sPVRJTable.pfnPVRSRVRemoveDCDevice(g_psLCDInfo->ui32DisplayID);
	
	for (i=0; i<S3C_MAX_BACKBUFFERRS; i++)
		FreeLinearMemory(g_psLCDInfo->asBackBuffers[i].byteSize,
			(IMG_UINT32 *)g_psLCDInfo->asBackBuffers[i].bufferVAddr,
			(IMG_UINT32 *)g_psLCDInfo->asBackBuffers[i].bufferPAddr.uiAddr);

	if (g_psLCDInfo)
		kfree(g_psLCDInfo);

	g_psLCDInfo = NULL;
	
#ifdef USE_IOREMAP
	if(LCDControllerBase)
		iounmap(LCDControllerBase);
	if(regs)
		iounmap((void*)regs);
#endif

}

