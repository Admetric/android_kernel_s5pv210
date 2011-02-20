/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#if !defined(__PVR_DRM_H__)
#define __PVR_DRM_H__

#include "pvr_drm_shared.h"

#if defined(SUPPORT_DRI_DRM)
#define	PVR_DRM_MAKENAME_HELPER(x, y) x ## y
#define	PVR_DRM_MAKENAME(x, y) PVR_DRM_MAKENAME_HELPER(x, y)

IMG_INT PVRCore_Init(IMG_VOID);
IMG_VOID PVRCore_Cleanup(IMG_VOID);
IMG_INT PVRSRVOpen(struct drm_device *dev, struct drm_file *pFile);
IMG_INT PVRSRVRelease(struct drm_device *dev, struct drm_file *pFile);
IMG_INT PVRSRVDriverSuspend(struct drm_device *pDevice, pm_message_t state);
IMG_INT PVRSRVDriverResume(struct drm_device *pDevice);

IMG_INT PVRSRV_BridgeDispatchKM(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);

#if defined(SUPPORT_DRI_DRM_EXT)
#define	DRI_DRM_STATIC
IMG_INT PVRSRVDrmLoad(struct drm_device *dev, unsigned long flags);
IMG_INT PVRSRVDrmUnload(struct drm_device *dev);
IMG_INT PVRSRVDrmOpen(struct drm_device *dev, struct drm_file *file);
IMG_VOID PVRSRVDrmPostClose(struct drm_device *dev, struct drm_file *file);
IMG_INT PVRDRMIsMaster(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);
IMG_INT PVRDRMUnprivCmd(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);
IMG_INT PVRDRM_Dummy_ioctl(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);
#else
#define	DRI_DRM_STATIC	static
#endif	

#if defined(DISPLAY_CONTROLLER)
extern int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Init)(struct drm_device *);
extern void PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Cleanup)(struct drm_device *);
#endif

#if defined(PDUMP)
int dbgdrv_init(void);
void dbgdrv_cleanup(void);
IMG_INT dbgdrv_ioctl(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile);
#endif

#if !defined(SUPPORT_DRI_DRM_EXT)
#define	PVR_DRM_SRVKM_IOCTL	_IO(0, PVR_DRM_SRVKM_CMD)
#define	PVR_DRM_IS_MASTER_IOCTL _IO(0, PVR_DRM_IS_MASTER_CMD)
#define	PVR_DRM_UNPRIV_IOCTL	_IO(0, PVR_DRM_UNPRIV_CMD)
#define	PVR_DRM_DBGDRV_IOCTL	_IO(0, PVR_DRM_DBGDRV_CMD)
#endif	

#endif	

#endif 


