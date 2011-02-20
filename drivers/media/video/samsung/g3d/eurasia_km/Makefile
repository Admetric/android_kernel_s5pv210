#
# Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
# 
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
# 
# This program is distributed in the hope it will be useful but, except 
# as otherwise stated in writing, without any warranty; without even the 
# implied warranty of merchantability or fitness for a particular purpose. 
# See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
# 
# The full GNU General Public License is included in this distribution in
# the file called "COPYING".
#
# Contact Information:
# Imagination Technologies Ltd. <gpl-support@imgtec.com>
# Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
# 
#
# This file is based on eurasia_km/services4/srvkm/env/linux/kbuild/Makefile
# To merge in kernel as a built-in, small modification was done by Samsung Electronics. 2010.
#
# As stated above, this file is subject to the terms and conditions of the GNU General Public
# License. See the file COPYING in the main directory of this archive
# for more details.

EURASIAROOT := drivers/media/video/samsung/g3d/eurasia_km/
EXTRA_CFLAGS = 	$(INCLUDES) \
		-DLINUX -DPVR_BUILD_DATE="\"Fri May 28 KST 2010\"" -DPVR_BUILD_TYPE="\"release\"" \
		-DRELEASE -DPVRSRV_MODNAME="\"pvrsrvkm"\" -DSERVICES4 \
		-D_XOPEN_SOURCE=600 -DPVR2D_VALIDATE_INPUT_PARAMS -DDISPLAY_CONTROLLER=s3c_lcd \
		-DDEBUG_LOG_PATH_TRUNCATE=\"/home/scsuh/opengl/img-final-ed3029/img-ddk/eurasia_km\" \
		-DSUPPORT_SRVINIT -DSUPPORT_SGX -DSUPPORT_PERCONTEXT_PB \
		-DSUPPORT_LINUX_X86_WRITECOMBINE -DTRANSFER_QUEUE -DSUPPORT_DRI_DRM \
		-DSYS_USING_INTERRUPTS -DPVR_SECURE_HANDLES -DPVR_SECURE_DRM_AUTH_EXPORT \
		-DUSE_PTHREADS -DSUPPORT_SGX_EVENT_OBJECT -DPVR2D_ALT_2DHW \
		-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING -DSUPPORT_LINUX_X86_PAT \
		-DSUPPORT_CACHE_LINE_FLUSH \
		-DSUPPORT_PDUMP_DELAYED_INITPHASE_TERMINATION \
		-DPVR_PROC_USE_SEQ_FILE -DSGX540 \
		-DSUPPORT_SGX540 -DSGX_CORE_REV=120 \
		-DPVR_BUILD_DIR="\"s5pc110_linux\"" 

obj-y += pvrsrvkm.o

KBUILDROOT := ./services4
PVR_SYSTEM := s5pc110

INCLUDES = 	-I$(EURASIAROOT)/include4 \
			-I$(EURASIAROOT)/services4/include \
			-I$(EURASIAROOT)/services4/srvkm/env/linux \
			-I$(EURASIAROOT)/services4/srvkm/include \
			-I$(EURASIAROOT)/services4/srvkm/bridged \
			-I$(EURASIAROOT)/services4/system/$(PVR_SYSTEM) \
			-I$(EURASIAROOT)/services4/system/include 

pvrsrvkm-y := 	$(KBUILDROOT)/srvkm/env/linux/osfunc.o \
		$(KBUILDROOT)/srvkm/env/linux/mutils.o \
		$(KBUILDROOT)/srvkm/env/linux/mmap.o \
		$(KBUILDROOT)/srvkm/env/linux/module.o \
		$(KBUILDROOT)/srvkm/env/linux/pdump.o \
		$(KBUILDROOT)/srvkm/env/linux/proc.o \
		$(KBUILDROOT)/srvkm/env/linux/pvr_bridge_k.o \
		$(KBUILDROOT)/srvkm/env/linux/pvr_debug.o \
		$(KBUILDROOT)/srvkm/env/linux/mm.o \
		$(KBUILDROOT)/srvkm/env/linux/mutex.o \
		$(KBUILDROOT)/srvkm/env/linux/event.o \
		$(KBUILDROOT)/srvkm/env/linux/osperproc.o

pvrsrvkm-y     += $(KBUILDROOT)/srvkm/common/buffer_manager.o \
		$(KBUILDROOT)/srvkm/common/devicemem.o \
		$(KBUILDROOT)/srvkm/common/deviceclass.o \
		$(KBUILDROOT)/srvkm/common/handle.o \
		$(KBUILDROOT)/srvkm/common/hash.o \
		$(KBUILDROOT)/srvkm/common/metrics.o \
		$(KBUILDROOT)/srvkm/common/pvrsrv.o \
		$(KBUILDROOT)/srvkm/common/queue.o \
		$(KBUILDROOT)/srvkm/common/ra.o \
		$(KBUILDROOT)/srvkm/common/resman.o \
		$(KBUILDROOT)/srvkm/common/power.o \
		$(KBUILDROOT)/srvkm/common/mem.o \
		$(KBUILDROOT)/srvkm/common/pdump_common.o \
		$(KBUILDROOT)/srvkm/bridged/bridged_support.o \
		$(KBUILDROOT)/srvkm/bridged/bridged_pvr_bridge.o \
		$(KBUILDROOT)/srvkm/common/perproc.o \
		$(KBUILDROOT)/../services4/system/$(PVR_SYSTEM)/sysconfig.o \
		$(KBUILDROOT)/../services4/system/$(PVR_SYSTEM)/sysutils.o \
		$(KBUILDROOT)/srvkm/common/lists.o \
		$(KBUILDROOT)/srvkm/common/mem_debug.o

INCLUDES += -I$(EURASIAROOT)/services4/srvkm/hwdefs 

SYM_VERS_DEPS = $(EURASIAROOT)/tools/intern/debug/dbgdriv/linux


#
# Device dependent
#

#ifeq ($(SUPPORT_SGX), 1)

INCLUDES 	+=	-I$(EURASIAROOT)/services4/srvkm/bridged/sgx \
				-I$(EURASIAROOT)/services4/srvkm/devices/sgx

pvrsrvkm-y     +=	$(KBUILDROOT)/srvkm/bridged/sgx/bridged_sgx_bridge.o \
				$(KBUILDROOT)/srvkm/devices/sgx/sgxinit.o \
				$(KBUILDROOT)/srvkm/devices/sgx/sgxpower.o \
				$(KBUILDROOT)/srvkm/devices/sgx/sgxreset.o \
				$(KBUILDROOT)/srvkm/devices/sgx/sgxutils.o \
				$(KBUILDROOT)/srvkm/devices/sgx/sgxkick.o \
				$(KBUILDROOT)/srvkm/devices/sgx/sgxtransfer.o \
				$(KBUILDROOT)/srvkm/devices/sgx/mmu.o \
				$(KBUILDROOT)/srvkm/devices/sgx/pb.o

#else  

ifeq ($(SUPPORT_VGX), 1)

INCLUDES 	+=	-I$(EURASIAROOT)/services4/srvkm/bridged/vgx \
				-I$(EURASIAROOT)/services4/srvkm/devices/vgx

#FIXME: We need to cerate a subset of vgxdefs.h from $(EURASIAROOT)/hwdefs 
# and put it into srvkm/hwdefs for GPL compliance
INCLUDES 	+=	-I$(EURASIAROOT)/hwdefs

SOURCES		+= 	$(KBUILDROOT)/srvkm/bridged/vgx/bridged_vgx_bridge.c \
				$(KBUILDROOT)/srvkm/devices/vgx/vgxinit.c \
				$(KBUILDROOT)/srvkm/devices/vgx/vgxreset.c \
				$(KBUILDROOT)/srvkm/devices/vgx/mmu.c \
				$(KBUILDROOT)/srvkm/devices/vgx/pb.c \
				$(KBUILDROOT)/srvkm/env/linux/pdump_vgx.c
endif

#endif # SUPPORT_SGX

#ifeq ($(SUPPORT_DRI_DRM),1)
pvrsrvkm-y     += $(KBUILDROOT)/srvkm/env/linux/pvr_drm.o
	INCLUDES += -I$(KERNELDIR)/include/drm \
		-I$(EURASIAROOT)/services4/include/env/linux

ifneq ("$(DISPLAY_CONTROLLER_DIR)","")
include $(EURASIAROOT)/services4/$(DISPLAY_CONTROLLER_DIR)/makefile.linux.common
endif

ifeq ($(PDUMP),1)
include $(EURASIAROOT)/tools/intern/debug/dbgdriv/linux/makefile.linux.common
endif

#endif

pvrsrvkm-y     += $(KBUILDROOT)/3rdparty/s3c_lcd/s3c_displayclass.o \
		$(KBUILDROOT)/3rdparty/s3c_lcd/s3c_lcd.o

#include $(EURASIAROOT)/eurasiacon/build/linux/kbuild/Makefile.kbuild_subdir_common
