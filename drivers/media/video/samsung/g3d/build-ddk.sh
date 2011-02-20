#!/bin/sh
export CODE=`pwd`
export DDK_ROOT=$CODE/eurasia
export KERNELDIR=/home/scsuh/asus/latest
export DISCIMAGE=$CODE/output
export PATH=$PATH:/usr/local/arm/arm-2009q3/bin
export XORG_TOOLCHAIN=tarballs-arm
export BUILD=release
export CROSS_COMPILE=arm-none-linux-gnueabi-
export SUPPORT_XORG=1
export SUPPORT_OPENGL=1
export SUPPORT_OPENGLES1=1
export SUPPORT_OPENGLES2=1
export SUPPORT_OPENVG=0
export SUPPORT_OGL_TEXTURE_STREAM=1
export ARCH=arm

#set -e

case $1 in
km_clean)
  export EURASIAROOT=$CODE/eurasia_km
  cd $EURASIAROOT/eurasiacon/build/linux/s5pc110_linux/kbuild
  make SUPPORT_XORG=1 clobber
;;
modules)
  cd $KERNELDIR
  make INSTALL_MOD_PATH=$DISCIMAGE modules_install
;;
km_copy)
  export EURASIAROOT=$CODE/eurasia
  cd $EURASIAROOT/eurasiacon/binary_s5pc110_linux_$BUILD/
  echo "$EURASIAROOT/eurasiacon/binary_s5pc110_linux_$BUILD/*.ko --> $DISCIMAGE/lib/modules/2.6.31/kernel/drivers/char/"
  cp -vf *.ko $DISCIMAGE/lib/modules/2.6.31/kernel/drivers/char/
  echo "drmc110.ko --> $DISCIMAGE/lib/modules/"
  cp -vf $CODE/drmc110/drmc110.ko $DISCIMAGE/lib/modules/
;;
km)
  export EURASIAROOT=$CODE/eurasia_km
  cd $EURASIAROOT/eurasiacon/build/linux/s5pc110_linux/kbuild
  make SUPPORT_XORG=1
  export EURASIAROOT=$CODE/drmc110
  cd $EURASIAROOT
  echo now build drmc110 ko module
  #rm *.o *.ko
  make SUPPORT_XORG=1
  
  cd $CODE
  mkdir -p eurasia_km/eurasiacon/binary_s5pc110_linux_$BUILD/ 
  mkdir -p eurasia/eurasiacon/binary_s5pc110_linux_$BUILD/
  cp -vf eurasia_km/eurasiacon/binary_s5pc110_linux_$BUILD/* eurasia/eurasiacon/binary_s5pc110_linux_$BUILD/
#  cp drmc110/drmc110.ko $CODE/eurasia/eurasiacon/binary_s5pc110_linux_$BUILD/  
;;

esac
