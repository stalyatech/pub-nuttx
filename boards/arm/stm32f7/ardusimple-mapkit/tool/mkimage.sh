#/bin/sh

export NXIMAGE=../../../../../../apps/boot/tinyboot/tools/nximage.py
export IMAGERAW=../../../../../nuttx.bin
export IMAGEVER="1.0.0"
export IMAGEPRI=primary.img
export IMAGESEC=secondary-$IMAGEVER.img

#python3 $NXIMAGE -v --version $IMAGEVER --header_size 0x200 --primary $IMAGERAW $IMAGEPRI 
python3 $NXIMAGE -v --version $IMAGEVER --header_size 0x200 $IMAGERAW $IMAGESEC
