#!/bin/bash

if [ ! -d sysroot ]; then

	apt install debootstrap quemu-user-static
	debootstrap --foreign --arch=armhf stretch sysroot http://deb.debian.org/debian/
	cp /usr/bin/qemu-arm-static sysroot/usr/bin/
	cp /etc/resolv.conf sysroot/etc/
	chroot sysroot/ /debootstrap/debootstrap --second-stage
	chroot sysroot/ apt install -y libc6-dev libboost-filesystem-dev
fi

make CC=arm-linux-gnueabihf-g++ SYSROOT=sysroot/