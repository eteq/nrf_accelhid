#!/bin/bash

set -o errexit
: "${GPIONUM:=1}"
: "${BOOTPATH:=ITSY840BOOT}"

cargo objcopy --release --bin nrf_accelhid -- -O binary target/toflash.bin
uf2conv target/toflash.bin --base 0x26000 --family 0xADA52840 --output target/toflash.uf2
rm target/toflash.bin

# get the chip in reset mode
gpioset -m time -u 5000 $GPIONUM 0=0 1=0 2=0 3=0
gpioset -m time -u 5000 $GPIONUM 0=1 1=1 2=1 3=1
if ! [ -z ${SINGLERESET+x} ]; then
gpioset -m time -u 5000 $GPIONUM 0=0 1=0 2=0 3=0
gpioset -m time -u 5000 $GPIONUM 0=1 1=1 2=1 3=1
fi

echo wait for USB to catch up
sleep 4 

cp target/toflash.uf2 /run/media/erik/$BOOTPATH/
