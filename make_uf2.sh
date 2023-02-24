#!/bin/bash

set -o errexit

cargo objcopy --release --bin nrf_accelhid -- -O binary target/toflash.bin
uf2conv target/toflash.bin --base 0x26000 --family 0xADA52840 --output target/toflash.uf2
rm target/toflash.bin
cp target/toflash.uf2 /run/media/erik/ITSY840BOOT/
