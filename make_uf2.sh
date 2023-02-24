#!/bin/bash

set -o errexit

cargo objcopy --release --bin nrf_accelhid -- -O binary toflash.bin
uf2conv toflash.bin --base 0x26000 --family 0xADA52840 --output toflash.uf2
rm toflash.bin
#cp toflash.uf2 /run/media/erik/ITSYBOOT/
