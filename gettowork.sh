#!/bin/bash

set -e

./build/qemu-system-hppa \
    -accel tcg,thread=multi \
    -m 512 -drive if=scsi,bus=0,index=6,file=OS_test/HPUX/new_hpux.img,format=raw \
    -net nic,model=lasi_82596 -net user -boot c -serial mon:stdio -nographic

