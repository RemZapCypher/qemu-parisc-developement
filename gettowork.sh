#!/bin/bash

set -e

echo "Building QEMU..."
make -j$(nproc)

# cd roms/seabios-hppa/
# ./build-pa.sh
# cd ../../

echo "Running QEMU..."

# ./build/qemu-system-hppa \
#     -drive file=OS_test/debian-10/Linux-hppa-hdd-image.img \
#     -kernel OS_test/debian-10/vmlinux-6.15.ok-32bit \
#     -append "root=/dev/sda5 cryptomgr.notests panic=10 apparmor=0 no_hash_pointers" \
#     -serial mon:stdio -smp cpus=4 -machine B160L \
#     -nographic -net nic,model=lasi_82596 -net user
#
# ---HPUX with ODE--- #
# gdb --args ./build/qemu-system-hppa \
#     -accel tcg,thread=multi \
#     -trace "i82596_*" \
#     -trace "lasi_*" \
#     -cdrom OS_test/ODE_2006_ohne_Passwort.iso \
#     -m 512 \
#     -boot d \
#     -drive if=scsi,bus=0,index=4,file=OS_test/new_hpux.img,format=raw \
#     -net nic,model=lasi_82596 \
#     -net user \
#     -serial mon:stdio \
#     -nographic \
#     -D hpuxtrace.log

# ---HPUX Normal--- #
# gdb --args ./build/qemu-system-hppa \
#     -trace "ncr53c710_*" \
#     -trace "i82596_*" \
#     -trace "lasi_*" \
#     -accel tcg,thread=multi -m 512 \
#     -drive if=scsi,bus=0,index=4,file=OS_test/hpux/new_hpux.img,format=raw \
#     -net nic,model=lasi_82596 -net user -boot c \
#     -serial mon:stdio -nographic -D hpuxtrace.log


# ---Debian Helge config--- #
# gdb --args ./build/qemu-system-hppa \
#     -drive file=OS_test/debian-10/Linux-hppa-hdd-image.img \
#     -trace "i82596_*" \
#     -trace "lasi_*" \
#     -kernel OS_test/debian-10/vmlinux-6.15.ok-32bit  \
#     -append "root=/dev/sda5 cryptomgr.notests panic=10 apparmor=0 no_hash_pointers" \
#     -serial mon:stdio -smp cpus=4 -machine B160L  \
#     -nographic -net nic,model=lasi_82596 -net user \
#     -D trace.log


#---Debian Normal config--- #
./build/qemu-system-hppa \
    -drive file=OS_test/debian-10/Linux-hppa-hdd-image.img \
    -kernel OS_test/debian-10/vmlinux \
    -append "root=/dev/sda5 cryptomgr.notests panic=10 apparmor=0 no_hash_pointers console=ttyS0" \
    -serial mon:stdio \
    -smp cpus=4 \
    -machine B160L \
    -nographic \
    -net nic,model=lasi_82596 \
    -net user

# ./build/qemu-system-hppa \
#     -kernel OS_test/debian-10/vmlinux \
#     -drive file=OS_test/debian-10/Linux-hppa-hdd-image.img,format=raw,if=scsi,media=disk \
#     -nographic \
#     -serial mon:stdio \
#     -accel tcg,thread=multi \
#     -smp cpus=4 \
#     -append "root=/dev/sda1 rw console=ttyS0 scsi_mod.max_luns=1 scsi_mod.default_dev_flags=0x240" \
#     -net nic,model=lasi_82596 \
#     -net user
