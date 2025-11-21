#!/bin/bash

set -e  # Exit on error

# Build QEMU
echo "Building QEMU..."
make -j"$(nproc)"

QEMU="./build/qemu-system-hppa"
gdb --args $QEMU \
      -trace "i82596_*" \
      -trace "lasi_*" \
      -accel tcg,thread=multi -m 512 \
      -drive if=scsi,bus=0,index=4,file=OS_test/HPUX/new_hpux.img,format=raw \
      -net nic,model=lasi_82596 -net user -boot c \
      -serial mon:stdio -nographic -D hpuxtrace.log

# echo "Select which environment to run:"
# echo "1) ODE ISO"
# echo "2) HP-UX 10.20"
# echo "3) Linux (Helge)"
# echo "4) Linux (Debian 12 test)"
# read -rp "Enter choice [1-4]: " choice

# Common QEMU binary

# case "$choice" in
#   1)
#     echo "Running ODE ISO..."
#     gdb --args $QEMU \
#       -accel tcg,thread=multi \
#       -trace "i82596_*" \
#       -trace "lasi_*" \
#       -cdrom OS_test/hpux/ODE_2006_ohne_Passwort.iso \
#       -m 512 \
#       -boot d \
#       -drive if=scsi,bus=0,index=6,file=OS_test/hpux/new_hpux.img,format=raw \
#       -net nic,model=lasi_82596 \
#       -net user \
#       -serial mon:stdio \
#       -nographic \
#       -D hpuxtrace.log
#     ;;
#   2)
#     echo "Running HP-UX 10.20..."
#     gdb --args $QEMU \
#       -trace "i82596_*" \
#       -trace "lasi_*" \
#       -accel tcg,thread=multi -m 512 \
#       -drive if=scsi,bus=0,index=4,file=OS_test/HPUX/new_hpux.img,format=raw \
#       -net nic,model=lasi_82596 -net user -boot c \
#       -serial mon:stdio -nographic -D hpuxtrace.log
#     ;;
#   3)
#     echo "Running Linux (Helge)..."
#     gdb --args $QEMU \
#       -drive file=OS_test/debian-10/Linux-hppa-hdd-image.img \
#       -trace "i82596_*" \
#       -trace "lasi_*" \
#       -kernel OS_test/debian-10/vmlinux-6.15.ok-32bit \
#       -append "root=/dev/sda5 cryptomgr.notests panic=10 apparmor=0 no_hash_pointers" \
#       -serial mon:stdio -smp cpus=4 -machine B160L \
#       -nographic -net nic,model=lasi_82596 -net user \
#       -D trace.log
#     ;;
#   4)
#     echo "Running Linux (Debian 12 test)..."
#     $QEMU \
#       -trace "i82596_*" \
#       -trace "lasi_*" \
#       -kernel OS_test/vmlinux \
#       -drive file=OS_test/debian-12-hdd-2023.img \
#       -nographic \
#       -serial mon:stdio \
#       -accel tcg,thread=multi \
#       -smp cpus=4 \
#       -append "root=/dev/sda5 rw console=ttyS0 debug ignore_loglevel" \
#       -net nic,model=lasi_82596 \
#       -net user \
#       -D trace.log
#     ;;
#   *)
#     echo "Invalid choice."
#     exit 1
#     ;;
# esac
# 
