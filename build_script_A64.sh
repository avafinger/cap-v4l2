#!/bin/bash
echo "Building cap with kernel header version" $(uname -r) "...."
echo "Searching for: /usr/src/linux-headers-"$(uname -r) "...."
gcc -D_V4L2_KERNEL_ -I/usr/src/linux-headers-$(uname -r) cap.c -o cap $(pkg-config --libs --cflags opencv) -lm -O3
echo "done!"
