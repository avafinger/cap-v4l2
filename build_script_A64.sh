#!/bin/bash
echo "Building cap with kernel header version 3.10.102...."
gcc -I/usr/src/linux-headers-3.10.102 cap.c -o cap $(pkg-config --libs --cflags opencv) -lm -O3
echo "done!"
