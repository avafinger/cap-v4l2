#!/bin/bash
echo "Building cap with local header...."
g++ cap.c -o cap $(pkg-config --libs --cflags opencv4) -lm -O3
echo "done!"
echo ""
echo "run: ./cap 1280 768 4 1 -999 -1 -1"
echo ""
