#!/bin/bash

w=(3264 3264 1920 1600 1280 800 640)
h=(2448 1836 1080 1200 960 600 480)

function reload() {
    echo "Reloading OV8865, wait..."
    sleep 1
	sudo modprobe -r -v vfe_v4l2
    sleep 2
	sudo modprobe -r -v ov8858_4lane
    sleep 1
    sudo modprobe -r -v vm149c_act
    sleep 1
    sudo modprobe vm149c_act
    sleep 1
	sudo modprobe ov8858_4lane
    sleep 2
	sudo modprobe vfe_v4l2
    echo "done!"
}

function load() {
    echo "Loading OV8865, wait..."
    sudo modprobe vm149c_act
    sudo modprobe ov8858_4lane
	sudo modprobe vfe_v4l2
    echo "done!"
}

function unload() {
    echo "Unloading OV8865, wait..."
	sudo modprobe -r -v vfe_v4l2
	sudo modprobe -r -v ov8858_4lane
    sudo modprobe -r -v vm149c_act
    echo "done!"
}

for index in ${!w[*]}
do
	# reload "$1"
	echo "TEST $index ( ${w[$index]} x ${h[$index]} )"
	# ./cap <width> <height> <buffers [4,8]> <video mode [0,1]> <exposure [-4,4]> <hflip [0,1]> <vflip [0,1]>
	./cap "${w[$index]}" "${h[$index]}" "8" "1" "-999" "-1" "-1"
done
