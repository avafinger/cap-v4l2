#!/bin/bash

width=(2592 2048 1920 1600 1280 1280 1024 800 640 320 176)
heigth=(1936 1536 1080 1200 960 720 768 600 480 240 144)

function reload() {
	echo "Reloading OV5640, wait..."
	sleep 1
	sudo modprobe -r -v vfe_v4l2
	sleep 2
	sudo modprobe -r -v ov5640
	sleep 1
	sudo modprobe ov5640 frame_rate="$1"
	sleep 2
	sudo modprobe vfe_v4l2
	sleep 1
	echo "done!"
}

function load() {
	echo "Loading OV5640, wait..."
	sleep 1
    sudo modprobe ov5640 frame_rate="$1"
	sleep 2
	sudo modprobe vfe_v4l2
	sleep 1
	echo "done!"
}

function unload() {
	echo "Unloading OV5640, wait..."
	sleep 1
	sudo modprobe -r -v vfe_v4l2
	sleep 2
	sudo modprobe -r -v ov5640
	sleep 1
	echo "done!"
}

for index in ${!width[*]}
do
	# reload "$1"
	echo "TEST $index - ( ${width[$index]} x ${heigth[$index]} )"
	# ./cap <width> <height> <buffers [4,8]> <video mode [0,1]> <exposure [-4,4]> <hflip [0,1]> <vflip [0,1]>
	./cap "${width[$index]}" "${heigth[$index]}" "4" "1" "-999" "-1" "-1"
done
