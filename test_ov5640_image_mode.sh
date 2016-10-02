#!/bin/bash

widht=(2592 2048 1920 1600 1280 1280 1024 800 640 320)
heigth=(1936 1536 1080 1200 960 720 768 600 480 240)

function reload() {
	echo "Re-Loading ov5640..."
	sleep 1
	modprobe -r -v vfe_v4l2
	sleep 2
	modprobe -r -v ov5640
	sleep 1
	modprobe ov5640 frame_rate="$1"
	sleep 2
	modprobe vfe_v4l2
	sleep 1
	echo "done!"
}

function load() {
	echo "Loading ov5640..."
	modprobe vfe_v4l2
	sleep 2
	modprobe ov5640 frame_rate
	sleep 1
	echo "done!"
}

function unload() {
	echo "Unloading ov5640.."
	sleep 1
	modprobe -r -v vfe_v4l2
	modprobe -r -v ov5640
	echo "done!"
}

for index in ${!widht[*]}
do
	# reload "$1"
	echo "TEST $index"
	echo "Widht : ${widht[$index]}, Heigth : ${heigth[$index]}"
	# ./cap <width> <height> <buffers [4,8]> <video mode [0,1]> <exposure [-4,4]> <hflip [0,1]> <vflip [0,1]>
	./cap "${widht[$index]}" "${heigth[$index]}" "4" "0" "-999" "-1" "-1"
done