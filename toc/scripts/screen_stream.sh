#!/bin/sh

export DISPLAY=:0 

cvlc -vvv --no-audio screen:// --screen-fps 1 --sout '#transcode{vcodec=MJPG,vb=800}:standard{access=http,mux=mpjpeg,dst=:18223/}' --sout-http-mime='multipart/x-mixed-replace;boundary=--7b3cc56e5f51db803f790dad720ed50a' > /dev/null 2>&1 
