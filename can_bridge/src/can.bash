#!/bin/bash

sudo modprobe peak_usb # kernel driver, since 3.11

sudo modprobe peak_pci # kernel driver

sudo modprobe pcan #PEAK vendor driver 

sudo modprobe esd_usb2 #kernel driver 

sudo ip link set can0 up type can bitrate 1000000 #adjust bitrate as needed