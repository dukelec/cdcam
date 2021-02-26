#!/bin/sh

st-flash --reset write build/cam_bl.bin 0x08000000

