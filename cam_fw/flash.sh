#!/bin/sh

st-flash --reset write build/cam_fw.bin 0x08006800

