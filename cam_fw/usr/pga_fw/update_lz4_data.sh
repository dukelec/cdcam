#!/bin/bash

cd "$(dirname "$(realpath "$0")")"
BIN_FILE=../../../cam_pga/cam_prj/cam_prj_Implmnt/sbt/outputs/bitmap/cam_spi_bitmap.bin

rm pga_fw_*

split -d -b 24K $BIN_FILE pga_fw_

for i in pga_fw_*; do
    echo -e "\n$i:"
    lz4 -l -12 $i $i.lz4
    dd if=$i.lz4 of=${i}_lz4 bs=1 skip=8
    xxd -i ${i}_lz4 $i.h
    rm $i $i.lz4 ${i}_lz4
    sed -i 's/unsigned char/static const uint8_t/g' $i.h
    sed -i 's/unsigned int/static const uint16_t/g' $i.h
done

[[ "$i" != "pga_fw_02" ]] && echo -e "\nerror!\n"

