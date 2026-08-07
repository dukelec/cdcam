#!/bin/bash

echo "make sure esp-idf work space clean!"
[[ "$IDF_PATH" == "" ]] && { echo "please source esp-idf/export.sh first!"; exit; }

cd "$(dirname "$(realpath "$0")")"
PATCH_PATH=`pwd`

echo "apply esp_video_components patchs"
cp ov5647_*.h ../esp_video_components/esp_cam_sensor/sensors/ov5647/private_include/

cd $IDF_PATH
echo "apply patch_spi.patch"
git apply $PATCH_PATH/patch_spi.patch

echo "done"
