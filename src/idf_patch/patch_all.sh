#!/bin/bash

echo "make sure esp-idf work space clean!"
[[ "$IDF_PATH" == "" ]] && { echo "please source esp-idf/export.sh first!"; exit; }

cd "$(dirname "$(realpath "$0")")"
PATCH_PATH=`pwd`

cd $IDF_PATH
echo "apply patch_spi.patch"
git apply $PATCH_PATH/patch_spi.patch

echo "done"
