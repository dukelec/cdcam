CDCAM
=======================================

An ESP32P4-based CDBUS (RS-485) Camera.


## Protocol

CDCAM is an open-source serial camera that communicates over an RS485 interface.
 - Default baud rate: 115200 bps
 - Maximum speed: 50 Mbps
 - Default address: 0xfe

The underlying protocol is CDBUS, with the following frame format:  
`src, dst, len, [payload], crc_l, crc_h`

Each frame includes a 3-byte header, a variable-length payload, and a 2-byte CRC (identical to Modbus CRC).  
For more information on the CDBUS protocol, please refer to:
 - https://cdbus.org

The payload is encoded using the CDNET protocol. For detailed information, please refer to:
 - https://github.com/dukelec/cdnet
 - https://github.com/dukelec/cdnet/wiki/CDNET-Intro-and-Demo


## GUI Tool

CDBUS GUI Tool: https://github.com/dukelec/cdbus_gui

Default sensor: OV5647.


## Hardware

Schematic: <a href="hw/cdcam_v4.0.pdf">cdcam_v4.0.pdf</a>


## Build Instructions

Based on IDF v6.0-beta1, run `source esp-idf/export.sh`, then execute `src/idf_patchs/patch_all.sh` once.  
After that, enter the `src` directory, run `idf.py set-target esp32p4` (only required the first time), and then execute `idf.py build`.

Firmware can be upgraded by:
 - Using the CDBUS GUI tool to perform RS-485 IAP with the HEX file in the build directory.
 - Running `tests/ble_ota.py` for OTA upgrade (or OTA via UDP).
 - Via the USB debug port.

After reboot, CD-ESP will automatically switch to the new firmware.
