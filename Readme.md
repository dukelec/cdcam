CDCAM Introduction
=======================================

<img src="doc/cdcam3.jpg">

RS-485 wire housing: Molex 5264 (4 pin)

Download this project:
```
git clone --recursive https://github.com/dukelec/cdcam.git
```


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

In the current configuration, the frame rate of 800x600 pictures is tested at 2.5 fps.  
In previous tests, it was possible to reach 10 fps,
but when shooting targets with a lot of detail (when the jpg file is relatively large), the picture tends to tear.  

The fps is limited by the performance of the STM32G0.

Default sensor: OV2640.

<img src="doc/cdbus_gui.png">

Notes:  
After modifying the configuration, write 1 to `save_conf` to save the configuration to flash.  
If you need to restore the default configuration, change `magic_code` to another value and save it to flash. Then reapply power.


## Hardware

Schematic: <a href="hardware/cdcam_sch_v1.10.pdf">cdcam_sch_v1.10.pdf</a>

