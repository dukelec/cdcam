CDCAM Introduction
=======================================

<img src="doc/cdcam3.jpg">

RS-485 wire housing: Molex 5264 (4 pin)

Download this project:
```
git clone --recurse-submodules https://github.com/dukelec/cdcam.git
```

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

Schematic: <a href="hardware/cdcam_sch_v1.3.pdf">cdcam_sch_v1.3.pdf</a>

