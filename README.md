# fd-capture-lite

## ! This project is still WIP and in early stage !

This project is a simplified version of [floppy_disk_shield_2d](https://github.com/yas-sim/floppy_disk_shield_2d).
The FD-capture-lite has the following features:
- 2D/2DD floppy image capturing feature
- Create a floppy disk image in ['MFM' image](https://github.com/yas-sim/fdc_bitstream#mfm-image-data-format) file
- Only you need is an [Arduino Uno R3](https://docs.arduino.cc/hardware/uno-rev3) and a floppy disk drive. You don't need to have a special Arduino shield or any other hardware.
- The host program is written in Python. The host program communicates with Arduino via the onboard USB-serial interface on Arduino.

## Caveat
- ONLY *genuine Arduino Uno R3* may work.
    - This project uses AVR inline assembly code. The program has deep dependency on ATMega328 CPU.
- The Arduino Uno R3 clone boards may work (as long as the CPU is ATMega328) but the board should support 2Mbps serial I/F speed.

## Hardware preparation - Arduino-FDD wiring
You just need to use some jump-wires to connect FDD to Arduino directry.

----
|Signal|[Arduino shield pins](https://docs.arduino.cc/static/6ec5e4c2a6c0e9e46389d4f6dc924073/2f891/Pinout-UNOrev3_latest.png)|FDD 34pin conn|Note|
|---|---|---|---|
|STEP|D2|20||
|HEAD LOAD|D3|4|You can connect this signal to GND. The Arduino firmware drives this signal to LOW all the time.|
|MOTOR ON|D4|16|You can connect this signal to GND. The Arduino firmware drives this signal to LOW all the time.|
|SIDE1|D5|32||
|INDEX|D6|8||
|TRK00|D7|26||
|RD|D8|30||
|DIR|D9|18||
|DS0<br>DS1<br>DS2<br>DS3|GND|10<br>12<br>14<br>6|Depends on the setting of your FDD drive. You can connect all of them to GND if you have no idea which one to connect.<br>The generic FDDs for IBM PCs use DS2 for drive select, and old 8-bit PCs may use DS0 or DS1 depending on which side the drive was installed.|
|GND|GND|1,5,7,9,11,...|One of a odd pins except pin3|

------------------------
## Other information
- The primary goal of this project is to provide **minimum** features and capabilities to capture the FD data and create a FD image file.
    - The host program doesn't take any command options. You may need to modify the source code to change the settings.
    - The Arduino firmware is not configurable at the runtime. You need to modify the source code, build-and-reprogram to change its behavior and settings.
- This project uses 'MFM' floppy image format that is defined and developed by me. Please refer to the following projects for the details.
    - [Floppy disk shield 2D](https://github.com/yas-sim/floppy_disk_shield_2d)
    - [FDC bitstream](https://github.com/yas-sim/fdc_bitstream)
    - [MFM image format](https://github.com/yas-sim/fdc_bitstream#mfm-image-data-format)
- You can convert the 'MFM' floppy image file into other floppy image formats with '[image converter](https://github.com/yas-sim/fdc_bitstream/tree/master/image_converter)' in the 'FDC bitstream' project.

------------------------

## Log example
~~~sh
>python fd-capture-lite.py
Searching for Arduino
Arduino is found on "COM8"
Floppy media type = 2D
Peaks = 54 84 88
Estimated cell size = 17
  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  48  49  50  51  52  53  54  55  56  57  58  59  60  61  62  63  64  65  66  67  68  69  70  71  72  73  74  75  76  77  78  79
Image read completed.
Converting read data into MFM disk image data
Completed -> "image.mfm".
~~~
