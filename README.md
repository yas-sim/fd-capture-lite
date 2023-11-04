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

## Prerequisites
The host program requires 'pyserial' module.
~~~sh
pip install -r requisites.txt
~~~

----
|Signal|[Arduino shield pins](https://docs.arduino.cc/static/6ec5e4c2a6c0e9e46389d4f6dc924073/2f891/Pinout-UNOrev3_latest.png)|FDD 34pin conn|Note|
|:-:|:-:|:-:|---|
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

## Log example 1
~~~sh
python fd-capture-lite.py
** FD-CAPTURE-LITE
[HOST] Searching for Arduino
[HOST] Arduino is found on "COM8"
[HOST] Floppy media type setting = 2D
[ARDUINO] Floppy media type setting = 2D
[ARDUINO] FDD type setting = 2HD
[ARDUINO] Number of step pulse(s) per track = 2
[ARDUINO] Checking pulse condition of the floppy disk.
[ARDUINO] Peaks = 54 88 120
[ARDUINO] Estimated cell size = 33
[ARDUINO] Estimated input capture offset = 12
[ARDUINO] Recommended total offset = 28
0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79
[HOST] Image read completed.
[HOST] Converting read data into MFM disk image data
0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79
[HOST] Completed -> "image.mfm".
~~~

## Log example 2
Reading a disk image with fd-capture-lite, and convert the generated MFM file into D77 disk image format using the image_converter in the fdc_bitstream project.
~~~sh
N:\work\fd_capture_lite>python fd-capture-lite.py
** FD-CAPTURE-LITE
[HOST] Searching for Arduino
[HOST] Arduino is found on "COM8"
[HOST] Floppy media type setting = 2D
[ARDUINO] Floppy media type setting = 2D
[ARDUINO] FDD type setting = 2HD
[ARDUINO] Number of step pulse(s) per track = 2
[ARDUINO] Checking data pulse condition of the floppy disk.
[ARDUINO]     Peaks = 54 88 120
[ARDUINO]     Estimated cell size = 33
[ARDUINO]     Estimated input capture offset = 12
[ARDUINO]     Recommended total offset = 28
0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79
[HOST] Image read completed.
[HOST] Converting read data into MFM disk image data
0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79
[HOST] Completed -> "image.mfm".

N:\work\fd_capture_lite>c:\Users\yas_s\source\repos\fdc_test\bin\Release\image_converter.exe -i image.mfm -o image.d77 -v
Reading image.mfm.
VFO type : 5
Gain L=0.3 , Gain H=1
   0:  16/   0     1:  16/   0     2:  16/   0     3:  16/   0     4:  16/   0
   5:  16/   0     6:  16/   0     7:  16/   0     8:  16/   0     9:  16/   0
  10:  16/   0    11:  16/   0    12:  16/   0    13:  17/   1    14:  16/   0
  15:  16/   0    16:  15/   0    17:  16/   0    18:  16/   0    19:  16/   0
  20:  16/   0    21:  16/   0    22:  16/   0    23:  16/   0    24:  16/   0
  25:  16/   0    26:  16/   0    27:  16/   0    28:  16/   0    29:  16/   0
  30:  16/   0    31:  16/   0    32:  16/   0    33:  16/   0    34:  16/   0
  35:  16/   0    36:  16/   0    37:  16/   0    38:  16/   0    39:  16/   0
  40:  16/   0    41:  16/   0    42:  16/   0    43:  16/   0    44:  16/   0
  45:  16/   0    46:  16/   0    47:  16/   0    48:  16/   0    49:  16/   0
  50:  16/   0    51:  16/   0    52:  16/   0    53:  16/   0    54:  16/   0
  55:  16/   0    56:  16/   0    57:  16/   0    58:  16/   1    59:  16/   0
  60:  16/   0    61:  16/   0    62:  16/   0    63:  16/   0    64:  16/   0
  65:  16/   0    66:  16/   0    67:  16/   1    68:  16/   0    69:  16/   0
  70:  16/   0    71:  16/   0    72:  16/   0    73:  16/   0    74:  16/   0
  75:  16/   0    76:  16/   0    77:  16/   0    78:  16/   1    79:  16/   0

**TOTAL RESULT(GOOD/BAD):1276 4
image.mfm, -> image.d77
~~~

## Note
- The host program is tested on a Windows 11 system.
