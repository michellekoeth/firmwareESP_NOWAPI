# firmware

To flash the firmware into the ESP32 board you will need to install the Arduino IDE and all the required libraries from the Libraries directory. 

Here you can find a tutorial on how to install the libraries: https://www.arduino.cc/en/Guide/Libraries

Installation instructions using Arduino IDE Boards Manager
==========================================================

Starting with 1.6.4, Arduino allows installation of third-party platform packages using Boards Manager. We have packages available for Windows, Mac OS, and Linux (32 and 64 bit).

- Install the current upstream Arduino IDE at the 1.8 level or later. The current version is at the [Arduino website](http://www.arduino.cc/en/main/software).
- Start Arduino and open Preferences window.
- Enter ```https://dl.espressif.com/dl/package_esp32_index.json``` into *Additional Board Manager URLs* field. You can add multiple URLs, separating them with commas.
- Open Boards Manager from Tools > Board menu and install *esp32* platform (and don't forget to select your ESP32 board from Tools > Board menu after installation).

ESP32 Board Pin out
===================
![ESP32 Board Pin out](https://github.com/diybar/firmware/blob/master/images/esp32_pinout.jpg)