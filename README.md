# IMC Proftaak
### Configure the project

```
idf.py menuconfig
```

* Set WiFi SSID and WiFi Password and Maximum retry under Example Configuration Options.
* Set the Rotary Encoder Address.
* Set the LCD Address.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
