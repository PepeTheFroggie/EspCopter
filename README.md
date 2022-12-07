# EspCopter

Project using the ESP8266 and MPU6050 to control a quadcopter. Done with arduino for esp8266 ide. RC uses the ack-less ESPNOW protocol. See "espnow_RC_TX" for transmitter.

Serial Konze 0xF5 protocol:
The output to esc's is wired to the serial output pin #2 of the esp. Name is D4, GPIO02, TXD1. Baudrate is 128kb.

Normal PWM output: (#define PWMOUT)
The output to esc's is wired to the pins 14, 12, 13, 15. Refresh rate is about 6ms, pulses from 1ms to 2ms.
Important! For normal PWM use 80mhz cpu frequency for ESP8266. Use serial mode to write internal eeprom lik eacc calib, it pwm mode it will crash.

RC pulse input sequence is adaptable, default is orangerx sequence: 
* define ROL 0
* define PIT 1
* define THR 2
* define RUD 3
* define AU1 4

The copter will only arm after 1 second zero throttle. The copter will shut down motors RC data fails for more than 100ms.

The PID for level (acc) and gyro can be adjusted individually.  

Serial console commands:
* A - acc calib
* D - write default PID to eeprom
* R - read actual PID
* Wpxx, Wixx, Wdxx - write gyro PID
* WPxx, WIxx, WDxx - write level PID
* WS - Store PID in EEPROM

Data display over serial console (use arduino plot function)
* 0 - off
* 1 - Gyro values
* 2 - Acc values
* 3 - Angle values
* 4 - RC values
* 5 - Cycle time, RC time

Be sure you do acc_calib and write default params to eeprom before taking off! 
Acc_calib is done by placing the copter on a level surface and press A on the serial console.
Default PID are written by press P on the serial console. Read them with R.

6050 wiring: 
* SCL to D1 
* SDA to D2 
* VCC to 3.3V 
* GND to GND.
 
![wiring.png](Wiring.png "Wiring")

![DSC02280.jpg](DSC02280.jpg "Testcopter")

[Testflight](https://youtu.be/OhVVPzNwx6M)   
[Telemetry](https://youtu.be/0AWHVxgIqno)   

