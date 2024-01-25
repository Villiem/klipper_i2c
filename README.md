# i2c sensors on klipper

Prerequisites

- A working instance of Klipper, Moonraker and / or Mainsail/Fluidd
- Access to the Raspberry PI/CB1/Orange pi terminal.
- If using the GPIO pins [set as a secondary mcu] (https://www.klipper3d.org/RPi_microcontroller.html)
- If using something else to run kipper i.e. a virtual machine we are using a raspberry pico.

## **IMPORTANT**

This guide is aimed towards i2c temp sensors, but should work for most i2c devices such as accelerometers.

## Overview
### Why i2c sensors instead of a thermistor?
Normal thermistors are perfectly fine and easy to setup but most sensors allow you to track temperatures, humidity and in some cases VOC's.

### Example 
![](/images/chamber-dashboard.png)

## Connection.
All the i2c sensors use 4 cables, VCC, GND, SDA, SCL.
if your sensor has more ignore them.

![](/images/mount.jpeg)

### **IMPORTANT**
Some sensors use 3.3V or 5V. PLEASE double check voltages, some boards like de CB1 have an output of 1.8V instead of 3.3v in some pins (dunno why).


### If using a raspberry pi
Easiest to setup.

VCC -> 3.3V (or 5v)

GND -> any gnd pin

SDA -> PIN3

SCL -> PIN5

![](/images/pi-wiring.png)
### If using a CB1

VCC -> 3.3V or 5V (Remember sometimes this board uses 1.8v instead of 3.3v)

GND -> Any ground pin

SDA -> pin 10

SDL -> pin 8

![](/images/cb1-wiring.png)
### If using a pico

Any sda and scl pin should work. I used pin 1 and 2.

![](https://service.robots.org.nz/wiki/attach/RaspberryPiPico/Raspberry-Pi-Pico-pinout-diagram.svg)


## Config

### RASPBERRY PI
1. Enable i2c.
```
sudo raspi-config
```
Option 3) interface options -> I4 I2C -> Enable

2. Get the address (my sensor has an address of x77)
While I was writing this I found out Wiringpi has been deprecated, as I read the story I feel bad for the author, so here's the story. 

https://web.archive.org/web/20220405225008/http://wiringpi.com/wiringpi-deprecated/

The original command was:
```
i2cdetect -y 1
```
if using an orange pi check
https://github.com/orangepi-xunlong/wiringOP

### CB1
1. CB1 has no interface like the pi so go to:
Boot/BoardEnv.txt and uncomment:
overlays=light 
or if using more than one, separate them with a space
i.e.
overlays=light tft35_spi

I used
```
sudo nano /boot/BoardEnv.txt
```
ctrl+x and hit yes

3. Come back after everything is set, and type
```
cat /sys/kernel/debug/gpio
```
you should see klipper is using the pins

![](/images/cb1-gpio.png)

### Pico

1. Flash the pico following this guie 
https://www.klipper3d.org/Measuring_Resonances.html?h=pico#flash-the-pico-firmware

2. get the serial

## Klipper

go to the config in mainsail/fluidd and make a new file like bme.cfg and copy
### RPi
```
[mcu rpi] # Not needed if used before
serial: /tmp/klipper_host_mcu 

[temperature_sensor chamber]
sensor_type: BME280
i2c_address: 119 # RIP WIRINGPI
#   Default is 118 (0x76). Some BME280 sensors have an address of 119
#   (0x77).
i2c_mcu: rpi
i2c_bus: i2c.1
#i2c_speed:
#   See the "common I2C settings" section for a description of the
#   above parameters.

[gcode_macro QUERY_CHAMBER]
gcode:
    {% set sensor = printer["bme280 chamber"] %}
    {action_respond_info(
        "Temperature: %.2f C\n"
        "Pressure: %.2f hPa\n"
        "Humidity: %.2f%%\n"
        "Gas Level: %.2f VOC" % (
            sensor.temperature,
            sensor.pressure,
            sensor.humidity,
            sensor.gas_level))}
```

### CB1
```
[mcu host]
serial: /tmp/klipper_host_mcu

[temperature_sensor chamber]
sensor_type: BME280
i2c_address: 119
#   Default is 118 (0x76). Some BME280 sensors have an address of 119
#   (0x77).
i2c_mcu: host
i2c_software_scl_pin: gpio224
i2c_software_sda_pin: gpio225
#i2c_bus: i2c.1
#i2c_speed:
#   See the "common I2C settings" section for a description of the
#   above parameters.

[gcode_macro QUERY_CHAMBER]
gcode:
    {% set sensor = printer["bme280 chamber"] %}
    {action_respond_info(
        "Temperature: %.2f C\n"
        "Pressure: %.2f hPa\n"
        "Humidity: %.2f%%\n"
        "Gas Level: %.2f VOC" % (
            sensor.temperature,
            sensor.pressure,
            sensor.humidity,
            sensor.gas_level))}
```
### pico
```
[mcu rp2040]
serial: /dev/serial/by-id/<YourDevice>
[temperature_sensor chamber]
sensor_type: BME280
i2c_address: 119
#   Default is 118 (0x76). Some BME280 sensors have an address of 119
#   (0x77).
i2c_mcu: rp2040

[gcode_macro QUERY_CHAMBER]
gcode:
    {% set sensor = printer["bme280 chamber"] %}
    {action_respond_info(
        "Temperature: %.2f C\n"
        "Pressure: %.2f hPa\n"
        "Humidity: %.2f%%\n"
        "Gas Level: %.2f VOC" % (
            sensor.temperature,
            sensor.pressure,
            sensor.humidity,
            sensor.gas_level))}
```

### Add [include bme.cfg] to your printer.cfg and you should be able to see your sensor in the dashboard n.n



