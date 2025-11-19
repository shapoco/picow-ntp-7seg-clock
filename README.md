# [WIP] Animated 7-segment Clock

<p align="center"><img src="./img/cover.gif" width="1280"></p>

NTP clock with animated 7-segment display.

## Components

|Part|Type|
|:--|:--|
|Board|RaspberryPi Pico2W|
|LED|anode common 7-segment LEDs (\*),<br>and same color chip LEDs|
|EEPROM|[AT24C32E](https://akizukidenshi.com/catalog/g/g115715/)|
|RTC|[RX-8025NB](https://akizukidenshi.com/catalog/g/g108585/)|
|Light Sensor|[NJL7502L](https://akizukidenshi.com/catalog/g/g102325/)|
|Switch|tactile switch|
|Battery|CR2032|
|Others|some resistors/capacitors/diodes|

(\*) I used `SS512UWWB`, which I bought at a local parts store, but I couldn't find its datasheet.

## Connection

![](./img/connection.png)

> [!NOTE]
> The value of the LED's current-limiting resistor should basically be determined based on V<sub>F</sub> and I<sub>F</sub>, but keep in mind that because it is a dynamic scanning method, the LED will appear much dimmer than if current were to flow continuously.

> [!WARNING]
> To avoid burning your Raspberry Pi Pico, I recommend inserting a buffer IC between the LED and the Pico (I'm lazy so I just wired them directly, but so far I haven't had any problems).

|Port|Connection|
|:--|:--|
|GP0|Digit\[0\]|
|GP1|Digit\[1\]|
|GP2|Digit\[2\]|
|GP3|Digit\[3\]|
|GP4|Digit\[4\]|
|GP5|Digit\[5\]|
|GP6|Digit\[6\]|
|GP7|Digit\[7\]|
|GP8|Digit\[8\]|
|GP9|Digit\[9\]|
|GP10|Digit\[10\]|
|GP11|Digit\[11\]|
|GP12|Digit\[12\]|
|GP13|Digit\[13\]|
|GP14|Segment 'c'|
|GP15|Segment 'dp'|
|GP16|Segment 'd'|
|GP17|Segment 'e'|
|GP18|Segment 'g'|
|GP19|Segment 'f'|
|GP20|Segment 'a'|
|GP21|Segment 'b'|
|GP22|Setup Switch|
|RUN|Reset Switch|
|GP26 (i2c1_sda)|EEPROM / RTC|
|GP27 (i2c1_scl)|EEPROM / RTC|
|GP28 (ADC2)|Light Sensor|

## Building Firmware

1. Install [Pico SDK](https://github.com/raspberrypi/pico-sdk) and set `PICO_SDK_PATH` environment variable.
2. Run `make` in `firmware/` directory.

    ```sh
    cd firmware
    make
    ```

3. `ntp_clock.uf2` generated in `firmware/bin/` directory.

## Setting Wi-Fi and NTP server

### via USB-Serial

1. Connect Raspberry Pi Pico to PC with USB cable.
2. Wait until the 7seg-display lights up.
3. Open a serial terminal with baudrate 115.2 kHz.
4. Press the setup switch ("ConFiG" will appear on the display).
5. Follow the prompts in the serial terminal to complete your settings.
6. The display will go blank for a few seconds, then show the time once the Wi-Fi connection and NTP synchronization are successful.

### via Light Sensor

1. Open the setup page from the link below and fill the form.

    - [Japan](https://shapoco.github.io/vlconfig/#form:%7Bt:Setup%20NTP%20Clock,e:%5B%7Bk:c,t:t,l:WiFi%20Country%20Code,v:JP,p:%28Worldwide%29%7D,%7Bk:s,t:t,l:SSID%7D,%7Bk:p,t:p,l:Password%7D,%7Bk:n,t:t,l:NTP%20Server,v:ntp.nict.jp,p:%28pool.ntp.org%29%7D,%7Bk:z,t:t,l:UTC%20Offset%20%28HHMM%29,v:%220900%22,p:%28UTC%29%7D%5D%7D)
    - [United States (EST)](https://shapoco.github.io/vlconfig/#form:%7Bt:Setup%20NTP%20Clock,e:%5B%7Bk:c,t:t,l:WiFi%20Country%20Code,v:US,p:%28Worldwide%29%7D,%7Bk:s,t:t,l:SSID%7D,%7Bk:p,t:p,l:Password%7D,%7Bk:n,t:t,l:NTP%20Server,p:%28pool.ntp.org%29%7D,%7Bk:z,t:t,l:UTC%20Offset%20%28HHMM%29,v:%22-0400%22,p:%28UTC%29%7D%5D%7D)

2. Press the setup switch ("ConFiG" will appear on the 7seg-display).
3. Place the light sensor over the lamp on the setup page and click the Submit button.
4. Wait until the light stops flashing.
5. The display will go blank for a few seconds, then show the time once the Wi-Fi connection and NTP synchronization are successful.

The received configuration data is stored in EEPROM.

> [!WARNING]
> The light signal and the data in the EEPROM are not encrypted, so be careful not to let someone spy on you watching the flashing or steal your clock.

Once configured, NTP synchronization will occur at a random time between 4:00 and 5:00 every morning.

For more information about the configuration interface, see the [VLConfig](https://github.com/shapoco/vlconfig) repository.

## Brightness Calibration

The clock learns changes in the room's brightness to determine the display brightness. The maximum brightness is learned within one minute after startup, and the minimum brightness is learned over time.

To calibrate the display brightness, follow these steps:

1. Place the clock in the desired location.
2. Turn on the power or press the reset switch.
3. Turn the room lights on for 30 seconds
4. Turn the room lights off or shield the light sensor for 30 seconds.

## Debug Mode

Press the reset switch while holding down the setup button to enter debug mode, in which trace information is displayed on the UART.

