# NTP 7-segment Clock

NTP clock with 7-segment display.

## Connection

|Connection|Port||Port|Connection|
|:--|:--|:--|:--|:--|
|Digit\[0\]|GP0||||
|Digit\[1\]|GP1||||
|Digit\[2\]|GP2||||
|Digit\[3\]|GP3||||
|Digit\[4\]|GP4||3V3_OUT||
|Digit\[5\]|GP5||GP28 (ADC3)|Light Sensor|
|Digit\[6\]|GP6||GP27 (i2c1_scl)|EEPROM|
|Digit\[7\]|GP7||GP26 (i2c1_sda)|EEPROM|
|Digit\[8\]|GP8||RUN|Reset Switch|
|Digit\[9\]|GP9||GP22|Setup Switch|
|Digit\[10\]|GP10||GP21|Segment 'b'|
|Digit\[11\]|GP11||GP20|Segment 'a'|
|Digit\[12\]|GP12||GP19|Segment 'f'|
|Digit\[13\]|GP13||GP18|Segment 'g'|
|Segment 'c'|GP14||GP17|Segment 'e'|
|Segment 'dp'|GP15||GP16|Segment 'd'|

## Parts

|Part|Type|
|:--|:--|
|MCU|RaspberryPi Pico2W|
|LED|SS512UWWB (Anode Common White LED)|
|EEPROM|[AT24C32E](https://akizukidenshi.com/catalog/g/g115715/)|
|Light Sensor|[NJL7502L](https://akizukidenshi.com/catalog/g/g102325/)|
|Switch|Tactile Switch (Active Low)|

## 7-segment LED

|Digit\[13:10\]|Digit\[9:8\]|Digit\[7:6\]|Digit\[5:4\]|Digit\[3:2\]|Digit\[1:0\]|
|:--:|:--:|:--:|:--:|:--:|:--:|
|YYYY|MM|DD|HH|MM|SS|

## Setup Page

[Setup Page](https://shapoco.github.io/vlconfig/#form:%7Bt:WiFi%20Setup,e:%5B%7Bk:c,t:t,l:Country,v:JP%7D,%7Bk:s,t:t,l:SSID%7D,%7Bk:p,t:p,l:Password%7D,%7Bk:n,t:t,l:NTP%20Server,v:ntp.nict.jp%7D,%7Bk:z,t:t,l:Time%20Offset,v:%220900%22%7D%5D%7D)
