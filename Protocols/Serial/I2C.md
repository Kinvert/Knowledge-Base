---
aliases:
  - I2C
  - I²C
  - Inter-Integrated Circuit
tags:
  - protocol
  - communication
  - digital
  - hardware
title: I2C Protocol
---

# I²C (Inter-Integrated Circuit) Protocol
---

## Overview
I²C is a synchronous, multi-master, multi-slave, packet switched, single-ended, serial communication protocol designed by Philips Semiconductor (now NXP). It uses just two bidirectional lines:
- **SCL** (Serial Clock Line)
- **SDA** (Serial Data Line)

## Key Characteristics
- **Speed**: 
  - Standard Mode: 100 kbit/s
  - Fast Mode: 400 kbit/s
  - Fast Mode Plus: 1 Mbit/s
  - High Speed Mode: 3.4 Mbit/s
- **Addressing**: 7-bit or 10-bit addressing
- **Pull-up Resistors**: Required on both SDA and SCL lines
- **Maximum Devices**: Theoretically 128 devices (7-bit address)

## Hardware Configuration
```
VCC (3.3V/5V)
    │
    ├─[R1]─── SDA ───┬────┬────┬────
    │                │    │    │
    ├─[R2]─── SCL ───┬────┬────┬────
    │                │    │    │
    │             Device1 Device2 Device3
    │
GND
```
- R1, R2: Pull-up resistors (typically 4.7kΩ for 100kHz)

## Protocol Details

### Basic Transaction Structure
1. **Start Condition**: SDA goes LOW while SCL is HIGH
2. **Address Frame**: 7/10-bit address + R/W bit
3. **Acknowledgment Bit**
4. **Data Frame(s)**: 8-bit data + ACK/NACK
5. **Stop Condition**: SDA goes HIGH while SCL is HIGH

### Timing Diagram
```
    START           ADDR + R/W    ACK     DATA       ACK    STOP
    ┌─┐
SDA ┘ └──┐_______┌───────────┐__┌─────────────┐__┌─┘
         │       │           │  │             │  │
SCL ────┐______┌┘─┐_┌─┐_┌─┐_└┐_┌─┐_┌─┐_┌─┐_└┐─────
        │      │  │ │ │ │ │  │ │ │ │ │ │ │  │
```

## Common Applications
- Sensor interfaces
- [[EEPROM]] communication
- Real-time clock modules
- LCD displays
- GPIO expanders

## Implementation Examples

### [[Arduino]] Example
```cpp
#include <Wire.h>

void setup() {
  Wire.begin();        // Join I2C bus as master
  Serial.begin(9600);  // Start serial for output
}

void loop() {
  Wire.beginTransmission(0x68);  // Start transmission to device 0x68
  Wire.write(0x00);             // Send register address
  Wire.endTransmission();       // End transmission
  
  Wire.requestFrom(0x68, 1);    // Request 1 byte from device 0x68
  
  if (Wire.available()) {
    byte data = Wire.read();    // Read received byte
    Serial.println(data);
  }
  
  delay(1000);
}
```

### [[Raspberry Pi]] Example (Python)
```python
import smbus
import time

bus = smbus.SMBus(1)  # Use I2C bus 1
address = 0x68        # Device I2C address

def read_sensor():
    # Read 1 byte from register 0x00
    return bus.read_byte_data(address, 0x00)

while True:
    data = read_sensor()
    print(f"Received: {data}")
    time.sleep(1)
```

### [[Spin]] Driver
```spin
'' SOURCE: https://obex.parallax.com/obex/p1-i2c-driver/
'' =================================================================================================
''
''   File....... jm_i2c.spin
''   Purpose.... Low-level I2C routines (requires pull-ups on SCL and SDA)
''   Author..... Jon "JonnyMac" McPhalen
''               Copyright (c) 2009-2013 Jon McPhalen
''               -- elements inspired by code from Mike Green
''   E-mail.....  
''   Started.... 28 JUL 2009
''   Updated.... 07 APR 2013
''
'' =================================================================================================


con
 
  EE_SDA = 29                                                   ' boot eeprom
  EE_SCL = 28

 
con

  #0, ACK, NAK


var

  long  scl                                                     ' buss pins
  long  sda
    

pub setup

'' Setup I2C using Propeller EEPROM pins

  setupx(EE_SCL, EE_SDA)
         

pub setupx(sclpin, sdapin)

'' Define I2C SCL (clock) and SDA (data) pins

  longmove(@scl, @sclpin, 2)                                    '  copy pins
  dira[scl] := 0                                                '  float to pull-up
  outa[scl] := 0                                                '  write 0 to output reg
  dira[sda] := 0
  outa[sda] := 0

  repeat 9                                                      ' reset device
    dira[scl] := 1
    dira[scl] := 0
    if (ina[sda])
      quit
  
    
pub wait(id) | ackbit

'' Waits for I2C device to be ready for new command

  repeat
    start
    ackbit := write(id)
  until (ackbit == ACK)


pub start

'' Create I2C start sequence
'' -- will wait if I2C buss SDA pin is held low

  dira[sda] := 0                                                ' float SDA (1)
  dira[scl] := 0                                                ' float SCL (1)
  repeat while (ina[scl] == 0)                                  ' allow "clock stretching"

  dira[sda] := 1                                                ' SDA low (0)
  dira[scl] := 1                                                ' SCL low (0)

  
pub write(i2cbyte) | ackbit

'' Write byte to I2C buss
'' -- leaves SCL low

  i2cbyte := (i2cbyte ^ $FF) << 24                              ' move msb (bit7) to bit31
  repeat 8                                                      ' output eight bits
    dira[sda] := i2cbyte <-= 1                                  ' send msb first
    dira[scl] := 0                                              ' SCL high (float to p/u)
    dira[scl] := 1                                              ' SCL low

  dira[sda] := 0                                                ' relase SDA to read ack bit
  dira[scl] := 0                                                ' SCL high (float to p/u)  
  ackbit := ina[sda]                                            ' read ack bit
  dira[scl] := 1                                                ' SCL low

  return (ackbit & 1)


pub read(ackbit) | i2cbyte

'' Read byte from I2C buss

  dira[sda] := 0                                                ' make sda input

  repeat 8
    dira[scl] := 0                                              ' SCL high (float to p/u)
    i2cbyte := (i2cbyte << 1) | ina[sda]                        ' read the bit
    dira[scl] := 1                                              ' SCL low
                             
  dira[sda] := !ackbit                                          ' output ack bit 
  dira[scl] := 0                                                ' clock it
  dira[scl] := 1

  return (i2cbyte & $FF)


pub stop

'' Create I2C stop sequence 

  dira[sda] := 1                                                ' SDA low
  dira[scl] := 0                                                ' float SCL
  repeat while (ina[scl] == 0)                                  ' hold for clock stretch
  
  dira[sda] := 0                                                ' float SDA


dat

{{

  Terms of Use: MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be included in all copies
  or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 

}}
```
## Troubleshooting

### Common Issues
1. **No Communication**
   - Check pull-up resistors
   - Verify power supply
   - Confirm correct address
   - Check wire connections

2. **Intermittent Failures**
   - Check for noise on lines
   - Verify cable length
   - Check pull-up resistor values
   - Look for bus capacitance issues

3. **Bus Lockup**
   - Reset all devices
   - Check for stuck SCL/SDA lines
   - Verify master release of bus

## Components Using I2C
- https://ww1.microchip.com/downloads/en/devicedoc/21754m.pdf

## Related Topics
- [[SPI]] - Alternative serial protocol
- [[Sensors]] - Common I²C devices
- [[Electronics]] - Hardware implementation
- [[Arduino]] - Microcontroller implementation

## References
- [I²C Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
- [[Digital Protocols]]
- [[Serial Protocols]]
