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
- EEPROM communication
- Real-time clock modules
- LCD displays
- GPIO expanders

## Implementation Examples

### Arduino Example
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

### Raspberry Pi Example (Python)
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

## Related Topics
- [[SPI]] - Alternative serial protocol
- [[Sensors]] - Common I²C devices
- [[Electronics]] - Hardware implementation
- [[Arduino]] - Microcontroller implementation

## References
- [I²C Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
- [[Digital Protocols]]
- [[Hardware]]
