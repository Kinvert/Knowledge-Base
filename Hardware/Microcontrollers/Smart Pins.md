# Parallax Propeller 2 – Smart Pins Reference

The **Propeller 2 (P2)** microcontroller from Parallax introduces **Smart Pins**, a powerful and flexible feature that enhances GPIO capabilities by integrating dedicated hardware functionality into each I/O pin. These pins offload tasks from the CPU and allow for deterministic, high-speed I/O operations.

## Overview

- Each of the 64 I/O pins on the P2 can function as a **Smart Pin**.
- Smart Pins are configured via registers and can be set to perform a wide range of autonomous operations.
- This significantly reduces CPU load, especially for timing-sensitive or high-frequency I/O tasks.

## Configuration

Smart Pins are configured via:

- `DIRx`: Direction register (input/output)
- `PINx`: Raw pin state
- `SMARTPINx`: Smart Pin mode configuration
- `X`, `Y`, and `Z` registers: Provide input values and retrieve results depending on mode

Configuration involves:
1. Selecting a **mode** (e.g., serial receive, NCO, pulse width measurement).
2. Setting mode-specific parameters in `Y` and `Z`.
3. Enabling the Smart Pin with the `WXPIN` instruction or equivalent high-level setup.

## Notable Smart Pin Modes

| Mode ID | Function                        | Description                                                                 |
|--------:|----------------------------------|-----------------------------------------------------------------------------|
| 0..3    | Input modes                     | Basic digital/analog input with filtering or Schmitt triggers              |
| 4       | ADC                              | Analog-to-Digital conversion                                               |
| 5       | Comparator                       | Voltage comparison with optional hysteresis                                |
| 6..7    | Digital Output                   | Standard output or open-drain                                              |
| 8..11   | NCO (Numerically Controlled Oscillator) | Output pulses at precise frequencies                                |
| 12..15  | PWM                              | Pulse Width Modulation generation                                          |
| 16..19  | Pulse and transition measurement | Measure pulse width, high/low times, transitions, etc.                     |
| 20..23  | Capture                          | Record timestamps of pin transitions                                       |
| 24..27  | Serial TX                        | Asynchronous serial transmit                                               |
| 28..31  | Serial RX                        | Asynchronous serial receive                                                |
| 32..35  | Quadrature Encoder               | Decode signals from rotary encoders                                        |
| 36..39  | Smart Logic                      | Bitwise logic operations with shift registers                              |
| 40..43  | SPI / I²C / Custom serial        | Flexible serial communication                                              |
| 44..47  | Pattern Match / Streamer Trigger | Match bit patterns on input                                                |

*Note: The actual mode values must be set precisely via assembler or high-level wrappers.*

## Benefits

- **High precision timing**: Sub-clock-cycle accurate for many modes
- **Parallelism**: All 64 pins can operate independently
- **Low CPU overhead**: Frees up cogs for higher-level logic
- **Determinism**: Hardware-timed, not subject to software jitter

## Example Use Cases

- Generate PWM signals for motor control
- Read rotary encoders with built-in quadrature decoding
- Receive and decode serial data at high speeds without CPU load
- Measure pulse widths for ultrasonic distance sensors
- Create frequency generators or capture audio signals with ADC mode

## Code Examples

**Input Pulse Width Measurement (PASM2)**
```pasm
' Measure pulse width on P8 using Smart Pin in POS detector mode

              .coginit 0, @entry

entry         mov       pin, #8
              wrpin     #%0000_0000_000_000000_000_000_000_000, pin  ' Set pin to POS edge smart mode
              wxpin     #0, pin
              dirh      pin
              wypin     #0, pin            ' Arm the smart pin
              
loop          rdpin     width, pin
              jmp       #loop

pin           long      8
width         long      0
```

**Square Wave Output (Spin2)**
```spin2
PUB start_square_wave(pin, freq)
  smartcfg := %0000_0000_000_000000_000_000_101_000  ' NCO mode
  frq := clkfreq / freq

  wrpin(smartcfg, pin)
  wxpin(frq << 16, pin)
  dirh(pin)
```

**Serial Receive (PASM2)**
```pasm
' Setup pin for 115200 baud serial receive on P10

              mov       pin, #10
              wrpin     #%0000_0000_000_000000_000_000_010_100, pin  ' Async RX
              wxpin     ##(clkfreq / 115200), pin
              dirl      pin
              wypin     #0, pin

recv_loop     testp     pin  wz
        if_nz jmp       #recv_loop        ' Wait for data
              rdpin     rxval, pin
              jmp       #recv_loop

pin           long      10
rxval         long      0
```

**High Speed Counting (Spin2)**
```spin
PUB start_counter(pin)
  smartcfg := %0000_0000_000_000000_000_000_001_001  ' Count A events
  wrpin(smartcfg, pin)
  wxpin(0, pin)
  dirl(pin)
  wypin(0, pin)
```

## Considerations

- You may need to coordinate Smart Pin setup with Streamer (for high-speed data), interrupts, or cog-specific code
- Incorrect register setup may cause undefined behavior
- Some modes require a clock source or calibration

## Related

- [[Parallax Propeller 2]]
- [[Parallax Propeller 1]]
- [[Microcontrollers]]

## Resources

- [Propeller 2 Documentation](https://docs.parallax.com/)
- [PNut Tool (official assembler/development tool)](https://www.parallax.com/propeller-2/)
- Example code on [Parallax forums and GitHub](https://forums.parallax.com/)
