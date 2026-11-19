# Chirp Spread Spectrum (CSS)

Chirp Spread Spectrum encodes symbols using linear up/down frequency sweeps (chirps) and then performs correlation-based demodulation over a known chirp bank. It is the core physical-layer idea behind LoRa-style modulation families and many low-power long-range systems.

---

## ⚙️ Overview

In CSS, each symbol maps to a time-frequency sweep shifted or offset in phase/frequency. Receiver correlation against expected chirp patterns gives resilience against narrowband interference and clock offsets.

This matters for drones and long-range links because:

- Multipath and Doppler around moving airframes is common.
- You often have weak-signal margin more than raw data bandwidth.
- Lower spectral efficiency can still be acceptable for commands + sparse telemetry.

---

## 🧠 Core Concepts

- **Sweep rate / bandwidth**: determines resolution and symbol duration.
- **Spreading factor**: controls symbol time and sensitivity (higher = longer but more robust).
- **Chirp generation and synchronization**: front-end synchronization is central to good bit error performance.
- **Doppler tolerance**: sweep-based demodulation handles some drift better than many phase-sensitive schemes.

---

## 📊 Comparison Chart

| Property | CSS | FHSS | DSSS | OFDM | Narrowband FSK |
|---|---|---|---|---|---|
| Narrowband jammer tolerance | High | High | Medium | Medium | Low |
| Link range at low SNR | High | Medium-High | Medium | Medium | Low |
| Payload rate | Low-Med | Low-Med | Low-Med | High | Medium |
| Sync complexity | Medium | Low | Medium | High | Low |
| Typical drone use | Telemetry/position updates | RC backup links | Mixed | Video/faster telemetry | Simple legacy links |

---

## 🧪 Practical Implications

For agentic optimization loops:

- `bandwidth` and `spreading_factor` are primary knobs.
- You usually trade:
  - Higher spread -> fewer bytes/sec but lower packet error at range,
  - Lower spread -> more throughput but sensitivity drops.

If your objective is CFD loop reliability, CSS-style links often improve mission completion rate even with fewer bytes per frame.

---

## ✅ Strengths

- Good non-line-of-sight and urban canyon behavior at low data rates.
- Built-in resilience to interference spikes.
- Simpler fallback behavior than complex adaptive MIMO stacks in many simple modules.

---

## ❌ Weaknesses

- Raw throughput limits can become bottleneck for payload-heavy telemetry.
- Heavily tuned parameters are hardware-specific.
- Demodulator compute can be non-trivial on tiny MCUs if not offloaded.

---

## 📚 Related Concepts

- [[LoRa]]
- [[Spread Spectrum Methods]]
- [[Adaptive Frequency Hopping]]
- [[RF Jamming and Anti-Jamming]]

---

## 🌐 External Resources

- [Semtech LoRa / CSS technical overview](https://www.semtech.com/lora)
- [LoRaWAN Specification overview](https://lora-alliance.org/resource-hub)

