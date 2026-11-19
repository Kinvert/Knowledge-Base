# Spread Spectrum Methods

Spread spectrum is a family of techniques that intentionally spreads signal energy across a larger frequency dimension than minimum-bandwidth modulation. The payoff is usually robustness to interference, lower per-channel detectability, and better coexistence in shared spectrum.

---

## ⚙️ Overview

The common methods are:

- **Frequency Hopping Spread Spectrum (FHSS)**: switches carrier across many channels.
- **Direct Sequence Spread Spectrum (DSSS)**: multiplies data by a higher-rate pseudorandom chip sequence.
- **Chirp Spread Spectrum (CSS)**: encodes data with frequency-swept chirps.
- **Hybrid variants**: practical systems sometimes mix these concepts with coding and forward error correction.

For drone control systems, FHSS and CSS are especially common due to high interference resistance versus implementation complexity.

---

## 🧠 Core Tradeoffs

- **Robustness vs efficiency**:
  - Higher spread factor or larger hopspace improves interference tolerance but lowers raw bit rate.
- **Synchronization burden**:
  - Spreading often needs tighter sync than simple narrowband links.
- **Regulatory context**:
  - Unlicensed bands often impose spectral occupancy, duty-cycle, and power limits.
- **Antenna and hardware impacts**:
  - Spreading schemes are sensitive to front-end linearity and oscillator stability.

---

## 📊 Comparison Chart

| Method | Interference Robustness | Receiver Complexity | Throughput Efficiency | Typical Drone-Relevant Examples | Best For |
|---|---|---|---|---|---|
| FHSS | High | Low-Med | Med | Legacy RC, proprietary RC modems | Narrowband blockers, channel reuse |
| DSSS | High | Med | Med | Wi-Fi legacy variants, GNSS-like systems | Multipath-heavy and moderate interference |
| CSS | High | Med | Med-High | Long-range LPWAN (LoRa family) | Long range and low-power links |
| OFDM | Med | High | High | High-rate links, Wi-Fi-like stacks | Throughput over robust coding |
| FSK/GMSK narrowband | Low-Med | Low | High | Very short-range specialized links | Simple short-range links |

---

## 🛠 Use Cases

- Long-range telemetry where every extra dB margin matters.
- C2 links crossing mixed-spectrum environments (RF + Wi-Fi + Bluetooth).
- Resilient swarm relay where link availability is more important than bitrate.

---

## ✅ Strengths

- Good fit for shared bands where deterministic channel planning is hard.
- Often improves survivability against narrowband interference attacks.
- Enables fallback patterns (switch between robust mode and efficiency mode).

---

## ❌ Weaknesses

- Increased implementation complexity for synchronization and timing.
- Potentially lower deterministic throughput vs uncoded OFDM.
- More sensitive to oscillator mismatch and timing jitter than purely narrowband.

---

## 📚 Related Concepts

- [[Frequency Hopping Spread Spectrum]]
- [[Adaptive Frequency Hopping]]
- [[Chirp Spread Spectrum (CSS)]]
- [[RF Jamming and Anti-Jamming]]
- [[MIMO for Wireless Links]]

---

## 🌐 External Resources

- [Semtech LoRa (CSS family context)](https://www.semtech.com/lora)
- [NIST overview on spread spectrum for non-harmful interference concepts](https://www.nist.gov)
- [IEEE wireless PHY reference pages](https://www.ieee.org)

