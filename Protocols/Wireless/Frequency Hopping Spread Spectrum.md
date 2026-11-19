# Frequency Hopping Spread Spectrum (FHSS)

FHSS transmits data by changing carrier frequency over time according to a pseudorandom hop sequence known to both transmitter and receiver. It trades raw spectral efficiency for interference resistance and coexistence in noisy or crowded ISM bands, which is why it keeps coming up for drones and remote-control links.

---

## ⚙️ Overview

- Time is divided into hop slots.
- Each slot uses one frequency from a predefined hopping set.
- The sequence can be:
  - **Pseudo-random** for interception resistance,
  - **Deterministic** for synchronization,
  - **Adaptive** in systems that react to interference (AFH).
- If a slot is corrupted, only that symbol period is usually lost, not the whole packet stream.

Compared to static channels, FHSS reduces sustained impact from narrowband jammers and narrowband interferers.

---

## 🧠 Core Mechanisms

- **Hopset**: pool of channels/frequencies (can include guard-bands and notched channels).
- **Dwell time**: how long transmitter stays on each channel.
- **Hop rate**: hops/sec or hops/packet.
- **Sequence sync**: pre-shared algorithm/seed and timing alignment.
- **Fallback and reseed**: critical for long missions; if clock drift or missed slots happen, links must reacquire the sequence quickly.

---

## 📊 Comparison Chart

| Property                     | FHSS | DSSS | OFDM | Chirp Spread (CSS) | Direct Sequence / DSSS Hybrid |
|-----------------------------|------|------|------|--------------------|-------------------------------|
| Narrowband interferer impact | Low  | Medium | Medium | Low-Med | Low |
| Coexistence in crowded bands | Medium-High | Medium | Medium | Medium | Medium-High |
| Peak throughput             | Medium | Medium | High | Medium-High | Medium |
| Latency/jitter (low SNR)    | Low jitter | Low | Low | Medium | Medium |
| Ease of compliance in ISM    | High | High | Medium | High | High |

---

## 🛠 Use Cases

- RC and telemetry links where interference resilience is more important than max throughput.
- Mesh and relay links that need independent short-hop links under varying local interference.
- Backhaul fallback for UAV swarms when a primary Wi-Fi or LTE path degrades.
- Low-cost, unlicensed deployments where predictable channel planning is hard.

---

## ✅ Strengths

- Works well in non-stationary interference environments.
- Easy to add frequency diversity without complex equalization stacks.
- Interference effects tend to be bursty in time rather than persistent.
- Can be combined with error correction and retransmissions.

---

## ❌ Weaknesses

- Reduced per-hop spectral efficiency relative to wide-carrier OFDM links.
- Sequence coordination overhead and timing sensitivity are non-trivial.
- Regulatory constraints apply to dwell time and minimum channel spacing in some regions.
- If all channels are crowded, performance degrades to random.

---

## 🔧 Related Notes

- [[Adaptive Frequency Hopping]]
- [[Spread Spectrum Methods]]
- [[Chirp Spread Spectrum (CSS)]]
- [[ExpressLRS]]
- [[MAVLink]]
- [[RF Jamming and Anti-Jamming]]

---

## 🌐 External Resources

- [Bluetooth Core Specification Vol 6, Part B (AFH family context)](https://www.bluetooth.com/specifications/specs/bluetooth-core-specification/)
- [Semtech Adaptive FH discussions in LoRa gateways and PHY docs](https://www.semtech.com/products/wireless-rf/lora-transceivers)
- [Wikipedia FHSS overview (high-level refresh)](https://en.wikipedia.org/wiki/Frequency-hopping_spread_spectrum)

