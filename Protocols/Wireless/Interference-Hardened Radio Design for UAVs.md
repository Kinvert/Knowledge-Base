# Interference-Hardened Radio Design for UAVs

This is the hardware/software co-design view for radios that need to keep C2 and telemetry alive under dense interference, dynamic motion, and self-generated electrical noise.

---

## ⚙️ Overview

You cannot fully solve interference in software if the RF front-end, layout, and power architecture are weak. Hardened design has five pillars:

1. **Spectrum policy** (what channels and bands you are allowed to operate),
2. **RF integrity** (filters, LNA, PA linearity, PCB layout),
3. **Antenna system design** (polarization and placement),
4. **Adaptive protocol behavior** (AFH/CSS + retries + timeout policy),
5. **Operational strategy** (fallback channel plans and failsafe).

---

## 🧠 Hardware Design Levers

- **Front-end filtering**: reject nearby harmonics and in-band emitters.
- **Antenna placement**: reduce coupling to ESC/motor current spikes.
- **Cable and connector integrity**: avoid poorly shielded feed.
- **Diversity options**: if size allows, use multiple antennas for redundancy.
- **Grounding and return path quality**: lowers noise figure and spur susceptibility.
- **Power path coupling**: isolate RF supply noise from motor drivers.

---

## 📊 Design Tradeoff Matrix

| Lever | Robustness gain | Cost / complexity | Typical caveat |
|---|---|---|---|
| Band-pass filtering | High | Low-Med | Insertion loss risk |
| Antenna isolation spacing | High | Medium | Form-factor constraints |
| Shielding and partitioning | Medium | Medium | Weight and assembly cost |
| Antenna polarization diversity | Medium | Medium | Mechanical mounting complexity |
| Frequency agility + hopping | High | Medium | Tight timing sync requirement |
| Logging + telemetry hooks | High | Low | Data management overhead |

---

## 🛠 Implementation Pattern for AI Loops

Use parameter files to describe hardware-safe operational envelopes:

- `allowed_bands`,
- `min_rssi_for_retry`,
- `recovery_frequency_set`,
- `max_consecutive_bad_frames`,
- `grounded_antenna_exclusion_zones`.

That makes experiments actionable and prevents overfitting to one “nice” flight condition.

---

## ✅ Strengths

- Better mean-time-between-failures from hardening at the signal path.
- Less sensitivity to transient RF events.
- More predictable safety behavior across mission sets.

---

## ❌ Weaknesses

- Physical constraints (payload, nacelle geometry) can block ideal placements.
- More expensive and longer build loops.
- Over-tuning can reduce agility if every edge-case gets locked by policy.

---

## 📚 Related Concepts

- [[RF Jamming and Anti-Jamming]]
- [[Frequency Hopping Spread Spectrum]]
- [[Adaptive Frequency Hopping]]
- [[MIMO for Wireless Links]]
- [[Mesh Networking]]

---

## 🌐 External Resources

- [FCC RF design and emissions references](https://www.fcc.gov/)
- [Analog Devices RF design considerations (general)](https://www.analog.com/en/landing-pages/001/whitepapers.html)
- [General RF EMC and layout guidance](https://www.itu.int/en/ITU-T/)

