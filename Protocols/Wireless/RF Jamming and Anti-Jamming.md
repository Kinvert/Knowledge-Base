# RF Jamming and Anti-Jamming

RF jamming is intentional interference that raises the noise floor or blocks channel access. Anti-jamming for drones usually combines physical-layer hardening, protocol adaptation, routing redundancy, and control-plane policies.

---

## ⚙️ Overview

There are two broad classes:

- **Barrage jamming**: broad-spectrum energy raising noise floor.
- **Spot jamming**: targets known channels/signals with high duty-cycle energy.

Anti-jamming is never single-tech; practical systems blend techniques across PHY, MAC, and application logic.

---

## 🧠 Core Threats

- Wideband noise in 2.4 GHz consumer bands.
- Replay/spoof attempts against weak authentication paths.
- Gateway denial through control-plane saturation.
- Localized denial from high-gain directional emitters.

---

## 📊 Countermeasure Table

| Countermeasure | Layer | What it helps | Tradeoff |
|---|---|---|---|
| AFH/FHSS | PHY/MAC | Narowband spot interference | More complexity / timing requirements |
| CSS mode / larger spreading | PHY | Low-rate robustness | Lower throughput |
| Redundant links | Network | Single-link outages | Power + complexity |
| Message signing / cryptography | Application | Spoofing | Latency + key management |
| Frequency diversity | PHY | Jamming on one slice | More channel scan logic |
| Power control | PHY | Link margin in weak spots | Battery impact |
| Spatial reroute via mesh | Network | Localized jammer dead zones | Routing overhead |

---

## 🛠 Use Cases

- Mission-critical control with adversarial RF threat.
- Events with dense consumer RF contention.
- Large-area operations where no single clear channel remains stable.

---

## ✅ Strengths

- Layered defense reduces probability of single failure mode.
- Can keep at least low-rate fail-safe alive during attack.

---

## ❌ Weaknesses

- Overhead can be high for tiny MCU-only links.
- Badly tuned adaptation loops may amplify instability.
- If one radio front-end is badly filtered, no software fix fully solves hardware noise floor issues.

---

## 📚 Related Concepts

- [[Adaptive Frequency Hopping]]
- [[Spread Spectrum Methods]]
- [[RF Link Resilience Stack for Drones]]
- [[Interference-Hardened Radio Design for UAVs]]
- [[Drone C2 Radio Protocols]]

---

## 🌐 External Resources

- [FCC interference and unauthorized operations summary](https://www.fcc.gov/consumers/guides/causing-noise)
- [NIST wireless interference baseline resources](https://www.nist.gov/)
- [NATO-style EW and anti-jam references](https://www.nato.int/)

