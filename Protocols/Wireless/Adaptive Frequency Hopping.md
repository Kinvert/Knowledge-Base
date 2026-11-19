# Adaptive Frequency Hopping (AFH)

Adaptive frequency hopping adjusts an FHSS hopset dynamically based on live channel quality, interference reports, and packet health. The result is a more robust control link under variable interference, especially in 2.4 GHz environments where Bluetooth, Wi-Fi, radar, and consumer noise dominate.

---

## ⚙️ Overview

AFH is used to avoid bad channels without changing the higher-level protocol framing. A link keeps sending on a base FHSS framework but marks channels as “bad,” “uncertain,” or “good” and alters the usable channel map.

In practical systems this is usually:

- Monitor link quality over a sliding window (RSSI/CRC/error counters/snr history),
- Penalize unstable channels,
- Build a reduced whitelist/priority list,
- Re-negotiate or update hop sequences with minimal interruption.

---

## 🧠 Core Mechanisms

- **Channel quality scoring**: recent packet error events and noise floor are combined into per-channel fitness.
- **Blacklist / graylisting**: bad channels are temporarily removed to avoid repeated failures.
- **Hopping map compression**: only a reduced subset of frequencies is retained when interference is severe.
- **Reinforcement-like adaptation**: aggressive channels are reintroduced if conditions improve.
- **Synchronization safety**: both endpoints must converge on the same map quickly.

Most AFH implementations are bounded by available packet payload; map updates are usually compact.

---

## 📊 Comparison Chart

| System | AFH Behavior | Typical Hop Dynamics | Good For | Limitation |
|---|---|---|---|---|
| Bluetooth LE (classic AFH-style model) | Periodic channel classification | ~Adaptive over connection intervals | Dense consumer noise, low-latency control | 802.11 co-channel crowding can still saturate |
| ExpressLRS-like RC systems | Dynamic frequency selection + hopping patterns | Configurable by firmware | RC/telemetry with LOS LOS jitter | Vendor-specific implementation details |
| Generic FHSS radios | Often static maps with optional runtime re-mapping | Moderate | Industrial telemetry | Map management overhead |
| Wi-Fi dynamic channel selection | Coarse channel selection, not hop-level | Minute-scale | Throughput path optimization | Slower adaptation to burst interference |
| MIMO/FEC-only designs | No AFH, rely on coding/spatial | None | Higher-rate links | Vulnerable to narrowband persistent blockers |

---

## 🛠 Use Cases

- Race drones and long-range RC where a 2.4 GHz channel gets polluted temporarily by local Wi-Fi noise.
- UGV/UAV missions with nearby industrial radios (forklifts, BLE beacons, industrial WLAN).
- Testbeds where agents perform repeated sweeps and may induce self-interference.

---

## ✅ Strengths

- Lower packet loss in variable interference versus fixed hopping.
- Better channel reuse because links naturally avoid occupied slots.
- Can dramatically reduce CRC failures on cheap low-power transceivers.

---

## ❌ Weaknesses

- Additional control-plane burden to keep TX/RX maps synchronized.
- Can be gamed if attackers infer and poison adaptation metrics.
- Over-aggressive blacklisting can reduce diversity too much.

---

## 🔧 Design Notes for Agents

For agent-driven parameter sweeps, expose AFH behavior as parameters:

- `scan_window_ms`
- `blacklist_threshold`
- `reprobe_period_ms`
- `max_blacklisted_pct`

This lets an optimization loop search robust maps for specific terrain and mission profiles.

---

## 🌐 External Resources

- [Bluetooth Core Spec (AFH-related behavior under frequency management)](https://www.bluetooth.com/specifications/specs/bluetooth-core-specification/)
- [LoRa/FSK transceiver configuration patterns on Semtech platforms](https://www.semtech.com/lora)
- [Broadcom Bluetooth adaptive frequency hopping whitepaper excerpts (vendor reference)](https://www.broadcom.com/products/bluetooth)

