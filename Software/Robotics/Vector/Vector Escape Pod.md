# Vector Escape Pod

**Vector Escape Pod** concerns the local-backend ecosystem and compatible firmware for [[Anki Vector]]. Separate the historical DDL Escape Pod product, a robot image compatible with that protocol/workflow, and community alternatives such as [[Vector WirePod]].

An `ep` firmware suffix is not evidence of root SSH, arbitrary onboard execution or a developer-unlocked boot chain.

---

## 🧭 Terminology Comparison

| Term | Meaning in this ecosystem | Does it establish developer/root access? |
|---|---|---|
| Escape Pod product | DDL's local-backend offering/workflow | Not by name alone |
| Escape Pod-compatible firmware | Robot software able to use the relevant local backend | No |
| `ep` suffix | Compatibility marker described by WirePod's production setup guide | No |
| WirePod | Community external backend | No |
| OSKR | Official developer-unlock ecosystem | Separate access category |
| Generic production unlock | Community developer-access route | Separate installation/security decision |
| WireOS | Community onboard OS/applications | Depends on the target/install category, not the word Escape Pod |

This terminology prevents a backend setup task from silently becoming a bootloader project.

---

## ⚙️ Firmware Compatibility Is Not a Running Service

Compatible robot firmware determines which backend behavior the robot supports. A separately running external host must still provide and configure the backend.

The WirePod installation guide describes a DDL-produced production firmware variant for Escape Pod compatibility. It explicitly says merely having a `2.0.*` production image is not enough for that documented workflow.

That is evidence for a particular compatibility path—not a universal mapping from every firmware string to identical behavior. Record the full image/version and follow the current instructions applicable to the robot category.

---

## 🔓 Why Unlocking Is Different

A compatible production image can still retain production execution/signing restrictions. A working voice backend or SDK connection therefore does not demonstrate that the owner can replace `vic-engine` or install an arbitrary onboard helper.

Developer access is discussed in [[Vector OSKR]]; bootloader/recovery risks belong to [[Vector ABOOT]].

Do not perform an unlock merely because the desired task is to collect SDK measurements already present in public state.

---

## 🛠️ Setup and Migration Boundaries

The community installation wiki separates:

1. Robot preparation for production devices.
2. External host installation/configuration.
3. Bot authentication with the backend.
4. Different treatment for OSKR/developer-unlocked devices.

Its production preparation includes firmware installation and a user-data clearing step. Entering recovery and clearing user data are not the same operation.

Before migration, preserve the applicable device configuration and understand which identities, settings and SDK pairing material may change. Do not treat a backend transition as guaranteed nondestructive just because the server runs elsewhere.

---

## 📡 SDK and Privacy Implications

[[Vector SDK]] provides robot state/actions through its authenticated interface. Backend compatibility alone does not add raw IMU history, raw encoders or a new sample clock.

A local backend can change where voice processing occurs, but it is not proof that every optional integration, update or robot service is offline. Review the actual selected implementation and configuration.

Keep credentials, certificates and robot identifiers out of public notes. A familiar backend hostname is not a substitute for establishing the intended peer and authentication path.

---

## ⚠️ Current-Service Uncertainty

Source and installation documentation establish compatibility concepts and community workflows. They do not by themselves establish current commercial product availability, entitlement, support, activation-server availability or guarantees from the original vendor.

Use current primary project documentation for the chosen deployment. Do not present historical purchase/activation instructions as a tested present-day service.

---

## 🔗 Related Notes

- [[Vector WireOS]] — onboard OS and application images.
- [[Vector Firmware Architecture]] — backend versus SDK process roles.
- [[Vector Partitions and Recovery]] — user-data and image-change boundaries.
- [[Vector Firmware Development]] — when developer access is actually necessary.

---

## 📚 External Resources

- [WirePod README](https://github.com/kercre123/wire-pod) — project relationship to Chipper and Escape Pod.
- [WirePod installation guide](https://github.com/kercre123/wire-pod/wiki/Installation) — compatible production firmware, `ep` marker and developer-device distinctions.
- [WireOS installation categories](https://os-vector.github.io/vector-docs/2.-Available-CFW/1-WireOS.html).
- [Current developer-unlock status](https://github.com/kercre123/unlocking-vector) — separate from local-backend compatibility.
