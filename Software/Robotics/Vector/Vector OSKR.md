# Vector OSKR

**Vector OSKR** (Open Source Kit for Robots) refers to the official developer-unlock ecosystem for [[Anki Vector]]. It is not another name for SDK authentication, a local voice backend or every form of community developer access.

The key distinction is permission to install/run modified onboard software, rather than permission to call an existing API.

---

## 🧭 Access Categories

| Category | What it provides or describes | What it does not establish |
|---|---|---|
| Production robot | Normal signed firmware and supported interfaces | Arbitrary onboard application execution |
| SDK credentials | Access to implemented authenticated APIs | Root SSH or an unlocked boot chain |
| Escape Pod-compatible firmware | Compatibility with a local backend | OSKR/developer unlock |
| Official OSKR unlock | Historical official developer transition | A reversible return to production state |
| Generic production unlock | Community developer-access route documented since 2025 | Identical identity/signing state to official OSKR |
| Prototype/development robot | A distinct manufacturing/developer category | Compatibility with any downloaded image |

Choose images according to the actual category, not merely the fact that an SSH connection works.

---

## 📜 Historical Official Workflow

The official unlock documentation describes obtaining a robot-specific unlock image using QSN and ESN identifiers. It explicitly warns that unlocking is permanent and that modifying ABOOT and recovery can leave the robot unable to boot if interrupted.

It also records historical functionality changes, including disabling Alexa mode. This is evidence about that official workflow—not proof that old per-device image delivery, accounts or commercial support remain available now.

Do not combine selected steps from an old OSKR guide with a current community unlock guide.

---

## 🔓 Community Unlocking Changed the Landscape

The unlock maintainer's archived README says generic unlocking became available in **April 2025**, and explicitly retracts older claims that Vector cannot be unlocked.

The linked current guide identifies `Unlock-Prod.ota`, followed by installation of custom firmware. It separately describes SSH as optional for extra developer features.

This establishes a documented community route. It does **not** authenticate a particular downloaded image, establish its suitability for a specific unit or prove a recovery method if bootloader installation fails.

See [[Vector ABOOT]] for why that risk is different from replacing an ordinary Linux executable.

---

## 📦 OSKR Versus WireOS Image Selection

[[Vector WireOS]] documentation distinguishes:

- `dev` images for production robots unlocked through the generic production-unlock path.
- `oskr` images for robots unlocked through the official OSKR program.
- Separate instructions for certain prototypes/development states.

These are compatibility categories, not interchangeable quality levels. A source build or downloaded OTA must match the selected device's actual state and the maintainer's current installation instructions.

---

## 🛡️ Security and Identity

Developer access changes who can run software and access files. Treat factory calibration, identity keys, authentication tokens and SSH material as per-device secrets.

The community guide offers a shared root-key path. Shared convenience authentication is not private owner-specific authentication. Review accepted keys and host identity while retaining a proven access/restoration route; do not blindly remove the only working key.

A working SDK or local backend does not justify publishing credentials or opening unauthenticated debug services.

---

## ✅ Before Crossing the Unlock Boundary

- Identify the robot generation, current image and existing developer state.
- Use one applicable, current maintainer workflow rather than an improvised combination.
- Establish artifact provenance and keep stable power/network conditions.
- Separate accessible backups from data that cannot be obtained before unlock.
- Accept that an untested backup is not a demonstrated unbrick method.
- Keep another robot or setup unchanged when a reference device is available.

**Stopping an application, rebooting or removing a runtime override does not reverse an unlock.** No successful unlock or guaranteed hard-brick recovery is asserted here.

---

## 🔗 Related Notes

- [[Vector Partitions and Recovery]] — what slots and per-device storage protect.
- [[Vector Firmware Development]] — lower-risk work after developer access exists.
- [[Vector SDK]] — capabilities available without arbitrary onboard execution.
- [[Vector Escape Pod]] — local-backend firmware compatibility.
- [[SSH]] — remote access and authentication foundations.

---

## 📚 External Resources

- [Historical official unlock instructions](https://support.anki.bot/article/571-detailed-unlock-steps) — permanence, QSN/ESN and boot/recovery warnings.
- [Maintainer status correction](https://github.com/kercre123/unlocking-vector) — April 2025 generic unlock and obsolete older advice.
- [Current community unlock guide](https://unlock-prod.skittle.dev/) — consult directly before considering any operation.
- [WireOS device-category installation instructions](https://os-vector.github.io/vector-docs/2.-Available-CFW/1-WireOS.html).
