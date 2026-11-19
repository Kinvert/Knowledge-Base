---
title: ESP32
aliases: [Espressif ESP32, ESP32 Microcontroller]
tags: [Microcontrollers, WiFi, Bluetooth, Espressif]
---

# ESP32

The **ESP32** is a powerful, low-cost microcontroller with built-in **Wi-Fi** and **Bluetooth** capabilities. Designed by **Espressif Systems**, it is widely used in IoT, robotics, embedded systems, and hobbyist projects.

---

## 🔧 Key Features

- Dual-core Tensilica Xtensa LX6 (or LX7 in newer chips)
- Clock speed: up to 240 MHz
- RAM: ~520 KB SRAM
- Flash: External SPI flash (typically 4MB or 8MB)
- Connectivity:
  - **Wi-Fi**: 802.11 b/g/n
  - **Bluetooth**: v4.2 BR/EDR and BLE
- GPIO: 34+ programmable pins
- ADC, DAC, I2C, SPI, UART, CAN, PWM
- Touch sensor support
- Deep sleep mode for low power applications
- Available in many modules: ESP32-WROOM-32, ESP32-WROVER, etc.

---

## 📦 Common Development Boards

| Board Name            | Key Traits                          |
|-----------------------|-------------------------------------|
| **ESP32 DevKitC**     | Basic, widely supported dev board   |
| **ESP32-WROVER**      | With PSRAM and external antenna     |
| **TTGO T-Display**    | With onboard display                |
| **LilyGO T-Beam**     | With GPS and LoRa                   |

---

## 🛠️ Programming Options

### 🔹 ESP-IDF (Espressif IoT Dev Framework)
- Official development framework.
- C/C++ based, low-level access.
- Full control over all peripherals and RTOS features.

### 🔹 Arduino Core for ESP32
- Arduino-style programming.
- Easier for hobbyists and fast prototyping.
- Supports most Arduino libraries.

### 🔹 MicroPython
- Python interpreter running directly on the ESP32.
- Great for rapid development and scripting.
- Limited performance compared to native C/C++.

---

## 🔌 Peripherals & Protocols

The ESP32 supports a huge number of protocols:

- **UART** (up to 3)
- **I2C** (2)
- **SPI** (4)
- **CAN**
- **PWM**
- **ADC/DAC**
- **Hall effect sensor**
- **Capacitive touch sensors**

It also has built-in **Timers**, **Watchdogs**, and **DMA** support.

---

## 🧠 Use Cases

- Home automation (e.g. ESPHome, Tasmota)
- IoT devices with MQTT
- Robot controllers with wireless telemetry
- DIY smart displays
- LoRaWAN nodes (with external LoRa modules)
- Bluetooth Low Energy (BLE) sensors or gateways

---

## 📚 Resources

- [Espressif Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 Arduino Core GitHub](https://github.com/espressif/arduino-esp32)
- [MicroPython ESP32 Docs](https://docs.micropython.org/en/latest/esp32/)
- [Random Nerd Tutorials](https://randomnerdtutorials.com/projects-esp32/) - Great beginner projects
- [Hackaday Projects](https://hackaday.io/projects/tag/ESP32)

---

## 🔗 See Also

- [[Microcontrollers]]
- [[I2C]]
- [[WiFi]]
- [[Bluetooth]]
- [[ESP8266]]
- [[MQTT]]
- [[LoRa]]
- [[MicroPython]]
- [[Arduino]]
- [[ESPHome]]

---

## ESP32 Buying Guide — Black Friday 2026

**November 2026 price guide.** US dollars before tax and shipping. Prices below are examples, not stock or promotional guarantees. No boards were physically tested.

**Scope clarification:** the earlier overview primarily describes the **original ESP32**, not every ESP32-family chip. S2, S3, C3, and C6 differ substantially in CPU, Bluetooth, USB, memory, and peripherals. Adafruit is a board manufacturer; **Feather** and **QT Py** are board formats, not processor architectures.

**Quick picks:** QT Py **S3 with 2MB PSRAM** for compact general experimentation; **Feather S3** for battery-powered prototypes and expansion; **C6** for Thread/Zigbee; an inexpensive classic ESP32 for legacy Bluetooth Classic projects. Buy a separate LoRa-equipped board only when long-range radio is actually needed.

An ESP32 can collect orchard sensor readings, operate a local control loop, or send telemetry to an agent computer. It is **not** a replacement for the Linux/Windows browser-and-Python workers in [[Small Agent Computers]]. MicroPython is not desktop CPython, and an ESP32 does not run omp, Chromium, pandas, or Office.

---

### Chip families: choose capabilities before a board

CPU entries describe the main application processor, not auxiliary low-power cores. SRAM is on-chip working memory; external **PSRAM is separate**. The wireless column describes silicon capability, not guaranteed support in every firmware build. [Espressif family comparison](https://docs.espressif.com/projects/esp-techpedia/en/latest/esp-friends/get-started/board-selection.html)

| Chip family | Main CPU | Internal SRAM | Wi-Fi | Bluetooth / other radio | USB capability | Best reason to choose it |
|---|---|---|---|---|---|---|
| Original ESP32 | Xtensa LX6, commonly 2 cores, up to 240MHz | 520KB | 2.4GHz Wi-Fi 4 | Bluetooth Classic + BLE 4.2 | No native USB; boards normally use a USB-UART bridge | Legacy libraries, Classic Bluetooth/SPP, inexpensive general I/O |
| ESP32-S2 | Xtensa LX7, 1 core, 240MHz | 320KB | 2.4GHz Wi-Fi 4 | **No Bluetooth** | Programmable full-speed USB OTG | USB HID/MIDI devices when Bluetooth is unnecessary |
| ESP32-S3 | Xtensa LX7, 2 cores, 240MHz | 512KB | 2.4GHz Wi-Fi 4 | BLE 5; **no Classic Bluetooth** | Programmable full-speed USB OTG + Serial/JTAG | General-purpose development, cameras, displays, audio, PSRAM-heavy projects |
| ESP32-C3 | RISC-V, 1 core, 160MHz | 400KB | 2.4GHz Wi-Fi 4 | BLE 5 | Fixed-function USB Serial/JTAG, not OTG | Cheap, small connected sensors and controls |
| ESP32-C6 | RISC-V, 1 main core, 160MHz | 512KB | **2.4GHz** Wi-Fi 6 | BLE 5 + IEEE 802.15.4 for Thread/Zigbee | Fixed-function USB Serial/JTAG, not OTG | Matter/Thread/Zigbee experimentation |
| ESP32-H2 | RISC-V, 1 main core, 96MHz | 320KB | **None** | BLE 5 + IEEE 802.15.4 | USB Serial/JTAG; not a general HID controller | Thread/Zigbee end devices without Wi-Fi |
| ESP32-C5 | RISC-V, 1 main core, up to 240MHz | 384KB | **2.4GHz and 5GHz Wi-Fi 6** | BLE 5 + IEEE 802.15.4 | USB Serial/JTAG | Projects that genuinely require 5GHz networking |
| ESP32-P4 | RISC-V, 2 main cores, up to 400MHz | 768KB | **None on-chip** | **None on-chip** | High-speed and full-speed USB OTG + Serial/JTAG | Rich camera/display/USB applications; wireless requires a companion chip |

**Important distinctions:**

- C6's **Wi-Fi 6 does not mean 5GHz**. C5 is the dual-band option in this comparison. [C5 datasheet](https://documentation.espressif.com/esp32-c5_datasheet_en.pdf)
- S2/S3 do not inherit the original ESP32's Bluetooth Classic/SPP support. An old Bluetooth serial example may require a different transport, not merely another board selection.
- Matter is an application-layer ecosystem; Thread and Zigbee use 802.15.4. Having the radio does not automatically install a Matter application or make a board a complete border router.
- **No listed chip includes LoRa.** LoRa boards add a separate Semtech-compatible radio such as SX1276 or SX1262.
- DAC, touch, Hall sensor, CAN/TWAI, ADC behavior, and pin counts vary by family. The original overview's peripheral list must not be applied unchanged to an S3 or C6. TWAI/CAN signaling also needs an external bus transceiver.
- Newer catalog entries include C61 and S31; check firmware support and board availability before choosing them.

---

### Board comparison: memory, USB, GPIO, and price

Prices below come from the linked manufacturer's listing, except the explicitly marked Micro Center Inland offer. **Flash / PSRAM** are separate capacities, not interchangeable RAM totals. “USB-C” describes the connector; the USB column explains what it can do.

| Board / exact version | Chip; flash / PSRAM | USB | Exposed GPIO / expansion | Observed price | Buying judgment |
|---|---|---|---|---|---|
| [Adafruit QT Py S3, product 5700](https://www.adafruit.com/product/5700) | S3; **4MB / 2MB** | USB-C; native programmable USB | **13 GPIO:** 11 edge pads + 2 on STEMMA QT | **$12.50** direct | Best compact all-round pick; retail example below is $12.99 |
| [Adafruit QT Py S2, product 5325](https://www.adafruit.com/product/5325) | S2; **4MB / 2MB** | USB-C; native programmable USB | **13 GPIO:** 11 edge pads + 2 on STEMMA QT | **$12.50** direct | Same direct price as S3, but single-core and no Bluetooth; choose for a specific existing project |
| [Adafruit Feather S3, product 5477](https://www.adafruit.com/product/5477) | S3; **4MB / 2MB** | USB-C; native programmable USB | Larger header layout, STEMMA QT, FeatherWing ecosystem | **$17.50** direct | Useful premium for LiPo charging, battery monitor, and easier wiring |
| [Adafruit Feather S2, product 5000](https://www.adafruit.com/product/5000) | S2; **4MB / 2MB** | USB-C; native programmable USB | Feather headers + STEMMA QT; LiPo charging | **$17.50** direct | Keep for compatibility; S3 is generally the better new purchase at equal price |
| [Adafruit Feather C6, product 5933](https://www.adafruit.com/product/5933) | C6; **4MB / no PSRAM** | USB-C; fixed Serial/JTAG | Feather headers + STEMMA QT; some header labels share GPIO | **$14.95** direct; direct listing out of stock | Good $14.99 option for Thread/Zigbee and battery nodes |
| [Seeed XIAO ESP32C3](https://www.seeedstudio.com/Seeed-XIAO-ESP32C3-p-5431.html) | C3; **4MB / no PSRAM** | USB-C; Serial/JTAG | **11** header GPIO; tiny footprint; battery charging | **$4.99** direct | Low-cost Wi-Fi/BLE nodes; add shipping before comparing |
| [Seeed XIAO ESP32S3](https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html) | S3; **8MB / 8MB** | USB-C; native programmable USB | **11** header GPIO; tiny footprint; battery charging | **$7.49** direct | Strong memory-per-dollar alternative; less convenient than QT Py for plug-in I2C sensors |
| [Seeed XIAO ESP32C6](https://www.seeedstudio.com/Seeed-Studio-XIAO-ESP32C6-p-5884.html) | C6; **4MB / no PSRAM** | USB-C; Serial/JTAG | **11** header GPIO; PCB/external antenna selection | **$5.99** direct | Budget Thread/Zigbee experiments |
| [Seeed XIAO ESP32S3 Sense](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html) | S3; **8MB / 8MB** | USB-C; native programmable USB | Camera, microphone, microSD expansion; peripherals consume pins | **$19.99** direct | Buy for camera/audio, not as a basic sensor board |
| Inland ESP32-WROOM board, KS5019 / MC product 704601 | Original ESP32; verify module flash; no PSRAM established | USB-C through **CH340 USB-UART bridge** | Larger breadboard-style headers; not every physical pin is GPIO | **$7.99** at Micro Center | Cheap classic ESP32; documentation at [Keyestudio](https://docs.keyestudio.com/projects/KS5019/en/latest/docs/KS5019.html) |
| [LILYGO T3 LoRa32 V1.6.1](https://wiki.lilygo.cc/products/t3-series/t3-lora32/) | Original ESP32; **4MB flash**; SX127x radio | USB-C through CH9102 serial bridge | Headers + 0.96-inch OLED; radio/display reserve pins | **$29.99** at retail; see band warning below | Useful integrated LoRa board, not an S3 |
| [Heltec WiFi LoRa 32 V4 / V4-R8](https://heltec.org/project/wifi-lora-32-v4/) | S3 + SX1262; **16MB flash**, **2MB / 8MB PSRAM** respectively | USB-C; verify board routing for the intended USB function | OLED, battery/solar connectors; revision-specific pin map | No configured US price verified | More modern LoRa alternative; do not mix standard V4 and V4-R8 firmware |

**Adafruit variant trap:** “QT Py S3” also names the **8MB flash / no PSRAM** product 5426. More flash does not compensate for missing working memory. The **4MB flash / 2MB PSRAM product 5700** is the pick above. Feather S3 also comes in different memory variants; specify the product number, not just “S3.”

**DevKit alternative:** Espressif S3-DevKitC boards expose more signals than tiny QT Py/XIAO boards. Buy an explicit memory variant, such as an **N8R8** module, and check which connector reaches native USB versus a serial bridge. The previously checked [Adafruit N8R2 listing](https://www.adafruit.com/product/5310) is **no longer stocked**; its displayed $17.50 is not an actionable offer.

---

### USB: the feature most likely to cause a wrong purchase

| Desired behavior | Appropriate hardware | Catch |
|---|---|---|
| Upload firmware and read serial logs | Most development boards | Use a data cable; USB-UART boards may need CH340/CH9102/CP210x drivers |
| Act as a keyboard, mouse, or USB MIDI device | **S2 or S3** with native USB routed to the connector | C3/C6 Serial/JTAG cannot be turned into arbitrary HID/MIDI |
| CircuitPython `CIRCUITPY` drag-and-drop drive | Supported **S2/S3** board firmware | Requires native USB; UF2 bootloader support is a separate board-specific question |
| C6 CircuitPython development | Supported C6 `.bin` build; serial/web workflow | No native USB mass-storage drive or UF2 drag-and-drop bootloader |
| Connect a USB keyboard or other peripheral to the ESP | S2/S3 OTG host or a suitable P4 board | Needs host software, VBUS supply, and correct wiring; USB-C alone proves none of these |
| High-speed USB rather than 12Mbps full-speed | Suitable **P4** design | Ordinary S2/S3 USB is full-speed, not 480Mbps |

Sources: [Espressif USB controller comparison](https://docs.espressif.com/projects/esp-iot-solution/en/latest/usb/usb_overview/usb_overview.html), [C6 CircuitPython build and transfer limitations](https://circuitpython.org/board/adafruit_feather_esp32c6_4mbflash_nopsram/).

For a USB HID project, start with S3 and avoid a board whose only connected port is a USB-UART bridge. An ESP32 configured as a keyboard is still **not a video-capture device or IP KVM**.

---

### GPIO, memory, and power gotchas

- **Count unique usable signals, not header holes or chip maxima.** Ground, 3V3, 5V, reset, duplicate labels, and internally occupied pins are not spare GPIO.
- QT Py S2's manufacturer specifies **11 edge GPIO plus 2 on STEMMA QT**, rather than 13 free edge pins. QT Py/XIAO pin layouts are compact; check the exact pinout before swapping boards.
- **Feather C6 duplicates:** A2 is the same GPIO as IO6, and A3 is the same as IO5. These labels cannot drive independent signals. [Manufacturer warning](https://www.adafruit.com/product/5933)
- **Classic ESP32 GPIO34–39 are input-only**; some are absent from particular modules/boards. ADC2 is shared with Wi-Fi on the classic chip. Boot-strapping pins can prevent startup when pulled to the wrong level.
- Native USB, flash/PSRAM, onboard LEDs, battery monitoring, cameras, SD cards, OLEDs, and LoRa radios consume or constrain pins. A camera board is not automatically the best high-GPIO board.
- **GPIO uses 3.3V logic, not 5V-tolerant inputs.** Use appropriate level shifting and drivers. Do not connect motors, solenoids, or pumps directly to GPIO; use rated switching hardware and protection.
- Flash holds firmware/files; **PSRAM holds runtime buffers** and is slower than internal SRAM. Not every DMA allocation or peripheral buffer can live there. Camera/display/Python projects benefit from PSRAM much more than a simple temperature publisher.
- A **LiPo connector is not proof of a charger**, regulator suitability, or matching polarity. Feather and XIAO features differ; QT Py boards generally need an add-on charger for rechargeable battery use.
- Deep-sleep figures are **configuration-specific**, not Wi-Fi-active consumption. Regulator quiescent current, USB bridges, LEDs, sensors, and displays can dominate a sleeping development board.
- Notable board-revision issue: QT Py S2 **rev B** had roughly **6mA** deep-sleep draw; **rev C** fixed it, with the manufacturer reporting about **70µA**. Do not budget battery life from the chip datasheet alone. [Revision history](https://www.adafruit.com/product/5325)
- For farm installations, add enclosure, condensation protection, antenna clearance, suitable power conditioning, and watchdog/failsafe behavior. Keep time-critical or safety-related control local rather than dependent on an API agent.

Pinout references: [Feather S3](https://learn.adafruit.com/adafruit-esp32-s3-feather/pinouts), [Feather C6](https://learn.adafruit.com/adafruit-esp32-c6-feather/pinouts), [XIAO S3/Sense](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/), [XIAO C6](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/), [classic ESP32 GPIO reference](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html).

---

### Programming languages and firmware choices

| Option | What runs on the board | Best fit | Important limitation |
|---|---|---|---|
| **ESP-IDF** | Compiled C/C++ using Espressif drivers and FreeRTOS | Production firmware, networking, low-level peripherals, debugging | More setup; target and IDF release must support the chip |
| **Arduino-ESP32** | Compiled C/C++ with Arduino APIs | Fast prototyping and the broad hobby library ecosystem | Old examples may target Arduino core 2.x or classic ESP32 only |
| **MicroPython** | Embedded Python interpreter | REPL-driven sensor/control experiments | Not full CPython; firmware builds, PSRAM settings, and peripheral coverage vary |
| **CircuitPython** | Embedded Python with Adafruit libraries | STEMMA QT sensors and approachable USB workflows | Exact board build matters; C6 does not offer an S3-style USB disk |
| **Rust** | Compiled firmware through `esp-hal` or ESP-IDF-based crates | Embedded Rust, ownership-based design, lower-level learning | RISC-V and Xtensa toolchains differ; check chip/peripheral support |
| **ESPHome** | YAML configuration compiled to firmware, with optional C++ | Home Assistant sensors, switches, OTA-managed nodes | Not an interactive Python runtime; framework and component support vary by chip |
| **Meshtastic / MeshCore / LoRaWAN firmware** | Purpose-built C/C++ firmware | Long-range telemetry or messaging | These are different protocols/stacks, not interchangeable modes of a generic “LoRa app” |

Start with **Arduino or CircuitPython on S3** for general experiments. Use **ESP-IDF** for direct hardware/networking work and **ESPHome** when declarative sensor integration is the objective. For Thread/Zigbee/Matter, follow the chosen stack's exact C6/H2 board requirements.

Current supported downloads exist for [MicroPython S3](https://micropython.org/download/ESP32_GENERIC_S3/) and [MicroPython C6](https://micropython.org/download/ESP32_GENERIC_C6/). The Feather C6 [CircuitPython page](https://circuitpython.org/board/adafruit_feather_esp32c6_4mbflash_nopsram/) currently offers stable **10.3.2**, including Wi-Fi and BLE modules; this does **not** establish Python support for every 802.15.4 use case.

Useful official references: [Arduino-ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/), [Rust toolchain setup](https://docs.espressif.com/projects/rust/book/getting-started/toolchain.html), [ESPHome ESP32 platform](https://esphome.io/components/esp32/).

---

### LoRa choices for orchard and field experiments

**LoRa** is the radio modulation; **LoRaWAN**, **Meshtastic**, and **MeshCore** add different networking behavior. A Feather C6's Thread/Zigbee radio cannot substitute for a sub-GHz LoRa radio.

| Approach | What it adds | Good use | Buying caveat |
|---|---|---|---|
| Plain S3/C6 plus SPI LoRa module | Separate radio, antenna connector, extra wiring | Custom sensor/control firmware | Check radio driver, RF band, voltage, and remaining SPI/control pins |
| LILYGO **T3 LoRa32 V1.6.1** | Classic ESP32 + SX127x + OLED | Inexpensive integrated telemetry experiments | **SX1276 is 868/915-class; SX1278 is 433-class**; verify exact box/module |
| Heltec **WiFi LoRa 32 V4** | S3 + SX1262 + OLED + battery/solar interfaces | Modern integrated long-range node | Standard V4 has 2MB PSRAM; V4-R8 has 8MB and different firmware/pins |
| LILYGO **T-Deck Plus** | S3 + SX1262 + screen/keyboard/GPS-oriented handheld hardware | Standalone messaging/interface device | Expensive and bulky for a fixed unattended sensor |
| LoRaWAN gateway plus end nodes | Multi-node network infrastructure | Larger orchard sensor deployments | A single SX127x/SX1262 dev board is **not** a normal multi-channel LoRaWAN gateway |

For US Meshtastic experiments, select the **US 915MHz hardware and `US` region**, compatible firmware, and an antenna for that band. Merely selecting a software region does not convert a 433MHz radio into 915MHz hardware. Connect a suitable antenna before transmitting; follow applicable RF limits.

The linked T3 product listing does **not** establish the radio band, so it is not an unconditional “buy this” recommendation. LILYGO's documentation also contains a misleading GPIO33-and-up input-only remark: use Espressif's actual rule, **GPIO34–39**, instead.

Heltec V4 requires **V4-specific firmware**; V4-R8 requires its own target. The manufacturer currently calls for Meshtastic newer than 2.7.20 for standard V4 and newer than 2.7.25 for V4-R8. Follow the current flasher rather than assuming a V3 image is interchangeable.

Sources: [LILYGO T3 specifications and revision table](https://wiki.lilygo.cc/products/t3-series/t3-lora32/), [Heltec V4/V4-R8 hardware and firmware](https://heltec.org/project/wifi-lora-32-v4/), [Meshtastic LILYGO devices](https://meshtastic.org/docs/hardware/devices/community-supported/lilygo/), [Meshtastic regional configuration](https://meshtastic.org/docs/configuration/radio/lora/).

---

### Retail board comparisons

Use the manufacturer part number to distinguish memory and radio variants. Check current prices and availability with the seller.

| Board | SKU / manufacturer part | Price | Judgment |
|---|---|---|---|
| [QT Py S3 with 2MB PSRAM](https://www.microcenter.com/product/668983/adafruit-industries-qt-py-s3-with-2mb-psram-wifi-dev-board-with-stemma-qt) | **664102 / 5700** | **$12.99** | Best compact general-purpose option |
| [QT Py S2](https://www.microcenter.com/product/646717/adafruit-industries-qt-py-esp32-s2-wifi-dev-board-with-stemma-qt) | **242883 / 5325** | **$12.99** | S3 is preferable at the same price unless compatibility dictates S2 |
| [Feather C6](https://www.microcenter.com/product/698255/adafruit-industries-esp32-c6-feather-stemma-qt) | **901866 / 5933** | **$14.99** | Good Thread/Zigbee addition |
| [Feather S2, 4MB flash / 2MB PSRAM](https://www.microcenter.com/product/634751/adafruit-industries-esp32-s2-feather) | **248839 / 5000** | **$17.99** | Battery-ready, but not the first new all-round choice |
| [Feather S2 with BME280](https://www.microcenter.com/product/668761/adafruit-industries-esp32-s2-feather-with-bme280-sensor-stemma-qt-4mb-flash-2mb-psram) | **901595 / 5303** | **$24.99** | Convenient temperature/humidity/pressure prototype |
| [Inland classic ESP32-WROOM USB-C board](https://www.microcenter.com/product/704601/inland-esp32-wroom-core-board-with-usb-c-wifi-bluetooth-dual-core-microcontroller-for-arduino) | **961177 / KS5019** | **$7.99** | Low-cost general ESP32 board |
| [Elegoo ESP32 development-board 3-pack](https://www.microcenter.com/product/704603/elegoo-esp-32-development-board-3-pack) | Verify package revision | **$19.99** | **$6.66 per board**; inspect connector, module, and pinout before standardizing |
| [Seeed XIAO ESP32-S3 3-pack](https://www.microcenter.com/product/713921/seeed-studio-xiao-esp32-s3-(3-pack)) | **689773 / 102010573** | **$22.99** | About **$7.66 per board** |
| [Seeed XIAO ESP32-S3 Sense](https://www.microcenter.com/product/713919/seeed-studio-xiao-esp32-s3-sense) | **689757 / 113991115** | **$22.99** | Buy only for camera/audio use |
| [LILYGO T3 LoRa32 V1.6.1](https://www.microcenter.com/product/706483/t3-lora32-v161) | **982405** | **$29.99** | Verify **915MHz** radio and antenna before buying |
| [LILYGO T-Deck Plus, S3 / SX1262 / 915MHz](https://www.microcenter.com/product/703092/lilygo-t-deck-plus-esp32-s3-915mhz-lora-89-x-162-full-box-gps-28-inch-display-ttgo-development-board-black-keyboard) | Check box revision | **$89.99** | Handheld experiment, not the value sensor-node choice |


**Black Friday assessment:** November 27 is Black Friday in 2026, but that calendar fact does not establish an active sale. Searches of Adafruit/Seeed announcements did not verify a current ESP32-specific promotional code. Seeed's surfaced Thanksgiving/Black Friday article was **2025**, not evidence of a 2026 deal. Direct XIAO prices and retail three-packs are useful comparisons, not invented holiday discounts.

---

### What to buy

**For a small bench kit today:**

1. **One QT Py S3, product 5700 — $12.99.** Native USB, Wi-Fi/BLE, and 2MB PSRAM cover the broadest compact experiments.
2. **One Feather C6 — $14.99.** Adds Thread/Zigbee and Wi-Fi 6 experimentation rather than duplicating the S3.
3. **One Inland classic ESP32 — $7.99**, if legacy Bluetooth Classic or a cheap larger breadboard board is useful.

Those three total **$35.97 before tax**. Skip the classic board if you have no Classic-Bluetooth/library need; the S3+C6 pair is **$27.98**. Cables, headers, breadboard, sensors, and battery hardware are extra.

**Change the selection for a specific goal:**

- **Battery-powered I2C prototype:** Feather S3 4MB/2MB direct, with its charging circuit and battery monitor; QT Py's smaller price excludes those conveniences.
- **Several cheap nodes:** compare the **$19.99 Elegoo 3-pack** against shipped XIAO C3/C6/S3 orders. Standardize one exact board/revision only after testing a sample.
- **Camera/audio:** XIAO S3 Sense, not the plain XIAO S3; verify camera revision and supported firmware.
- **Orchard long-range telemetry:** buy **two matching US-915 LoRa nodes** for a useful radio experiment. Check radio markings and antenna; listed T3 band remains unverified.
- **USB keyboard/mouse/MIDI:** S3 native USB. C6 is the wrong choice despite its USB-C socket.
- **Rich UI:** S3 with PSRAM and a suitable display; P4 only when its extra hardware capability justifies a more complex board.

**Bottom line:** there is no reason to wait for a questionable “Black Friday” label to save a dollar on one development board. Buy the correct chip, memory variant, connector routing, and RF band. For deployed farm control, the power system, enclosure, wiring, firmware recovery, and actuator safety matter more than the board's sticker price.

Related notes: [[ESP32-S3]], [[Microcontrollers]], [[LoRa]], [[MQTT]], [[MicroPython]], [[Arduino]], [[ESPHome]], [[Small Agent Computers]].
