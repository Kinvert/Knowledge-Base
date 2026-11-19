# Small Agent Computers

Small, inexpensive computers for API-backed agents such as [[oh-my-pi]]: browser work, spreadsheets, email, scheduling, Jira, research, data entry, and local Python scripts. **No local LLM inference requirement.** The useful hardware is usually an ordinary mini desktop, not an “AI PC” or a Raspberry Pi cluster.

**November 2026 US price examples**, before tax and shipping. Manufacturer specifications, independent measurements, advertised prices and suggested buying ceilings are distinguished below. No machines were physically benchmarked for this note.

---

## Bottom line

1. **Best general-purpose value: refurbished AMD business minis**, particularly **Lenovo ThinkCentre M75q Gen 2 with Ryzen 5 PRO 4650GE or 5650GE**, 16GB RAM, and a replaceable SSD. These offer more CPU concurrency and expansion headroom than four-core Intel N-series boxes without requiring a large desktop.
2. **Best simple low-power appliance: Intel N100/N150 mini PCs**, provided the complete-machine price is genuinely low. GMKtec G3/G3 Plus and Beelink Mini S12 Pro/S13/EQ14 are candidates, not interchangeable SKUs. Current checked prices are much worse than old review prices; do not buy the label alone.
3. **Best tiny GPU machine: a factory-configured ThinkStation P350 Tiny with T600 or T1000.** This is a real 1L workstation, not an SBC. Buy it complete with the proper GPU cooling, riser, and power adapter.
4. **Best flexible GPU machine: a used SFF workstation plus a low-profile, slot-powered card.** Favor Turing or newer NVIDIA hardware for CUDA. An RTX A1000 or A2000 is useful when the software actually uses it; an RTX 3050 **6GB** is a different power proposition from the **8GB** card.
5. **Best fleet architecture when physical separation is unnecessary: two expandable hosts running ten isolated workers.** Ten agents do not inherently require ten motherboards. Conversely, ten independent Windows desktops are not ten lightweight containers.

**Reuse before buying:** pilot an existing laptop for API workloads before adding dedicated workers. A discrete GPU is a separate purchase decision for workloads such as HIP or CUDA programming.

---

## What actually determines the hardware floor

These are **starting allocations, not measured agent benchmarks**. Browser complexity and concurrent applications matter more than the number of agent conversations.

| Workload | Sensible starting allocation | What changes the requirement |
|---|---|---|
| API calls, email/Jira integrations, scheduled Python | 2 vCPU, 2–4GB RAM, 32–64GB disk per isolated Linux worker | Large dataframes, document parsing, many subprocesses |
| One active browser-based office/research worker | 2–4 vCPU, 8GB RAM, 64–128GB disk per VM | Heavy tabs, multiple browser profiles, screenshots, downloads |
| Dedicated physical browser-agent box | Four modern small cores or a used six-core CPU; **16GB RAM, 256–512GB SSD** | Multiple simultaneous browser workers |
| Windows desktop Office plus browser and communication apps | 4 vCPU, 8–16GB RAM, 128GB+ disk; prefer **16GB per physical box** | Excel recalculation, large workbooks, Teams/Zoom video, add-ins |
| Shared host running five browser VMs | Six or more capable CPU cores, **64GB RAM**, 1TB SSD as an initial configuration | Five workers continuously busy rather than waiting for APIs |
| Real GPU-enabled batch processing | Enough CPU/RAM for the worker plus VRAM sized to the actual batch | CUDA support, transfer overhead, memory capacity |

- **256GB is usable; 512GB is comfortable.** A 1TB drive is optional for an individual office worker, more useful on a VM host or for retained downloads and recordings.
- **DDR4 is fine.** Capacity, replaceability, and total system cost matter more here than buying DDR5 for its own sake.
- An integrated GPU already handles basic display output and may accelerate video. A discrete GPU is not required to render a spreadsheet or keep an API connection open.
- Ten API requests can overlap on a modest CPU. Ten active browser instances or Excel recalculation jobs are a different workload.
- A coding-agent harness does not supply working Outlook, Jira, or franchise-management integrations by itself. Authentication, tools, permissions, and application-specific behavior still need implementation.

### Choose the application path before the operating system

**Linux x86-64 is the boring default** for Python, API integrations, browser automation, and containerized workers. Current omp installation documentation explicitly supports macOS, Linux, and Windows.[^omp]

**Windows x86-64 is the safer compatibility choice** when work really requires desktop Excel, Windows-only add-ins, VBA/COM automation, or a vendor's Windows client. Editing `.xlsx` files in Python is not equivalent to running Excel: `openpyxl` explicitly does not calculate formulas.[^openpyxl]

Microsoft Graph can eliminate a lot of desktop clicking, but inspect each endpoint's authentication model. For example, Excel `workbook/createSession` currently supports delegated work/school permissions and **does not support application permissions**. Do not assume every Excel operation works with a client-credentials service principal.[^graph]

Unattended Microsoft 365 desktop automation has separate licensing requirements, and Microsoft documents modal dialogs, identity problems, single-threading, and stability risks. Its unattended Office license is for Windows and covers one physical or virtual operating-system environment; noninteractive server-side COM automation remains unsupported. A Windows license bundled with a mini PC does not include Microsoft 365 or RPA licensing.[^office-license][^office-risks]

**Tern caveat:** Stencil's Tern is a native workspace with remote-server functionality. Treat the exact desktop, daemon, browser, and architecture build you use as a pilot prerequisite—not as automatically portable because omp is. The operator's Tern interface can be separate from Linux/Windows worker machines. There are unrelated products named Tern; do not infer compatibility from another product's documentation.[^tern]

---

## Hardware and architecture comparison

“Tiny/Micro” usually means approximately 1L with no ordinary GPU expansion. **SFF is larger but normally has real PCIe slots.** A workstation Tiny is a special case; a generic Tiny with a vaguely similar chassis is not automatically a GPU-ready equivalent.

| Computer / platform | Architecture and useful configuration | Memory and storage | Local diagnostics / networking | Fit for this project |
|---|---|---|---|---|
| **ThinkCentre M75q Gen 2** | AMD x86-64; 4650GE/5650GE, 6 cores / 12 threads | Two DDR4 SO-DIMMs; Lenovo specifies up to 64GB; M.2 NVMe plus 2.5-inch bay | HDMI + DP, six standard USB ports including USB-C; GbE; optional Wi-Fi | **First used-machine choice**; one worker now, several later [^m75-spec] |
| **ThinkCentre M720q / M920q class** | Intel x86-64; seek i5-8500T/9500T rather than the cheapest Pentium configuration | Replaceable DDR4 and SSD; verify model-specific limits | M720q listings commonly include HDMI, USB, GbE; Wi-Fi is configuration-dependent | Useful only when clearly cheaper than newer Ryzen/Core minis [^m720-offer] |
| **OptiPlex 3000 Micro** | Intel x86-64; checked offer has i5-12500T, 6 cores / 12 threads | Two DDR4-3200 SO-DIMMs, official 64GB maximum; replaceable SSD | HDMI + DP, six USB-A, GbE; inspect actual wireless option | More expensive but attractive CPU headroom and documented serviceability [^dell-memory][^dell-ports] |
| **GMKtec G3 Plus** | Intel x86-64; N150, 4 cores / 4 threads | DDR4; vendor advertises up to 32GB; 2280 NVMe + 2242 SATA | Dual HDMI, USB-A, 2.5GbE, Wi-Fi 6; tested unit used RTL8852BE | Good single-worker appliance at a low price; not at every current asking price [^g3plus][^g3-test] |
| **Beelink Mini S13** | Intel x86-64; N150, 4 cores / 4 threads | Tested version: 16GB DDR4, one SO-DIMM; two M.2 positions; supplied SSD was SATA | Two HDMI, four USB-A, GbE and Wi-Fi 6 on reviewed version | Compact browser worker; check revision and do not assume NVMe or 2.5GbE [^s13] |
| **Beelink EQ14** | Intel x86-64; N150 | Officially up to 16GB DDR4; two M.2 2280 positions; current page says supplied SSD is SATA | Two HDMI, dual 2.5GbE on checked variant; Intel AX101 Wi-Fi 6; internal PSU | Fewer external power bricks; current checked price is unattractive [^eq14] |
| **ACEMAGIC F2A, current X7 358H version** | Intel x86-64 Panther Lake; 16 cores / 16 threads (**4P + 8E + 4 LP-E**), Arc B390 integrated GPU | **32GB LPDDR5X soldered, not upgradeable**; 1TB supplied NVMe, two M.2 2280 positions per manufacturer | HDMI + DP, four USB-A, USB4 and another USB-C; dual 2.5GbE; Wi-Fi 7; manufacturer specifies **100W PD input** | Premium CPU/graphics/PD option, **not a cheap API worker**; exact linked SKU is $1,099.99 in the linked listing. Do not substitute older F2A 125H specifications [^f2a-mc][^f2a-spec] |
| **ThinkStation P350 Tiny** | Intel x86-64; choose a factory-supported CPU/GPU combination | Two DDR4 SO-DIMMs, up to 64GB; two M.2 SSDs | HDMI + DP, six USB including USB-C; optional T600/T1000 adds mini-DP; optional Wi-Fi | **Tiny CUDA-capable worker**, with more heat and component constraints [^p350] |
| **SFF workstation + low-profile GPU** | Intel/AMD x86-64; e.g. Precision or ThinkStation SFF | Model-dependent; normally easier RAM/SSD/GPU service than 1L units | Standard USB and display outputs; Wi-Fi may need a card and antennas | Best GPU upgrade path; verify slot power, clearance, PSU, and cooling |
| **Mac mini M4, used/refurbished** | Apple ARM64; 16GB/256GB is already useful | Unified RAM is not upgradeable; no ordinary internal GPU upgrade | HDMI, five USB-C/TB ports, GbE, Wi-Fi 6E; USB-A needs an adapter | Strong macOS/Tern operator or Mac-specific worker; **not a CUDA box** [^mac-spec] |
| **Raspberry Pi 5 16GB** | ARM64; four Cortex-A76 cores | Soldered LPDDR4X; SSD needs USB enclosure or PCIe adapter/HAT | Two **micro-HDMI**, four USB-A, GbE, dual-band Wi-Fi 5 | Poor current value for this 16GB office-worker brief [^pi-spec][^pi-price] |
| **Orange Pi 5 Plus** | ARM64 RK3588; four A76 + four A55 cores | Soldered LPDDR4/4X; native M.2 2280 NVMe | Two HDMI outputs, four USB-A, dual 2.5GbE; **Wi-Fi module extra** | An ARM experiment, not the compatibility-first fleet default [^orange] |
| **Dell Wyse 5070** | Older Intel low-end x86-64 thin client | Check installed RAM carefully; M.2 storage is **SATA**, not NVMe; tiny eMMC configurations common | DP-oriented outputs, USB and Ethernet; wireless optional | Worth considering only as a very cheap Linux API/scheduler appliance [^wyse] |

### Why the used Ryzen mini is the default

The 4650GE/5650GE combination of six cores, twelve threads, two RAM slots, a normal SSD, and mature business-desktop documentation is useful for **multiple Python processes and concurrent browsers**. You are paying for CPU/RAM flexibility rather than inference hardware.

For an M75q Gen 2, Lenovo lists multiple possible wireless cards—including Intel AX200/AX210, Intel 9260, and Realtek options—and also configurations with **no wireless hardware**. “M75q Gen 2” is not a guarantee of a particular Wi-Fi chipset.[^m75-spec]

**Suggested buying ceiling, not a live quote:** roughly **$175–225 complete** for a clean 4650GE/16GB/256GB unit, with a return window and correct adapter. Consider a higher ceiling for a 5650GE or meaningful warranty. At a much higher price, compare a newer Core mini or one shared host rather than paying a nostalgia premium.

For old Intel business minis, a cheap “i5” listing is insufficient: verify the generation and exact CPU. Prefer an eighth-generation or newer configuration if Windows 11 is part of the plan, and verify TPM/Secure Boot eligibility. Old Windows 10-era listings need an explicit supported-OS plan.

### Intel N-series: useful, but do not overpay

N100 and N150 are small four-core, four-thread CPUs. They are adequate candidates for one modest browser worker, not a reason to cram ten full desktops into a 16GB machine.

- Compare **complete 16GB/SSD/PSU prices**, not barebones starting prices.
- Do not pay a large premium merely to move from N100 to N150. Check actual workflow latency and thermals.
- Product memory limits differ: Beelink specifies 16GB for the checked EQ14, while GMKtec advertises 32GB for G3 Plus. Do not extrapolate one vendor's RAM support to another board.
- A second M.2 connector may be SATA-only. M.2 describes the form factor, not a guarantee of NVMe.
- A “Wi-Fi 6” badge does not identify the chipset, number of streams, or whether the card can be replaced.
- Small fans and power limits can dominate sustained performance. The firsthand G3 Plus investigation documented thermal-paste and airflow changes; its performance-tuned results are not a guarantee for an untouched unit.[^g3-test]

**Suggested buying ceiling:** around **$180–225 for a complete 16GB/256–512GB N100/N150 machine**, with some premium acceptable for a strong return policy or better documented hardware. This is a value threshold, **not a claim that such a live US offer was verified in this snapshot**.

### ARM: valid architecture, weaker fit for this purchase

ARM64 can run Python and browser automation. Playwright currently documents supported Debian/Ubuntu releases on both x86-64 and ARM64. That does not guarantee every third-party browser channel, extension, native wheel, desktop application, or RPA tool supports a particular ARM board image.[^playwright]

**Pi economics are especially poor right now:** Raspberry Pi's November announcement raises the 16GB Pi 5 board to **$385**, also reflected in the current product brief. The power supply, case, cooling, SSD, and SSD interface still cost extra. Even the 8GB board is listed at $175 before accessories. GPIO/camera projects can justify it; office-agent hosting generally cannot.[^pi-price][^pi-spec]

Orange Pi 5 Plus provides substantially more I/O than a bare Pi 5, including native NVMe and dual 2.5GbE, but requires an added Wi-Fi module. Its Mali GPU and Rockchip NPU are **not CUDA devices**, and advertised accelerator TOPS do not accelerate ordinary Python scripts. Validate the actual maintained OS image, browser, kernel, and graphics stack before buying multiples.[^orange]

A used **16GB M4 Mac mini** is the more interesting ARM alternative if macOS is useful. Its GPU supports Apple's compute stack, not NVIDIA CUDA. Do not assume Windows ARM virtualization reproduces an x86 Windows Office/add-in environment. Avoid an 8GB M1/M2 bargain if the intended role needs several heavy applications. Apple's current store lists newer M6/M5 Pro models; M4 is deliberately a used-value candidate here, not a claim about the latest model.[^mac-spec][^apple-store]

---

## Tiny GPUs: buy for an accelerated task, not “parallel stuff” generally

**Concurrency is not the same thing as GPU parallelism.** Browser sessions, HTTP requests, normal Excel operations, and ordinary Python loops do not move onto CUDA simply because a GPU is installed. More CPU cores and RAM usually help these workloads first.

A discrete GPU can be justified for explicit CUDA/CuPy kernels, supported GPU dataframe operations, rendering, image/video processing, or a specific accelerated application. For conferencing/transcoding, check the integrated GPU's media support first. GPU memory is separate from system RAM and can become the limiting resource.

The powers below are **manufacturer board limits, not measured system idle power**.

| GPU | VRAM | Board power | Physical class / outputs | Buying judgment |
|---|---|---|---|---|
| **NVIDIA T600** | 4GB | 40W | Low-profile single-slot class; four mini-DP | Sensible cheap used Turing CUDA option; no RTX Tensor-core proposition [^p350] |
| **NVIDIA T1000** | 4GB or 8GB | 50W | Low-profile single-slot class; four mini-DP | More headroom, especially 8GB, if priced well [^p350] |
| **RTX A400** | 4GB | 50W | Low-profile, single-slot, four mini-DP | Modern small Ampere option; limited VRAM and modest compute [^a400] |
| **RTX A1000** | 8GB | 50W | Low-profile, single-slot, four mini-DP | **Best neat single-slot target** when an actual CUDA workload justifies its price [^a1000] |
| **RTX A2000** | 6GB or 12GB | 70W | Low-profile, **dual-slot**, four mini-DP | Useful used SFF compute card; 12GB adds dataset room, not lower power [^a2000] |
| **RTX 3050 6GB** | 6GB | 70W reference | Low-profile versions exist; exact cooler varies; MSI LP has HDMI + DP | Good budget CUDA candidate if a real used/refurb deal and chassis fit align [^3050][^3050-offer] |
| **RTX 3050 8GB** | 8GB | 130W reference | Normally larger; reference specification requires an 8-pin power lead | **Not interchangeable with the 6GB card** for a tiny low-power build [^3050] |
| **RTX 4050 Laptop GPU** | 6GB | Laptop-specific power configuration | Soldered/mobile GPU in a complete laptop or purpose-built mini PC | Not a normal desktop add-in upgrade; evaluate the entire machine [^4050] |

**Used-price targets, not observed market offers:** T600 around $75–100; T1000 around $100–150 depending on VRAM; low-profile 3050 6GB around $150–180. These are thresholds for this modest workload, not promises of availability. At high workstation-card prices, keep the money or buy a better CPU host.

### Two practical GPU computer shapes

**A. ThinkStation P350 Tiny, factory T600/T1000 configuration**

- Approximately **37 × 183 × 179mm**, genuinely 1L.
- Official options include T600 4GB, T1000 4GB, and T1000 8GB; GPU/CPU/PSU combinations are constrained.
- Two SO-DIMM slots and two SSD positions make it useful beyond one worker.
- The standard motherboard HDMI port simplifies a diagnostic monitor, while GPU outputs are mini-DP.
- Buy the matching riser, cooling assembly, backplate, and 170W/230W adapter **as part of the complete configuration**. Do not assume a cheap bare chassis accepts an arbitrary retail low-profile card.
- Prefer it when tiny size is worth a premium. No sufficiently attractive live US used T600-equipped P350 offer was verified for this snapshot.[^p350]

**B. SFF workstation plus a standard low-profile card**

Look at Dell Precision SFF, HP Z2 SFF, or Lenovo ThinkStation SFF configurations. Choose the exact model using its service manual, not merely the badge.

Before ordering the GPU, check **all** of these: PCIe slot power allowance, PSU capacity, low-profile bracket, card length, one-slot versus two-slot clearance, whether the PSU blocks the fan, and motherboard video behavior with a discrete card. A physical x16 connector does not prove the OEM allows a 70W card. Do not use SATA-to-PCIe power adapters to force an unsuitable machine into service.

**Avoid cheap Pascal-era Quadro P-series cards as a new CUDA standard.** CUDA 13 removed offline compilation for architectures before compute capability 7.5. Existing older-toolkit applications can still run with supported drivers, but buying a legacy stack problem to save a few dollars is poor fleet standardization. Turing and Ampere are the more sensible starting points.[^cuda]

Intel Arc low-profile cards are another graphics/media path, but are not drop-in CUDA alternatives. Buy one only when the chosen software explicitly supports its acceleration API; do not choose it for unspecified Python acceleration.

**Fleet recommendation:** ordinary integrated-graphics workers plus **one shared GPU worker**. Queue accelerated jobs there. Standard GPU passthrough typically assigns a whole device to one VM; do not budget on splitting a cheap GeForce into ten independently accelerated VMs.

### AMD programming example: Dell RX 6500 SFF

The RX 6500 is a small target for learning AMD GPU programming, not a performance proxy for an MI300X. HIP C++ is the CUDA-like kernel/runtime programming interface; ROCm is the surrounding compiler, runtime, libraries, and tooling stack. Writing GPU kernels is distinct from developing the kernel-mode graphics driver.

The following **Dell Refurbished** configurations were previously inspected and their **50% discounts verified in temporary carts with `EARLYBF50`**. Prices and stock are dated observations, not a fresh reservation or guarantee of availability.[^dell-gpu-deals][^dell-coupon]

| Computer / condition | CPU | RAM / SSD | Dedicated GPU | List → cart price | Observed stock |
|---|---|---|---|---:|---:|
| **Precision 3260 Compact, Grade B**, listing `000019` | i5-12600, 6 cores / 12 threads | 16GB / 512GB | Quadro RTX 3000, 6GB, Turing Tensor cores | $869 → **$434.50** | 9 |
| **Precision 3260 Compact, Grade A**, listing `000018` | i5-12600, 6 cores / 12 threads | 16GB / 512GB | Quadro RTX 3000, 6GB, Turing Tensor cores | $979 → **$489.50** | 30+ |
| **Precision 3460 SFF, Grade B**, listing `000015` | i7-12700, 12 cores / 20 threads | 32GB / 512GB | T1000, 4GB, no Tensor cores | $949 → **$474.50** | 4 |
| **OptiPlex 7010 SFF, Grade B**, listing `000059` | i5-13500, 14 cores / 20 threads | 16GB / 256GB | **Radeon RX 6500, 4GB** | $989 → **$494.50** | 1 |

The selected Radeon configuration included **Wi-Fi + Bluetooth**; the selected Precisions did not include WLAN. The RX 6500 is AMD hardware, **not CUDA-capable**. These are complete-system prices, not GPU-only prices. No matched wall-power or workload benchmarks were run on these units.

**AMD learning value and limits:**

- AMD's target mapping identifies the **RX 6500 as RDNA2 `gfx1034`**. TheRock's Linux support table marks that target build-passing, sanity-tested, and release-ready; its installation documentation includes **`device-gfx1034`** packages.[^amd-targets][^therock-support][^therock-install]
- The RX 6500 is **not listed in the production ROCm compatibility matrix**. A tested development target is not the same assurance as a qualified GPU/OS/framework combination. Library, debugger, and profiler support must be checked individually; this Dell was not hardware-tested.[^rocm-matrix]
- Learn explicit memory allocation/transfers, kernels, indexing, shared memory, reductions, atomics, synchronization, streams, events, and AMD compiler/runtime setup. These are relevant skills beyond this small card.
- Portable HIP source and programming concepts remain useful on an **MI300X** or **Radeon AI PRO R9700**. Recompile for the target and retest; code objects, performance tuning, matrix instructions, and hardware capabilities are not interchangeable. The R9700 is RDNA4 `gfx1201`, while MI300X is a different CDNA accelerator architecture. “9700” should not be confused with an RX 9070/9070 XT.[^amd-targets][^hip-porting]
- **4GB VRAM and a small RDNA2 GPU** limit datasets and training. This is an AMD software-stack laboratory, not a representative MI300X performance platform. Start with a verified maintained Linux/driver/ROCm combination and test a small kernel with CPU-checked results before adding frameworks.
- Current native **PufferLib 5.0 is CUDA code**, not an AMD program simply because ROCm is installed. Running that path on AMD requires a separate port. On the two Turing NVIDIA cards above, the source-indicated starting point is **FP32 `--float`** rather than default BF16; that is source-based guidance, not a successful hardware test.[^puffer-build][^puffer-precision]

An RX 6500 system can serve as a HIP development machine as well as an API worker. The RTX 3000 alternative supports CUDA instead; neither discrete GPU is required for API calls.

### Reusing an older laptop: Latitude E6430

Dell's E6430 specifications list third-generation Intel processors, configurations up to **16GB DDR3**, and Intel HD 4000 or optional NVIDIA NVS 5200M graphics.[^e6430]

- **[INFERENCE]** With a healthy SSD and sufficient RAM, an existing E6430 is worth piloting for one modest API/Python/browser worker or as an SSH/editor terminal for the AMD machine. Heavy browser concurrency is a separate requirement.
- **Replace Windows 7 before connecting business accounts.** Install a maintained OS, identify the actual Wi-Fi adapter, inspect SSD/battery health and cooling, and remove swollen batteries from service.
- No AMD GPU means these laptops are **not local ROCm execution machines**. They can control a separate AMD host.
- Measure with the battery charged and screen off under the real workload. Old laptops are not automatically lower-power than modern minis; battery charging distorts wall readings. Built-in screen/keyboard/Wi-Fi reduce initial accessory spending, but laptops stack less neatly.

---

## Measured power: compare systems, not TDP labels

A CPU's 6W/15W/35W rating is not whole-computer wall consumption. RAM, SSD, wireless, Ethernet PHYs, firmware settings, power-adapter losses, fans, and GPU state all matter.

These are **reported measurements or manufacturer wall-power figures**, with different configurations and test procedures. They are not one controlled cross-platform benchmark.

| Exact source configuration | Idle | Reported loaded figure | Evidence / limitation |
|---|---:|---:|---|
| GMKtec G3 Plus N150, 16GB; reviewer's replacement SSD and documented tuning | **9.3W** | 25W `stress-ng`; 28.5W HPL | Firsthand wall measurements; workload and modifications documented [^g3-test] |
| ThinkCentre M75q Gen 2 **4750GE** review unit | **About 12W** | About 58W | STH measured at 120V; not a measurement of every 4650GE/5650GE refurb [^m75-power] |
| ThinkCentre M720q **Pentium G5400T** review unit | **11–14W** | Under 50W in testing | STH; do not relabel as an i5-8500T result [^m720-power] |
| ThinkStation P340 Tiny discrete-GPU review unit | **22–23W initially; 32–33W after extended idle** | Just over 150W | STH observation; related GPU Tiny, **not a P350 guarantee** [^p340-power] |
| Mac mini M4, 16GB/256GB | **4W** | 65W | Apple's wall-power figures; idle is Finder-only, not an active browser agent [^mac-power] |

The GPU Tiny example is the warning: **tiny enclosure does not imply tiny idle consumption**. It may be better to buy one GPU worker and leave the other machines simpler.

**F2A power evidence:** no independently verified idle/load wall measurements were obtained for the **X7 358H configuration** linked here. ACEMAGIC lists 25–65W CPU power modes and a 19V/6.32A DC supply; neither is an idle-power measurement. Older F2A 125H reviews describe different hardware and cannot fill this gap.[^f2a-spec]

### Ten-node electricity sensitivity

Calculated for **ten computers, 24/7, 365 days, $0.15/kWh**. “Average” includes all operating states; these are scenarios, not predictions for a named model. Switch/AP/UPS losses and cooling are excluded.

| Average watts per computer | Fleet watts | Fleet kWh/year | Electricity/year |
|---:|---:|---:|---:|
| 5 | 50 | 438 | $65.70 |
| 10 | 100 | 876 | $131.40 |
| 15 | 150 | 1,314 | $197.10 |
| 20 | 200 | 1,752 | $262.80 |

Formula: `annual cost = node count × average watts × 8.76 × electricity price per kWh`.

At this scale, paying a large hardware premium to save a few watts per node can take many years to recover. Measure the representative workload with a plug-in meter; do not extrapolate from a completely idle desktop or the PSU's printed wattage.

---

## Wi-Fi that is easy to live with

**For ten machines in one stack, use Ethernet internally and retain Wi-Fi for flexibility.** A 16-port gigabit switch is sufficient for ordinary API/browser traffic and leaves spare ports. If an Ethernet cable cannot reach the stack, a dedicated Wi-Fi client bridge/router with Ethernet downstream is another option; it creates a shared uplink and failure point.

For machines deployed separately, Wi-Fi matters much more:

- Prefer a documented, replaceable **Intel AX200 or AX210** when the host supports a normal PCIe/USB M.2 wireless card. Linux's `iwlwifi` documentation lists support starting at kernels 5.1 and 5.10 respectively; use current distribution firmware as well.[^iwlwifi]
- **AX201/AX211 are not generic replacements.** Their platform-specific interface requirements differ; AX211 is CNVio2. Check motherboard compatibility, not just the M.2 connector shape.[^ax211]
- Older Intel 9260/8265 cards can be adequate for API agents; raw Wi-Fi bandwidth is rarely the limiting resource here.
- A factory AX101 is a plausible low-cost client, but it is not equivalent to a two-stream AX200/AX210. The checked EQ14 explicitly identifies AX101.[^eq14]
- Realtek is not automatically unusable. The tested G3 Plus used RTL8852BE successfully on Ubuntu with a 6.12 kernel. That is evidence for that unit and software stack—not every Realtek adapter or old Linux installer.[^g3-test]
- Ask for the **chipset and antenna configuration**. Refurbisher “Wi-Fi included” may mean a cheap USB dongle rather than the original internal module.
- For broad router compatibility, retain 2.4GHz support and use 5GHz when practical. Do not require 6GHz or Wi-Fi 7 for this workload. Test enterprise authentication, captive portals, and reconnection on the actual destination network.
- An internal card upgrade needs antenna leads, suitable connectors, and sometimes an external antenna assembly. A bare card alone may not fix a no-Wi-Fi refurb.

**Virtualization gotcha:** ordinary Wi-Fi client links do not behave like Ethernet bridges for VM MAC addresses. Proxmox explicitly recommends avoiding WLAN for the hypervisor uplink and documents NAT/workarounds. Use Ethernet for a shared VM host; if forced onto Wi-Fi, design routed/NAT networking instead of assuming a normal bridge will work.[^pve-wlan]

---

## USB-C-powered mini computers

**Require explicit USB-C PD power input, not just a USB-C socket.** USB data, display output, charging a phone, and accepting power for the computer are different capabilities. Ordinary OptiPlex Micro and ThinkCentre Tiny configurations should not be assumed to accept USB-C power.

| Candidate | Verified power-input capability | Configuration / buying judgment |
|---|---|---|
| **Minisforum UM760 Slim** | Manufacturer specifies USB4 with **65–100W PD input**, alongside its normal 19V DC input | Ryzen 5 7640HS, replaceable RAM, HDMI/DP, Wi-Fi 6E. Useful expandable shared host, not a cheap dedicated worker [^um760-pd] |
| **ACEMAGIC F2A, X7 358H version** | Manufacturer explicitly specifies **100W PD input** on USB4 and the other full-function USB-C port; also has 19V DC input | Current Micro Center unit is **32GB/1TB, $1,099.99**, with fixed RAM. A documented native-PD option, but substantially more expensive than UM760 Slim. Confirm that the boxed hardware matches this revision; PD operation was not physically tested [^f2a-mc][^f2a-spec] |
| **ThinkCentre M90n-1 Nano, non-IoT** | Rear USB-C explicitly supports **20V, greater than 45W power-in**; choose a suitable 65W PD source | Seek i5-8265U/8365U or i7-8665U and **16GB already installed**: RAM is soldered. DP rather than native HDMI; Wi-Fi optional. Genuine refurb candidate, but no actionable US 16GB offer was verified in this check [^nano-pd] |
| **HP Elite Mini 800 G9, 35W CPU / integrated graphics** | Optional rear Flex IO USB-C module supports **up to 100W PD intake**; HP limits its single-cable scenario to 35W UMA configurations | Look for the installed **100WPD Flex IO v2, part 13L60AA**. A front USB-C port or generic “Flex IO” listing is insufficient. No qualifying complete refurb price was verified [^hp-pd] |
| **MeLE Quieter 4C N150, SE Grade** | Full-function USB-C explicitly supports **PD 3.0 input**; separate USB-C-shaped DC input is listed as 12V/2A | Fanless, 16GB/512GB, dual HDMI, Wi-Fi 5; direct page showed **$489.99** and Add to cart. Technically suitable, **poor price for an N150**. Confirm accepted PD profiles for the exact revision before standardizing chargers [^mele-pd] |
| **GMKtec G2 Plus N150** | A firsthand test demonstrated third-party PD operation and recommends at least **15V/3A, 45W** | Tested version has **12GB soldered RAM**, below the preferred 16GB. Supplied adapter was measured as fixed 12V, **not compliant USB-PD**. Treat this as a caveated experiment, not the fleet PD default; no current US deal verified [^g2-pd] |

**Important negative:** ASUS explicitly says **NUC 14 Essential cannot be powered through its USB-C port**; it requires its 19V barrel input.[^nuc-no-pd]

### Charger and deployment rules

- For UM760-class machines, choose a reputable **100W PD supply and a 5A e-marked USB-C cable** rated for 100W or 240W. The supply rating is available capacity, not continuous system consumption.
- Match the required **voltage profile and per-port wattage**, not merely the charger's total wattage. Do not assume every “65W” charger or monitor supplies the required profile.
- A multiport charger may redistribute power or interrupt outputs when another device connects. Test cold boot, full load, reconnect, and AC-loss recovery with the exact charger/cable combination. One shared charger also becomes a shared failure point.
- **Never reuse a fixed-12V USB-C-shaped supply as a phone charger.** The G2 Plus review measured precisely this dangerous adapter behavior; the computer itself did negotiate PD with suitable third-party sources.[^g2-pd]
- A PD-to-barrel trigger cable is an adapter workaround, not native USB-C input. Voltage, polarity, connector, current capacity, and any OEM adapter-identification requirements still apply; do not make this the untested fleet standard.

**Economical alternative:** a Latitude 5420 already supplies the desired Wi-Fi, diagnostic screen, keyboard, battery backup, and **USB-C charging**. Dell documents 65W and 90W USB-C adapters. A laptop may meet the common-charger objective more cheaply than forcing a premium USB-C mini into the budget.[^latitude-power]

---

## KVM, headless recovery, and physical stacking

“KVM” has two relevant meanings here:

- **Keyboard/video/mouse:** a local HDMI/USB switch or a hardware KVM-over-IP device for firmware and recovery.
- **Kernel-based Virtual Machine:** Linux virtualization, commonly managed through Proxmox or libvirt. It does not provide a physical motherboard's BIOS console when the host itself is broken.

### Cheap local access

A portable HDMI monitor and USB keyboard are sufficient for initial experiments. Choose full-size HDMI when possible; DP-only business boxes and mini-DP GPUs can work with the appropriate adapter. VGA adds no agent capability and often needs an active conversion path into an HDMI capture device.

If a host is physically accessible, a laptop screen or spare monitor and USB keyboard may suffice. A local KVM shares desk peripherals; an IP KVM addresses remote recovery instead.

An ordinary KVM switch does not automate Excel/Outlook or add tools to omp. An IP KVM can provide screenshots and keyboard/mouse control for a separately implemented automation system, but does not provide browser DOM access, application APIs, or automatic agent integration. Prefer APIs, browser automation, and local scripts for routine work.

For a rack of ten, an HDMI/USB KVM switch can reduce monitor cabling, but check **EDID persistence**. Switching the monitor away can change desktop resolution or display availability. Use a consistent virtual display or EDID emulator where needed for GUI automation; a dummy HDMI plug is not required for a genuinely headless browser workflow.

### Remote recovery

- SSH and [[Tailscale]] are the normal Linux management path; RDP or another appropriate remote desktop handles Windows GUI diagnosis. Keep management services private.
- Hardware KVM-over-IP is useful when the OS/network configuration is broken or firmware access is necessary. JetKVM's published updated MSRP is **$103**, or **$119 for PoE**—not the old launch price frequently repeated in guides.[^jetkvm]
- One KVM-over-IP plus a suitable multiport HDMI/USB switch can serve a stack, **if** switching, BIOS resolution, EDID, and USB behavior are verified. It is not automatically ten independent remote consoles.
- Remote video/input does not necessarily supply remote power-button control. Tiny PCs may lack a standard exposed ATX header; check the exact supported power-control accessory or use a managed outlet with verified AC-restore behavior.
- Select and test **power on after AC loss**, boot without keyboard/monitor, service startup after reboot, and remote access after wireless reconnect. Do not assume every budget BIOS implements these correctly.
- Wake-on-LAN is not equivalent to guaranteed wake over Wi-Fi. Intel vPro/AMT can be useful on an explicitly supported, provisioned configuration, but an Intel CPU sticker is not proof of remote KVM capability.
- Full-disk encryption can stop at a preboot unlock prompt after an outage. Decide the remote-unlock and physical-security policy before an off-site deployment.
- A capture device cannot recover a display signal the GPU no longer produces. This matters when experimenting with AMD drivers: IP KVM is useful for visible boot/BIOS output, not a guarantee of video during every GPU fault.

### Intel vPro: what it means here

**vPro is Intel's business-PC platform designation, not a faster CPU mode, an AI accelerator, or a synonym for virtualization.** It combines supported hardware, firmware, security, stability, and management capabilities. The useful feature for an unattended fleet is **Intel AMT (Active Management Technology)**: management firmware that can operate independently of the host OS.[^vpro]

**Example:** an agent computer freezes at another location. SSH/RDP no longer works. On a correctly configured AMT machine, you may still power-cycle it and use a remote keyboard/video/mouse console to inspect startup or BIOS without visiting it. The computer still needs power and a working management-network path.

| Technology / label | What it provides | Limitation |
|---|---|---|
| SSH / RDP / ordinary remote desktop | Routine access to the running OS | Usually unavailable when the OS or its network stack is broken |
| Wake-on-LAN | A wake signal on a supported network/power configuration | Not a BIOS video console or general remote recovery interface |
| **vPro Enterprise / AMT** | Out-of-band power control, inventory, and supported KVM/boot-redirection features | Requires an eligible complete platform, provisioning, and an AMT-capable management console |
| **vPro Essentials / Intel Standard Manageability** | A smaller management feature set | Intel's comparison excludes AMT's out-of-band KVM and wireless management; not equivalent to full AMT [^amt-tiers] |
| External hardware KVM-over-IP | Independent video/input capture and optional power accessories | Extra hardware and cabling; power control still needs a compatible connection |

For refurbished purchasing:

- **Verify the exact OEM configuration supports AMT and KVM.** An eligible i5/i7 model, Intel Wi-Fi card, or listing keyword alone does not establish a complete working vPro platform.
- Have the seller remove previous-owner provisioning and BIOS/MEBx passwords; update OEM BIOS and Intel ME/CSME firmware before enrollment.
- Provision and test it locally before off-site deployment. **Client Control Mode requires local user consent for KVM/redirection**; that can defeat unattended recovery. Admin Control Mode supports different policies but needs deliberate, secure setup.[^amt-setup][^amt-consent]
- Prefer built-in wired Ethernet. Wireless AMT needs supported Intel WLAN hardware and provisioned network profiles; available power-state behavior is platform-dependent.
- Use TLS, unique credentials, current firmware, and a private management network. **Do not expose AMT directly to the public internet.** A VPN running only on the failed worker cannot provide its own recovery path; use a surviving gateway/management host or a properly deployed management service.

**Buying judgment:** a useful tie-breaker for remotely deployed workers, not a reason to pay a large premium for a supervised pilot. Linux virtualization uses VT-x/VT-d rather than requiring the vPro badge. AMD PRO is a separate platform, not Intel AMT.

### Stackable is an airflow requirement

Use a shelf/rack with unobstructed intakes and exhausts, not ten cases pressed together. Preserve underside clearance on bottom-intake designs, including EQ14. Arrange power adapters outside the hot exhaust path, label both ends of every cable, and keep one compatible spare adapter or spare node. Use properly rated power distribution; do not improvise a shared DC supply without matching voltages, polarity, connectors, and per-output protection.

A UPS and automatic power-loss recovery are useful; they do not replace backups of worker state, credentials, and outputs.

### Comparing UPS and KVM options

Example US retail prices, not current stock guarantees.[^apc-mc][^cyberpower-mc][^local-kvm-mc][^comet-mc]

| Item | Price | Capability and buying decision |
|---|---:|---|
| **APC Back-UPS Pro BN1500M2**, SKU **608661** | **$149.99**, advertised against $259.99 | **1500VA / 900W**, stepped approximation to sine wave on battery. **Value pick if backup is needed now**; test actual load compatibility |
| **CyberPower CP1500PFCLCD**, SKU **180554** | **$239.99**, advertised against $259.99 | **1500VA / 1000W**, pure sine wave on battery, AVR. Technically suitable, **not a compelling sale**; costs $90 more than the APC |
| **IOGear GCS32HU**, SKU **414342** | **$42.99** | Two-PC HDMI/USB **local** KVM, maximum **1080p/60Hz**, integrated cables. Optional desk convenience, not network remote access |
| **GL.iNet Comet PoE GL-RM1PE**, SKU **073296** | **$115.99** | HDMI/USB **IP KVM** with network console and virtual media. One attached computer, not ten independent consoles |

**Waveform versus deal value:** APC's model-specific data sheet explicitly says **“Stepped approximation to a sinewave.”** That is its inverter output on battery, not a claim that normal utility power is always converted into a stepped waveform. CyberPower specifies sine-wave output for CP1500PFCLCD. Pure sine reduces a power-supply compatibility concern, but **active PFC alone does not prove a stepped-wave UPS will fail**. The CyberPower is **60% more expensive** than the $149.99 APC; do not pay that premium merely because its small markdown is called a sale.[^apc-spec][^cyberpower-spec]

**Before relying on a UPS:** Save work, charge the UPS, then follow its manufacturer's procedure to test battery operation and switchover with the actual equipment at expected load during the return window. If the computer drops out or behaves abnormally, return/exchange for a compatible model rather than assuming all stepped-wave units or all PC supplies behave alike.

- **VA/W capacity is not runtime.** Use the runtime curve at measured combined load, leave capacity headroom, and count battery-backed outlets rather than total outlets. The APC has **six battery-backed plus four surge-only outlets**. This is not a sizing recommendation for a future high-power inference box.[^apc-spec]
- Put the router, modem/ONT, and relevant network switch on battery backup too if agents must remain connected. Configure UPS-triggered graceful shutdown; USB charging ports are not the UPS data/management connection.
- The E6430 batteries already provide local ride-through if healthy, but do not keep the network running. A UPS is not a backup of files, credentials, or worker checkpoints.
- If IP KVM becomes useful later, Comet PoE needs suitable network/power arrangements; confirm the included adapter or use a compatible PoE switch/injector. **Hard power/reset control requires an accessory**. Verify Dell front-panel/header compatibility before buying an ATX board. Keep management access private behind LAN/VPN; do not expose it directly to the internet.[^comet-docs]

---

## Current Black Friday research and live listing checks

**Black Friday 2026 is November 27.** This snapshot is during early promotions, not the final event. A crossed-out price is not verified historical savings. Listed inventory can change, and no coupon below was validated through a completed checkout.

### Offers and configurations actually found

| Listing / promotion | Observed configuration and asking price | Availability / verification | Judgment |
|---|---|---|---|
| **Dell Refurbished `EARLYBF50`** | Official coupons page advertises **50% off Small and Micro desktops** | Current official early-BF promotion; Hot Deals excluded; coupons do not combine [^dell-coupon] | Real promotion; judge the resulting machine price, not the percentage |
| **OptiPlex 3000 Micro**, listing `000001` | i5-12500T, 16GB, 256GB, Win 11 Pro, Grade A; **$799 list → $399.50 if coupon applies** | Category/product pages showed **30+ available**; browser configuration confirmed **Wi-Fi + Bluetooth**. Effective price is calculated from advertised terms, **not checkout-verified** [^dell-stock] | Capable repeat-purchase option with included wireless, but not the cheapest ten-node route |
| **OptiPlex 3000 SFF**, listing `000041` | i5-12500, 16GB, 256GB, Win 11 Pro, Grade B; **$709 list → $354.50 if eligible** | Category page showed 17 remaining; same coupon caveat [^dell-stock] | Interesting shared-host base; **not a prevalidated GPU build** |
| **M75q Gen 2 on Newegg marketplace** | 4650GE, 16GB, **128GB** SSD, Wi-Fi adapter; **$269.99** | Product page offered Add to cart; sold/shipped by **Munir Metal & Electronics**, not Newegg; seller showed 3.5/5 from eight ratings [^m75-offer] | Usable current asking price, not an outstanding deal; small SSD and unspecified Wi-Fi |
| **M75q Gen 2 eBay lead**, item `236111048822` | 4650GE, 16GB, 256GB; search result displayed **$199.99** | **Unverified lead**: direct fetch blocked and browser showed an error page; stock, seller terms, and final price unconfirmed [^ebay-lead] | Worth manually checking; **not counted as an available fleet deal** |
| **GMKtec G3 Plus direct** | N150, 16GB/512GB; **$329.99** | Product page explicitly said **Sold out** [^g3plus] | Not an actionable bargain; do not quote its old $159.99 review price as current |
| **Beelink EQ14 direct** | N150, 16GB/500GB, dual 2.5GbE; **$389**, with $479 comparison price | Selected variant labelled **Pre-order** [^eq14] | Poor fit for the cheapest-worker objective at this asking price |
| **GMKtec G11 Plus direct** | Ryzen 5 3500U; **$169.99 starting price is barebones** | Configured options exist, but their selected price/stock was not independently verified [^g11] | Not a $169.99 complete 16GB computer; add RAM, SSD, OS if needed |
| **MSI RTX 3050 LP 6G OC at Newegg** | **$279.99 new**, shown against $296.99 | Live browser page: sold/shipped by **Newegg**, Add to cart; advertised 5% saving [^3050-offer] | Real current listing, weak value for this experiment; complete PC costs extra |
| **PNY RTX A400 at B&H** | **$269 new**, used link from **$241.95** | Product page showed **In Stock**; used condition/stock not separately checked [^a400-offer] | Tiny 50W card, but a substantial premium for only 4GB VRAM |
| **Dell T600 used at PCSP** | **$177.61**, high-profile bracket shown | Explicitly **out of stock** [^t600-offer] | Exclude from actionable deals; bracket also matters for an SFF build |

Newegg also displayed a **“1 Refurbished from $149.99”** teaser on the MSI 3050 page. The individual offer's seller, condition, and availability did not resolve during inspection. Treat it as a lead, not a validated $149.99 purchase recommendation.[^3050-offer]

### GMKtec sale terms: useful, but internally inconsistent

The official announcement describes **early access November 5–22**, Black Friday week **November 23–27**, and its branded Cyber Monday event **November 28–December 2**. It advertises 30-day early-access price protection and a G/M/K-series multi-buy discount of $15 for two, $25 for three, or $50 for five.[^gmk-bf]

However, the live sale page also shows **4% off $600 / 6% off $1,300 / 8% off $2,200+**, alongside the per-series multi-buy offer, with non-combination restrictions. The announcement and footer also differ on the newsletter coupon amount. **Do not add these promotions together or assume five-unit savings automatically double for ten units.** Obtain the applicable cart total and written price-protection conditions before a batch order.[^gmk-sale]

The announcement's $229.99 Celeron N5095 G5S and $269.99 Pentium 7505 G3 Ultra are not automatically good deals because they appear in a Black Friday article. The G5S collection entry specifies **8GB soldered RAM**; that fails the preferred upgradeable 16GB configuration.[^gmk-bf][^gmk-gseries]

**Deal conclusion:** the strongest verified promotion here is Dell's official code, while the most interesting cheap Ryzen offer remains a lead requiring manual validation. No current, in-stock, complete 16GB N150 US offer under the suggested value ceiling was verified. Waiting for better pricing is reasonable; buying a poor configuration because of a sale banner is not.

---

## Micro Center configuration examples

These are example US retail prices before tax, not quotes or availability guarantees.

### Small desktop options

| Role | Exact Micro Center configuration | Price | Main tradeoff |
|---|---|---:|---|
| **Best balanced business mini** | [Lenovo ThinkCentre M80Q refurbished](<https://www.microcenter.com/product/716766/lenovo-thinkcentre-m80q-desktop-computer-(refurbished)>), SKU **098285**; i5-10500T, 6 cores / 12 threads, 16GB/256GB | **$349.99** | HDMI + DP, six USB ports including USB-C, listed 64GB RAM ceiling; **Wi-Fi not specified** |
| **Lower-cost refurbished mini** | [HP EliteDesk 800 G4 MFF refurbished](<https://www.microcenter.com/product/704294/hp-elitedesk-800-g4-mff-desktop-computer-(refurbished)>), SKU **957902**; i5-8500T, 6 cores / 6 threads, 16GB/256GB | **$299.99** | Older CPU; two DP outputs rather than listed HDMI; **Wi-Fi not specified** |
| **New machine with advertised Wi-Fi included** | [GMKtec NucBox M8](https://www.microcenter.com/product/708606/gmktec-nucbox-m8-mini-pc), SKU **006080**; Ryzen 5 PRO 6650H, 6 cores / 12 threads, 16GB/512GB, Radeon 660M | **$449.99** | HDMI + DP + USB4, dual 2.5GbE, advertised Wi-Fi 6E; **16GB onboard RAM** limits expansion |
| **Roomier Intel business worker** | [Dell OptiPlex 7090 refurbished](<https://www.microcenter.com/product/699102/dell-optiplex-7090-desktop-computer-(refurbished)>), SKU **906206**; i5-11500T, 6 cores / 12 threads, 16GB/512GB | **$399.99** | Reasonable alternative to the M80Q; retailer's detailed chassis, display-port, and wireless information is incomplete |
| **Expandable shared-host candidate** | [Minisforum UM760 Slim](https://www.microcenter.com/product/689621/minisforum-um760-slim-mff-mini-pc), SKU **794602**; Ryzen 5 7640HS, 6 cores / 12 threads, 16GB/1TB, Radeon 760M | **$599.99** | Two DDR5 slots, retailer-listed 96GB maximum, HDMI + DP + USB4, advertised Wi-Fi 6E; RAM upgrades still cost extra |
| **Premium 32GB native-PD mini** | [ACEMAGIC F2A](https://www.microcenter.com/product/711594/acemagic-f2a-mff-mini-pc), SKU **044271**, retailer Mfr Part **AE3 PRO**; Core Ultra X7 358H, 16 cores / 16 threads, **32GB LPDDR5X-7500/1TB**, Arc B390 | **$1,099.99** | HDMI + DP, four USB-A, USB4, dual 2.5GbE, Wi-Fi 7; official current F2A documentation specifies **100W PD input**. RAM is **soldered**, and the retailer part-number mismatch needs checking [^f2a-mc][^f2a-spec] |
| **Larger chassis for expansion** | [HP ProDesk 600 G6 SFF refurbished](<https://www.microcenter.com/product/700377/hp-prodesk-600-g6-sff-desktop-computer-(refurbished)>), SKU **916957**; i5-10500, 6 cores / 12 threads, 16GB/512GB | **$359.99** | Four RAM slots, listed 128GB ceiling, PCIe x16 and x4; approximately 269 × 302 × 94mm, not a 1L mini; GPU fit/power **not validated** |
| **ARM/macOS operator or worker** | [Mac mini M4, MU9D3LL/A](https://www.microcenter.com/product/688173/apple-mac-mini-mu9d3ll-a-(late-2024)-desktop-computer), SKU **776963**; 16GB/256GB | **$599.99** | HDMI and Wi-Fi 6E; fixed RAM, USB-C rather than USB-A, no CUDA; not the cheapest fleet worker |
| **Current-generation ARM alternative** | [Mac mini M6, MHQK4LL/A](https://www.microcenter.com/product/715409/apple-mac-mini-mhqk4ll-a-(mid-2026)-desktop-computer), SKU **087015**; 16GB/256GB | **$699.99** | Advertised Wi-Fi 7 and 2.5GbE; check order limits before planning a fleet |

The PC configurations above list Windows 11 Pro; the Macs run macOS. Refurbished products sometimes appear under Micro Center's generic “NEW IN STOCK” wording: that stock label does **not** change the item's refurbished condition.

### Choosing by workload

- **One CPU-only experimental worker:** the **M80Q** is the balanced pick if Ethernet is acceptable or the actual wireless hardware is confirmed. Its newer six-core/twelve-thread CPU is preferable to the older six-thread G4 when the price difference is acceptable. It is not a discrete-GPU upgrade chassis.
- **Wi-Fi-ready new worker:** choose the **M8**, provided 16GB is enough for its lifetime. Its integrated Radeon is useful for graphics/media, **not CUDA**. The exact Wi-Fi chipset and Linux behavior still need verification; “Wi-Fi 6E” alone does not establish that.
- **Several workers per computer:** the **UM760 Slim** is a better foundation than a fixed-16GB box. Budget additional RAM and validate concurrency. Do not mistake its stock 16GB configuration for a finished five-VM host.
- **Premium graphics/PD workstation rather than a cheap worker:** the **F2A X7 358H** is a real option when its Arc graphics, CPU platform, and native 100W USB-C PD solve an identified need. For API agents, benchmark before paying a premium for graphics or choosing fixed RAM over an expandable host.
- **Expansion over minimum size:** inspect the **HP SFF**. Its four RAM slots are useful for consolidation. Treat a GPU installation as a separate compatibility decision, not a guaranteed drop-in build.
- **Native macOS/Tern use:** either Mac is credible as an operator/control machine. The M6 is a real early-Black-Friday option, but neither Mac price makes ten dedicated light-duty workers the economical default.

**M8 specification trap:** Micro Center calls the installed RAM onboard LPDDR5 yet also lists “Maximum Memory Supported: 64GB” and “Expandability: Memory.” [GMKtec's own product page](https://www.gmktec.com/products/gmktec-m8-amd-ryzen-5-pro-6650h-mini-pc) explicitly specifies **16GB LPDDR5 onboard**. Treat this unit as fixed-16GB, not as a verified upgradeable 64GB host.

**F2A revision and value caveats:** the linked Micro Center page currently specifies **Core Ultra X7 358H / Arc B390**, not the older **Core Ultra 5 125H F2A** found in many reviews. ACEMAGIC's current official F2A page corroborates the new processor, 32GB soldered memory, dimensions, networking, and PD input, but Micro Center's manufacturer-part field says **AE3 PRO**. Verify the box label before purchase. The manufacturer lists two NVMe slots; that does **not** make the RAM expandable. Its rear USB-C speed is inconsistently described as 10Gbps/20Gbps on the manufacturer's page, so do not buy it for an unverified port-speed detail.[^f2a-mc][^f2a-spec]

**F2A buying judgment:** technically suitable, economically weak for ten dedicated API workers. One 32GB unit could host several appropriately sized workers, but concurrency is unmeasured and it cannot grow into the note's 64GB shared-host configuration. Arc B390 is integrated graphics, **not CUDA**, and the NPU's TOPS do not accelerate ordinary Python/Excel/API calls. Its $400 retailer markdown is not proof of historical-low pricing.

**Wireless and warranty:** the checked M80Q, HP, and Dell refurb listings do not promise Wi-Fi. Do not budget them as wireless-ready without inspecting the card, antennas, or included adapter. The checked business refurbs list **90-day limited warranties**; M8 and the linked F2A list **one year**, and UM760 lists two years. The F2A's advertised Wi-Fi 7 does not identify its actual wireless module or establish Linux compatibility. Product pages show a **15-day desktop return window**. Confirm any applicable holiday exception rather than assuming a long trial period.

No new wall-power measurements were obtained for these configurations. CPU power profiles and a 65W/120W adapter rating are not system idle consumption.

### Low-profile GPU option

[MSI RTX 3050 LP 6G OC](https://www.microcenter.com/product/693900/msi-nvidia-geforce-rtx-3050-lp-dual-fan-6gb-gddr6-pcie-40-low-profile-graphics-card), SKU **846386**: **$289.99**. This is the 6GB, slot-powered, low-profile **dual-slot** version, approximately **174 × 69 × 42mm**, with two HDMI outputs and one DP. It fits the desired low-power CUDA category technically, but the asking price is weak for an experimental office-agent fleet.


None of the mini PCs above accepts that dual-slot retail card internally. The HP SFF has PCIe expansion, but its exact PSU, slot-power allowance, card clearance, and airflow must be established first. An M8 OCuLink eGPU needs an external dock, power supply, and cabling; it is not the simple stackable route.

### Other configurations and tradeoffs

- **GMKtec G10**, SKU **957225**: $349.99; Ryzen 5 3500U, 16GB/512GB, Wi-Fi 5. It works, but the same-price M80Q offers a more attractive CPU platform for concurrent work; choose the G10 only for a specific included-networking or form-factor reason.
- **ACEMAGIC E1 N150**, SKU **044289**: $399.99. A poor price for this CPU class; the stronger M8 is the more interesting new-machine option.
- **GMKtec K17**, SKU **027334**: $549.99; Core Ultra 5 226V, 16GB/512GB, Arc 130V. A modern integrated-graphics option, but fixed memory and a higher price weaken its case for these light workers. It is not a CUDA substitute.
- **HP EliteDesk 800 G6 mini**, SKU **958322**: $349.99, i5-10500T/16GB/256GB. A valid M80Q alternative, not a distinctly better deal; its listing does not promise Wi-Fi and omits the display-port details.

### Black Friday and ten-unit reality

[Micro Center's official announcement](https://www.microcenter.com/site/mc-news/article/micro-center-early-black-friday-deals.aspx) dates this early event **November 13–15**, the next early event **November 20–22**, Black Friday weekend **November 27–29**, and Cyber Week **November 30–December 6**. The announcement explicitly advertises the M6 Mac mini at $699.99 against $899.99. That does not establish historical-low pricing or guarantee every other listed SKU's price through the same dates.

For ten matching machines, request a business-sales quote including RAM, wireless hardware and adapters. Validate a single unit before placing a batch order.

---

## Buying laptops and replenishing a fleet over time

Favor direct sellers of off-lease business equipment over changing marketplace sellers. The supplier judgments below use published sourcing/support terms, **not audited failure rates or personal purchasing experience**. No refurbisher guarantees indefinite stock of one old SKU.

### Additional actionable offers

| Offer | Complete configuration / price before tax | Stock and deal status | Buying judgment |
|---|---|---|---|
| **Discount Computer Depot Latitude 5430**, SKU `96287-3` | i5-1235U, 16GB/512GB, FHD, Wi-Fi, Win 11 Pro; **$349.99 with included one-year warranty** | Live browser had Add to Cart; exact quantity not published. Required warranty selector initially chose a three-year option at $454.99; selecting **one year** produced $349.99 [^depot-laptop] | Newer CPU and longer included coverage than the DFS 5420. A regular current offer, not a verified historical-low BF discount |
| **Dell Refurbished Latitude 5420**, listing `000608` | i7-1185G7, 16GB/512GB, FHD, Wi-Fi + Bluetooth, **Grade B, $359** | **30+ available**, live browser; current Hot Deal against $519 list. Grade A listing `000311` was $409 with 30+ available. No additional coupon on Hot Deals [^latitude-deal] | Most clearly documented matching laptop lot. Ships listed with **Windows 10 Pro**: install a supported OS before deployment |
| **REFURB.io USA Latitude 5420**, SKU `049500453182` | i5-1145G7, 16GB/512GB, Grade A, adapter; **$419 listed → $377.10 if advertised extra 10% applies** | Product page showed **12 in stock**. Extra 10% on orders over $200 ends **November 18, 2026, 11:59 PM EST**; cannot combine offers; calculated price **not checkout-verified** [^refurbio-deal] | One-year system and 90-day battery coverage. Listing conflicts between Windows 10/11 fields: get the shipped OS confirmed |
| **Dell Refurbished OptiPlex 7090 Micro**, listing `000187` | **Corrected selected configuration: i5-10600T**, 16GB/256GB, Grade A, **No WLAN**, Windows 10 Pro; **$649 → $324.50** | Follow-up browser showed **30 remaining**; adding **ten** to a temporary cart and applying `EARLYBF50` discounted every unit. Cart cleared, no order placed [^7090-deal][^dell-coupon] | Earlier i5-10500T description was incorrect. Install a supported OS; networking extra. Ten units total **$3,245 before tax**, not an accepted purchase order |
| **Lenovo Outlet promotion** | Official laptop Outlet page advertises **extra 15% with `OUTLET15`** | No complete, in-stock target ThinkPad/ThinkCentre quote verified. Coupon applicability must be checked on the chosen configuration [^lenovo-outlet] | A manufacturer-backed watch channel, **not an invented discounted machine price** |

The DFS 7090 page also offered a Grade B i5-10600T/16GB/256GB unit, listing `000188`, at $579 before coupon, but **only one remained** and it lacked WLAN. Its conditional $289.50 price is a pilot opportunity, not evidence of repeatable fleet stock.[^7090-deal]

**Coupon and returns follow-up:** the official page still advertised `EARLYBF50` but published **no expiration date**; do not assume it lasts through Black Friday. No numerical per-customer cap was found in the checked terms; a successful ten-unit test cart does not guarantee order acceptance. Dell's return window begins **30 days from the invoice date**, not delivery. RMA required; defective/seller-error returns have no restocking fee, while remorse returns may incur **15% plus applicable shipping**. Buy one pilot rather than treating ten machines as a free trial.[^dell-coupon][^dfs-terms][^dfs-faq-returns]

**Watch, do not count as available:** Wisetek's T14 Gen 2 Ryzen 5 PRO 5650U/16GB/256GB page displayed **$256.02 but Unavailable/backordered**. Its price is attractive; both stock and business-use warranty terms need resolution before recommending it.[^wisetek-deal][^wisetek-terms]

### Sources to use repeatedly

| Supplier | Published protection | Fleet fit / important catch |
|---|---|---|
| **Dell Refurbished / Dell Financial Services** | **100-day** hardware warranty; paid extension advertised as 360 days/one year costs **$49 laptop / $39 desktop**. Batteries excluded. **30-day** returns; remorse returns may incur **15%**, defective/seller-error returns get free labels and no restocking fee [^dfs-terms] | Best documented direct off-lease channel here; FAQ says inventory arrives several times weekly. Fixed prepacked configurations, **no customization**. Formal bulk program requires **100 assets**, not ten: grow this fleet through normal retail orders [^dfs-buying] |
| **Lenovo Certified Refurbished / Outlet** | Official certification provides **one-year Lenovo warranty**; program advertises **minimum 80% battery performance**. US shopping FAQ specifies **14-day Outlet returns**, distinct from ordinary Lenovo.com [^lenovo-certified][^lenovo-returns] | Good manufacturer-backed ThinkPad/ThinkCentre channel; verify the actual certified item and its grade. Stock rotates |
| **Discount Computer Depot** | **One-year** parts/labor; batteries excluded from warranty but policy says tested to **at least 80%**. Published 30-day refund process; customer pays return shipping after 30 days [^depot-terms] | Useful laptop/mini catalog and a business-account channel. Warranty excludes unauthorized internal modifications. Refund wording varies between order/receipt dates and includes a remorse-return disclaimer: obtain written pilot/upgrade terms rather than assuming a free compatibility trial [^depot-business] |
| **REFURB.io USA** | Generally **one-year** laptop/desktop warranty, **90 days for laptop batteries**; product pages may specify exceptions. **30-day** refunds, prepaid continental-US returns, **15% restocking unless defective** [^refurbio-terms] | A direct supplier with explicit inventory counts; useful alternative when DFS configuration/coverage is unattractive. Read grade-specific coverage rather than assuming every listing gets one year |
| **PC Server & Parts / PCSP** | Configured desktops/workstations/servers advertise **one-year parts, labor, shipping**, extendable up to five years. **90-day** returns; **20% restocking** for order errors or “defective” units found operational [^pcsp-terms] | Quote-based option for matching desktop lots and parts. No qualifying cheap mini lot independently verified here. Ask for an actual mini quote; **do not buy an old rack server just because the supplier specializes in them** [^pcsp-quote] |
| **Micro Center** | Checked business refurbs above: **90-day** warranty and **15-day** desktop returns | Retail availability does not establish a replenishment agreement. Use business sales for matching future batches |

**Return-policy risks for pilot hardware:**

- **Discount Electronics:** published online returns are **store credit only**, with **15% restocking**; even pre-shipment cancellation can incur that fee and store-credit restriction. Its checked 7070 Micro “early BF” page asked $565 before Wi-Fi and described preorder timing. Store credit limits the value of a compatibility trial.[^discount-electronics]
- **Wisetek Market:** retail warranty wording specifies **personal use**, excludes its reseller program, and says installing manufacturer-unapproved software or opening the device can void coverage. The product listing promises a charger, while warranty terms separately exclude chargers/accessories. Obtain written coverage for business agents, Linux, and upgrades; do not rely on its one-year banner alone.[^wisetek-terms]

### A repeatable buying process

1. **Choose a family, not an immortal SKU:** Latitude 5420/5430 or ThinkPad T14 AMD for laptops; OptiPlex Micro, M75q Gen 2, or EliteDesk Mini for desktops. Approve one successor model after testing rather than accumulating ten unrelated bargains.
2. Buy **one pilot**, then batches of two or three as jobs become useful. Keep a compatible cold spare once downtime matters. A matching lot today is convenient, not a commitment to purchase ten immediately.
3. Request **delivered price**, exact CPU, 16GB RAM, SSD size, adapter wattage, internal Wi-Fi chipset/antennas versus USB dongle, HDMI/DP, and warranty in writing. For USB-C mini purchases, add the **PD input voltage/wattage and exact optional module**.
4. Require **no BIOS/admin passwords, no prior AMT enrollment, and no previous-owner Autopilot/MDM tenant registration**. An erased SSD alone is not proof of a clean corporate device.
5. During the return window, inspect battery health/swelling and SSD health, update firmware, reinstall the chosen OS, and exercise the actual agent workflow, Wi-Fi reconnect, sleep settings, cold boot, and outage recovery. Confirm opening/upgrading permissions before touching warranty-sensitive hardware.
6. Track serial/service tag, purchase date, supplier, warranty end, BIOS version, charger, and OS image. Restore worker state to replacement hardware instead of treating the original computer as irreplaceable.

For replenishment, compare repeat-batch quotes with manufacturer outlet offers. A desktop SKU, laptop SKU and expandable native-PD host solve different deployment problems.

---

## Two ways to deploy ten workers

### Option 1: ten physical computers

Choose this when workers go to different sites, need independent Windows desktops, require local peripherals, or when a failure should stop only one physical worker.

- Standardize on one CPU-only model, OS image, adapter, and wireless configuration after the pilot.
- Use one identity, browser profile, working directory, and credential boundary per worker/business.
- A **planning budget of $200–250 per complete used node** gives **$2,000–2,500 for ten**, before network/power accessories. This is a procurement target, not ten units of verified live stock.
- At the checked $269.99 marketplace asking price, ten machines would instead be **$2,699.90**, still with only 128GB SSDs and without proof of ten-unit availability.
- If ten physical workers really are needed, a cold spare is an eleventh computer. Do not quietly count a spare or management-only machine as one of the ten productive workers.
- A GPU-equipped worker can replace one CPU-only worker when appropriate; buy its CPU/RAM for the office job as well as its GPU for acceleration.

### Option 2: two shared hosts, five isolated workers each

Choose this when all agents live at one site and mostly use APIs or browser applications. This reduces boards, idle overhead, power bricks, and management endpoints, but one host outage affects several workers.

A concrete starting point is **two M75q Gen 2 or newer Core minis, each with 64GB RAM and a 1TB SSD**, with Linux/KVM virtualization. Allocate five Linux browser VMs at 8GB each per host, leaving memory for the host and bursts. Two 16GB machines do **not** provide this capacity.

- This is a **capacity hypothesis**, not a measured claim of ten busy workers. Benchmark concurrent page loads, data processing, and memory peaks before committing.
- API-only workers can use much smaller allocations. Heavy Windows desktops may need 12–16GB each, making this sizing inadequate without more hosts or fewer workers per host.
- Containers are lighter, but share a kernel. Use VMs for stronger separation between businesses or risky browsing; browser profiles alone are not a security boundary.
- Two independent hosts are not automatic high availability. Do not add shared storage, Ceph, or Kubernetes simply to run ten experiments.
- A shared GPU worker is a separate acceleration role unless it also hosts one of the ten office workers. Keep the worker count and machine count explicit.
- Price **complete 64GB/1TB builds**, including replacement memory and storage. Two cheap barebones/refurb base units are not the finished two-host bill.

### Minimal operating discipline

- Use a restartable service for each worker, not an unattended shell window as the only lifecycle mechanism.
- Disable sleep for always-on hosts, schedule updates, and verify startup after reboot and power loss.
- Retain task checkpoints and bounded logs; save structured outputs and only the browser evidence needed for diagnosis.
- Separate client/company credentials and OAuth scopes. Start with read-only access or drafts; require approval for payments, commitments, destructive changes, or bulk external messages.
- Check API budgets and application rate limits independently of hardware capacity.
- Test one real end-to-end job: collect data, process a spreadsheet, draft/send to a test mailbox, update a test Jira issue, reconnect after a network interruption, and resume after a reboot. Add meeting/video participation only if the job actually requires it.
- Pilot both an API path and a browser fallback. Do not choose the hardware solely from a synthetic CPU score.

**Final purchasing rule:** spend first on a reliable x86 box, adequate RAM, a replaceable SSD, known networking, and recovery access. Spend on a tiny GPU only for an identified accelerated job. Do not spend on SBC novelty, local-inference hardware, or “AI PC” branding for an API agent that mostly waits on the network.

---

## Related notes

- [[oh-my-pi]] — agent harness and local tool execution.
- [[Playwright]] — browser automation; relevant to OS/architecture choices.
- [[Tailscale]] — private remote management.
- [[Docker]] — lightweight worker packaging, not equivalent to VM isolation.
- [[Raspberry Pi 5]] — SBC comparison, not the default recommendation here.
- [[Lenovo ThinkCentre M900]] — older business-desktop family; distinguish Tiny from SFF/tower.

---

## Sources

Sources below were used for specifications, reported measurements, compatibility limits, and dated listing checks. Review-era prices are not current quotes; advertised discounts are not independent price-history verification.

[^omp]: [omp upstream README and installation platforms](https://github.com/can1357/oh-my-pi).
[^tern]: [Stencil Tern: native clients, persistent daemon, remote servers](https://stencil.so/tern).
[^openpyxl]: [openpyxl formula documentation](https://openpyxl.readthedocs.io/en/stable/simple_formulae.html).
[^graph]: [Microsoft Graph: workbook/createSession permissions](https://learn.microsoft.com/en-us/graph/api/workbook-createsession?view=graph-rest-1.0).
[^office-license]: [Microsoft: unattended Microsoft 365 Apps licensing](https://learn.microsoft.com/en-us/microsoft-365-apps/licensing-activation/overview-unattended).
[^office-risks]: [Microsoft: unattended Office automation considerations](https://learn.microsoft.com/en-us/office/client-developer/integration/considerations-unattended-automation-office-microsoft-365-for-unattended-rpa).
[^m75-spec]: [Lenovo M75q Gen 2 PSREF, including memory, ports, wireless options](https://psref.lenovo.com/syspool/Sys/PDF/ThinkCentre/ThinkCentre_M75q_Gen_2/ThinkCentre_M75q_Gen_2_Spec.pdf).
[^m720-offer]: [Joy Systems M720q i5-8500T/16GB/256GB specification listing](https://joysystemspc.com/products/lenovo-thinkcentre-m720q-tiny-intel-core-i5-8500t-2-1ghz-16gb-ddr4-256gb-ssd-internal-wifi-and-bluetooth-refurbished). Checked asking price was $449.99: a specification reference, not a recommended deal.
[^dell-memory]: [Dell OptiPlex 3000 Micro memory specifications](https://www.dell.com/support/manuals/en-us/optiplex-3000-micro/optiplex_3000_mff_setup_specs/memory?guid=guid-e9d2ea83-38a4-431d-803c-96d63c1dbc34&lang=en-us).
[^dell-ports]: [Dell OptiPlex 3000 Micro ports](https://www.dell.com/support/manuals/en-us/optiplex-3000-micro/optiplex_3000_mff_setup_specs/external-ports-and-slots?guid=guid-4c4cc1ba-66a4-45f0-8dfc-6013a624ddc0&lang=en-us).
[^g3plus]: [GMKtec G3 Plus direct product listing](https://www.gmktec.com/products/nucbox-g3-plus-enhanced-performance-mini-pc-with-intel-n150-processor).
[^g3-test]: [Jeff Geerling: G3 Plus test log, wall power, Linux, thermals, network chipset](https://github.com/geerlingguy/sbc-reviews/issues/64). Its $159.99 price is explicitly a January 2025 test-era price.
[^s13]: [Frandroid: firsthand Beelink Mini S13 review, SSD and I/O observations](https://www.frandroid.com/produits-android/ordinateurs/pc-fixes/2491676_test-beelink-mini-s13).
[^eq14]: [Beelink EQ14 direct specifications and selected preorder](https://www.bee-link.com/products/beelink-eq14-n150).
[^p350]: [Lenovo P350 Tiny PSREF: GPU options, power adapters, dimensions, riser](https://psref.lenovo.com/syspool/Sys/PDF/ThinkStation/ThinkStation_P350_Tiny/ThinkStation_P350_Tiny_Spec.PDF).
[^mac-spec]: [Apple Mac mini 2024 specifications](https://support.apple.com/en-us/121555).
[^apple-store]: [Apple current Mac mini store configurations](https://www.apple.com/shop/buy-mac/mac-mini). No current purchase price was reliably extracted from this dynamic page.
[^pi-spec]: [Raspberry Pi 5 current product brief](https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-product-brief.pdf).
[^pi-price]: [Raspberry Pi: November 2026 16GB price increase](https://www.raspberrypi.com/news/price-rises-for-some-16gb-raspberry-pi-products/).
[^orange]: [Orange Pi 5 Plus official specifications](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-5-plus.html).
[^wyse]: [Dell Wyse 5070 official storage specifications](https://www.dell.com/support/manuals/en-us/wyse-5070-thin-client/5070_std_ug/storage?guid=guid-4ac695f0-a4bf-435c-b96f-dd4f47bb523d&lang=en-us).
[^playwright]: [Playwright supported operating systems and architectures](https://playwright.dev/docs/intro#system-requirements).
[^a400]: [NVIDIA RTX A400 specifications](https://www.nvidia.com/en-us/products/workstations/rtx-a400).
[^a1000]: [NVIDIA RTX A1000 specifications](https://www.nvidia.com/en-us/products/workstations/rtx-a1000).
[^a2000]: [NVIDIA RTX A2000 6GB/12GB datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/products/workstations/nvidia-rtx-a2000-12gb-datasheet.pdf).
[^3050]: [NVIDIA RTX 3050: 6GB versus 8GB full specifications](https://www.nvidia.com/en-eu/geforce/graphics-cards/30-series/rtx-3050).
[^4050]: [NVIDIA GeForce RTX 40-series laptop GPUs](https://www.nvidia.com/en-us/geforce/laptops/40-series/).
[^cuda]: [NVIDIA: CUDA architecture support and pre-7.5 compilation removal](https://developer.nvidia.com/blog/navigating-gpu-architecture-support-a-guide-for-nvidia-cuda-developers/).
[^m75-power]: [ServeTheHome: M75q Gen 2 4750GE power measurements](https://www.servethehome.com/lenovo-thinkcentre-m75q-gen2-tiny-review-amd-changes-the-game/4/).
[^m720-power]: [ServeTheHome: M720q G5400T power measurements](https://www.servethehome.com/lenovo-thinkcentre-m720q-tinyminimicro-feature/3/).
[^p340-power]: [ServeTheHome: P340 Tiny GPU-system power and extended-idle observation](https://www.servethehome.com/lenovo-thinkstation-p340-tiny-1l-pc-with-nvidia-gpu-review-intel/3/).
[^mac-power]: [Apple Mac mini wall-power figures and test definitions](https://support.apple.com/en-us/103253).
[^iwlwifi]: [Linux Wireless: supported Intel chipsets and firmware requirements](https://wireless.docs.kernel.org/en/latest/en/users/drivers/iwlwifi.html).
[^ax211]: [Intel AX211 interface specification: CNVio2](https://www.intel.com/content/www/us/en/products/sku/204837/intel-wifi-6e-ax211-gig/specifications.html).
[^pve-wlan]: [Proxmox VE: WLAN limitations and guest networking](https://pve.proxmox.com/wiki/WLAN).
[^jetkvm]: [JetKVM: updated hardware and $103/$119 pricing](https://jetkvm.com/blog/new-internals-new-ports-price-update).
[^dell-coupon]: [Dell Refurbished official current coupon page](https://www.dellrefurbished.com/coupons).
[^dell-stock]: [Dell Refurbished desktop inventory: checked configuration prices and quantities](https://www.dellrefurbished.com/desktop-computers). [Micro listing 000001](https://www.dellrefurbished.com/item/dell-optiplex-3000-mff-000001/dell-optiplex-3000-mff/1.html); [SFF listing 000041](https://www.dellrefurbished.com/item/dell-optiplex-3000-sff-000041/dell-optiplex-3000-sff/1.html).
[^m75-offer]: [Newegg marketplace: M75q 4650GE/16GB/128GB, item 9SIBXJ9M129617](https://www.newegg.com/p/1VK-0003-1SSW9?Item=9SIBXJ9M129617).
[^ebay-lead]: [eBay item 236111048822, search-only price lead](https://www.ebay.com/itm/236111048822).
[^g11]: [GMKtec G11 Plus: barebones versus configured variants](https://www.gmktec.com/products/gmktec-g11-plus-amd-ryzen-5-3500u-mini-pc).
[^3050-offer]: [Newegg: MSI RTX 3050 LP 6G OC, N82E16814137879](https://www.newegg.com/msi-rtx-3050-lp-6g-oc-geforce-rtx-3050-6gb-graphics-card-double-fans/p/N82E16814137879). Price and seller observed in a live browser after the static reader was blocked.
[^a400-offer]: [B&H: PNY RTX A400, VCNRTXA400ATX-PB](https://www.bhphotovideo.com/c/product/1860620-REG/pny_vcnrtxa400atx_pb_nvidia_rtx_a400_graphics.html).
[^t600-offer]: [PCSP: used Dell T600, explicitly out of stock](https://pcserverandparts.com/dell-nvidia-quadro-t600-4gb-gddr6-graphics-card-used-4fkfp/).
[^gmk-bf]: [GMKtec official Black Friday 2026 announcement and terms](https://www.gmktec.com/blogs/buyers-guide/gmktec-black-friday-cyber-monday-2026).
[^gmk-sale]: [GMKtec live Black Friday landing page](https://www.gmktec.com/pages/black-friday).
[^gmk-gseries]: [GMKtec G-series collection and G5S fixed-memory specification](https://www.gmktec.com/collections/g-series).
[^um760-pd]: [Minisforum UM760 Slim official specifications: USB4 Alt PD 65–100W](https://store.minisforum.com/products/minisforum-um760-slim-mini-pc).
[^nano-pd]: [Lenovo M90n-1 Nano PSREF: 20V/>45W USB-C input, soldered RAM, optional WLAN](https://psref.lenovo.com/syspool/Sys/PDF/ThinkCentre/ThinkCentre_M90n_1_Nano/ThinkCentre_M90n_1_Nano_Spec.html).
[^hp-pd]: [HP Elite 800 G9 QuickSpecs, May 2026](https://h20195.www2.hp.com/v2/GetPDF.aspx/c08017769.pdf): 35W UMA restriction, 100W intake, Flex IO part 13L60AA. Retrieved with certificate verification bypassed after the reader could not validate HP's certificate.
[^mele-pd]: [MeLE Quieter 4C N150 SE Grade, 16GB/512GB, advertised PD input and separate 12V/2A power socket](https://store.mele.cn/products/mele-quieter-4c-se-grade-fanless-mini-pc-n150).
[^g2-pd]: [Gough's Tech Zone: G2 Plus teardown, tested PD negotiation, fixed-12V supply hazard](https://goughlui.com/2025/05/11/review-teardown-gmktec-nucbox-g2-plus-intel-n150-12256gb-au/). Review-era Australian price is not a current US offer.
[^nuc-no-pd]: [ASUS: NUC 14 Essential does not accept USB-C/Thunderbolt power input](https://www.asus.com/support/faq/1055242/).
[^latitude-power]: [Dell Latitude 5420 official 65W/90W USB-C power-adapter specifications](https://www.dell.com/support/manuals/en-us/latitude-5420-laptop/5420_sp14_setupspecs/power-adapter?guid=guid-aa08631d-8297-4127-97d5-a41eeafaa4fe&lang=en-us).
[^vpro]: [Intel: vPro Enterprise, AMT, out-of-band management, KVM, and complete-platform prerequisites](https://www.intel.com/content/www/us/en/business/enterprise-computers/resources/remote-management.html).
[^amt-tiers]: [Intel: AMT versus Standard Manageability / vPro Enterprise versus Essentials](https://www.intel.com/content/www/us/en/support/articles/000090499/technologies/intel-active-management-technology-intel-amt.html).
[^amt-setup]: [Intel: AMT provisioning, TLS, firmware, compatible networking, Client/Admin Control Modes](https://www.intel.com/content/www/us/en/developer/articles/guide/getting-started-with-active-management-technology.html). Older guide: use current OEM/console documentation for actual provisioning commands.
[^amt-consent]: [Intel: AMT user-consent requirement for KVM and storage redirection](https://www.intel.com/content/www/us/en/docs/active-management-technology/developer-guide/2021/user-consent.html).
[^depot-laptop]: [Discount Computer Depot Latitude 5430, SKU 96287-3](https://discountcomputerdepot.com/todays-top-deals-save-big/dell-latitude-5430-laptop-14-fhd-intel-core-i5-12th-gen-4-4ghz-10-core-16gb-ram-512gb-ssd-windows-11-pro/). $349.99 selected one-year configuration observed in browser, not inferred from the category price range.
[^latitude-deal]: [DFS Latitude 5420 Grade B 000608](https://www.dellrefurbished.com/item/dell-latitude-5420-000608/dell-latitude-5420/1.html); [5420 configuration inventory](https://www.dellrefurbished.com/laptops?model_f[]=Latitude%205420).
[^refurbio-deal]: [REFURB.io USA Latitude 5420, SKU 049500453182, inventory and pre-BF terms](https://us.refurb.io/products/copy-of-dell-latitude-5420-14-laptop-core-i5-1135g7-2-4-ghz-16-gb-512-gb-ssd-windows-10-pro-refurbished).
[^7090-deal]: [DFS 7090 Micro Grade A 000187](https://www.dellrefurbished.com/item/dell-optiplex-7090-mff-000187/dell-optiplex-7090-mff/1.html); [7090 Micro inventory](https://www.dellrefurbished.com/desktop-computers?chassis_type[]=Micro&model_f[]=OptiPlex%207090). Grade B 000188 selected and inspected in the same browser configurator.
[^lenovo-outlet]: [Lenovo US Outlet laptop page: OUTLET15 promotion](https://www.lenovo.com/us/outletus/en/laptops/).
[^lenovo-certified]: [Lenovo Certified Refurbished: warranty, battery criteria, and business contact](https://www.lenovo.com/us/en/solutions/sustainability-solutions/circularity/certified-refurbishment-devices/).
[^lenovo-returns]: [Lenovo US shopping FAQ: Outlet return window](https://www.lenovo.com/us/en/shopping-faq/).
[^dfs-terms]: [DFS limited warranty and extension prices](https://www.dellrefurbished.com/limited-warranty); [returns and restocking](https://www.dellrefurbished.com/returns).
[^dfs-buying]: [DFS FAQ: restocking, no customization, laptop adapters](https://www.dellrefurbished.com/faq); [formal bulk program: minimum 100 assets](https://www.dellrefurbished.com/bulk-discount).
[^depot-terms]: [Depot warranty, battery testing, modification restrictions](https://discountcomputerdepot.com/warranty-info/); [shipping and refunds](https://discountcomputerdepot.com/shipping-returns/).
[^depot-business]: [Discount Computer Depot business accounts](https://discountcomputerdepot.com/business-accounts).
[^refurbio-terms]: [REFURB.io USA warranty](https://us.refurb.io/pages/warranty); [refund policy](https://us.refurb.io/policies/refund-policy).
[^pcsp-terms]: [PCSP system warranty](https://pcserverandparts.com/pcsp-standard-warranty/); [90-day returns and 20% restocking conditions](https://pcserverandparts.com/return-policy/).
[^pcsp-quote]: [PCSP business, configuration service, and quote link](https://pcserverandparts.com/).
[^wisetek-deal]: [Wisetek T14 Gen 2 5650U/16GB/256GB, unavailable listing](https://www.wisetekmarket.com/products/lenovo-thinkpad-t14-gen-2-14-16gb-256gb-ssd-amd-ryzen%E2%84%A2-5-pro-5650u-2-3ghz-win11p-black-refurbished-fair).
[^wisetek-terms]: [Wisetek warranty: personal-use/reseller distinction, software and opening restrictions](https://www.wisetekmarket.com/pages/terms-conditions).
[^discount-electronics]: [Discount Electronics warranty, store-credit returns, cancellation fees](https://discountelectronics.com/warranty-returns-shipping/); [checked 7070 Micro preorder listing](https://discountelectronics.com/refurbished-dell-optiplex-7070-micro-pc-i7-16gb-256gb-ssd-windows-11-pro/).
[^f2a-mc]: [Micro Center F2A, product 711594](https://www.microcenter.com/product/711594/acemagic-f2a-mff-mini-pc).
[^f2a-spec]: [ACEMAGIC current F2A specifications, Intel X7 358H section](https://acemagic.com/products/f2a-minipc): 4P+8E+4 LP-E/16 threads, fixed 32GB LPDDR5X, two NVMe positions, dual 2.5GbE, Wi-Fi 7, 100W PD input, dimensions, and CPU power modes. Separate AMD variant exists; specifications must not be mixed. Manufacturer-direct warranty terms are not automatically Micro Center warranty terms.
[^dell-gpu-deals]: Dell Refurbished selected GPU configurations, 50% coupon verified in temporary carts: [3260 Compact Grade B 000019](https://www.dellrefurbished.com/item/dell-precision-3260-compact-mff-000019/dell-precision-3260-compact-mff/1.html), [Grade A 000018](https://www.dellrefurbished.com/item/dell-precision-3260-compact-mff-000018/dell-precision-3260-compact-mff/1.html), [3460 SFF Grade B 000015](https://www.dellrefurbished.com/item/dell-precision-3460-sff-000015/dell-precision-3460-sff/1.html), [7010 SFF Radeon Grade B 000059](https://www.dellrefurbished.com/item/dell-optiplex-7010-sff-000059/dell-optiplex-7010-sff/1.html). No purchases made; stock is the earlier observed snapshot.
[^amd-targets]: [AMD TheRock GPU-to-LLVM target mapping](https://github.com/ROCm/TheRock/blob/main/build_tools/hack/env_check/AMDGPU_LLVM_TARGET.py): RX 6500 gfx1034; R9700 and RX 9070 family gfx1201.
[^therock-support]: [TheRock GPU support status](https://github.com/ROCm/TheRock/blob/main/SUPPORTED_GPUS.md): Linux gfx1034 build-passing, sanity-tested, release-ready at the check; development status is not production-matrix qualification.
[^therock-install]: [TheRock release installation and target-specific packages](https://github.com/ROCm/TheRock/blob/main/RELEASES.md): device-gfx1034, SDK development tools, and warnings about nightly/runtime support.
[^rocm-matrix]: [AMD ROCm production compatibility matrix](https://rocm.docs.amd.com/en/latest/compatibility/compatibility-matrix.html): RX 6500 absent from the checked GPU list; verify exact OS/framework/version combinations.
[^hip-porting]: [AMD HIP porting and compilation guide](https://rocm.docs.amd.com/projects/HIP/en/latest/how-to/hip_porting_guide.html): APIs, architecture-specific compilation, multi-target code objects, and feature differences.
[^puffer-build]: [PufferLib 5.0 build script](https://github.com/PufferAI/PufferLib/blob/5.0/build.sh): native CUDA trainer, explicit architecture and FP32 --float option. Not an AMD runtime compatibility claim.
[^puffer-precision]: [PufferLib native trainer precision implementation](https://github.com/PufferAI/PufferLib/blob/5.0/src/pufferl.cu): BF16 default versus FP32 build path. Source inspected, not executed on the Dell GPUs.
[^e6430]: [Dell Latitude E6430 official specification sheet](https://i.dell.com/sites/doccontent/shared-content/data-sheets/en/Documents/Dell_Latitude_E6430_spec-sheet.pdf): third-generation Intel, up to 16GB DDR3 configurations, Intel HD 4000 / optional NVS 5200M.
[^apc-mc]: [Micro Center APC BN1500M2, product 497071](https://www.microcenter.com/product/497071/apc-back-ups-pro-ups-%28bn1500m2%29).
[^apc-spec]: [APC BN1500M2 manufacturer data sheet, distributor-hosted PDF](https://cdn.adiglobaldistribution.us/pim/Original/October-2026/2G_BN1500M2_Product_Data_Sheet.pdf): 1500VA/900W, line-interactive, stepped approximation to sine wave, six battery-backed/four surge-only outlets, USB management.
[^cyberpower-mc]: [Micro Center CP1500PFCLCD, product 353897](https://www.microcenter.com/product/353897/cyberpower-systems-pfc-sinewave-series-ups-%28cp1500pfclcd%29).
[^cyberpower-spec]: [CyberPower CP1500PFCLCD specifications](https://www.cyberpowersystems.com/product/ups/pfc-sinewave/cp1500pfclcd): 1500VA/1000W, line-interactive, sine-wave output.
[^local-kvm-mc]: [Micro Center IOGear GCS32HU, product 649908](https://www.microcenter.com/product/649908/iogear-2-port-full-hd-kvm-switch-with-hdmi-and-usb).
[^comet-mc]: [Micro Center Comet PoE GL-RM1PE, product 714359](https://www.microcenter.com/product/714359/glinet-comet-poe-powered-over-ethernet-remote-kvm-with-32-emmc-tailscale-support).
[^comet-docs]: [GL.iNet Comet PoE console guide](https://docs.gl-inet.com/kvm/en/user_guide/gl-rm1pe/console_guide/): remote input, virtual media, Wake-on-LAN, and separate ATX/Fingerbot power accessories.
[^dfs-faq-returns]: [Dell Refurbished FAQ](https://www.dellrefurbished.com/faq) and [terms of sale](https://www.dellrefurbished.com/terms): restocking/seller-error distinctions and order-acceptance limitations.
