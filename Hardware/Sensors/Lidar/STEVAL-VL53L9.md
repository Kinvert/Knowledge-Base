# STEVAL-VL53L9

The **STEVAL-VL53L9** line is STMicroelectronics evaluation hardware for the **VL53L9** time-of-flight (ToF) sensor family.  
It is mainly used to evaluate multi-point ranging behavior before integrating ToF devices into a real robot.

Compared with spinning lidars, this is usually a **short-to-mid-range solid-state distance sensing path** with fixed geometry and tighter integration requirements.

---

## Core concepts

- **Time-of-Flight (ToF) ranging** measures distance from emitted/return light timing.
- **Multi-zone sampling** can provide coarse 2D maps of depth instead of a full 360° scan.
- **Bus-first architecture** typically uses I²C (or I3C on compatible designs).
- **Fixed-shape outputs** are useful for PPO-style and RL pipelines.

---

## Driver language and what that means

- The ST ToF software model for this family is based on a **C API**.
- ST ULD-style packages are written as portable C drivers with a thin platform abstraction (`platform_` functions for I²C/timing/mutex/etc.).
- In practice, you write a C/HAL glue layer for your target board and keep the sensor API clean and testable.
- That means if you are asking what language to write your low-level integration in, **it should be C** (or C-compatible wrappers).

---

## ROS2 compatibility

- There is no explicit official ROS2 package confirmed in the public docs for this line in the fetched material.
- This is normal for new/less-common ToF evaluation families.
- You can still use it in ROS2 by:
  - running the ST/ULD-style C API in a node process,
  - publishing your frame data to a ROS2 topic,
  - subscribing in your control node.

Recommended ROS2 topic plan:

- publish raw depth zone frame as a custom message or mapped `sensor_msgs` payload,
- publish confidence/invalid-rate metadata in a parallel topic,
- keep message shape fixed per node run.

That is the robust path when no official driver package exists.

---

## More similar sensors to evaluate

If you want options closer to `STEVAL-VL53L9`, use this quick ladder:

### Direct ST multizone family

- [VL53L9CX](https://www.st.com/search?query=VL53L9CX) — highest-comparison class: 2.3K zones, up to 9 m, same API shape and concept.
- [VL53L8CX](https://www.st.com/resource/en/datasheet/vl53l8cx.pdf) — 64/16 zones, lower cost and lower bandwidth footprint.
- [VL53L7CX](https://www.st.com/resource/en/datasheet/vl53l7cx.pdf) — single-board-friendly legacy multizone path.
- [VL53L5CX](https://www.st.com/resource/en/data_brief/vl53l5cx.pdf) — proven stable baseline for multi-zone ToF.

### ST alternatives if you can reduce requirements

- [VL53L4CD](https://www.st.com/search?query=VL53L4CD) and [VL53L3CX](https://www.st.com/search?query=VL53L3CX) — single / low-zone designs with lower BOM cost.
- [VL6180X](https://www.st.com/search?query=VL6180X) — classic single-point, low-cost ToF + ambient-light sensor style integration.
- [VL53L1X](https://www.st.com/search?query=VL53L1X) and [VL53L0X](https://www.st.com/search?query=VL53L0X) — older single-zone ST designs.

### Broader similar-class modules (non-ST)

- [[YDLIDAR T-mini Plus]] — compact scanning ToF with wider field coverage than a single VL53x cone.
- [[RPLIDAR C1]] — low-cost 2D rotating ToF alternative when you need sweep-style obstacle mapping.
- [[Hobbyist Lidar Units]] for quick replacement candidates if your robot can tolerate a different scan model.

### Sensor comparison snapshot

| Candidate | Resolution / map detail | Range | FoV / geometry | Interface |
|---|---|---|---|---|
| [VL53L9CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l9cx.html) | 54 × 42 = 2,268 points (2.3k zones), depth/IR/confidence outputs | **0.05 to 8.8 m** (up to **9 m** max class) | 55° × 42° (**71° diagonal**), 100 Hz | I3C / MIPI CSI |
| [VL53L8CX](https://www.st.com/resource/en/datasheet/vl53l8cx.pdf) | 8 × 8 = 64 points, optional 4 × 4 mode | up to **4.0 m** | 65° diagonal, 60 Hz | SPI up to 3 MHz, I²C up to 1 MHz |
| [VL53L7CX](https://www.st.com/resource/en/datasheet/vl53l7cx.pdf) | 8 × 8 = 64 points, optional 4 × 4 mode | up to **3.5 m** | 60° × 60° square (**90° diagonal**) | I²C |
| [VL53L5CX](https://www.st.com/resource/en/data_brief/vl53l5cx.pdf) | 8 × 8 or 4 × 4 zones | up to **4.0 m** | 65° diagonal | I²C |
| [VL53L4CD](https://www.st.com/search?query=VL53L4CD) | Single-zone, no depth grid | **1 mm to 1200 mm** | 18° diagonal, low-FOV proximity class | I²C |
| [VL53L3CX](https://www.st.com/search?query=VL53L3CX) | Single-zone, multitarget depth | up to **3.0 m** (some catalog entries show up to **5 m**) | no fixed FoV map | I²C |
| [VL6180X](https://www.st.com/search?query=VL6180X) | Single-point + ambient-light sensor (no zone map) | **0 to ~100 mm** typical (longer possible under good targets) | narrow, single-point cone | I²C |
| [VL53L1X](https://www.st.com/search?query=VL53L1X) | Single-point with ROI/multizone host control options | up to **4.0 m** | 27° typical FoV | I²C |
| [VL53L0X](https://www.st.com/search?query=VL53L0X) | Single-point | up to **2.0 m** | narrow FoV class | I²C |
| [[YDLIDAR T-mini Plus]] | 2D rotating scan, ~0.54° angular step | 0.05 to 12 m | 360° sweep | UART, 4000 Hz ranging / 6-12 Hz scan |
| [[RPLIDAR C1]] | 2D rotating scan | around **12 m** class | 360° sweep | UART |

Choose order:
1. stay closest in behavior: `VL53L9CX`.
2. keep multi-zone but simplify integration: `VL53L8CX`/`VL53L7CX`/`VL53L5CX`.
3. move to simpler single-zone designs: `VL53L4CD`/`VL53L3CX`/`VL53L1X`.
4. switch to 2D scanning behavior: `YDLIDAR T-mini Plus` or `RPLIDAR C1`.

---

## How to get raw sensor data into a C robot

### 1) Physical integration checklist

- set `3V3` and ground stable and clean,
- wire I²C/I3C pins and optional `XSHUT`, `INTR`, `SYNC_IN`,
- confirm bus speed and pull-up levels,
- verify AP clock if required by your board variant.

### 2) Software read flow

- init bus + GPIO + optional interrupts,
- init sensor and apply ranging config,
- start ranging,
- wait for ready/data-valid,
- read distance frame + status/confidence if available,
- normalize / validate before feeding policy.

```c
typedef struct {
    uint8_t zone_cols;
    uint8_t zone_rows;
    uint16_t status;
    uint16_t range_mm[54 * 42];
    uint8_t conf[54 * 42];
} vl53l9_raw_frame_t;
```

### 3) Minimal C loop (conceptual)

```text
driver_init();
while (running) {
    if (!sensor_ready()) continue;
    frame = read_frame();
    if (!valid(frame)) {
        fallback_or_hold();
        continue;
    }
    push_to_control_loop(frame);
}
```

### 4) What to do in the control stack

- Keep a fixed-size vector every step.
- Include an `invalid_ratio` channel instead of directly trusting empty values.
- gate controls when confidence drops.
- start with conservative behavior (reduce speed) on repeated invalid frames.

---

## Custom ROS2 wrapper node (simple and clear)

Treat your sensor driver as a plain C library, then add a thin C++ ROS2 node that publishes frames.

1. Build the C driver as a shared library (`libvl53l9.a` or `.so`).
2. Keep a fixed message size so your consumers never reallocate.
3. Publish raw distances + confidence as arrays.

```cpp
// src/tof_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
extern "C" int vl53l9_read_frame(vl53l9_raw_frame_t* out);  // C API

class Vl53L9Node : public rclcpp::Node {
public:
  Vl53L9Node() : Node("vl53l9_node") {
    pub_frame_ = create_publisher<std_msgs::msg::UInt16MultiArray>("vl53l9/range_mm", 10);
    pub_invalid_ = create_publisher<std_msgs::msg::Float32>("vl53l9/invalid_ratio", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() { tick(); });
  }
  void tick() {
    vl53l9_raw_frame_t f{};
    if (vl53l9_read_frame(&f) != 0) return;
    std_msgs::msg::UInt16MultiArray msg;
    msg.data.assign(std::begin(f.range_mm), std::end(f.range_mm)); // fixed size
    pub_frame_->publish(msg);
    float invalid = 0.0f; /* compute from status/confidence or sentinels */
    std_msgs::msg::Float32 invalid_msg;
    invalid_msg.data = invalid;
    pub_invalid_->publish(invalid_msg);
  }
private:
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_frame_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_invalid_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

```bash
# CMakeLists.txt idea
add_library(vl53l9_wrapper SHARED src/vl53l9_wrapper.c)
ament_target_dependencies(vl53l9_node rclcpp std_msgs)
target_link_libraries(vl53l9_node vl53l9_wrapper)
```

Consumer node (or controller) subscribes to:
- `vl53l9/range_mm` (`UInt16MultiArray`)
- `vl53l9/invalid_ratio` (`Float32`)

This is the minimum viable ROS2 path when no official vendor node exists.

---

## Comparison chart

| Candidate | Sensing model | Range shape | Interface | Best for |
|---|---|---|---|---|
| **[[STEVAL-VL53L9]]** | multizone ToF | cone/zone map | I²C/I3C | compact obstacle and edge sensing |
| [[YDLIDAR T-mini Plus]] | spinning TOF-style 2D | 360° sweep | UART | full obstacle ring on small robot |
| [[LDROBOT LD06]] | 2D lidar | 360° sweep | UART | compact navigation |
| [[LDROBOT STL-06P]] | 2D lidar | 360° sweep | UART | very small platforms |
| [[Mini Pupper STL-06P Lidar Module]] | 2D lidar | 360° sweep | UART | mini-robot ecosystems |
| [[RPLIDAR A1M8]] | 2D lidar | 360° sweep | USB/UART | mature community ROS stack |
| [[RPLIDAR A2M6]] | 2D lidar | 360° sweep | UART | denser scan rates |

If your use case is forward obstacle confidence, this is often cleaner than spinning lidar.  
If you need full surround coverage, spinning lidar is usually easier.

---

## Common mistakes

- fixed shape changes mid-run,
- feeding invalid ranges as real distances,
- mixing bus timing between modules on shared rails,
- assuming 360° coverage from a ToF zone sensor.

---

## Related notes

- [[Lidar]]
- [[Line Lidar for Balancing Robots]]
- [[Hobbyist Lidar Units]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Extended Kalman Filter]]
- [[Linear Quadratic Regulator]]

---

## External resources

- ST product and development portal: https://www.st.com/search?query=VL53L9
- STEVAL-VL53L9 data brief: https://www.st.com/resource/en/data_brief/steval-vl53l9.pdf
- Digi-Key STEVAL-VL53L9: https://www.digikey.com/en/products/detail/stmicroelectronics/STEVAL-VL53L9/29294599
- VL53L9CX datasheet: https://www.st.com/resource/en/datasheet/vl53l9cx.pdf
- STEVAL-VL53L9 product page: https://www.st.com/en/evaluation-tools/steval-vl53l9.html
