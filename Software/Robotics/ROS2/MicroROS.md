---
title: Micro-ROS
aliases: [MicroROS, micro-ros]
tags: [robotics, ros2, mcu, embedded, microcontrollers, communication]
---

# Micro-ROS

Micro-ROS is the ROS 2 stack for microcontrollers and deeply constrained embedded devices. It exposes ROS 2 node concepts on MCUs through the C client API (`rcl` + `rclc`) and a transport path based on Micro XRCE-DDS.

---

## Overview

| Field | Details |
|---|---|
| Goal | Run ROS 2-compatible nodes on resource-constrained microcontrollers |
| Client API | `rcl` + `rclc` (C-oriented API for MCUs) |
| Bridge component | `micro_ros_agent` on a host machine |
| Transport defaults | Serial/UART, USB-CDC, UDP |
| Standard ROS 2 concepts | Nodes, publishers, subscriptions, services, parameters, QoS |
| Typical RTOS targets | FreeRTOS, Zephyr, NuttX (plus dedicated board support) |
| Official sources | [micro-ROS Features and Architecture](https://micro.ros.org/docs/overview/features/), [Supported Hardware](https://micro.ros.org/docs/overview/hardware/), [Supported RTOSes](https://micro.ros.org/docs/overview/rtos/) |

---

## Architecture

Micro-ROS follows the ROS 2 layered architecture, but replaces the normal DDS middleware path with a constrained XRCE profile:

- MCU-side client is a micro-ROS RMW implementation backed by micro-ROS client libraries.
- The client does not directly join a DDS domain. It talks to an XRCE agent.
- The agent bridges MCU entities into normal ROS 2 graph tooling (`ros2 topic`, `ros2 node`, etc.).
- Micro XRCE-DDS is explicitly described as a client-server model where agents store entities and mediate operations; this is what lets tiny nodes appear as normal ROS 2 entities without full DDS state on-device.

Data path (logical):

`MCU micro-ROS node <-Serial/USB/UDP-> micro-ROS Agent <-> DDS / ROS 2 host graph`

---

## Why use micro-ROS on MCUs

A full ROS 2 runtime is not shaped for tiny RAM/flash footprints and hard-constrained scheduling profiles. Micro-ROS specifically trims this for MCU use:

- reduced memory dynamic behavior (docs explicitly call out low memory and static allocation patterns in client use),
- explicit transport setup rather than broad host assumptions,
- first-class support for MCU + RTOS targets with POSIX-like behavior,
- and host-agent bridge semantics so you get ROS 2 interoperability without full middleware on-device.

It is not the same as running full desktop ROS 2 on the MCU; it is the MCU-adapted ROS 2 path.

---

## Supported hardware and RTOS stance

micro-ROS publishes official and community hardware categories. Official support is the only path with guarantee language; others are community-supported.

- **Official hardware:** curated list in micro-ROS docs (including ESP32, Portenta H7, RP2040/Pico, several STM32 variants, and others).
- **Official RTOSes:** FreeRTOS, Zephyr, NuttX.
- **Bare-metal/experimental:** available through Arduino-first and platform-specific pathways, but often treated as experimental or less-strongly guaranteed.

Implication for **Parallax Propeller**: it is not a listed official item in the published micro-ROS hardware matrix. It may still be connected as part of a broader stack (as a peripheral under a host), but that becomes a custom/informal path and usually requires substantial transport and build integration work.

---

## Build and agent setup (practical)

Common workflow on ROS 2 hosts:

- create firmware workspace for RTOS/platform in `micro_ros_setup`,
- configure transport and static memory/execution options,
- build firmware image,
- run an agent on the host for transport back to ROS 2.

Common commands:

```bash
# Host-run agent via ROS package
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Container variant (also commonly documented):

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -v6
```

`micro_ros_agent` is the standard bridge process; some environments expose the agent as a container entrypoint instead of the ROS 2 command path.

---

## Language/runtime nuances (Python, C, C++)

For MCUs, micro-ROS is typically C-centric:

- Client APIs/firmware patterns are designed around `rclc` and related C headers.
- Host-side orchestration still uses standard ROS 2 tooling (`rclcpp`/`rclpy` ecosystem).
- Python is practical and normal on host/Edge nodes, but it is not the normal MCU firmware path.

Conceptual C publisher skeleton:

```c
rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t pub;

rclc_support_init(&support, 0, NULL, &allocator);
rclc_node_init_default(&node, "mcu_node", "", &support);
rclc_publisher_init_default(
    &pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "example_topic");
```

---

## Where to use micro-ROS

- Distributed low-level nodes (motor, battery, sensors, safety IO).
- Deterministic embedded I/O that must participate in robot-level ROS 2 flows.
- Hybrid SBC+MCU architectures (Jetson / Raspberry Pi / x86 host + MCU edge board).

It is a strong fit when your architecture already assumes host-node orchestration and MCU nodes mainly provide interfaces/control loops.

---

## Comparison

| Approach | Target class | API model | Bridge requirement | Typical transports | Notes |
|---|---|---|---|---|---|
| micro-ROS | MCU/embedded | C (`rclc` + `rcl`) | `micro_ros_agent` bridge to ROS 2 | UART/USB/UDP | Native ROS 2 interop from tiny nodes |
| Full ROS 2 (Linux host) | SBC/desktop | C++ / Python | native DDS graph | IP / DDS | Richer tooling, higher overhead |
| ros1 `rosserial` | MCU + ROS 1 graph | custom (Arduino/vendor APIs) | rosbridge-like node on host | Serial | Legacy pattern, ROS 2 migration gap |
| Plain UART custom protocol | Any MCU | custom protocol stack | custom software | UART/CAN/etc | Lowest overhead, lowest interop |
| MQTT telemetry | MCU + cloud/edge | app-specific | broker | TCP/UDP | Not ROS graph-native |
| Custom XRCE client | MCU | custom XRCE integration | custom bridge | custom transport | Possible, but no micro-ROS ecosystem |

---

## Strengths / Weaknesses

### Strengths

- Keeps MCU side small while preserving ROS 2 semantics.
- Clear split between constrained edge and host compute stack.
- Transport options include serial/USB/UDP and room for custom transport implementations.

### Weaknesses

- Not all MCU families are officially supported.
- Runtime/build setup is more toolchain-heavy than ultra-minimal bare-metal code.
- Requires a host process and transport endpoint management.
- You should validate toolchain/RTOS version matrices per micro-ROS release.

---

## Common pitfalls

- **“Can I run full ROS 2 desktop code on my MCU?”**  
No. Use micro-ROS for MCU compatibility and reduced middleware expectations.
- **“Can I treat ROS 2 like a standard serial protocol library?”**  
It remains a distributed graph model. transport is required, but behavior and QoS are not just message framing.
- **“Will a not-official MCU work automatically?”**  
Not automatically. If absent from the official matrix, expect custom integration and lower guarantees.

---

## Hello-world / starter tutorials

- **Official first-project sequence**
  - https://micro.ros.org/docs/tutorials/core/overview/
  - https://micro.ros.org/docs/tutorials/core/first_application_linux/
  - https://micro.ros.org/docs/tutorials/core/first_application_rtos/
  - https://micro.ros.org/docs/tutorials/core/zephyr_emulator/
  - https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/
- **Related micro-ROS references**
  - https://micro.ros.org/docs/overview/features/
  - https://micro.ros.org/docs/overview/hardware/
  - https://micro.ros.org/docs/overview/rtos/
- **Source and build docs**
  - https://github.com/micro-ROS/micro_ros_setup
  - https://github.com/micro-ROS/micro_ros_platformio
  - https://github.com/micro-ros/micro_ros_agent

## Related Notes

- [[ROS2]]
- [[DDS]]
- [[ROS2 Node]]
- [[ROS2 Topics]]
- [[RTOS]]

## References

- https://micro.ros.org/docs/overview/features/
- https://micro.ros.org/docs/overview/hardware/
- https://micro.ros.org/docs/overview/rtos/
- https://micro-xrce-dds.docs.eprosima.com/en/latest/
- https://micro.ros.org/docs/overview/
- https://github.com/micro-ROS/micro_ros_setup
- https://github.com/micro-ROS/micro_ros_platformio
- https://github.com/micro-ROS/micro_ros_agent
