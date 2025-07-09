# IDL

IDL (Interface Definition Language) is a formal language used to define data structures and communication interfaces in a platform- and language-independent manner. In ROS2, IDL plays a central role as it underpins the definition of `.msg`, `.srv`, and `.action` files, ensuring compatibility with the DDS (Data Distribution Service) middleware.

---

## 📚 Overview

IDL was originally developed as part of the CORBA standard and has since become widely adopted in middleware systems like DDS. It defines structured data types and interfaces that can be mapped to multiple programming languages. ROS2 uses a subset of OMG IDL 4.2 for defining its interface types behind the scenes, enabling real-time, type-safe, distributed communication.

---

## 🧠 Core Concepts

- **IDL Files**: Text files using `.idl` extension that describe structured data.
- **Primitive Types**: `int32`, `float64`, `boolean`, `string`, etc.
- **Composite Types**: Structs, sequences (arrays), nested types.
- **Language Bindings**: IDL gets compiled to C++, Python, and others via code generators.
- **DDS Compatibility**: IDL is used to generate DDS-compatible types for transport.

---

## 🧰 Use Cases

- Defining reusable data types for communication
- Describing ROS2 interfaces in a DDS-compliant way
- Generating client stubs and server skeletons in middleware systems
- Enabling introspection and cross-platform communication

---

## ✅ Pros

- Language-agnostic and platform-independent
- Strongly typed definitions enable safe communication
- Supported by many real-time systems and middleware
- Enables tooling for introspection and message serialization

---

## ❌ Cons

- Limited expressiveness compared to some modern languages
- Syntax differs from typical programming languages
- Requires additional tooling to compile and integrate

---

## 📊 Comparison Chart

| Feature              | IDL                    | ROS2 `.msg`             | Protobuf               | JSON Schema              |
|----------------------|------------------------|--------------------------|------------------------|--------------------------|
| **Purpose**          | Middleware communication | ROS2 topics/services     | Data serialization     | Web APIs, config         |
| **Type-Safe**        | ✅ Yes                 | ✅ Yes                  | ✅ Yes                 | ⚠️ Limited              |
| **Binary Format**    | ✅ Yes (via DDS)        | ✅ Yes (via DDS)         | ✅ Yes                 | ❌ No (text-based)       |
| **Language Binding** | ✅ Many supported       | ✅ C++, Python, more     | ✅ Many supported       | ✅ JS, Python, etc.       |
| **ROS2 Use**         | ✅ Native DDS type base | ✅ Frontend interface    | ❌ Not used natively    | ❌ Not used              |

---

## 🔧 Compatible Items

- `.idl` files in ROS2 interface packages (auto-generated from `.msg`, `.srv`, `.action`)
- `rosidl_adapter`, `rosidl_generator_dds_idl`
- [[ROS2 Interface Definition]] (Built on top of IDL)
- [[DDS]] (Uses IDL as its type system)
- [[ROS2 Messages]], [[ROS2 Services]], [[ROS2 Actions]]

---

## 🔗 Related Concepts

- [[ROS2 Interface Definition]] (ROS2 front-end for defining IDL types)
- [[DDS]] (IDL is the native type system for DDS)
- [[ROS2 Messages]] (Compiled to IDL-compatible representations)
- [[Serialization]] (IDL definitions enable binary serialization)
- [[ROS2 QoS Policies]] (Apply to IDL-defined data types)

---

## 🛠 Developer Tools

- `rosidl_adapter`: Converts `.msg` to `.idl`
- DDS tools (e.g., RTI Connext IDL parsers)
- IDL compilers and code generators (e.g., `omgidl`, `fastddsgen`)
- Use `ros2 interface show` to inspect types in ROS2

---

## 📚 Further Reading

- [OMG IDL 4.2 Specification](https://www.omg.org/spec/IDL/)
- [ROS2 Interface Definition and IDL](https://design.ros2.org/articles/interface_definition.html)
- [DDS and RTPS Design](https://design.ros2.org/articles/ros_on_dds.html)

---
