# 🟣 sensor_msgs/Image

The `sensor_msgs/Image` message is a core message type in [[ROS2]] (and ROS 1) used to represent raw 2D image data from cameras or image sensors. It is part of the [[sensor_msgs]] package and serves as the primary structure for transmitting image frames between nodes in robotics, computer vision, and autonomous systems.

---

## 🧠 Summary

- Represents uncompressed image data.
- Includes full metadata: resolution, encoding, step size, timestamp, frame ID.
- Commonly used with camera drivers, computer vision tools, and image transport pipelines.

---

## ⚙️ Message Fields

| Field        | Type     | Description |
|--------------|----------|-------------|
| `header`     | `std_msgs/Header` | Includes timestamp and frame ID |
| `height`     | `uint32` | Image height (rows) |
| `width`      | `uint32` | Image width (columns) |
| `encoding`   | `string` | Encoding format (e.g. `rgb8`, `mono8`, `bgr8`) |
| `is_bigendian` | `uint8` | Endianness of data (usually 0) |
| `step`       | `uint32` | Full row length in bytes |
| `data`       | `uint8[]`| Actual pixel data, row-major order |

---

## 📸 Common Encodings

- `mono8` — grayscale, 8-bit
- `rgb8` — RGB, 8-bit
- `bgr8` — BGR, 8-bit (common for OpenCV)
- `mono16`, `rgba8`, `yuv422`, etc.

See `sensor_msgs/image_encodings.hpp` or `image_transport` docs for full list.

---

## 🧪 Use Cases

- Publishing live camera streams from CSI or USB webcams.
- Bridging images to/from [[OpenCV]] or deep learning models.
- Logging visual data for post-processing or [[Visual Odometry]].
- Input for [[sensor fusion]], [[SLAM]], or navigation stacks.
- Converting to other formats (e.g., [[sensor_msgs/CompressedImage]]).

---

## 🔄 Interfacing With

- [[cv_bridge]] — Converts between `sensor_msgs/Image` and `cv::Mat`.
- [[image_transport]] — Handles compressed transport (`compressed`, `theora`, etc.)
- [[rclpy]] / `rclcpp` — Standard ROS 2 APIs to publish or subscribe.
- [[rosbag2]] — Record and playback image data.
- [[Foxglove]] and [[RViz]] — Visualization.

---

## 🏆 Strengths

- Widely supported in the ROS ecosystem.
- Full metadata with timestamps and frame reference.
- Interoperable with vision tools and robotics frameworks.

---

## ⚠️ Weaknesses

- Uncompressed — large data sizes.
- Encoding mismatches can cause display or processing issues.
- Requires careful synchronization with other sensor types.

---

## 🔗 Related Notes

- [[sensor_msgs]]
- [[sensor_msgs-CompressedImage]]
- [[cv_bridge]]
- [[image_transport]]
- [[Foxglove]]
- [[RViz]]
- [[OpenCV]]
- [[rclpy]]

---

## 🌐 External References

- [ROS 2 `sensor_msgs/Image` Docs](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)
- [ROS Wiki (ROS 1)](http://wiki.ros.org/sensor_msgs/Image)
- [image_encodings.hpp](https://github.com/ros-perception/image_common/blob/ros2/image_transport/include/image_transport/image_transport.hpp)

---
