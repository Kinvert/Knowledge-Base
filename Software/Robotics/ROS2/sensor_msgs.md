# 🟣 sensor_msgs

**sensor_msgs** is a standard [[ROS2]] (and ROS 1) message package that defines commonly used message types for representing sensor data. These messages provide a consistent structure for exchanging information from sensors like cameras, LiDARs, IMUs, and more between ROS nodes.

---

## 🧠 Summary

- Contains message definitions for a wide range of sensors.
- Designed to promote interoperability between nodes and systems.
- Used extensively in robotics applications involving perception, navigation, and mapping.

---

## ⚙️ Common Message Types

| Message Type              | Description                                              |
|---------------------------|----------------------------------------------------------|
| `Image`                   | 2D image data (e.g. from a camera)                       |
| `CompressedImage`          | JPEG/PNG-compressed image data                           |
| `CameraInfo`               | Camera calibration and intrinsic parameters              |
| `LaserScan`                | 2D LiDAR or range scanner data                          |
| `PointCloud` / `PointCloud2`| 3D point cloud data (e.g. from LiDAR or depth cameras) |
| `Imu`                      | Inertial Measurement Unit data (accel, gyro, orientation)|
| `NavSatFix`                | GPS position data                                       |
| `Range`                    | Simple range data (e.g. ultrasonic sensor)              |
| `FluidPressure`            | Barometric pressure readings                            |
| `MagneticField`            | Magnetometer data                                       |
| `JointState`               | Joint position, velocity, and effort (often in `sensor_msgs` though conceptually fits elsewhere) |

---

## 🚀 Example Use Cases

- Publishing camera feeds using `sensor_msgs/Image` or `sensor_msgs/CompressedImage`.
- Exchanging LiDAR scans via `LaserScan` or `PointCloud2`.
- Sharing IMU data between perception and control nodes.
- Broadcasting GPS location using `NavSatFix`.

---

## 🏆 Strengths

- Standardized message formats simplify integration between components.
- Wide tool support (e.g. RViz, [[Foxglove]], ROS bag recording).
- Well-documented and mature.

---

## ⚠️ Weaknesses

- Some messages (e.g. `PointCloud`) are older and superseded by more efficient alternatives (e.g. `PointCloud2`).
- Can require careful configuration (e.g. coordinate frames, timestamps) to ensure correct interpretation.

---

## Examples

**Publish Image**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        # Create a dummy image (black image with OpenCV)
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher_.publish(ros_img)
        self.get_logger().info('Published dummy image.')

def main():
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscribe to Image**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info(f'Received image of size {cv_image.shape}')

def main():
    rclpy.init()
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Publish IMU**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Imu()
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = math.radians(45.0)  # Example: 45 deg/sec rotation
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # Simulate gravity
        self.publisher_.publish(msg)
        self.get_logger().info('Published dummy IMU data.')

def main():
    rclpy.init()
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscribe to IMU**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Orientation: [{msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}]'
        )

def main():
    rclpy.init()
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔗 Related Notes

- [[ROS2]]
- [[ROS2 Topics]]
- [[PCL]]
- [[OpenCV]] (often works alongside `sensor_msgs/Image`)
- [[Foxglove]]
- [[RViz]]

---

## 🌐 External References

- [ROS 2 sensor_msgs API](https://docs.ros2.org/latest/api/sensor_msgs/index.html)
- [ROS Wiki sensor_msgs (ROS 1, still relevant)](http://wiki.ros.org/sensor_msgs)

---
