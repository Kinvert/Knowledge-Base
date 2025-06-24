# 🟣 cv_bridge

**cv_bridge** is a ROS (ROS1 and [[ROS2]]) library that facilitates conversion between ROS image messages (such as `sensor_msgs/Image`) and OpenCV image types (`cv::Mat` in C++ or `numpy.ndarray` in Python). It allows seamless integration of ROS imaging pipelines with OpenCV processing.

---

## 🧠 Summary

- Bridges the gap between ROS image message formats and OpenCV image data structures.
- Supports multiple encodings like `bgr8`, `rgb8`, `mono8`, `16UC1`.
- Available for both C++ and Python (via `rclcpp`, `rclpy`).

---

## ⚙️ Key Features

| Feature                       | Description                                                    |
|--------------------------------|----------------------------------------------------------------|
| ROS <-> OpenCV conversion      | Convert `sensor_msgs/Image` ↔ `cv::Mat` / `numpy.ndarray`      |
| Encoding support               | Handles common formats like `bgr8`, `rgb8`, `mono8`, etc.     |
| ROS integration                | Works directly with `sensor_msgs/Image` topics                |
| Cross-language support         | Available in both C++ and Python                              |
| Supports depth & color images  | Can handle multi-channel and 16-bit depth formats             |

---

## 🚀 Example Use Cases

- Convert camera image topics for OpenCV processing (e.g., object detection).
- Visualize or modify ROS image messages in OpenCV before republishing.
- Apply computer vision algorithms directly inside ROS nodes.

---

## 🏆 Strengths

- Simplifies image handling in robotics perception stacks.
- Well-supported and widely used in ROS ecosystem.
- Compatible with tools like [[RViz]] and [[Foxglove]] when republishing processed images.

---

## ⚠️ Weaknesses

- Requires careful handling of encodings (e.g., mismatches between `rgb8` and `bgr8`).
- Performance overhead for conversion in high-frequency pipelines.

---

## Examples

**Publisher**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        # Create a dummy image (black image with size 640x480)
        cv_image = cv2.imread('your_image.jpg')  # Replace with actual capture
        if cv_image is None:
            cv_image = 255 * np.ones((480, 640, 3), dtype=np.uint8)

        img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher_.publish(img_msg)
        self.get_logger().info('Published image')

def main():
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'camera/image', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Received Image', cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

**Publisher C++**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()
    : Node("image_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ImagePublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        cv::Mat image = cv::imread("your_image.jpg");  // Replace or generate
        if (image.empty())
            image = cv::Mat::zeros(480, 640, CV_8UC3);

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Published image");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

**Subscriber C++**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
    : Node("image_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image",
            10,
            std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1)
        );
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::imshow("Received Image", image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

---

## 🔗 Related Notes

- [[sensor_msgs]]
- [[OpenCV]]
- [[ROS2]]
- [[ROS2 Topics]]
- [[rclpy]]
- [[Foxglove]]
- [[RViz]]

---

## 🌐 External References

- [ROS2 cv_bridge docs](https://docs.ros.org/en/rolling/p/cv_bridge/)
- [ROS Wiki cv_bridge (ROS1)](http://wiki.ros.org/cv_bridge)

---
