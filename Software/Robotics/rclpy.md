# 🟣 rclpy

**rclpy** is the official Python client library for [[ROS2]] (Robot Operating System 2). It allows developers to write ROS 2 nodes, interact with topics, services, actions, and parameters, and control the ROS 2 middleware from Python code.

---

## 🧠 Summary

- Provides Python bindings for ROS 2 core functionality.
- Enables rapid development and prototyping of robotics applications.
- Built on top of the underlying ROS Client Library (RCL), which interacts with [[DDS]] (Data Distribution Service).

---

## ⚙️ Key Features

- Create and manage ROS 2 nodes in Python.
- Publish and subscribe to [[ ROS2 Topics]].
- Define and call [[ROS2 Services]].
- Use [[ROS2 Actions]] for long-running tasks.
- Work with ROS 2 parameters.
- Integrate with QoS (Quality of Service) policies.

---

## 🚀 Example Use Cases

- Prototyping and testing algorithms quickly.
- Educational projects and teaching robotics.
- Scripting for robot control, logging, or monitoring.
- Lightweight nodes for system orchestration.

---

## 🏆 Strengths

- Easy and quick to develop in Python.
- Ideal for prototyping and small applications.
- Access to the vast Python ecosystem (e.g., NumPy, OpenCV).

---

## ⚠️ Weaknesses

- Slower performance compared to [[rclcpp]] (the C++ client library).
- Higher CPU usage may be noticeable in resource-constrained systems.
- Some ROS 2 features are first implemented in rclcpp and appear in rclpy later.

---

## 📊 Comparison with Other ROS 2 Client Libraries

| Client Library | Language | Performance | Ease of Use | Notes                    |
|----------------|----------|-------------|-------------|--------------------------|
| rclpy          | Python   | Moderate     | Easy        | Great for scripting, prototyping |
| [[rclcpp]]     | C++      | High         | Moderate    | Best for production-grade, real-time systems |
| rcljava        | Java     | Moderate     | Moderate    | Less common, industrial use cases |

---

## Examples

**Basic rclpy Node**
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node has been started.')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Publisher Example**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS2 from rclpy!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main():
    rclpy.init()
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber Example**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main():
    rclpy.init()
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Server Example**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: {request.a} + {request.b} = {response.sum}')
        return response

def main():
    rclpy.init()
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 🔗 Related Notes

- [[ROS2]]
- [[ROS2 Node]]
- [[ROS2 Topics]]
- [[ROS2 Services]]
- [[ROS2 Actions]]
- [[DDS]] (Data Distribution Service)

---

## 🌐 External References

- [ROS 2 rclpy Documentation](https://docs.ros.org/en/rolling/How-To-Guides/Using-rclpy-with-ROS2.html)
- [rclpy GitHub](https://github.com/ros2/rclpy)

---
