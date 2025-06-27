# 🟣 v4l2-ctl

**`v4l2-ctl`** is a command-line utility that interacts with the **Video4Linux2 (V4L2)** API, allowing users to control and query video devices such as webcams, capture cards, and other V4L2-compatible video inputs. It is part of the `v4l-utils` package.

---

## 🧠 Summary

- Used to configure and debug video devices.
- Supports querying device capabilities, listing supported formats, changing resolution, and adjusting controls like brightness or gain.
- Works with most Linux systems with V4L2-enabled drivers.
- Commonly used with webcams, CSI/USB cameras, and the Raspberry Pi Camera Module.

---

## ⚙️ Common Use Cases

| Use Case | Example Command |
|----------|------------------|
| List all video devices | `v4l2-ctl --list-devices` |
| Show all device capabilities | `v4l2-ctl -d /dev/video0 --all` |
| List all supported formats | `v4l2-ctl -d /dev/video0 --list-formats-ext` |
| Set resolution and pixel format | `v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=YUYV` |
| Capture a single frame to a file | `v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=1 --stream-to=frame.raw` |
| List all adjustable controls | `v4l2-ctl -d /dev/video0 --list-ctrls` |
| Adjust brightness (example value 100) | `v4l2-ctl -d /dev/video0 --set-ctrl=brightness=100` |
| Query current value of gain | `v4l2-ctl -d /dev/video0 --get-ctrl=gain` |

---

## 🏆 Strengths

- Lightweight and fast.
- Excellent for debugging and camera configuration.
- Scriptable for automation or integration with capture pipelines.

---

## ⚠️ Weaknesses

- Interface can be cryptic for beginners.
- Output and accepted values vary depending on the specific camera driver.
- Requires low-level understanding for advanced use.

---

## 🧪 Helpful Tips

- You can use `--list-formats-ext` to see available resolutions and frame intervals.
- Some devices don't expose all controls unless using specific drivers.
- Always double-check the `/dev/videoX` number if you have multiple devices connected.

---

## 🔗 Related Notes

- [[Raspberry Pi Camera]]
- [[OpenCV]]
- [[GStreamer]]
- [[Video Capture Pipelines]]
- [[CSI Interface]]
- [[USB Cameras]]

---

## 🌐 External References

- [v4l-utils GitHub](https://github.com/linuxtv-org/v4l-utils)
- [LinuxTV wiki](https://linuxtv.org)
- `man v4l2-ctl`

---
