# 🟣 libjpeg-dev

`libjpeg-dev` is the development package for the **libjpeg** library — a widely used C library for handling JPEG (Joint Photographic Experts Group) image compression and decompression. This package provides the headers and symbolic links required to compile programs using libjpeg.

---

## 🧠 Summary

- Enables development of software that reads/writes JPEG images.
- Provides C API and headers for JPEG manipulation.
- Frequently used in image processing pipelines, computer vision, and robotics (e.g., [[OpenCV]], [[ROS2]], embedded vision systems).

---

## 📦 Contents of `libjpeg-dev`

- Header files (`jpeglib.h`, `jerror.h`, etc.)
- Static libraries (e.g. `libjpeg.a`)
- Symlinks to shared libraries (`libjpeg.so`)
- Documentation for API usage (often in `/usr/share/doc`)

---

## ⚙️ Typical Usage

Install via apt on Debian/Ubuntu:

`sudo apt install libjpeg-dev`

Used by:

- [[OpenCV]]: for loading/saving `.jpg` images.
- [[cv_bridge]]: to convert between ROS image formats and OpenCV.
- Image compression tools, media frameworks.
- Embedded systems using camera input (e.g. JPEG-encoded feeds).

---

## 🧩 Related Packages

| Package         | Description                                               |
|-----------------|-----------------------------------------------------------|
| `libjpeg-turbo` | A faster JPEG codec implementation using SIMD optimizations |
| `libjpeg8-dev`  | Alternative version (specific version bindings)           |
| `libjpeg62-dev` | Legacy support for older applications                     |

---

## ✅ Pros

- Stable, well-supported and widely used.
- Works across many platforms.
- Required by many upstream dependencies.

---

## ⚠️ Caveats

- Can conflict with other JPEG libraries (e.g., `libjpeg-turbo-dev`) if not managed carefully.
- Not a high-level API — low-level usage may require understanding of JPEG internals.

---

## 🔗 Related Notes

- [[OpenCV]]
- [[cv_bridge]]
- [[sensor_msgs]]
- [[Image Compression]]
- [[libjpeg-turbo]]
- [[Image Transport]]

---

## 🌐 External References

- [libjpeg Manual](http://libjpeg.sourceforge.net/)
- [Ubuntu libjpeg-dev package info](https://packages.ubuntu.com/libjpeg-dev)

---
