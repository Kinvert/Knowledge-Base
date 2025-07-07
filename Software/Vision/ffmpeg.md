# 🟣 FFmpeg

**FFmpeg** is a powerful open-source multimedia framework used to decode, encode, transcode, mux, demux, stream, filter, and play nearly all types of audio and video formats. It is widely used in video processing, robotics (e.g., for encoding camera streams), machine learning datasets, and media pipelines.

---

## 🧠 Summary

- Command-line utility and underlying libraries for media handling.
- Supports most known video/audio formats.
- Provides capabilities such as video editing, format conversion, streaming, frame extraction, and real-time encoding/decoding.

---

## 🧰 Common Tools and Libraries

| Component         | Purpose                                                        |
|-------------------|----------------------------------------------------------------|
| `ffmpeg`          | Command-line tool for conversion and processing                |
| `ffprobe`         | Tool for media metadata inspection                             |
| `libavcodec`      | Codec library (used for encoding/decoding)                     |
| `libavformat`     | Demuxing/muxing library                                        |
| `libavutil`       | Utility library for common functions                           |
| `libswscale`      | Image scaling and pixel format conversion                      |
| `libswresample`   | Audio resampling, format conversion                            |
| `libavfilter`     | Filtering capabilities (e.g. blur, overlay, crop, etc.)        |

---

## 💻 Example Use Cases

- Extract frames from video for labeling:
  `ffmpeg -i input.mp4 -vf fps=1 frame_%04d.png`

- Convert `.mov` to `.mp4`:
  `ffmpeg -i input.mov output.mp4`

- Stream camera feed:
  `ffmpeg -f v4l2 -i /dev/video0 http://localhost:8090/feed.ffm`

- Combine audio and video:
  `ffmpeg -i video.mp4 -i audio.wav -c:v copy -c:a aac output.mp4`

---

## ✅ Pros

- Extremely versatile and powerful.
- Fast and optimized, supports hardware acceleration.
- Widely adopted and well-documented.
- Available for Linux, macOS, and Windows.

---

## ⚠️ Cons

- Steep learning curve for beginners due to broad functionality.
- Default configurations may not always yield best quality.
- Legal/licensing issues with some codecs in commercial applications.

---

## 🔗 Related Tools

- [[OpenCV]] – Uses FFmpeg under the hood for video I/O.
- [[cv_bridge]] – Can interact with FFmpeg via OpenCV.
- [[sensor_msgs/Image]] – Image streams can be encoded using FFmpeg.
- [[Image Compression]]
- [[Video Streaming]]
- [[GStreamer]] – Another multimedia processing framework often compared to FFmpeg.

---

## 🧪 Alternatives

| Tool         | Strengths                                  | Use Case                            |
|--------------|---------------------------------------------|--------------------------------------|
| GStreamer    | Pipeline-based architecture, modular        | Complex multimedia applications      |
| VLC          | GUI and CLI support, built on FFmpeg        | Media playback and streaming         |
| avconv       | Fork of FFmpeg (deprecated, older distros)  | Legacy systems                       |

---

## 🌐 External References

- [FFmpeg Official Site](https://ffmpeg.org/)
- [FFmpeg Documentation](https://ffmpeg.org/documentation.html)
- [FFmpeg Wiki](https://trac.ffmpeg.org/wiki)

---
