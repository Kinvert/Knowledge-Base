# FFprobe

`ffprobe` is the FFmpeg companion CLI for inspecting media files and streams without re-encoding. It is for metadata/stream introspection, while `ffmpeg` does filtering/transcoding.

---

## Overview

- Fast way to inspect container/containerization details, stream codecs, durations, bitrates, timestamps, and frame data.
- Supports output in compact text, CSV, JSON, XML, and flat formats.
- Commonly used for dataset QC, validation pipelines, and stream debugging.

---

## Common Usage

- Show full file metadata:
  `ffprobe -v error -show_format -show_streams input.mp4`
- Print JSON for scripts:
  `ffprobe -v error -print_format json -show_format -show_streams input.mp4`
- List duration/codec only:
  `ffprobe -v error -select_streams v:0 -show_entries stream=codec_name,codec_type,width,height,duration -of json input.mp4`
- Count packets:
  `ffprobe -v error -show_packets -select_streams a:0 -of csv=input.mp4`
- Check first stream errors/format:
  `ffprobe -v error -show_error -show_format input.mp4`

`-v error` is usually used to keep output clean and non-noisy.

---

## Useful Flags

| Flag | Use |
| --- | --- |
| `-show_format` | Container-level metadata (duration, bitrate, tags, size) |
| `-show_streams` | Per-stream metadata (codec, fps, sample rate, channels) |
| `-show_frames` | Per-frame info (useful for debugging timing/framesize issues) |
| `-select_streams` | Limit inspection to specific streams (e.g., `v:0`, `a:0`) |
| `-of json` | Output format for scripts/automation |
| `-v error` | Suppress noisy warnings and errors only |
| `-print_format` | Output formatter (`json`, `xml`, `default`, `flat`, etc.) |

---

## Typical Automation Patterns

- Validate batches:
  `for f in *.mp4; do ffprobe -v error -show_format -show_streams "$f" > /tmp/$f.json; done`
- Quick sanity check before training/processing:
  `ffprobe -v error -show_streams -select_streams v:0 -print_format json input.mp4`
- Detect stream corruption from CI/CD pipelines:
  `ffprobe -v error -show_error input.mp4` (non-zero exit if stream reports errors)

---

## Related

- [[FFmpeg]]
- [[GStreamer]]
- [[Image Compression]]

---

## External References

- [FFmpeg ffprobe Documentation](https://ffmpeg.org/ffprobe.html)
- [FFmpeg Docs](https://ffmpeg.org/documentation.html)
