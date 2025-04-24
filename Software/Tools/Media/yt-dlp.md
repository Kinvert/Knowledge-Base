---
title: yt-dlp
tags: [software, tools, video-downloading, media, open-source]
aliases: [youtube-dlp, youtube-dl fork, yt-dlp commands]
---

# 📥 yt-dlp: Advanced Video Downloader

## 🧭 Overview

**yt-dlp** is a powerful, open-source command-line tool for downloading videos and audio from a wide variety of websites. It is a fork of the popular **youtube-dl** project, with additional features, bug fixes, and performance improvements. It supports downloading from hundreds of websites, including YouTube, Vimeo, Twitch, and many more.

yt-dlp is widely used for personal media archiving, offline viewing, and extracting audio from video files.

---

## 🛠️ Key Features

1. **Wide Website Support**:
   - Supports hundreds of video and audio platforms.
   - Includes support for YouTube playlists, channels, and live streams.

2. **Advanced Format Selection**:
   - Allows users to choose specific video and audio formats.
   - Supports merging video and audio streams into a single file.

3. **Subtitle and Metadata Extraction**:
   - Downloads subtitles in various formats (e.g., SRT, VTT).
   - Extracts metadata like titles, descriptions, and thumbnails.

4. **Post-Processing**:
   - Integrates with tools like FFmpeg for format conversion and merging.
   - Supports embedding metadata and thumbnails into media files.

5. **Authentication Support**:
   - Handles login credentials for downloading private or restricted content.

6. **Customizable**:
   - Offers extensive configuration options via command-line arguments or configuration files.

7. **Performance Improvements**:
   - Faster downloads and better handling of rate limits compared to youtube-dl.

---

## 📦 Common Use Cases

1. **Downloading Videos**:
   - Save videos for offline viewing.
   - Download entire playlists or channels.

2. **Audio Extraction**:
   - Extract audio tracks from videos for music or podcasts.

3. **Archiving Content**:
   - Preserve online content before it is removed or restricted.

4. **Subtitle Downloads**:
   - Download subtitles for accessibility or language learning.

5. **Custom Media Processing**:
   - Convert downloaded media to specific formats or resolutions.

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Feature-Rich**: Includes advanced options for format selection, metadata, and post-processing.
- **Open Source**: Actively maintained by a community of contributors.
- **Wide Compatibility**: Works on Windows, macOS, and Linux.
- **Extensive Website Support**: Covers a broad range of platforms beyond YouTube.

### ❌ Disadvantages
- **Command-Line Interface**: May be intimidating for non-technical users.
- **Dependency on FFmpeg**: Some features require additional tools like FFmpeg.
- **Website-Specific Issues**: Occasionally breaks when websites change their APIs or structures.

---

## 🆚 Comparisons with Similar Tools

| Feature                | yt-dlp            | youtube-dl        | JDownloader       | Video DownloadHelper |
|------------------------|-------------------|-------------------|-------------------|-----------------------|
| **Website Support**    | 🌟 Extensive      | 🌟 Extensive      | Moderate          | Moderate             |
| **Format Selection**   | ✅ Advanced       | ✅ Basic          | ❌ Limited        | ❌ Limited           |
| **Post-Processing**    | ✅ Yes            | ✅ Yes            | ❌ No             | ❌ No                |
| **Ease of Use**        | Moderate          | Moderate          | Easy              | Easy                 |
| **Open Source**        | ✅ Yes            | ✅ Yes            | ❌ No             | ❌ No                |

---

## 🛠️ Common Commands

- **Download a video**:  
  `yt-dlp <URL>`

- **Download audio only**:  
  `yt-dlp -x --audio-format mp3 <URL>`

- **Download a playlist**:  
  `yt-dlp <PLAYLIST_URL>`

- **Select specific video quality**:  
  `yt-dlp -f "bestvideo+bestaudio" <URL>`

- **Download subtitles**:  
  `yt-dlp --write-subs --sub-lang en <URL>`

- **Use a configuration file**:  
  `yt-dlp --config-location <CONFIG_FILE>`

---

## 🔗 Related Topics

- [[FFmpeg]]
- [[youtube-dl]]
- [[Video Processing Tools]]
- [[Media Streaming Protocols]]

---

## 📚 Further Reading

- [yt-dlp GitHub Repository](https://github.com/yt-dlp/yt-dlp)
- [yt-dlp Documentation](https://github.com/yt-dlp/yt-dlp#readme)
- [FFmpeg Documentation](https://ffmpeg.org/documentation.html)

---

## 🧠 Summary

yt-dlp is a versatile and feature-rich tool for downloading and processing online media. Its advanced capabilities, active development, and extensive website support make it a top choice for video and audio downloading. While it may have a learning curve for beginners, its flexibility and power are unmatched in the domain of media downloading tools.
