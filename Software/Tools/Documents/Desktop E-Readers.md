# PDF / MOBI / EPUB Readers on Ubuntu

Document readers that work well in Ubuntu for viewing and managing formats like [[PDF]], [[MOBI]], and [[EPUB]]. This document covers both pure readers and full management suites, with tables comparing features relevant for reading, annotation, and conversion.

---

## Overview

Many document formats are common when reading ebooks or technical documents on Linux. Ubuntu has solid support for most of these formats. The goal here is simple, reliable tools that are easy to install and use.

Formats covered include:
- [[PDF]]
- [[EPUB]]
- [[MOBI]]
- [[DJVU]]
- AZW3
- FB2
- CBZ
- CBR

---

## Readers

### [[Okular]]

- KDE's document viewer.
- Supports many formats out of the box, with plugins for others.
- Strong annotation features.
- Available directly via `apt`.

### [[Evince]]

- GNOME's default document viewer.
- Lightweight and simple.
- Pre-installed on many Ubuntu systems.
- Primarily aimed at [[PDF]] and [[DJVU]] formats.

### [[Foliate]]

- Modern [[EPUB]] reader.
- Supports [[MOBI]], AZW3, FB2, and limited [[PDF]].
- Has customizable night mode, themes, and dictionaries.
- Available via Flatpak.

### [[Calibre]]

- Full ebook management suite.
- Reads nearly any ebook format.
- Highly customizable reader interface.
- Includes full format conversion tools.

### [[KOReader]]

- Originally built for e-ink devices.
- Now available as Flatpak or AppImage for desktop.
- Excellent support for both [[PDF]] and [[EPUB]].
- Highly configurable reading environment.

### [[Zathura]]

- Lightweight, keyboard-driven viewer.
- Supports [[PDF]] natively; [[EPUB]] and [[DJVU]] via plugins.
- Extremely fast and efficient.
- Highly scriptable and configurable.

### [[Bookworm]]

- Simple, clean reader.
- Supports [[EPUB]], [[PDF]], [[MOBI]], CBZ, and CBR.
- Available as Flatpak.
- Lighter feature set than [[Foliate]] or [[KOReader]].

---

## Comparison - Reading Features

| Application | Formats | Night Mode | Custom Colors | Notes |
| ------------| --------| -----------| --------------| ------|
| [[Okular]] | [[PDF]], [[EPUB]] (plugin), [[MOBI]] (plugin), [[DJVU]], CBZ | Yes | Limited | Strong for academic work |
| [[Evince]] | [[PDF]], [[DJVU]], TIFF, DVI, XPS | Yes | No | GNOME default |
| [[Foliate]] | [[EPUB]], [[MOBI]], AZW3, FB2, [[PDF]] (basic) | Yes | Yes | Excellent for EPUB |
| [[Calibre]] | [[PDF]], [[EPUB]], [[MOBI]], AZW3, CBZ, CBR, etc | Yes | Yes | Full suite |
| [[KOReader]] | [[PDF]], [[EPUB]], [[MOBI]], FB2, [[DJVU]], CBZ, CBR | Yes | Yes | Highly customizable |
| [[Zathura]] | [[PDF]] (native), [[EPUB]] (plugin), [[DJVU]] | Yes | Yes (via config) | Lightweight, fast |
| [[Bookworm]] | [[EPUB]], [[PDF]], [[MOBI]], CBR, CBZ | Yes | Yes | Simple interface |

---

## Annotation / Editing

| Application | Annotation | Highlights | Editing | Notes |
| ------------| -----------| ----------| --------| ------|
| [[Okular]] | Yes | Yes | Limited | Great academic annotator |
| [[Evince]] | Limited | No | No | Lightweight |
| [[Foliate]] | Bookmarks, highlights | Yes | No | Good basic annotations |
| [[Calibre]] | Yes (via plugins) | Yes | Yes | Full management suite |
| [[KOReader]] | Yes | Yes | No | Good for touch annotation |
| [[Zathura]] | No | No | No | Pure reader |
| [[Bookworm]] | Bookmarks | No | No | Lightweight |

---

## Conversion / Management

### [[Calibre]]

- The dominant suite for ebook management.
- Converts between most formats.
- Handles metadata editing, cover images, series, syncing to devices, etc.
- GUI and command-line tools available.

### ebook-convert (Calibre CLI)

- Ships with [[Calibre]].
- Full conversion functionality available via terminal.
- Supports batch conversion with scripting.

### [[Pandoc]]

- Limited support for [[EPUB]] creation.
- Better for technical documents than ebook library management.
- Excellent when working with Markdown or LaTeX sources.

### pdf2epub / epub2pdf tools

- Lightweight single-purpose converters.
- Useful for quick format changes.
- Less robust than [[Calibre]].

---

## Comparison - Conversion / Management

| Tool | Converts | Metadata | Batch? | Notes |
| ---- | -------- | -------- | ------ | ----- |
| [[Calibre]] | Yes | Yes | Yes | Full-featured |
| ebook-convert | Yes | Yes | Yes | CLI version of [[Calibre]] |
| [[Pandoc]] | Limited | No | Yes | Great for text/Markdown |
| pdf2epub / epub2pdf | Yes | No | No | Lightweight tools |

---

## Installation

### Apt

```
sudo apt install okular evince zathura calibre
```

### Flatpak

```
flatpak install flathub com.github.johnfactotum.Foliate
flatpak install flathub com.github.babluboy.bookworm
flatpak install flathub org.koreader.KOReader
```

### AppImage

- [[KOReader]] offers portable AppImages.

---

## Notes

- Flatpak often offers newer versions than Ubuntu’s apt packages.
- [[Calibre]]'s GUI and CLI tools cover nearly every ebook management task.
- [[Zathura]] excels at keyboard-driven [[PDF]] reading.
- [[Foliate]] and [[KOReader]] are top choices for heavy [[EPUB]] usage.
