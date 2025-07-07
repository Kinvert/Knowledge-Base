# 📸 Omnivision Global Shutter Sensor Comparison

A curated table of various **OmniVision** global shutter CMOS sensors, frequently used in robotics, industrial imaging, stereo vision, AR/VR, and embedded systems. This chart includes details like resolution, pixel size, chroma configuration, and interface options to help compare models effectively.

---

## 🧠 Summary Table

| Part Number   | Resolution     | Pixel Size | Key Tech Features                                     | Optical Format | Frame Rate (Max)                      | CFA       | Output Format            | Interface          | Package         |
|---------------|----------------|------------|--------------------------------------------------------|----------------|---------------------------------------|-----------|---------------------------|---------------------|-----------------|
| OG01A1B       | 1.3MP          | 2.2 µm     | Global Shutter, Nyxel®, PureCel®Plus‑S                | 1/5"           | 120 fps                               | B&W       | RAW                       | MIPI                | COB, RW         |
| OG01H1B       | 1.55MP         | 2.2 µm     | Global Shutter, Nyxel®, PureCel®Plus‑S                | 1/4.51"        | 120 fps (MIPI), 30 fps (DVP)          | B&W       | —                         | DVP, MIPI           | CSP             |
| OG02B         | 2MP            | 3.0 µm     | Global Shutter, OmniPixel®3‑GS                        | 1/2.9"         | 60 fps                                | B&W, Color| RAW                       | DVP, MIPI           | COB, RW         |
| OG02C         | 2MP            | 3.45 µm    | DCG™, HDR, Nyxel®, PureCel®Plus‑S                     | 1/2.53"        | 120/180/240/300 fps (various modes)   | B&W, Color| 10/12-bit RAW             | LVDS, MIPI          | CSP             |
| OG03A         | 3MP            | 3.45 µm    | DCG™, Nyxel®, PureCel®Plus‑S                          | 1/1.8"         | 150 fps                               | B&W, Color| 10/12/14-bit RAW          | LVDS, MIPI          | CLGA            |
| OG05B         | 5MP            | 2.2 µm     | Nyxel®, PureCel®Plus‑S                                | 1/2.53"        | 60 fps                                | B&W, Color| 10-bit RAW                | DVP, MIPI           | CSP             |
| OG05C         | 5MP            | 3.45 µm    | DCG™, HDR, Nyxel®, PureCel®Plus‑S                     | 1/1.45"        | 60/120 fps                            | B&W, Color| 10/12/14-bit RAW          | LVDS, MIPI          | CLGA            |
| OG09A         | 9MP            | 3.45 µm    | DCG™, HDR, Nyxel®, PureCel®Plus‑S                     | 1"             | 60 fps (30 fps in HDR mode)           | B&W, Color| 10/12-bit RAW             | LVDS                | CLGA            |
| OG0TB         | 400×400        | 2.2 µm     | Nyxel®, PureCel®Plus‑S                                | 1/14.46"       | 240 fps                               | B&W       | 8/10-bit RAW              | MIPI, SPI           | CSP             |
| OG0TC1B       | 400×400        | 2.2 µm     | DCG™, HDR, Nyxel®, PureCel®Plus‑S                     | 1/14.46"       | 240–480 fps                           | B&W       | 8/10/12/14-bit RAW        | MIPI                | CSP             |
| OG0VA         | VGA            | 2.2 µm     | Nyxel®, PureCel®Plus‑S                                | 1/10"          | 240 fps                               | B&W       | RAW                       | MIPI / LVDS         | COB, CSP, RW    |
| OG0VE         | VGA            | 3.006 µm   | OmniPixel®3‑GS                                        | 1/7.5"         | 120 fps                               | B&W       | 8/10-bit RAW              | MIPI                | COB, CSP, RW    |
| OV2310        | 1.3MP          | 3.0 µm     | OmniPixel®3‑GS                                        | 1/3.6"         | 60 fps                                | B&W       | 10-bit RAW                | DVP, MIPI           | a‑CSP™          |
| OV2311        | 2MP            | 3.0 µm     | OmniPixel®3‑GS                                        | 1/2.9"         | 60 fps                                | B&W       | RAW                       | DVP, MIPI           | a‑CSP™          |
| OV2312        | 2MP            | 3.0 µm     | OmniPixel®3‑GS                                        | 1/2.9"         | 60 fps                                | RGB-Ir    | RGB-Ir                    | DVP, MIPI           | a‑CSP™          |
| OV6211        | 400×400        | 3.0 µm     | OmniPixel®3‑GS                                        | 1/10.5"        | 120 fps                               | Mono      | RAW                       | MIPI                | CSP             |
| OV7251        | VGA            | 3.0 µm     | OmniPixel®3‑GS                                        | 1/7.5"         | 120 fps                               | B&W       | 10-bit RAW                | MIPI / LVDS         | COB, CSP        |
| OV7261        | VGA            | 3.0 µm     | OmniPixel®3‑GS                                        | 1/7.5"         | 100 fps                               | Mono      | RAW                       | MIPI                | a‑CSP™          |
| [[OV9281]]    | 1MP            | 3.0 µm     | OmniPixel®3‑GS                                        | 1/4"           | 120 fps                               | Mono      | RAW                       | DVP, MIPI           | CSP             |
| OV9282        | 1MP            | 3.0 µm     | OmniPixel®3‑GS                                        | 1/4"           | 120 fps                               | Mono      | RAW                       | DVP, MIPI           | RW              |
| OV9284        | 1MP            | 3.0 µm     | OmniPixel®3‑GS                                        | 1/4"           | 120 fps                               | B&W       | RAW                       | DVP, MIPI           | a‑CSP™          |
| [[OV9782]]    | 1MP            | 3.0 µm     | OmniPixel®3‑GS                                        | 1/4"           | 120 fps                               | RGB Bayer | 8/10-bit RAW              | DVP, MIPI           | COB, RW         |
| OX01H1B       | 1.5MP          | 2.2 µm     | Nyxel®, OmniPixel®4-GS, PureCel®Plus‑S                | 1/4.51"        | 90 fps                                | B&W       | RAW B&W                   | DVP, MIPI           | a‑CSP™          |
| OX01N1B       | 1.55MP         | 2.2 µm     | Nyxel®, OmniPixel®4-GS, PureCel®Plus‑S                | 1/4.51"        | 90 fps                                | B&W       | 10-bit RAW, YUV           | DVP, MIPI           | a‑CSP™          |
| OX02C1S       | 2.5MP          | 2.2 µm     | Nyxel®, OmniPixel®4-GS, RGB‑Ir, PureCel®Plus‑S        | 1/3.52"        | 90 fps                                | RGB-Ir    | RAW RGB-Ir                | DVP, MIPI           | a‑CSP™          |
| OX05B         | 5MP            | 2.2 µm     | HDR, Nyxel®, RGB‑Ir, PureCel®Plus‑S                   | 1/2.53"        | 60 fps (1944p)                         | RGB-Ir    | Linear output             | DVP, MIPI           | a‑CSP™          |

---

## 🔗 Related Notes

- [[Global Shutter]]
- [[Cameras]]
- [[Arducam]]
- [[MIPI CSI-2 Protocol]]
- [[Stereo Cameras]]
- [[Omnivision]]
- [[Raspberry Pi Camera]]
- [[Sensor Comparison Charts]]
- [[Arducam Camarray]]

---
