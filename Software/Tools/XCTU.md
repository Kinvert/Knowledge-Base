# XCTU 🛠️

XCTU is a free software tool provided by Digi International for configuring, testing, and managing their [[XBee]] and [[Zigbee]] modules. It provides a simple graphical user interface to work with Digi radio modules via USB or serial connection.

---

## 🚀 Features

| Feature                     | Description                                                                 |
|----------------------------|-----------------------------------------------------------------------------|
| **Module Configuration**   | Read/write firmware, set parameters, and update settings easily.            |
| **Network View** 🕸️         | Visualize the network topology of connected devices.                         |
| **Console Mode** 💻         | Interactive terminal for sending AT commands and monitoring serial data.     |
| **Range Testing** 📶        | Measure the wireless range and packet loss between modules.                  |
| **Firmware Update** 🔁     | Flash new firmware directly to the connected module.                         |
| **Spectrum Analyzer** 📡    | Check wireless signal strength across channels (on supported modules).       |
| **Profiles** 🧩             | Save and load configurations for multiple modules.                           |

---

## 🧰 System Requirements

- **OS Support**: Windows, macOS, and Linux
- **Java Runtime**: XCTU is Java-based and includes its own runtime
- **Connection**: USB or serial adapter (FTDI/CP2102, etc.)

---

## 🛠️ Common Use Cases

- Configuring XBee/XBee-PRO modules before deployment  
- Visualizing mesh topologies in Zigbee networks  
- Diagnosing wireless range issues in sensor networks  
- Developing and debugging custom wireless protocols  
- Teaching wireless communication in educational environments

---

## 🔁 XCTU vs Other Tools

| Feature / Tool        | **XCTU**                            | **CoolTerm**         | **RealTerm**       | **AT Command Line** |
|-----------------------|-------------------------------------|----------------------|--------------------|----------------------|
| GUI Configuration     | ✅ Yes                              | ❌ No                | ❌ No              | ❌ No                |
| Zigbee Network View   | ✅ Yes                              | ❌ No                | ❌ No              | ❌ No                |
| Serial Terminal       | ✅ Built-in                         | ✅ Yes               | ✅ Yes             | ❌ No                |
| Firmware Management   | ✅ Yes                              | ❌ No                | ❌ No              | ❌ No                |
| Cross-Platform        | ✅ Win/Mac/Linux                    | ✅ Yes               | ❌ Windows only    | ✅ Yes               |
| Visual Range Testing  | ✅ Yes                              | ❌ No                | ❌ No              | ❌ No                |

---

## 🔌 How to Use

### 🔧 Installation

1. Download from Digi’s official site: https://www.digi.com/xctu
2. Run the installer for your platform
3. Plug in your XBee via USB-to-serial adapter
4. Open XCTU and detect the connected radio module

### 🛠️ Basic Configuration

- Add your module via **Discover Radio Modules**
- Click **Read** to load its settings
- Modify PAN ID, baud rate, channel, etc.
- Click **Write** to save new settings
- Use **Console Mode** to test AT commands

### 🖥️ Network View

- Click on **Working Modes** → Network
- Scan to see surrounding devices
- Click devices to inspect routes and neighbors

---

## 🧪 Tips & Tricks

- Use **Profiles** to replicate configs across many modules  
- Set **API Mode** for use with libraries (like Python DigiMesh)  
- Check **DH / DL** for setting module addresses in point-to-point links  
- Use **ATND** to discover nearby devices manually in Console Mode  
- Use **+++\<pause\>AT** to enter command mode manually

---

## 📚 Related Tools

- **DigiMesh** – Mesh protocol supported by XCTU  
- **XBee Python Library** – Interact with XBee in Python  
- **MicroPython** – Often paired with XBee for IoT  

---

## 📎 Resources

- [Digi XCTU Official Page](https://www.digi.com/products/embedded-systems/digi-xbee/digi-xbee-tools/xctu)
- [XCTU Documentation (PDF)](https://www.digi.com/resources/documentation/digidocs/)
- [GitHub – XBee Python Lib](https://github.com/digidotcom/xbee-python)
