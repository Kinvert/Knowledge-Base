# nvidia-smi

**`nvidia-smi`** (NVIDIA System Management Interface) is a command-line utility provided with the NVIDIA Driver package. It allows users to monitor and manage GPU devices in real time, providing insights into GPU utilization, memory usage, temperature, power consumption, and active processes.

It is a critical diagnostic and monitoring tool for developers using GPUs in machine learning, simulation, robotics, and high-performance computing environments.

---

## 🧠 Overview

`nvidia-smi` interfaces directly with the installed NVIDIA Driver and supported NVIDIA GPUs. It outputs detailed statistics and can also be used to:

- Change performance states  
- Set power limits  
- Kill processes using the GPU  
- Monitor memory and temperature  
- Track the active CUDA context

---

## 🧪 Use Cases

- Checking GPU status during training or simulation  
- Diagnosing overheating or memory bottlenecks  
- Monitoring GPU usage across multiple users  
- Debugging stalled or runaway processes  
- Tracking GPU power efficiency  
- Integrating into monitoring systems for automation  

---

## ⚙️ Capabilities

- List all visible NVIDIA GPUs with name, memory, and utilization  
- Display per-process GPU memory usage  
- Show driver and CUDA versions  
- Change GPU power and clock settings (with elevated permissions)  
- Logging and CSV/JSON output for automation  
- Query individual metrics using flags (e.g. `--query-gpu`)  

---

## 📊 Common Commands

| Command                                  | Description                                 |
|------------------------------------------|---------------------------------------------|
| `nvidia-smi`                             | Show summary of all GPUs                    |
| `nvidia-smi -l 1`                        | Refresh stats every second                  |
| `nvidia-smi --query-gpu=utilization.gpu` | Query specific GPU attribute                |
| `nvidia-smi dmon`                        | Monitor device stats in real time           |
| `nvidia-smi pmon -i 0`                   | Per-process monitoring on GPU 0             |
| `nvidia-smi --format=csv`               | Output in CSV format                        |
| `nvidia-smi -i 0 -pl 150`                | Set power limit to 150W on GPU 0 (sudo)     |
| `nvidia-smi --help`                      | Show help menu                              |

---

## ✅ Pros

- Installed by default with NVIDIA drivers  
- Lightweight and scriptable  
- Useful for performance tuning and monitoring  
- Wide adoption in ML, HPC, and robotics environments  
- Supports output logging and scripting

---

## ❌ Cons

- Read-only for most users without root access  
- Limited in-depth metrics compared to tools like Nsight  
- No direct integration with GUI environments  
- Only supports NVIDIA GPUs

---

## 🔗 Related Concepts

- [[NVIDIA Driver]]  
- [[CUDA Toolkit]]  
- [[GPU Computing]]  
- [[cuDNN]]  
- [[PyTorch]]  
- [[TensorFlow]]  
- [[TensorRT]]  
- [[CUDA Streams]]

---

## 📚 Further Reading

- [NVIDIA Developer Docs – nvidia-smi](https://developer.nvidia.com/nvidia-system-management-interface)  
- [nvidia-smi Reference Guide (PDF)](https://developer.download.nvidia.com/compute/DCGM/docs/nvidia-smi-367.38.pdf)  
- `nvidia-smi -h` — for full help and option list  
- `man nvidia-smi` — manual page on Linux (if installed)

---
