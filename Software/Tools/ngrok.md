# ngrok

**ngrok** is a tunneling service that exposes local network services to the public internet via secure tunnels. Robotics engineers commonly use it to remotely access development machines, webhook endpoints, embedded devices, and robot control dashboards—especially when working behind NAT, firewalls, or restricted networks.

---

## 🌐 Overview

ngrok creates an encrypted tunnel from a local port (such as `localhost:8080`) to a public URL. This makes it possible to test cloud integrations, share services, or remotely access tools running on private networks without configuring routers or VPNs.

---

## 🧠 Core Concepts

- **Tunneling**  
  Maps a local port to a public endpoint via ngrok’s cloud service.

- **Authtokens**  
  Authenticate your machine to your ngrok account for persistent configuration.

- **Ingress**  
  ngrok’s term for the public-facing entrypoint (HTTP, TCP, TLS).

- **Local Services**  
  The application you want to expose (API server, telemetry, dashboard).

- **Regions**  
  Geographic endpoints for minimizing latency.

- **ACL & Access Controls**  
  Restrict who can reach your tunnels.

---

## 📊 Comparison Chart

| Tool / Service | Type | Authentication | Ease of Use | Common Use Case | Notes |
|----------------|------|----------------|-------------|------------------|-------|
| **ngrok** | Tunnel Service | Required (token) | Very Easy | Webhooks, remote dashboards | Rich features & stable |
| **LocalTunnel** | Tunnel Service | None | Easy | Quick sharing | Free, less reliable |
| **Cloudflare Tunnel** | Tunnel Service | Account-based | Medium | Secure zero-trust access | Enterprise-focused |
| **SSH Reverse Tunnel** | Manual Tunnel | SSH keys | Medium | Dev access, CLI workflows | No web UI |
| **[[Tailscale]]** | Mesh VPN | Account | Medium | Private access to devices | No public endpoints |

---

## 🧰 Use Cases

- Exposing a robotics dashboard running on `localhost`
- Connecting to embedded systems’ web UIs over NAT
- Testing robot cloud integrations with webhook callbacks
- Streaming telemetry from a robot on a private network to a cloud app
- Sharing local simulation environments (e.g., Gazebo, RViz plugins)
- Demonstrating robotics demos during development or presentations

---

## ⭐ Strengths

- Extremely easy to set up  
- Provides public, HTTPS-secured endpoints  
- Works through firewalls and NAT  
- Dashboard includes request inspection  
- Supports HTTP, TCP, TLS, and edge-based routing  
- Programmable API and configuration files  
- Good ecosystem for CI, robots, and cloud integrations  

---

## ⚠️ Weaknesses

- Requires relying on a hosted third-party service  
- Free tier has occasional limitations (session length, domain stability)  
- Not ideal for high-security environments without additional controls  
- Latency depends on region and cloud proximity  

---

## 🔧 Developer Tools

- CLI tools (`ngrok http 8080`)  
- YAML configuration for persistent tunnels  
- API for programmatically managing tunnels  
- Web UI dashboard for live inspection  
- Integrations: GitHub, AWS, GCP, Slack, IoT systems  

---

## 🏗️ How It Works

1. You run a command like `ngrok http 5000`.  
2. ngrok’s client connects to its cloud service.  
3. ngrok provides a public URL like `https://abc123.ngrok.io`.  
4. External users hit that URL.  
5. Requests travel through ngrok’s cloud→your local machine.  
6. Your local server responds, and ngrok pipes the response back to the public internet.

---

## 🧮 Compatible Items

- [[SSH]] for alternative tunneling  
- [[Tunneling]]
- [[REST APIs]] for webhook testing  
- [[Dev Boards]] (Raspberry Pi, Jetson, etc.)  
- [[Cloud and Web Protocols]]  
- [[Docker]] for containerized local services  
- [[CI-CD]] systems that need ephemeral endpoints  
- [[Tailscale]]

---

## 🔀 Variants

- **ngrok Agent**: CLI binary  
- **ngrok Cloud Edge**: advanced routing, domains, ACL  
- **ngrok API**: automation and dynamic tunnel creation  
- **Self-hosted “alternatives”**: not ngrok itself, but comparable solutions (LocalTunnel, FRP, Cloudflare Tunnel)

---

## 📚 Related Concepts / Notes

- [[SSH]] (reverse port forwarding)  
- [[Zero Trust Networking]] (if covered)  
- [[Cloud and Web]]  
- [[Local Networking]]  
- [[Webhooks]]  
- [[Dev Boards]] for robotics deployment  
- [[Docker]] for local-and-cloud hybrid development  

---

## 🔗 External Resources

- Official documentation: ngrok.com/docs  
- ngrok GitHub SDKs  
- CLI reference: `ngrok help`  
- Example config templates on their site  

---

## 📝 Summary

ngrok is one of the simplest ways to expose local robotics tools, dashboards, APIs, simulators, and embedded interfaces to the wider internet securely. Its combination of lightweight CLI usage, strong security options, and ease of deployment make it a standard tool in robotics development workflows—especially when working behind NAT or developing cloud-connected robotic systems.
