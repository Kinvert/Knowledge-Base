# Nginx (Engine-X)

**Nginx** is a high-performance, event-driven web server and reverse proxy, widely used for serving static content, load balancing, caching, and as a TLS termination proxy. Its asynchronous architecture makes it well-suited for handling high concurrency with low memory usage. Nginx has become a cornerstone of modern web infrastructure, powering millions of websites and APIs.

---

## ⚙️ Overview

Nginx was originally developed to solve the **C10k problem** (handling 10,000+ concurrent connections). Unlike Apache’s process/thread-based model, Nginx uses an event-driven, non-blocking architecture that scales efficiently under heavy load. Today, Nginx is used both as a standalone web server and as a reverse proxy in front of application servers.

---

## 🧩 Architecture & Core Concepts

- **Event-driven I/O**: Uses epoll/kqueue for asynchronous connection handling.
- **Master/Worker Model**: One master manages multiple worker processes; workers handle all connections.
- **Modules**: Core (built-in) and dynamic modules (e.g., `ngx_http_ssl_module`, `ngx_stream_module`).
- **Reverse Proxy**: Distributes traffic across backend servers (HTTP, FastCGI, gRPC, TCP, UDP).
- **Load Balancing**: Round-robin, least connections, IP hash, and health checks.
- **Caching**: Built-in disk-based content caching.
- **TLS Termination**: Offloading SSL/TLS from backend applications.

---

## 📊 Comparison Chart

| Feature / Server         | Nginx              | [[Apache]] | [[Caddy]] | [[HAProxy]] | [[Envoy]] |
|---------------------------|--------------------|------------------------|-----------|-------------|-----------|
| Architecture              | Event-driven       | Process/Thread-based   | Event-driven | Event-driven | Event-driven |
| Static Content            | Extremely fast     | Good                   | Fast       | Not designed | Not designed |
| Reverse Proxy / LB        | ✅ Built-in        | ✅ via mod_proxy        | ✅         | ✅ Advanced  | ✅ Advanced |
| Config Style              | Blocks (`nginx.conf`) | Directives (`*.conf`) | Simple (`Caddyfile`) | Declarative | YAML |
| TLS Automation            | External (Certbot, acme.sh) | mod_md or Certbot | Built-in   | External    | External |
| HTTP/2 Support            | ✅                 | ✅ (mod_http2)         | ✅         | ✅           | ✅ |
| HTTP/3/QUIC               | ✅ (1.25+)         | Experimental/proxied   | ✅         | ✅ (via patches) | ✅ |

---

## 🧪 Use Cases

- **Static File Hosting**: Images, video streaming, web assets with high concurrency.
- **Reverse Proxy**: TLS termination, header manipulation, path routing to backends.
- **Load Balancing**: Scaling web apps and APIs across multiple servers.
- **Edge Proxy**: Microservices ingress, API gateway functionality.
- **Caching**: Acting as a content cache to reduce backend load.
- **Security Frontend**: Rate limiting, IP whitelisting/blacklisting, DoS mitigation.

---

## ✅ Strengths

- Lightweight and highly efficient at scale.
- Handles static content faster than most alternatives.
- Robust reverse proxy and load balancing features.
- Strong community and commercial support (NGINX, Inc. → F5).
- Supports modern web standards (HTTP/2, HTTP/3, TLS 1.3).

---

## ❌ Limitations

- Configuration syntax can be less intuitive for beginners.
- Dynamic modules must be compiled or pre-installed (less plug-and-play than Apache).
- Limited `.htaccess`-style per-directory overrides (all configuration centralized).
- Advanced WAF/security needs external modules (e.g., ModSecurity).

---

## 🔧 Compatible & Related Items

- [[Apache]] (often compared; sometimes used together)
- [[Caddy]] (alternative with built-in TLS automation)
- [[HAProxy]] (stronger focus on load balancing)
- [[Envoy]] (cloud-native proxy and service mesh)
- [[TLS]] (Transport Layer Security for HTTPS)

---

## 📝 Summary

Nginx is a high-performance, event-driven web server and reverse proxy designed for scalability and efficiency. It excels at serving static content, proxying, and TLS termination, making it a standard choice for modern web applications and microservices architectures. While it lacks some of Apache’s granular per-directory configuration flexibility, Nginx’s simplicity and speed have made it the backbone of much of today’s web.
