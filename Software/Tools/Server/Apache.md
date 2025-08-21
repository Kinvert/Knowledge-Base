# Apache HTTP Server (httpd)

The **Apache HTTP Server** (often called **Apache** or **httpd**) is a widely used, modular web server for serving static and dynamic content, reverse proxying, and acting as a general HTTP(S) front end. It powers classic LAMP stacks, shared hosting, enterprise intranets, and modern microservices edges. Apache is known for its rich module ecosystem, per-directory configuration via `.htaccess`, and flexibility across Unix-like systems and Windows.

---

## ⚙️ Overview

Apache is a process/thread-based server with a pluggable architecture. It supports virtual hosting, TLS, compression, caching, proxying, and integration with app runtimes (PHP-FPM, Python, Node.js, Java, etc.). Configuration is text-based and typically split across `httpd.conf`, `conf.d/*.conf`, and `sites-available/sites-enabled` (Debian/Ubuntu).

---

## 🧩 Architecture & Core Concepts

- **MPMs (Multi-Processing Modules)**: Choose request handling model:
  - `event` (asynchronous keep-alive handling; best general-purpose)
  - `worker` (threads + processes)
  - `prefork` (process-per-connection; legacy, sometimes used with non-thread-safe modules)
- **Modules**: Functionality via `mod_*` (e.g., `mod_ssl`, `mod_http2`, `mod_proxy`, `mod_rewrite`, `mod_headers`, `mod_brotli`)
- **Virtual Hosts**: Name-based and IP-based hosts (`*:80`, `*:443`)
- **.htaccess**: Per-directory overrides (useful in shared hosting; avoid for performance if possible)
- **Pipelines**: Reverse proxy and load balancing via `mod_proxy`, `mod_proxy_http`, `mod_proxy_fcgi`, `mod_proxy_balancer`
- **HTTP/2**: Provided by `mod_http2` (optimized with `event` MPM)
- **TLS**: `mod_ssl` (OpenSSL) and ACME automation via `mod_md` or `certbot`

---

## 🔬 How It Works (High Level)

1. **Listener** on `Listen 80/443` accepts connections (accept mutex tuned by MPM).
2. **MPM** dispatches requests to workers/threads.
3. **Request processing** flows through input filters → handlers (static file, proxy, CGI/FCGI) → output filters (compression, chunking).
4. **Authorization/AuthN/Z** via `mod_authn_*`, `mod_authz_*`, optional SSO (Kerberos/OAuth via modules).
5. **Logging** via `mod_log_config` (access/error logs), metrics via `mod_status`.

---

## 🧰 Quick Commands Cheat Sheet

- Test config: `apachectl -t` or `httpd -t`
- List modules: `apachectl -M | sort`
- Enable/disable modules (Debian/Ubuntu): `a2enmod rewrite` / `a2dismod mpm_prefork`
- Enable site (Debian/Ubuntu): `a2ensite example.conf` then `systemctl reload apache2`
- Start/Reload: `systemctl restart httpd` (RHEL) or `systemctl reload apache2` (Debian)
- Check syntax & reload (one-liner): `apachectl -t && systemctl reload apache2`
- Tail logs: `tail -f /var/log/apache2/{access,error}.log`
- Basic Auth user: `htpasswd -c /etc/apache2/.htpasswd user`
- Show MPM: `apachectl -V | grep -i mpm`
- TLS test (local): `openssl s_client -connect localhost:443 -servername your.host`

---

## 🧠 Key Features

- **Reverse Proxy & LB**: `mod_proxy`, `mod_proxy_balancer`, `mod_proxy_http`, `mod_proxy_fcgi`
- **HTTP/2**: `mod_http2` with ALPN; server push deprecated in clients but supported historically
- **TLS**: `mod_ssl` with OCSP stapling; ACME automation via `mod_md`
- **Caching**: `mod_cache`, `mod_cache_disk`, `mod_file_cache`
- **Compression**: `mod_deflate` (gzip), `mod_brotli` (Brotli)
- **Rewriting & Routing**: `mod_rewrite`, `mod_alias`, `mod_proxy_express`
- **Security/WAF**: `mod_security` (third-party) with [[OWASP CRS]] (Core Rule Set)
- **Observability**: `mod_status`, per-`LogFormat`, conditional logging, `mod_dumpio` for debugging

---

## 🧪 Typical Use Cases

- **LAMP**: Apache + [[PHP-FPM]] + [[MySQL]]/[[MariaDB]] for CMS (WordPress, Drupal, MediaWiki)
- **Reverse Proxy Gateway**: Terminate TLS, route to [[Gunicorn]]/[[uWSGI]]/[[Node.js]]/[[Tomcat]]
- **Shared Hosting**: Fine-grained user overrides via `.htaccess`
- **Enterprise Intranet**: SSO (Kerberos/LDAP), authn/z gateways
- **Static & Media**: Efficient static delivery with HTTP/2, `Cache-Control`, and Brotli
- **Microservices Edge**: Canary routing, header manipulation, WAF, mTLS (via modules)
- **Legacy App Support**: CGI/SCGI/AJP (AJP is legacy; prefer HTTP/FCGI)

---

## ✅ Strengths

- Extremely **mature & stable**, broad OS support
- **Rich module ecosystem** and long-tail features
- **Granular config** (per-dir, per-vhost), **.htaccess** flexibility
- Strong **TLS** and enterprise auth integrations
- **First-class PHP** integration via `proxy_fcgi` (PHP-FPM)

---

## ⚠️ Limitations

- Process/thread architecture can be **heavier** than event-driven servers for high concurrency
- **.htaccess** hurts performance when heavily used
- Native **HTTP/3/QUIC** support is not mainstream in httpd (commonly fronted by a proxy for H3)
- Configuration syntax can be **verbose**; changes spread across many files

---

## 🚀 Performance Tuning Highlights

- Prefer **`mpm_event`**; avoid `prefork` unless required
- Run **PHP via FPM** (`ProxyPassMatch`/`SetHandler` with `proxy_fcgi`) instead of mod_php
- Disable unused modules: `a2dismod autoindex cgi negotiation ...`
- Keep-Alive tuning: `KeepAlive On`, `MaxKeepAliveRequests 1000+`, `KeepAliveTimeout 1-2`
- MPM sizing: tune `ServerLimit`, `ThreadsPerChild`, `MaxRequestWorkers` to CPU/RAM
- Filesystem: use `EnableSendfile Off` on virtualized/storage-backed setups if issues occur
- Prefer **vhost-wide config**; minimize `.htaccess` with `AllowOverride None` where possible
- Caching: `mod_cache_disk` for frequently requested static/dynamic with correct `Cache-Control`

---

## 🛡️ Security Hardening (quick wins)

- Hide version: `ServerTokens Prod`, `ServerSignature Off`
- Security headers: `Header always set X-Content-Type-Options "nosniff"`, `X-Frame-Options "SAMEORIGIN"`, `Referrer-Policy "strict-origin-when-cross-origin"`
- TLS: modern ciphers, OCSP stapling, HSTS where appropriate
- WAF: deploy `mod_security` with [[OWASP CRS]]
- Limit methods: `Require` directives, `LimitExcept` blocks, disable `TRACE`
- Least privilege: run as dedicated user, conf/keys permissions `600`
- Log and monitor: `mod_status` protected by IP/auth; centralize logs

---

## 🔄 Ecosystem & Variants

- **httpd 2.4.x**: Current mainstream series (modules like `mod_http2`, `mod_md`)
- **Windows builds**: Popular community builds (e.g., Apache Lounge); production prefers Unix-like OS
- **Sibling projects**: [[Apache Tomcat]] (Java), [[Apache Traffic Server]] (caching/proxy), distinct from httpd
- **Automation**: `mod_md` (built-in ACME) vs external `certbot --apache`

---

## 🔗 Interop & Compatible Items

- **Languages/Runtimes**: [[PHP-FPM]], Python (WSGI via Gunicorn/uWSGI), Node.js, Ruby (Puma/Unicorn), Java (Tomcat/JBoss)
- **Proxies/LB**: [[HAProxy]], [[Nginx]], [[Envoy]], [[Traefik]]
- **Databases**: [[MySQL]], [[PostgreSQL]], [[Redis]] (sessions via app)
- **CI/CD & Config Mgmt**: [[Ansible]], [[Terraform]], Docker, Kubernetes (as an ingress sidecar or behind ingress)
- **TLS/ACME**: Let’s Encrypt, ACME (via `mod_md`)

---

## 🧭 Troubleshooting & Diagnostics

- Validate config: `httpd -t -D DUMP_VHOSTS`, `apachectl -S`
- Dump active settings: `apachectl -V`, `apachectl -M`
- Check per-request details: `LogLevel trace8` (temporarily), `mod_dumpio` (careful with secrets)
- Status page: enable `mod_status` and check `/server-status` (restrict access)
- Benchmark: `ab -n 10000 -c 200 https://host/` or use `wrk`/`hey` (external tools)

---

## 🆚 Comparison Chart (Web Servers & Edge Proxies)

| Project           | Model / Concurrency         | HTTP/2 | HTTP/3 | ACME Built-in | Config Style        | Static Perf | Dynamic App Integration | Typical Role |
|-------------------|-----------------------------|--------|--------|---------------|---------------------|-------------|-------------------------|--------------|
| **Apache httpd**  | Process/threads (`event`)   | ✅     | Via front proxy/experimental | `mod_md` | Directives (`*.conf`) | Good        | Excellent (FCGI/Proxy) | General-purpose, legacy & enterprise |
| **Nginx**         | Event loop (epoll/kqueue)   | ✅     | ✅     | Via `certbot`/3rd | Blocks (`nginx.conf`) | Very high   | Very good (FastCGI/Proxy) | Static, reverse proxy/LB |
| **Caddy**         | Event loop (Go)             | ✅     | ✅     | ✅ (automatic) | Simple (`Caddyfile`) | High        | Good (reverse proxy)     | Dev UX, TLS automation |
| **LiteSpeed**     | Event-driven                | ✅     | ✅     | Built-in       | GUI + conf          | Very high   | Excellent (native PHP)   | High-perf PHP hosting |
| **OpenLiteSpeed** | Event-driven (open-source)  | ✅     | ✅     | Via plugins    | GUI + conf          | High        | Excellent (PHP)          | Free alt to LiteSpeed |
| **IIS**           | IOCP (Windows)              | ✅     | ✅     | OS-managed     | GUI + XML           | High (Windows) | Good (.NET/ISAPI)      | Windows enterprises |
| **Envoy**         | Event-driven (C++)          | ✅     | ✅     | Via extensions | YAML                | High        | Excellent (gRPC/HTTP)    | Service mesh/edge |
| **Traefik**       | Event-driven (Go)           | ✅     | ✅     | ✅             | TOML/YAML/Labels    | High        | Very good (containers)   | Cloud-native ingress |

*Notes*: Apache’s HTTP/3 commonly achieved by placing a QUIC-capable proxy (e.g., Nginx, Caddy, Envoy, HAProxy) in front. Apache offers unmatched `.htaccess` flexibility and module breadth; Nginx/Caddy excel in minimalism and default performance.

---

## 🧱 Related Concepts / Notes

- [[HTTP]] (Hypertext Transfer Protocol)
- [[TLS]] (Transport Layer Security)
- [[Reverse Proxy]]
- [[Load Balancing]]
- [[WAF]] (Web Application Firewall)
- [[MPM]] (Multi-Processing Module)
- [[PHP-FPM]] (FastCGI Process Manager)
- [[Nginx]] (Web server & reverse proxy)
- [[Caddy]] (Automatic HTTPS web server)
- [[HAProxy]] (High-performance TCP/HTTP proxy)
- [[Envoy]] (Service proxy)
- [[Traefik]] (Cloud-native edge router)
- [[Apache Tomcat]] (Java servlet container)

---

## 📚 External Resources

- Apache HTTP Server Project (docs, modules, how-tos)
- Security hardening guides (OWASP, distro-specific)
- Performance tuning guides for `mpm_event` and HTTP/2
- Let’s Encrypt ACME workflows (`mod_md`, `certbot`)
- Community Q&A and module repos

---

## 📝 Summary

Apache remains a versatile, production-grade web server with deep features and modules for almost any HTTP(S) scenario. While event-driven servers can outpace it in raw concurrency with simpler configs, Apache’s flexibility, module ecosystem, and enterprise integrations make it a dependable choice for LAMP apps, complex routing, and environments that benefit from `.htaccess` and fine-grained control.
