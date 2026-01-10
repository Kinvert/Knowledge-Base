# Tailscale

**Tailscale** is a mesh VPN built on [[WireGuard]] that creates secure private networks with zero configuration. Instead of the traditional hub-and-spoke VPN model, Tailscale establishes direct peer-to-peer encrypted connections between devices, with automatic NAT traversal, key management, and DNS. For developers, Tailscale eliminates the friction of accessing remote machines, sharing local services, and managing SSH keys across multiple devices.

---

## Overview

| Attribute | Value |
|-----------|-------|
| **Founded** | 2019 |
| **Headquarters** | Toronto, Canada |
| **Built On** | WireGuard protocol |
| **Architecture** | Mesh VPN (peer-to-peer) |
| **Coordination** | Cloud-hosted control plane (or self-hosted Headscale) |
| **Data Path** | Direct device-to-device (no traffic through Tailscale servers) |
| **Pricing** | Free tier available, paid plans for teams |
| **Open Source** | Client is open source (BSD-3), control plane is proprietary |

Tailscale was founded by former Google engineers who worked on infrastructure. The key insight: WireGuard is excellent but hard to configure; Tailscale adds the coordination layer that makes it usable by anyone.

---

## Architecture Deep Dive

### WireGuard Foundation

Tailscale is built entirely on WireGuard, the modern VPN protocol that's now part of the Linux kernel:

| Aspect | WireGuard | OpenVPN |
|--------|-----------|---------|
| **Codebase** | ~4,000 lines | ~100,000+ lines |
| **Cryptography** | ChaCha20, Poly1305, Curve25519 | Configurable (TLS) |
| **Connection** | Single UDP port | TCP or UDP, multiple ports |
| **Handshake** | 1 round-trip | Multiple round-trips |
| **Performance** | Kernel-space, very fast | User-space, slower |
| **Attack Surface** | Minimal | Large |

WireGuard's simplicity means fewer bugs and better performance. Tailscale inherits all of this.

### Control Plane vs Data Plane

Tailscale's architecture separates coordination from data:

```
┌─────────────────────────────────────────────────────────────┐
│                    CONTROL PLANE                             │
│     (Tailscale's servers OR self-hosted Headscale)          │
│                                                              │
│   • Authentication (SSO, OAuth)                              │
│   • Key exchange coordination                                │
│   • ACL enforcement                                          │
│   • Device discovery                                         │
│   • MagicDNS records                                         │
│                                                              │
│            Control traffic only (metadata)                   │
└──────────────────────┬──────────────────────────────────────┘
                       │
        ┌──────────────┴──────────────┐
        │                              │
        ▼                              ▼
┌───────────────┐              ┌───────────────┐
│   Device A    │◄────────────►│   Device B    │
│   (Laptop)    │   DIRECT     │   (Server)    │
│               │  WireGuard   │               │
└───────────────┘    Tunnel    └───────────────┘
                       │
               DATA PLANE
         (Your actual traffic)
         Never touches Tailscale
```

**Critical point:** Your data never passes through Tailscale's servers. The control plane only handles metadata—who can talk to whom, what their public keys are, where they are on the network.

### NAT Traversal

Getting two devices behind NAT to talk directly is hard. Tailscale uses multiple techniques:

| Technique | Description | Success Rate |
|-----------|-------------|--------------|
| **Direct** | Both devices have public IPs | 100% |
| **UDP Hole Punching** | Coordinate simultaneous outbound connections | ~85% |
| **STUN** | Discover NAT type and external IP | N/A (diagnostic) |
| **Port Mapping** | UPnP, NAT-PMP, PCP | Variable |
| **DERP Relay** | Fallback encrypted relay | 100% (but slower) |

The `tailscale netcheck` command shows your NAT situation:

```bash
$ tailscale netcheck
Report:
    * UDP: true
    * IPv4: yes, 203.0.113.50:41234
    * IPv6: yes, [2001:db8::1]:41234
    * MappingVariesByDestIP: false
    * HairPinning: true
    * PortMapping: UPnP, NAT-PMP
    * Nearest DERP: San Francisco
```

### DERP Relay Servers

When direct connections fail, traffic routes through DERP (Designated Encrypted Relay for Packets):

- **Encrypted end-to-end**: DERP relays WireGuard packets; it cannot read content
- **Globally distributed**: ~20+ DERP servers worldwide
- **Latency penalty**: Adds RTT to relay, but still works
- **Self-hostable**: Run your own DERP for air-gapped or low-latency needs

Use `tailscale ping` to see if you're going direct or through DERP:

```bash
$ tailscale ping my-server
pong from my-server (100.64.0.2) via 203.0.113.50:41234 in 12ms
#                                    ^^^^^^^^^^^^^^^^
#                                    Direct connection!

$ tailscale ping my-other-server
pong from my-other-server (100.64.0.3) via DERP(nyc) in 45ms
#                                          ^^^^^^^^^
#                                          Relayed through NYC
```

### Key Management

Raw WireGuard requires manually distributing public keys to every peer. Tailscale automates this:

- **Per-device keys**: Each device generates its own keypair
- **Automatic rotation**: Keys rotate periodically (configurable)
- **Revocation**: Disable a device, its key is immediately revoked network-wide
- **No shared secrets**: Never type or copy-paste keys

---

## Core Concepts

### Tailnet

Your **tailnet** is your private mesh network. Every device you add joins the same tailnet. By default, all devices can reach all other devices (modifiable via ACLs).

Each device gets a stable IP in the `100.64.0.0/10` (CGNAT) range. These IPs persist even if the device's physical IP changes.

### MagicDNS

MagicDNS provides automatic DNS for your tailnet:

| Query | Resolves To |
|-------|-------------|
| `laptop` | 100.64.0.1 |
| `laptop.tail1234.ts.net` | 100.64.0.1 |
| `server.internal.example.com` | Via split DNS |

No more remembering IPs or editing `/etc/hosts`. Just `ssh laptop`.

Enable with:
```bash
tailscale up --accept-dns
```

### Exit Nodes

An **exit node** routes all your internet traffic through another device:

```
You (coffee shop WiFi) ──► Exit Node (home server) ──► Internet
                              Encrypted tunnel
```

Use cases:
- Privacy on untrusted networks
- Access geo-restricted content
- Appear to be on your home network

```bash
# Use a device as exit node
tailscale up --exit-node=home-server

# List available exit nodes
tailscale exit-node list

# Advertise yourself as exit node
tailscale up --advertise-exit-node
```

### Subnet Routers

A **subnet router** exposes an entire network to your tailnet without installing Tailscale on every device:

```
┌─────────────────────────────────────┐
│         Office Network              │
│         10.0.0.0/24                 │
│                                     │
│  ┌─────────┐ ┌─────────┐ ┌───────┐ │
│  │ Printer │ │ NAS     │ │ IoT   │ │
│  │ 10.0.0.5│ │ 10.0.0.10│ │10.0.0.│ │
│  └─────────┘ └─────────┘ └───────┘ │
│                                     │
│  ┌─────────────────────────────┐   │
│  │ Subnet Router               │   │
│  │ Tailscale + advertise-routes│   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
            │
            ▼ Tailscale mesh
┌─────────────────────────────────────┐
│  Your Laptop (anywhere)             │
│  Can now reach 10.0.0.0/24          │
└─────────────────────────────────────┘
```

```bash
# On the subnet router
tailscale up --advertise-routes=10.0.0.0/24

# On clients that want to use the route
tailscale up --accept-routes
```

### ACLs (Access Control Lists)

ACLs are JSON policies that control who can access what:

```json
{
  "acls": [
    // Admins can access everything
    {"action": "accept", "src": ["group:admin"], "dst": ["*:*"]},

    // Developers can access dev servers on SSH and HTTP
    {"action": "accept", "src": ["group:dev"], "dst": ["tag:dev-server:22,80,443"]},

    // Everyone can access the shared printer
    {"action": "accept", "src": ["*"], "dst": ["printer:9100"]}
  ],

  "groups": {
    "group:admin": ["alice@example.com"],
    "group:dev": ["bob@example.com", "carol@example.com"]
  },

  "tagOwners": {
    "tag:dev-server": ["group:admin"]
  }
}
```

ACLs are enforced by the control plane—devices won't even attempt connections that aren't permitted.

### Taildrop

**Taildrop** is peer-to-peer file transfer without cloud storage:

```bash
# Send a file
tailscale file send report.pdf my-laptop

# Receive pending files
tailscale file get

# Wait for incoming files (daemon mode)
tailscale file get --loop
```

Files transfer directly between devices over the encrypted mesh. No size limits, no cloud storage, no third-party servers.

### Tailscale SSH

Tailscale SSH replaces traditional SSH key management:

| Traditional SSH | Tailscale SSH |
|-----------------|---------------|
| Generate key pairs | Automatic |
| Copy public keys to servers | Automatic |
| Manage `~/.ssh/authorized_keys` | Not needed |
| Key rotation | Manual | Automatic |
| Revoke access | Edit files everywhere | Click in admin console |

```bash
# Enable Tailscale SSH on a server
tailscale up --ssh

# Connect (no key setup needed!)
tailscale ssh user@server

# Or just use regular ssh (Tailscale intercepts it)
ssh user@server
```

Authentication uses your Tailscale identity (Google, Microsoft, GitHub, etc.), not SSH keys.

### Funnel

**Funnel** exposes a local service to the public internet with HTTPS:

```bash
# Expose local port 3000 to the internet
tailscale funnel 3000

# Your service is now available at:
# https://your-machine.tail1234.ts.net
```

Unlike ngrok, Funnel:
- Uses your tailnet's domain
- Integrates with your existing Tailscale ACLs
- Provides automatic HTTPS certificates

**Use cases**: Webhook testing, sharing demos, temporary public access.

### Serve

**Serve** is like Funnel but only to your tailnet (not public):

```bash
# Expose to your tailnet only
tailscale serve https / http://localhost:3000

# Show what's being served
tailscale serve status

# Turn off
tailscale serve off
```

Use Serve when you want teammates (but not the world) to access your local dev server.

### HTTPS Certificates

Tailscale provides free HTTPS certificates for your devices via Let's Encrypt:

```bash
# Get a cert for your machine's tailnet hostname
tailscale cert my-laptop.tail1234.ts.net

# Outputs:
#   my-laptop.tail1234.ts.net.crt
#   my-laptop.tail1234.ts.net.key
```

Use these with any web server (nginx, Caddy, etc.) for valid HTTPS within your tailnet.

---

## Comparison Charts

### VPN Solution Comparison

| Solution | Type | NAT Traversal | Key Management | Self-Host | Ease of Use | Performance |
|----------|------|---------------|----------------|-----------|-------------|-------------|
| **Tailscale** | Mesh | Excellent (automatic) | Automatic | Headscale | Very Easy | Excellent |
| **WireGuard** | Point-to-point | Manual | Manual | Native | Hard | Excellent |
| **OpenVPN** | Client-Server | Manual | Manual (PKI) | Native | Medium | Good |
| **ZeroTier** | Mesh | Good | Automatic | ZeroTier One | Easy | Good |
| **Nebula** | Mesh | Good | Manual (certs) | Native | Medium | Excellent |
| **IPsec** | Various | Complex | Manual (IKE) | Native | Hard | Good |
| **Cloudflare WARP** | Client-Server | Automatic | Automatic | No | Very Easy | Good |

### Remote Access Tool Comparison

| Tool | Primary Use Case | Public Endpoints | Private Networks | Auth | Pricing |
|------|------------------|------------------|------------------|------|---------|
| **Tailscale** | Private mesh VPN | Via Funnel | Native | SSO/OAuth | Free tier |
| **[[ngrok]]** | Expose local services | Native (primary) | No | Token | Free tier |
| **Cloudflare Tunnel** | Zero-trust access | Native | Via WARP | Cloudflare Access | Free tier |
| **[[SSH]] Tunneling** | Port forwarding | Manual setup | Manual | SSH keys | Free |
| **Teleport** | Infrastructure access | Via proxy | Native | SSO | Open source + paid |
| **Boundary** | HashiCorp zero-trust | Via workers | Native | OIDC | Open source + paid |

### Feature Comparison

| Feature | Tailscale | WireGuard | ZeroTier | Nebula | OpenVPN |
|---------|-----------|-----------|----------|--------|---------|
| **Auto NAT traversal** | Yes | No | Yes | Partial | No |
| **MagicDNS** | Yes | No | Yes | No | No |
| **File transfer** | Taildrop | No | No | No | No |
| **SSH replacement** | Yes | No | No | No | No |
| **ACLs** | JSON policies | No | Rules | Groups | Plugin |
| **Exit nodes** | Built-in | Manual | Yes | No | Yes |
| **Subnet routing** | Built-in | Manual | Yes | Yes | Yes |
| **Public exposure** | Funnel | No | No | No | No |
| **Mobile apps** | Yes | Yes | Yes | Yes | Yes |
| **Web admin** | Yes | No | Yes | No | No |

---

## CLI Cheatsheet

### Core Commands

```bash
# Connect to your tailnet
tailscale up

# Connect with specific options
tailscale up --authkey=tskey-auth-xxx    # Headless/automated auth
tailscale up --hostname=my-custom-name    # Set device name
tailscale up --shields-up                 # Block incoming connections
tailscale up --reset                      # Reset to default settings

# Disconnect (keeps authenticated)
tailscale down

# Full logout (requires re-auth)
tailscale logout

# Re-authenticate
tailscale login

# Show connection status
tailscale status                          # Human-readable
tailscale status --json                   # JSON for scripting
tailscale status --peers=false            # Only show self

# Show your Tailscale IP
tailscale ip                              # All IPs
tailscale ip -4                           # IPv4 only
tailscale ip -6                           # IPv6 only

# Look up a peer
tailscale whois 100.64.0.5                # Who owns this IP?
tailscale whois my-laptop                 # Info about device
```

### Network Diagnostics

```bash
# Ping a peer (shows direct vs relay path)
tailscale ping my-server                  # Basic ping
tailscale ping --c 10 my-server           # 10 pings
tailscale ping --until-direct my-server   # Wait for direct connection
tailscale ping --tsmp my-server           # Use TSMP instead of ICMP

# Check network conditions
tailscale netcheck                        # NAT type, DERP latency
tailscale netcheck --verbose              # More details

# Debug commands
tailscale debug prefs                     # Show preferences
tailscale debug portmap                   # Port mapping status
tailscale debug derp-map                  # DERP server info
tailscale debug peer-status               # Detailed peer info
tailscale debug local-creds               # Get local API credentials
tailscale debug metrics                   # Prometheus metrics

# Generate bug report
tailscale bugreport                       # Creates shareable report
```

### File Transfer (Taildrop)

```bash
# Send files
tailscale file send myfile.txt laptop           # Send to device
tailscale file send *.pdf laptop                # Send multiple files
tailscale file send largefile.zip laptop:       # Send to default dir

# Receive files
tailscale file get                              # Get pending files
tailscale file get --verbose                    # Show progress
tailscale file get --loop                       # Wait for files (daemon)
tailscale file get --wait=10s                   # Wait with timeout

# Check pending transfers
tailscale file get --peek                       # List without downloading
```

### Exit Nodes

```bash
# Use another device as exit node
tailscale up --exit-node=home-server
tailscale up --exit-node=100.64.0.5
tailscale up --exit-node=                       # Stop using exit node

# Allow LAN access while using exit node
tailscale up --exit-node=home --exit-node-allow-lan-access

# List available exit nodes
tailscale exit-node list

# Advertise yourself as exit node
tailscale up --advertise-exit-node

# Suggest exit node (for mobile apps)
tailscale exit-node suggest
```

### Subnet Routing

```bash
# Advertise routes
tailscale up --advertise-routes=10.0.0.0/24
tailscale up --advertise-routes=10.0.0.0/24,192.168.1.0/24

# Accept advertised routes
tailscale up --accept-routes

# Check route status
tailscale status --json | jq '.Peer[].PrimaryRoutes'
```

### SSH

```bash
# SSH to a device via Tailscale
tailscale ssh user@device                       # Interactive
tailscale ssh user@device "ls -la"              # Run command

# Enable Tailscale SSH server on this device
tailscale up --ssh

# Check SSH status
tailscale ssh --help
```

### Funnel & Serve (Exposing Services)

```bash
# Serve to tailnet only
tailscale serve https / http://localhost:3000       # Proxy to local HTTP
tailscale serve https /api http://localhost:8080    # Specific path
tailscale serve https:8443 / http://localhost:3000  # Custom port
tailscale serve tcp:5432 tcp://localhost:5432       # TCP proxy (e.g., Postgres)

# Show what's being served
tailscale serve status

# Turn off serving
tailscale serve off
tailscale serve off --yes                           # Skip confirmation

# Expose to public internet (Funnel)
tailscale funnel 443 on                             # Enable funnel
tailscale funnel off                                # Disable funnel
tailscale funnel status                             # Check funnel status

# Get HTTPS certificate
tailscale cert my-device.tail1234.ts.net
tailscale cert --cert-file=./server.crt --key-file=./server.key hostname
```

### Advanced & Scripting

```bash
# Multi-account switching
tailscale switch                                    # Interactive
tailscale switch my-work-account                    # Switch to account
tailscale switch --list                             # List accounts

# Version and updates
tailscale version                                   # Show version
tailscale update                                    # Update Tailscale
tailscale update --dry-run                          # Check for updates

# Tailnet lock (device attestation)
tailscale lock status
tailscale lock init
tailscale lock sign NODE_KEY

# Configure interactively
tailscale configure

# DNS configuration
tailscale dns status
tailscale up --accept-dns=false                     # Don't use MagicDNS

# Set tags (requires ACL permission)
tailscale up --advertise-tags=tag:server,tag:prod
```

### Environment Variables

```bash
# Common environment variables
TS_AUTHKEY=tskey-auth-xxx       # Headless authentication
TS_STATE_DIR=/var/lib/tailscale # State directory
TS_SOCKET=/var/run/tailscale/tailscaled.sock  # Socket path
TS_DEBUG_LOG=1                  # Enable debug logging
```

---

## Developer Workflow Use Cases

### Remote Dev Machine Access

Access your home workstation from anywhere:

```bash
# On home workstation (one-time setup)
tailscale up --ssh

# From anywhere in the world
tailscale ssh me@home-workstation

# Or use regular SSH (MagicDNS)
ssh me@home-workstation
```

No VPN client, no port forwarding, no dynamic DNS. Just works.

### Expose Local Dev Server to Team

Share your local development server with teammates:

```bash
# Start your dev server
npm run dev  # Running on localhost:3000

# Expose to your tailnet
tailscale serve https / http://localhost:3000

# Teammates can now access:
# https://your-laptop.tail1234.ts.net
```

Changes are visible immediately. No deployment needed.

### Public Webhook Testing (ngrok Alternative)

Test webhooks from services like Stripe or GitHub:

```bash
# Expose locally with Funnel
tailscale funnel 3000

# Configure webhook URL:
# https://your-laptop.tail1234.ts.net/webhook

# Persistent URL (doesn't change like ngrok free tier)
```

### Multi-Machine Development

Access databases, caches, and services across machines:

```bash
# Your setup:
# - laptop: frontend dev
# - desktop: runs PostgreSQL, Redis
# - server: runs microservices

# From laptop, connect to desktop's Postgres
psql -h desktop -U dev mydb

# Access server's API
curl http://server:8080/api/health
```

No port conflicts, no localhost juggling.

### Kubernetes Cluster Access

Use the Tailscale Kubernetes operator:

```yaml
# Expose your cluster via Tailscale
apiVersion: v1
kind: Service
metadata:
  name: my-service
  annotations:
    tailscale.com/expose: "true"
spec:
  type: ClusterIP
  # ...
```

Access cluster services by name from your laptop, no `kubectl port-forward`.

### CI/CD Self-Hosted Runners

Secure runners without public IPs:

```bash
# On runner machine
tailscale up --authkey=$TS_AUTHKEY --advertise-tags=tag:ci-runner

# Runner can now:
# - Access private resources (databases, registries)
# - Be accessed for debugging
# - No exposed ports or public IPs
```

### VSCode Remote Development

VSCode Remote SSH works seamlessly:

```json
// ~/.ssh/config
Host dev-server
    HostName dev-server  // MagicDNS name
    User me
    // No ProxyCommand or special config needed
```

Connect to any device in your tailnet from VSCode.

---

## Headscale: Self-Hosted Control Plane

**Headscale** is an open-source implementation of Tailscale's control server.

### When to Use Headscale

| Use Case | Hosted Tailscale | Headscale |
|----------|------------------|-----------|
| Quick setup | Best | Requires server |
| Free tier limits | 3 users, 100 devices | Unlimited |
| Air-gapped networks | No | Yes |
| Data sovereignty | Trust Tailscale | Full control |
| SSO integration | Built-in | Manual (OIDC) |
| Support | Official | Community |
| Uptime | 99.9%+ SLA | You manage |

### Quick Setup

```bash
# Docker
docker run -d \
  --name headscale \
  -v /path/to/config:/etc/headscale \
  -v /path/to/data:/var/lib/headscale \
  -p 8080:8080 \
  headscale/headscale:latest \
  serve

# Create a user
docker exec headscale headscale users create myuser

# Generate auth key
docker exec headscale headscale preauthkeys create --user myuser

# Connect clients
tailscale up --login-server=https://headscale.example.com
```

### Headscale Limitations

- No Tailscale SSH (use regular SSH)
- No Funnel (self-host your own ingress)
- No mobile app integration (use CLI)
- ACL syntax differs slightly

**GitHub**: https://github.com/juanfont/headscale

---

## Strengths

- **Zero configuration for users**: Install, authenticate, connected
- **Works through any NAT**: UDP hole punching + DERP fallback
- **Fast**: WireGuard performance (~500 Mbps+ typical)
- **Generous free tier**: 3 users, 100 devices
- **Cross-platform**: Linux, macOS, Windows, iOS, Android, FreeBSD
- **No port forwarding needed**: Works behind strict firewalls
- **MagicDNS**: No more remembering IPs
- **Automatic key rotation**: No manual key management
- **Tailscale SSH**: Eliminates SSH key management
- **Developer-friendly**: Great CLI, good docs, active community
- **Headscale option**: Self-host if needed

---

## Weaknesses

- **Control plane is SaaS**: Coordination requires Tailscale servers (unless Headscale)
- **Funnel bandwidth limits**: Not suitable for high-traffic public services
- **No IPv6-only tailnets**: Still requires IPv4 CGNAT range
- **ACL complexity**: JSON policies can get complex for large orgs
- **Enterprise features gated**: SSO, SCIM, audit logs require paid plans
- **DERP latency**: Relayed connections are slower (but usually <5% of connections)
- **Trust model**: Must trust Tailscale (or self-host Headscale)

---

## Platform Support

| Platform | Installation | Notes |
|----------|--------------|-------|
| **Linux** | apt, dnf, pacman, snap | Kernel WireGuard preferred |
| **macOS** | App Store, Homebrew, .pkg | Uses userspace WireGuard |
| **Windows** | MSI, winget | Runs as service |
| **iOS** | App Store | Full feature support |
| **Android** | Play Store, F-Droid | Full feature support |
| **FreeBSD** | pkg, ports | Community supported |
| **Synology** | Package Center | Native integration |
| **QNAP** | App Center | Native integration |
| **Docker** | Official image | For containerized services |
| **Kubernetes** | Operator | Automatic service exposure |

---

## Pricing (as of 2025)

| Plan | Users | Devices | Key Features | Price |
|------|-------|---------|--------------|-------|
| **Free** | 3 | 100 | All core features, MagicDNS, Taildrop | $0 |
| **Personal Pro** | 1 | Unlimited | Custom domain, more exit nodes | $48/year |
| **Starter** | Up to 3 | Unlimited | Team features, more ACLs | $60/user/year |
| **Premium** | Unlimited | Unlimited | SSO, SCIM, audit logs | Custom |

The free tier is generous enough for most individual developers and small teams.

---

## Related Notes

- [[SSH]] (Tailscale SSH replaces key management)
- [[ngrok]] (Funnel provides similar public exposure, but private-first)
- [[WireGuard]] (underlying VPN protocol)
- [[Docker]] (containerized services on tailnet)
- [[Dev Boards]] (Raspberry Pi, Jetson remote access)
- [[Kubernetes]] (Tailscale operator for cluster access)

---

## External Resources

- **Website**: https://tailscale.com/
- **Documentation**: https://tailscale.com/kb/
- **GitHub (client)**: https://github.com/tailscale/tailscale
- **Headscale**: https://github.com/juanfont/headscale
- **Blog**: https://tailscale.com/blog/
- **Discord**: https://discord.gg/tailscale

---

## Summary

Tailscale is the most accessible way to create secure private networks. By building on WireGuard and automating all the hard parts (NAT traversal, key management, DNS), it turns mesh VPN into something anyone can use. For developers, the killer features are MagicDNS (just `ssh laptop`), Tailscale SSH (no more key management), and Serve/Funnel (share local services instantly). The free tier covers most individual and small team needs. If you need full control, Headscale provides a self-hosted alternative.

---
