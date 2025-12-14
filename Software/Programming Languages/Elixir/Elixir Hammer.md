# Elixir Hammer

**Elixir Hammer** is a flexible rate-limiting library for Elixir applications, commonly used in Phoenix or Plug-based web applications. It allows developers to protect endpoints, APIs, or system resources from excessive requests using in-memory, ETS, or Redis-backed counters. Hammer is widely applied for throttling LiveView events, PubSub messages, RL telemetry endpoints, and external APIs.

---

## 📚 Overview

Hammer is designed to be **backend-agnostic** and highly configurable. At its core, it allows developers to track requests or events over time windows and limit actions per user, IP, or key. It is modular and integrates naturally with Phoenix, Plug, and other Elixir frameworks.

Key capabilities:
- [[ETS]], [[Redis]], or in-memory storage
- Configurable rate-limiting strategies
- Time-window or rolling-window counters
- Integration with Plugs (`Hammer.Plug`) for request throttling
- Atomic operations to avoid race conditions
- Support for multiple thresholds and scopes

---

## 🧠 Core Concepts

- **Bucket / Key**: Unique identifier for rate-limiting (e.g., user ID, IP)
- **Threshold**: Maximum allowed events per window
- **Window**: Time period in milliseconds for limiting
- **Backend**: Storage mechanism (`:ets`, `:redis`, `:con_cache`)
- **Plug Integration**: Middleware for HTTP request limiting
- **Atomic Increment**: Ensures counters are consistent across concurrent processes
- **Scoped Limiting**: Different limits per endpoint or user role

---

## 🔧 How It Works

1. **Configure Backend**:
   - ETS for lightweight, in-memory use
   - Redis for distributed rate-limiting across nodes
2. **Set Up Limits**:
   - Define key, threshold, and time window
   - Optionally use multiple buckets per resource
3. **Increment & Check**:
   - Each request or event increments the bucket counter
   - Hammer checks if the threshold is exceeded
4. **Plug Integration**:
   - `Hammer.Plug` intercepts HTTP requests
   - Returns `429 Too Many Requests` if limit exceeded
5. **Cleanup / Expiry**:
   - Buckets expire automatically after their time window
   - Supports both fixed and rolling windows

Example workflow:
- User hits an API endpoint
- Hammer.Plug increments the counter
- If the limit is exceeded, the request is blocked
- Counter automatically resets after the configured window

---

## 🛠️ Strengths

- High-performance counters using ETS or Redis
- Easily integrated into Phoenix and Plug pipelines
- Supports distributed, multi-node systems
- Flexible configuration per endpoint or user
- Provides both simple and advanced rate-limiting strategies
- Atomic operations prevent race conditions

---

## ⚠️ Weaknesses

- ETS backend is node-local, not distributed
- Redis backend introduces network latency
- Misconfigured thresholds can block legitimate traffic
- Fine-grained per-user rate-limiting may require additional keys and management
- Rolling window counters are slightly more complex to configure

---

## 📊 Comparison Chart

| Library / Tool | Storage | Distributed | Integration | Notes |
|----------------|---------|------------|------------|------|
| **Hammer** | ETS / Redis | Optional | Plug / Phoenix | Flexible Elixir rate-limiting |
| ExRated | ETS | No | Plug | Simpler, single-node |
| PlugAttack | ETS / Redis | Optional | Plug | Similar functionality, less maintained |
| RateLimiter | Redis | Yes | Phoenix / Plug | Distributed focused |
| Throttle | In-memory | No | Plug | Lightweight, simple |
| Rack::Attack (Ruby) | Redis | Optional | Rails | Similar concept in Ruby |

---

## 🧩 Use Cases

- Limiting API requests per user or IP
- Protecting LiveView events from spam
- Controlling PubSub message flood
- Throttling RL telemetry endpoints
- Preventing abusive login attempts
- Distributed rate-limiting in multi-node clusters

---

## 🛠️ Developer Tools

- `Hammer.Plug` for Plug/Phoenix middleware
- `Hammer.check_rate/4` for manual rate checks
- Redis or ETS backend configuration
- Telemetry events for monitoring limits
- Supervisor trees to manage Redis connections

---

## 📑 Documentation and Support

- HexDocs: https://hexdocs.pm/hammer
- GitHub repo: https://github.com/ExHammer/hammer
- Plug integration examples
- Phoenix and LiveView guides
- Community discussions on distributed rate-limiting

---

## 🔗 Related Concepts / Notes

- [[Elixir]]
- [[Phoenix]]
- [[Plug]]
- [[LiveView]]
- [[PubSub]]
- [[API]]
- [[ETS]]
- [[Redis]]
- [[Telemetry]]
- [[Rate Limiting]]

---

## 📝 Summary

Elixir Hammer is a versatile rate-limiting library designed for both local and distributed applications. Its Plug integration makes it ideal for protecting web endpoints, LiveView events, and PubSub channels. With ETS or Redis backends, developers can configure atomic, scoped, and rolling-window limits while maintaining BEAM’s concurrency model. Proper configuration ensures fair throttling without compromising system performance or user experience.
