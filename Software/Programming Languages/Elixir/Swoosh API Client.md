# Swoosh API Client ✉️

The **Swoosh API Client** is part of the Elixir Swoosh ecosystem, providing a unified interface for sending emails through various third-party providers. It abstracts away provider-specific HTTP details and gives developers a consistent, testable, and extensible way to send transactional or notification emails in Elixir applications, especially in conjunction with Phoenix.

---

## 📦 Overview
Swoosh is a flexible Elixir email library, and the Swoosh **API client** specifically handles sending emails over HTTP APIs rather than SMTP. Many modern providers—like SendGrid, Mailgun, Amazon SES, and others—recommend or require API-based delivery for performance, reliability, and analytics. Swoosh's API client architecture ensures each provider implements a consistent behaviour module so applications don’t need to manually handle HTTP, authentication, or JSON encoding.

---

## 🧩 Core Concepts
- **Deliveries via Providers:** Each API provider (SendGrid, Mailgun, Postmark, SparkPost, SES, etc.) implements Swoosh’s API behaviour.
- **Transport Configuration:** In `config/*.exs`, you specify provider credentials and options.
- **Email Struct:** Build emails using `Swoosh.Email`, then deliver via `Swoosh.ApiClient` or provider modules.
- **JSON Encoding:** API clients typically send JSON payloads; Swoosh handles encoding automatically.
- **Retries & Error Handling:** Providers normalize HTTP errors into consistent Swoosh error tuples.
- **Test Adapters:** Use the `Swoosh.Adapters.Test` adapter with `mix test` for safe, no-delivery testing.

---

## 📊 Comparison Chart

| Feature | API Client (Swoosh) | SMTP (Swoosh SMTP) | Bamboo (Alternative Library) | Direct Provider SDK (e.g., sendgrid-elixir) |
|--------|----------------------|---------------------|------------------------------|---------------------------------------------|
| Transport | HTTP API | SMTP protocol | Varies | HTTP API |
| Speed | Fast, async-friendly | Slower handshake | Moderate | Fast |
| Setup Complexity | Moderate | Simple | Moderate | Varies |
| Observability | Strong (provider web dashboards) | Weak | Moderate | Strong |
| Provider Flexibility | High | Medium | Medium | Tied to one provider |
| Reliability | High | Medium | High | High |

---

## 🏆 Use Cases
- Sending transactional emails from Phoenix applications
- Robotics dashboards or monitoring systems that notify operators via email
- Sending alerts from Elixir-based backend services
- API-based delivery where SMTP is blocked or undesirable
- Integrating multiple providers for redundancy or fallback strategies

---

## ✅ Strengths
- Consistent API across all providers
- Easy to test using built-in test adapters
- Supports many major providers out of the box
- Faster and more reliable than SMTP for cloud environments
- Rich provider features (templates, analytics, metadata)

---

## ❌ Weaknesses
- Requires setting up provider API keys
- Each provider may require slightly different fields
- Can be harder to debug than raw SMTP if misconfigured
- Depends on HTTP client behavior (Finch, Hackney, etc.)

---

## 🔧 Variants / Providers Supported
- Swoosh.Adapters.Sendgrid
- Swoosh.Adapters.Mailgun
- Swoosh.Adapters.Postmark
- Swoosh.Adapters.AmazonSES
- Swoosh.Adapters.SparkPost
- Swoosh.Adapters.Mandrill
- Swoosh.Adapters.SMTP (non-API)
- Swoosh.Adapters.Test (for `mix test`)

---

## 📚 Related Concepts / Notes
- [[Elixir]] (Language Swoosh is built for)
- [[Phoenix]] (Often paired with Swoosh for web apps)
- [[Mix]] (Used for running tests like `mix test`)
- [[ExUnit]] (Testing emails with Test Adapter)
- [[REST API]] (Underlying protocol used by API email providers)
- [[JSON]] (Common payload format for API email requests)

---

## 🛠️ Compatible Items
- Finch (recommended HTTP client)
- Hackney (older HTTP client option)
- Phoenix LiveView (email-triggered workflows)
- Elixir releases (avoids SMTP port issues)
- Dockerized environments that block outbound SMTP

---

## 🏗️ Developer Tools
- Swoosh debug logs
- Provider dashboards (SendGrid/Mailgun/etc.)
- Hex packages for optional plugins or adapters
- Email preview tools (e.g., `maildev` if using SMTP locally)

---

## 📖 Documentation and Support
- Official Swoosh Docs: https://hexdocs.pm/swoosh
- Hex package: https://hex.pm/packages/swoosh
- Provider API documentation (Mailgun, SendGrid, SES, etc.)
- Phoenix Guides for email sending

---

## 🌐 External Resources
- Swoosh GitHub repository
- Elixir Forum discussions on Swoosh
- Tutorials for Phoenix email flows
- Example repos using Swoosh API clients

---

## 🔑 Key Highlights
- Simple, unified interface for multiple email providers  
- Preferable to SMTP in most modern cloud environments  
- Integrates tightly with Phoenix and ExUnit  
- Supports templates, attachments, tags, metadata, and more  

---

## 🧪 Capabilities
- Send rich HTML/text emails
- Add attachments and inline images
- Use provider templating & personalization APIs
- Add metadata for analytics or categorization
- Plug-and-play provider switching

---

## 📚 Further Reading
- [[Phoenix]] for integrating email into web actions
- [[REST API]] to understand underlying HTTP interactions
- [[Mix]] for email testing flows
- [[ExUnit]] for verifying email content in tests
