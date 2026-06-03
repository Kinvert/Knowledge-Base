---
aliases:
  - ReqLLM
  - req_llm
tags:
  - elixir
  - llm
  - ai
title: ReqLLM
---

# ReqLLM 🤖

**ReqLLM** is an Elixir library that provides a unified, provider-agnostic interface for interacting with Large Language Models. Built on top of the [[Req]] HTTP client, it offers a consistent API for [[Anthropic]], [[OpenAI]], [[Google Gemini]], [[Groq]], [[OpenRouter]], [[Ollama]], and other providers, with first-class support for streaming, tool use, structured outputs, and usage tracking.

---

## 📦 Overview

ReqLLM treats LLM calls as ordinary [[Req]] requests, layering provider-specific plugins on top of a single core. Instead of juggling multiple SDKs with different conventions (one per provider), you build a `ReqLLM.Context` of messages and dispatch with `ReqLLM.generate_text/3` or `ReqLLM.stream_text/3`. The library normalizes responses, token counts, finish reasons, and tool-call schemas across providers — so swapping models is usually a one-line change.

It is part of the broader [[Elixir]] AI ecosystem alongside [[Bumblebee]] (local inference), [[Nx]] (tensors), and [[Axon]] (neural nets), but ReqLLM specifically targets *hosted* LLM APIs.

---

## 🧩 Core Concepts

- **Model Strings:** Compact identifiers like `"anthropic:claude-sonnet-4-5"` or `"openai:gpt-4o-mini"` route to the correct provider plugin.
- **Context:** A struct holding the system prompt and message history. Built with `ReqLLM.Context.new/1` or piped helpers.
- **Provider Plugins:** Each provider (Anthropic, OpenAI, Google, Groq, etc.) is a [[Req]] plugin that adapts the unified request shape to that vendor's API.
- **Generation API:** `generate_text/3`, `stream_text/3`, `generate_object/4`, `embed/3` — the four main entry points.
- **Structured Output:** `generate_object/4` accepts an [[Ecto]]-style schema or JSON Schema and returns a validated struct.
- **Tool Use:** Define tools as Elixir functions; ReqLLM handles the tool-call/tool-result round trip.
- **Streaming:** Token-by-token streaming via Elixir Streams or callback functions.
- **Usage Tracking:** Standardized token counts and cost estimates regardless of provider.
- **Keys via Config:** API keys are pulled from `Application.get_env/2` or environment variables; no per-call boilerplate.

---

## 📊 Comparison Chart

| Feature | ReqLLM (Elixir) | LangChain Elixir | instructor_ex | LiteLLM (Python) | Vercel AI SDK (JS/TS) | Direct Provider SDK |
|---------|-----------------|-----------------|---------------|------------------|----------------------|---------------------|
| Language | Elixir | Elixir | Elixir | Python | TS/JS | Varies |
| Multi-provider | ✅ Many | ✅ | ❌ (via others) | ✅ Most | ✅ Many | ❌ |
| Streaming | ✅ | ✅ | Limited | ✅ | ✅ | ✅ |
| Tool calling | ✅ Unified | ✅ | ✅ | ✅ | ✅ | ✅ |
| Structured output | ✅ Schema-based | Partial | ✅ (its focus) | Partial | ✅ | Provider-dependent |
| HTTP foundation | [[Req]] | Tesla/Req | Req | httpx | fetch | Various |
| Local model support | Via [[Ollama]] | Via Ollama | Limited | ✅ | Limited | N/A |
| Idiomatic to ecosystem | ✅ Very | ✅ | ✅ | ✅ | ✅ | ✅ |

---

## 🏆 Use Cases

- Phoenix and [[LiveView]] apps that integrate chat or generative features
- Switching between providers (cost optimization, fallback, A/B testing)
- Background LLM jobs orchestrated via [[Oban]]
- Agent workflows that need tool use against Elixir functions
- RAG pipelines paired with [[pgvector]] or [[Ecto]]-backed retrieval
- Structured data extraction from unstructured text
- [[Livebook]] notebooks experimenting with prompts and models
- Building [[Ash AI]]-style or [[Ash Framework]] LLM integrations

---

## ✅ Strengths

- Single API for many providers — easy model swaps
- Built on [[Req]], so you inherit retries, plugins, and Finch pooling
- Excellent fit for [[OTP]] supervision and concurrent calls
- Streaming integrates naturally with Elixir's Stream module and [[LiveView]]
- Schema-driven structured outputs play well with [[Ecto]] changesets
- Lightweight: just HTTP, no heavy abstractions
- Composable with other [[Req]] middleware (logging, tracing, caching)

---

## ❌ Weaknesses

- Hosted-only — does not run local models itself (delegates to [[Ollama]])
- Provider feature parity varies; some advanced features need provider-specific options
- Newer than equivalents in [[Python]]/[[JavaScript]] ecosystems, smaller community
- Cost/usage tracking accuracy depends on provider response fidelity
- Tool-calling JSON schema variations across providers can leak through

---

## 🔧 Supported Providers (Typical)

- [[Anthropic]] Claude
- [[OpenAI]] (GPT-4o, GPT-5, etc.)
- [[Google Gemini]]
- [[Groq]]
- [[OpenRouter]] (proxy for many models)
- [[Ollama]] (local inference)
- xAI Grok
- Cerebras
- Mistral
- DeepSeek

---

## 🛠️ Code Snippets

### Basic generation

```elixir
{:ok, response} =
  ReqLLM.generate_text(
    "anthropic:claude-sonnet-4-5",
    "Summarize the BEAM scheduler in two sentences."
  )

response.text
```

### Streaming

```elixir
ReqLLM.stream_text!("openai:gpt-4o-mini", "Tell me a story about Elixir.")
|> Stream.each(&IO.write/1)
|> Stream.run()
```

### Structured output

```elixir
schema = [
  name: [type: :string, required: true],
  port: [type: :integer, required: true]
]

{:ok, %{object: cfg}} =
  ReqLLM.generate_object(
    "anthropic:claude-sonnet-4-5",
    "Give me a config for a Phoenix server.",
    schema
  )
```

### Tool use

```elixir
tool =
  ReqLLM.tool(
    name: "get_weather",
    description: "Look up current weather",
    parameter_schema: [city: [type: :string, required: true]],
    callback: fn %{city: c} -> {:ok, "72°F in #{c}"} end
  )

{:ok, response} =
  ReqLLM.generate_text(
    "anthropic:claude-sonnet-4-5",
    "What's the weather in Cleveland?",
    tools: [tool]
  )
```

---

## 📚 Related Concepts / Notes

- [[Req]] (HTTP client ReqLLM is built on)
- [[Elixir]] (Host language)
- [[Phoenix Framework]] (Common integration target)
- [[LiveView]] (Streaming LLM responses to the browser)
- [[Oban]] (Background LLM job processing)
- [[Bumblebee]] (Local Elixir inference, complementary)
- [[Ash AI]] (Higher-level Ash framework AI integrations)
- [[Ecto]] (Schema validation for structured outputs)
- [[Anthropic]] (Major supported provider)
- [[OpenAI]] (Major supported provider)
- [[Ollama]] (Local model gateway)
- [[Livebook]] (Common experimentation environment)

---

## 🌐 External Resources

- Hex package: https://hex.pm/packages/req_llm
- HexDocs: https://hexdocs.pm/req_llm
- GitHub repository: https://github.com/agentjido/req_llm
- [[Req]] docs for underlying request behavior
- Elixir Forum threads on LLM integrations

---

## 🔑 Key Highlights

- Provider-agnostic LLM client for [[Elixir]]
- Built on [[Req]] — composable, testable, idiomatic
- Streaming, tools, and structured outputs all unified
- Pairs naturally with [[Phoenix Framework]], [[Oban]], and [[LiveView]]
- The pragmatic choice for hosted-LLM features in BEAM apps
