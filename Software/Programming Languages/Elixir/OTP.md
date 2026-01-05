# OTP (Open Telecom Platform)

**OTP** is a set of libraries, design principles, and tools bundled with [[Erlang]] that provide building blocks for concurrent, fault-tolerant, distributed systems. Despite the name, OTP is not telecom-specific—it's a general framework for building reliable applications. [[Elixir]] builds directly on OTP, making these patterns first-class citizens.

---

## 📚 Overview

OTP emerged from Ericsson's work on telephone switches in the 1980s-90s, where systems needed to run continuously with minimal downtime. The patterns that evolved—supervision trees, generic servers, state machines—became formalized as OTP.

Key highlights:
- Battle-tested in telecom (99.9999999% uptime claims)
- "Let it crash" philosophy with automatic recovery
- Supervision trees for fault isolation
- Hot code upgrades without stopping the system
- Built into [[BEAM]] VM

---

## 🧠 Core Behaviours

OTP defines standard "behaviours" (interfaces) that abstract common patterns:

- **GenServer**
  Generic server process for stateful request/response. See [[Elixir GenServer]]

- **Supervisor**
  Monitors child processes and restarts them according to strategy

- **Application**
  Bundles modules into a startable/stoppable unit with dependencies

- **GenStateMachine / gen_statem**
  Finite state machine with event handling

- **GenEvent** (deprecated)
  Event manager pattern, replaced by Registry + custom solutions

- **Task**
  Simple async computation. See [[Task.async_stream]]

- **Agent**
  Simple state wrapper around a process

---

## 📊 Comparison Chart

| Pattern | OTP Behaviour | Use Case | Alternatives |
|---------|---------------|----------|--------------|
| Stateful server | GenServer | Caching, sessions, coordination | Agent (simpler), ETS (faster reads) |
| Process monitoring | Supervisor | Fault tolerance, restart strategies | Manual Process.monitor |
| Async work | Task | One-off computations | spawn + receive |
| State machine | gen_statem | Protocols, workflows | GenServer with case statements |
| Pub/sub | Registry | Broadcasting events | Phoenix.PubSub, pg |
| Background jobs | None (use libs) | Queued work | [[Oban]], Exq |

---

## 🌳 Supervision Trees

The core OTP pattern: processes are organized in trees where parents supervise children.

```
Application
    └── Supervisor (top-level)
            ├── Supervisor (subsystem A)
            │       ├── GenServer (worker)
            │       └── GenServer (worker)
            └── Supervisor (subsystem B)
                    └── GenServer (worker)
```

**Restart Strategies:**

| Strategy | Behavior |
|----------|----------|
| `:one_for_one` | Restart only the crashed child |
| `:one_for_all` | Restart all children if one crashes |
| `:rest_for_one` | Restart crashed child and all started after it |

---

## 🔧 Supervisor Example

```elixir
defmodule MyApp.Supervisor do
  use Supervisor

  def start_link(opts) do
    Supervisor.start_link(__MODULE__, opts, name: __MODULE__)
  end

  def init(_opts) do
    children = [
      {MyApp.Cache, []},
      {MyApp.Worker, []}
    ]

    Supervisor.init(children, strategy: :one_for_one)
  end
end
```

---

## 🔧 Application Example

```elixir
# In mix.exs
def application do
  [
    mod: {MyApp.Application, []},
    extra_applications: [:logger]
  ]
end

# lib/my_app/application.ex
defmodule MyApp.Application do
  use Application

  def start(_type, _args) do
    children = [
      MyApp.Repo,
      MyApp.Supervisor
    ]

    opts = [strategy: :one_for_one, name: MyApp.Supervisor]
    Supervisor.start_link(children, opts)
  end
end
```

---

## 💡 "Let It Crash" Philosophy

OTP embraces failure as normal:

1. **Isolate failure** - Each process has its own memory; a crash doesn't corrupt others
2. **Detect failure** - Supervisors are notified immediately when children die
3. **Recover automatically** - Restart with known-good initial state
4. **Log and continue** - Don't try to handle every edge case defensively

This leads to simpler code—instead of handling every possible error, let the process crash and restart clean.

---

## 🔧 Use Cases

- Web application backends ([[Phoenix Framework]])
- Real-time systems (chat, notifications, live updates)
- IoT device management and coordination
- Background job processing ([[Oban]])
- Distributed systems and clustering
- Embedded systems ([[Nerves]])
- Game servers and multiplayer backends

---

## ✅ Pros

- Proven fault-tolerance patterns
- Automatic recovery from failures
- Process isolation prevents cascading failures
- Hot code upgrades possible
- Excellent for long-running systems
- Built into the VM, not bolted on

---

## ❌ Cons

- Learning curve for supervision design
- Overhead for simple scripts (overkill)
- Debugging distributed supervision trees can be complex
- Some patterns feel heavyweight for small tasks

---

## 🔩 Compatible Items

- [[BEAM]] / [[BEAM VM]] - Runtime
- [[Elixir]] - Primary modern interface
- [[Erlang]] - Original language
- [[Phoenix Framework]] - Web framework built on OTP
- [[Nerves]] - Embedded OTP
- [[Oban]] - Job processing
- [[ETS]] - In-memory storage
- [[Livebook]] - Interactive notebooks

---

## 🔗 Related Concepts

- [[Elixir GenServer]] (Core OTP behaviour)
- [[Elixir Concurrency]] (Process model)
- [[BEAM]] (Virtual machine)
- [[Erlang]] (OTP's origin)
- [[Actor Model]] (Theoretical foundation)
- [[Distributed Systems]] (OTP's strength)

---

## 📚 External Resources

- [Elixir Getting Started - OTP](https://elixir-lang.org/getting-started/mix-otp/introduction-to-mix.html)
- [Erlang OTP Design Principles](https://www.erlang.org/doc/design_principles/users_guide.html)
- [Learn You Some Erlang - OTP](https://learnyousomeerlang.com/what-is-otp)
- [The Little Elixir & OTP Guidebook](https://www.manning.com/books/the-little-elixir-and-otp-guidebook)
