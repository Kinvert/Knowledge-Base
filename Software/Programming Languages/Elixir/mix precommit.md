# mix precommit

The `mix precommit` task (usually provided through custom Mix aliases or community tools) acts as an automated quality gate before committing code. It bundles multiple checks such as compilation, formatting, dependency hygiene, and testing into a single command. This note outlines how precommit hooks function in the Elixir ecosystem and how they can integrate with engineering workflows—including RL tooling pipelines where correctness and consistency matter.

---

## ⚙️ Overview

A `mix precommit` workflow groups sanity checks into one command typically triggered manually, via Git hooks, or through CI. It ensures consistent formatting, prevents unused dependencies, catches compilation warnings, and runs tests before changes are committed. Precommit steps can drastically reduce runtime errors and failing pipelines downstream.

---

## 🧠 Core Concepts

- **Mix Alias** — A custom command defined in `mix.exs` that chains multiple Mix tasks.
- **Git Hooks** — Using `pre-commit` or `pre-push` scripts to enforce running Mix tasks locally.
- **Static Guarantees** — Running `compile --warnings-as-errors` ensures no warnings are tolerated.
- **Dependency Cleanup** — `deps.unlock --unused` removes lock entries for unused packages.
- **Consistency** — `mix format` ensures consistent code style.
- **Validation** — `mix test` guards correctness before a commit.

---

## 🧩 Example Task Steps (Recommended Defaults)

A common and highly effective configuration includes:

- `compile --warnings-as-errors` — Fail on any compiler warnings
- `deps.unlock --unused` — Remove unused dependencies from the lockfile
- `format` — Ensure code formatting matches project standards
- `test` — Run the test suite before allowing a commit

---

## 🔍 Comparison Chart

| Tool / Approach | Purpose | Strengths | Weaknesses |
|----------------|----------|-----------|------------|
| `mix precommit` (alias) | Local offline checks | Fast, customizable, no extra tooling | Not enforced unless hooked into Git |
| `lefthook` | Git hooks manager | Cross-language, fast, modern | Extra config overhead |
| `pre-commit` (Python tool) | Multi-language repo management | Rich ecosystem | Requires Python environment |
| CI pipelines | Server-side enforcement | Centralized, consistent for all devs | Slow feedback, requires infra |
| VSCode / Editor Tasks | On-save checks | Immediate feedback | Not standardized across team |

---

## 🧠 Additional Options for `mix precommit`

You can expand the alias with optional steps depending on your needs:

- `credo --strict` — Run static analysis
- `dialyzer` — Perform type checking
- `hex.outdated` — Show outdated dependencies
- `docs` — Build documentation before committing
- `format --check-formatted` — Validate formatting without writing
- `test --include integration` — Run extended test suite
- `xref deprecated` — Check for deprecated calls
- `xref unreachable` — Detect unreachable code
- `compile --force` — Ensure full recompilation for safety
- `coveralls.html` — Generate coverage reports
- `sobelow --exit` — Security linting for Phoenix apps
- `cmd "npm run build"` — For full-stack setups
- `ash.formatter` — For Ash DSL formatting  
- `ash.verify` — Validate Ash resource configurations

---

## 🧰 Developer Tools

- Git hooks in `.git/hooks/pre-commit`
- `mix alias` in `mix.exs`
- `lefthook.yml` for advanced hook management
- `asdf` to manage consistent Elixir/Erlang versions
- VSCode “Run on Save” actions for early catching

---

## 🛠️ How It Works

- You define a Mix alias in `mix.exs` under `def project do`.
- The alias maps `precommit` to a list of Mix tasks.
- When calling `mix precommit`, tasks run in sequence and the first failure aborts the chain.
- If used as a Git hook, the commit is canceled until the task passes.

---

## 📚 Use Cases

- Prevent breaking changes from reaching CI
- Maintain strict formatting and code hygiene
- Enforce dependency cleanliness in long-lived RL research repos
- Prevent unused deps and reduce lockfile churn
- Ensure Ash Framework APIs or resource definitions stay consistent

---

## 💪 Strengths

- Fast, local, automated
- Easy to extend
- Zero extra infrastructure required
- Reduces CI load and failures
- Encourages healthy engineering discipline

---

## ⚠️ Weaknesses

- Cannot guarantee everyone runs it unless enforced via Git hooks
- May slow down commits on large projects (e.g., heavy test suites)
- Requires maintenance as tasks evolve

---

## 👥 Compatible Items

- Git Hooks (`pre-commit`, `pre-push`)
- Phoenix projects
- Ash Framework (via `ash.verify`, `ash.formatter`)
- Nx-based projects
- Rebar3 (indirectly, if mixed BEAM tooling is used)

---

## 🔗 Related Concepts / Notes

- [[mix]] (Elixir’s build tool)
- [[Elixir]]
- [[CI-CD]] (Continuous Integration / Continuous Deployment)
- [[Static Analysis]] (General QA tools)
- [[Dialyzer]] (Type-level analysis)
- [[Ash Framework]] (For custom verifiers)
- [[Phoenix]] (Often paired with formatting + security checks)

---

## 🌐 External Resources

- HexDocs: Mix Tasks
- Git documentation on hooks
- Credo documentation
- Dialyxir (Dialyzer for Mix projects)
- Sobelow (Phoenix security tool)

---

## 🏁 Summary

`mix precommit` is a customizable workflow that enforces code quality by bundling together essential static checks, formatting, dependency hygiene, and tests. With strong integration into Mix and optional Git hook automation, it helps ensure that only high-quality, warning-free, dependency-clean code reaches the repository.
