# Spec Kit

Spec Kit is an open-source toolkit from GitHub that structures how developers work with AI coding agents. Instead of ad-hoc prompting, it provides a four-phase workflow where specifications become "living, executable artifacts" that guide AI code generation. Compatible with [[Claude Code]], GitHub Copilot, and Gemini CLI.

**Repository:** [github.com/github/spec-kit](https://github.com/github/spec-kit)

---

## 🎯 Core Concept

Traditional AI coding workflow: vague prompt → hope for good output → iterate endlessly.

Spec Kit workflow: structured specification → technical plan → discrete tasks → focused implementation.

The key insight is that LLMs excel at pattern completion but struggle with ambiguous requirements. Clear specifications eliminate guesswork by encoding requirements, constraints, and design decisions upfront before any code is generated.

---

## 🔄 Four-Phase Methodology

### 1. Specify

Define what you're building from the user's perspective:
- User journeys and experiences
- Problem statements and desired outcomes
- Success criteria
- Creates a "living artifact" that evolves with the project

### 2. Plan

Generate technical implementation strategy:
- Technology stack and architecture decisions
- Constraints (security, compliance, performance)
- Integration requirements
- Addresses real-world constraints before coding begins

### 3. Tasks

Break specification into actionable work items:
- Small, reviewable chunks
- Each task implementable and testable in isolation
- Example: "create user registration endpoint validating email format"

### 4. Implement

AI executes tasks sequentially:
- Focused reviews per task instead of reviewing large code blocks
- Developer verifies at explicit checkpoints
- Iterate and refine as needed

---

## ⌨️ Commands

| Command | Purpose |
|---------|---------|
| `/specify` | Generate full specification from high-level description |
| `/plan` | Create technical implementation strategy from spec |
| `/tasks` | Break down spec and plan into actionable tasks |

---

## 🛠️ Installation

```bash
uvx --from git+https://github.com/github/spec-kit.git specify init <PROJECT_NAME>
```

---

## 📊 Comparison Chart

| Approach | Structure | AI Guidance | Iteration Style | Best For |
|----------|-----------|-------------|-----------------|----------|
| **Spec Kit** | Four phases | High (specs + plans) | Task-by-task review | Greenfield, features, modernization |
| **Ad-hoc Prompting** | None | Low (vague prompts) | Trial and error | Quick prototypes |
| **CLAUDE.md Files** | Project context | Medium (conventions) | Conversation-based | Ongoing projects |
| **Custom Skills** | Workflow templates | Medium (domain knowledge) | Skill-specific | Repeated workflows |
| **[[TDD]]** | Test-first | None (tests guide human) | Red-green-refactor | Correctness focus |

---

## 💡 Use Cases

**Greenfield Projects**
- Ensures AI builds what you intend, not generic patterns
- Captures requirements before any code exists

**Feature Development**
- Maintains architectural consistency with existing codebase
- Prevents bolted-on additions that don't fit

**Legacy Modernization**
- Captures essential business logic first
- Rebuilds with fresh architecture while preserving behavior

---

## 🧑‍💻 Developer Role Shift

With Spec Kit, developers shift from writing code to:
- Steering AI agent direction
- Verifying outputs at checkpoints
- Critiquing generated artifacts for gaps and edge cases
- Iterating specifications based on learnings

The article describes this as moving from "code is the source of truth" to "intent is the source of truth."

---

## ✅ Why This Works

LLMs are described as "literal-minded pair programmers" - they follow instructions precisely but can't infer unstated requirements. Spec Kit addresses this by:

- Encoding security, compliance, and design requirements upfront
- Separating stable "what" (specification) from flexible "how" (implementation)
- Providing unambiguous instructions at each phase
- Creating reviewable checkpoints throughout the process

---

## 🔗 Related Concepts

- [[Claude Code]] - Compatible AI coding CLI
- [[TDD]] (Test-Driven Development) - Complementary methodology
- [[CI-CD]] (Continuous Integration / Continuous Delivery)
- [[Agile]]
- [[MCP]] (Model Context Protocol)

---

## 📚 External Resources

- [GitHub Blog: Spec-driven development with AI](https://github.blog/ai-and-ml/generative-ai/spec-driven-development-with-ai-get-started-with-a-new-open-source-toolkit/)
- [Spec Kit Repository](https://github.com/github/spec-kit)
