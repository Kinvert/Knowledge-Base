---
title: Agentic Coding Loops
tags: [ai, llm, agents, software-engineering, coding-agents]
aliases: [Agentic Coding, ReAct Loops, LLM Coding Agents, Agent Loops]
---

# Agentic Coding Loops

**Agentic coding loops** are workflows where an [[LLM]] repeatedly observes a software environment, reasons about the next step, takes tool actions, and evaluates the result. They are the core pattern behind coding agents such as [[Claude Code]], SWE-agent, GitHub Copilot coding agent, and terminal tools like aider.

---

## 🧠 Summary

- The agent is not just generating a code snippet; it is operating in a loop over a real codebase.
- Common actions include searching files, reading code, editing files, running tests, inspecting errors, and iterating.
- The loop is often described as **observe → reason/plan → act → observe**, with human review at checkpoints.
- [[ReAct]] (Reasoning + Acting) is a broad prompting pattern for interleaving reasoning traces and tool/environment actions.
- Coding agents adapt ReAct-style loops to software engineering by making the repository, shell, test suite, issue tracker, and git history part of the environment.

---

## 🔄 Core Loop

| Phase | Coding-Agent Meaning | Examples |
|-------|----------------------|----------|
| Observe | Gather state from the repo or tools | Read files, inspect errors, search symbols |
| Reason / Plan | Decide what likely matters next | Form hypothesis, choose files, split work |
| Act | Change or query the environment | Edit code, run command, call API, spawn subagent |
| Evaluate | Check whether the action helped | Run tests, lint, compare diff, read failure output |
| Iterate | Continue until the task is done or blocked | Fix regressions, narrow scope, ask human |

---

## ⚙️ ReAct vs Coding Agents

**ReAct** is the general LLM pattern: the model interleaves reasoning and actions so it can update its plan from external observations. In the original ReAct paper, actions might query Wikipedia or interact with benchmark environments.

**Agentic coding** is a domain-specific application of that idea. The action space is software-engineering work: filesystem reads/writes, shell commands, test execution, dependency inspection, git operations, browser automation, or calls through [[MCP]].

---

## 📊 Comparison Chart

| Pattern | Environment | Actions | Best Fit | Main Risk |
|---------|-------------|---------|----------|-----------|
| Chat Completion | Prompt only | Text response | Explanations, snippets | No direct verification |
| Chain-of-Thought | Prompt only | Internal reasoning | Math/planning prompts | Can reason without grounding |
| ReAct | Tools or environment | Search, lookup, interact | Multi-step grounded tasks | Bad tool choice compounds errors |
| Agentic RAG | Knowledge bases + tools | Retrieve, rerank, verify | Enterprise Q&A and research | Retrieval/debug complexity |
| Agentic Coding Loop | Repo + shell + tests | Read, edit, run, diff | Bug fixes, features, refactors | Unsafe edits or false test confidence |
| Multi-Agent Coding | Shared repo/worktrees | Delegate, review, integrate | Large parallel tasks | Coordination and merge conflicts |

---

## 🛠️ Typical Tool Surface

- **Search**: `rg`, symbol search, embeddings, codebase maps
- **Read**: source files, tests, docs, logs, dependency metadata
- **Edit**: patches, structured refactors, generated files
- **Execute**: unit tests, linters, typecheckers, build commands
- **Inspect**: diffs, git status, PR comments, CI output
- **Coordinate**: subagents, worktrees, issue trackers, [[MCP]] tools

---

## ✅ Strengths

- Can ground changes in real repository context
- Can verify work through tests and command output
- Handles multi-step debugging better than one-shot code generation
- Lets humans delegate narrow implementation tasks while retaining review control
- Scales with better tools, repository instructions, and CI feedback

---

## ❌ Weaknesses

- Context windows fill quickly on large repos
- Agents can chase noisy errors or overfit to weak tests
- Shell/file access creates security and data-loss risks
- Long loops can become expensive and hard to audit
- Human review is still needed for architecture, product intent, and subtle correctness

---

## 🧩 Useful Guardrails

- Keep repo instructions explicit: setup, tests, style, ownership, forbidden commands.
- Prefer small tasks with clear acceptance criteria.
- Require tests, typechecks, or reproducible manual verification when practical.
- Review diffs like human-authored code.
- Use sandboxes, limited credentials, branch protections, and CI before merging.

---

## 🔗 Related Notes

- [[Claude Code]]
- [[Multi-Agent Systems]]
- [[RAG]]
- [[MCP]]
- [[Worktree]]
- [[CI-CD]]
- [[LLM]]

---

## 🌐 External Resources

- [ReAct: Synergizing Reasoning and Acting in Language Models](https://arxiv.org/abs/2210.03629)
- [SWE-agent: Agent-Computer Interfaces Enable Automated Software Engineering](https://arxiv.org/abs/2405.15793)
- [Anthropic: Claude Code best practices](https://www.anthropic.com/engineering/claude-code-best-practices)
- [Claude Code docs: Best Practices](https://code.claude.com/docs/en/best-practices)
- [GitHub Docs: About GitHub Copilot coding agent](https://docs.github.com/en/copilot/concepts/coding-agent/coding-agent)
- [aider GitHub repository](https://github.com/Aider-AI/aider)
