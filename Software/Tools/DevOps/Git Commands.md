# Git Commands

This is a compact reference for commonly used Git operations in local + shared repos.

---

## Core Commands

- `git status` — inspect working tree and staged changes.
- `git diff` — view unstaged changes.
- `git add <path>` — stage file(s) for commit.
- `git commit -m "msg"` — create a commit.
- `git pull` — fetch + integrate from remote.
- `git push` — send local commits to remote.
- `git fetch` — update remote-tracking refs without merging.
- `git log --oneline --decorate --graph --all` — readable commit history view.

---

## Push Safety for Rewrites

### `--force-with-lease` (recommended over `--force`)

- `git push --force-with-lease`
  - Pushes rewritten history only if the remote branch still points to the commit you *originally expected* to be at the branch tip.
  - Prevents accidentally overwriting remote commits pushed by someone else.
  - Use on branches you control (for example feature branches) when you need to rewrite history.
  - Behavior: `--force` overwrites unconditionally, `--force-with-lease` overwrites only if remote tip is exactly what you fetched.

- Why this is safer:
  - `--force`: "replace whatever is currently on remote."
  - `--force-with-lease`: "replace remote only if it has not changed since I checked."
  - Works as a safety check against race conditions during rebase/squash/edit.

- How the lease is evaluated:
  - Git compares the remote ref (like `origin/feature-branch`) to the remote SHA it sees locally.
  - If that SHA changed since your `git fetch`, push is rejected.
  - This catches the common case where a teammate pushed new commits while you were rewriting.

- Typical workflow:
  1. `git fetch`
  2. `git rebase`/`git commit --amend`/squash locally
  3. `git push --force-with-lease`

- If step 3 fails:
  - Fetch latest remote state.
  - Inspect remote commits, resolve by rebase or merge as needed.
  - Retry once your branch is based on the correct remote tip.

- Explicit lease form for automation:
  `git push --force-with-lease=refs/heads/feature-branch:<old_remote_sha> origin feature-branch`
  - Tighter than default because it pins to a specific expected remote SHA.

- `git push --force` caveat:
  - Does not use lease safety checks and can wipe teammates’ remote work.
  - Avoid unless you own the branch and fully accept that risk.

- Practical defaults:
  - `git push --force-with-lease` works on the current branch's upstream by default (the branch configured by `branch.<name>.remote` and `branch.<name>.merge`).
  - If upstream is not set, provide the ref explicitly:
    `git push --force-with-lease origin feature-branch`
  - Prefer a small targeted fetch before rewriting:
    `git fetch origin feature-branch`

---

## Useful Comparison

| Task | Safe command | Safer note |
| --- | --- | --- |
| Push regular updates | `git push` | Requires no history rewrite |
| Rewrite feature branch | `git push --force-with-lease` | Preferred for safety |
| Emergency overwrite | `git push --force` | Avoid unless fully owned branch |
| Remove local unstaged changes | `git restore <path>` | Prefer before committing |
| View remote branches | `git branch -r` | Useful before cleanup |

---

## Related

- [[Git Rebase vs Merge]]
- [[GitHub Actions]]
- [[CI-CD]]

---

## External Reference

- [Git Push Manual](https://git-scm.com/docs/git-push)
