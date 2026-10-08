---
name: embedded-git-workflow
description: Inspect, organize, and safely prepare CurRobo Git changes. Use for branch planning, diff review, CubeMX regeneration review, commit grouping and messages, synchronization, merge preparation, release changes, or whenever firmware edits must be handed back without losing existing work.
---

# Embedded Git workflow

## Start read-only

Run or request:

- `git status --short --branch`
- `git log --oneline -5`
- relevant unstaged, staged, and untracked-file inspection

Identify pre-existing user changes. Keep them out of the task diff unless they
are inseparable and the user agrees. Never restore, overwrite, or reformat them.

## Review embedded artifacts

Check `.ioc` and CubeMX-generated changes together. Detect changed clocks, pins,
DMA, IRQ priorities, middleware, FDCAN Message RAM, and lost USER CODE sections.
Check that new sources build after CMake reconfiguration. Exclude build outputs,
ELF/HEX/BIN/MAP, local Ozone state, secrets, and absolute machine paths.

## Prepare atomic commits

Propose logical commit groups and Conventional Commit messages. Keep generated
configuration with the behavior that requires it when separating them would
create an unbuildable commit. Show file lists, summaries, and staged diff before
requesting commit authorization.

## Authorization boundary

Read-only Git commands are allowed. Do not execute `add`, `commit`, `switch`,
`checkout`, `merge`, `rebase`, `push`, branch deletion, `reset`, or `clean`
without explicit user approval. Treat force push and history rewriting as
separate high-risk approvals.

Do not default to `git pull`. Fetch, report ahead/behind and diverged commits,
then let the user choose merge or rebase. Stop on conflicts, list affected files
and conflict meaning, and wait for direction before resolving ambiguous intent.
