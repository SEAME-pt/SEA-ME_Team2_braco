# Guide: writing project documents

**Goal:** every document in the repo is easy to find, says the same thing as the code, and stays correct after the
issues and PRs behind it are closed.

These rules cover every tracked `.md` file: `docs/`, the ADRs, package `README.md`s and the root `README.md`. Issue and
PR descriptions are not repo files, so the rules do not apply to them.

**Contents:** [Where a document goes](#1-where-a-document-goes) · [Language](#2-language) ·
[No issue or PR references](#3-no-issue-or-pr-references) · [Open work](#4-open-work) ·
[One source of truth](#5-one-source-of-truth) · [Links](#6-links) · [Dates](#7-dates) ·
[Measurements](#8-measurements) · [Architecture changes](#9-architecture-changes) ·
[Package README](#10-package-readme) · [The check](#11-the-check)

## 1. Where a document goes

| Folder | What goes there |
|---|---|
| `docs/architecture/` | How the software is built today: nodes, topics, packages, deployment, measured resources and timing |
| `docs/decisions/` | ADRs: one decision each, copied from [ADR-TEMPLATE.md](../decisions/ADR-TEMPLATE.md) |
| `docs/guides/` | How to set up, operate and work on braco: steps a person follows |
| `docs/hardware/` | The physical arm: parts, serial protocol, URDF model |
| `ros2_ws/src/<package>/README.md` | One package: what it is and how to use it ([§10](#10-package-readme)) |
| `docker/README.md`, `web/README.md` | The folder they sit in |

- A new document gets a row in its folder's `README.md` index.
- Local notes (roadmaps, reviews, drafts) stay out of the repo.

## 2. Language

English only.

## 3. No issue or PR references

No issue or PR numbers or links in any repo file, ADRs included (their `## Links` too):

- not `#NN`, `PR #NN`, `issues/NN` or `pull/NN`;
- not in code spans or code blocks either.

Say **what** happened and **why**, in words. History lives in git, in the PRs and on the project board.

| Instead of | Write |
|---|---|
| "single `/joint_states` publisher (#NN)" | "single `/joint_states` publisher" (the rule says it all) |
| "added in PR #NN" | "added after review (2026-10-23)" |
| "see issue #NN" | a link to the document that explains it |

- Commit hashes are allowed when a document must point at an exact point in history.
- Issue and PR descriptions may link issues and PRs freely: that is where the links belong.

## 4. Open work

Describe the gap in words, with no issue number: "not measured yet", "not written yet", "planned". Tracking lives on
the project board.

## 5. One source of truth

Each fact has one owner document. Others link to it, they do not copy it. A copy goes stale the first time the owner
changes.

## 6. Links

- Relative paths: `../decisions/ADR-0002-unique-control.md`, not a GitHub URL to the same repo.
- Section anchors for a section: `architecture.md#5-control-of-the-arm`.
- Change a heading → fix the links to it.

## 7. Dates

Absolute dates, `YYYY-MM-DD` (`2026-11-19`). Never "last week", "this sprint" or "recently".

## 8. Measurements

A number someone measured says:

- **when** (the date);
- **on what hardware** (real arm, Jetson Nano, PC, simulation);
- **under which conditions** (what was running, load, settings).

Example: [timing-budget.md](../architecture/timing-budget.md), whose header states the setup and the dates once for
the whole page.

## 9. Architecture changes

A PR that changes nodes, topics, services, packages, folders, `make` commands or deployment updates
[architecture.md](../architecture/architecture.md) in the same PR, including its "Last checked against the code" date.
Which section to update: its
[Changing the architecture](../architecture/architecture.md#11-changing-the-architecture) table.

## 10. Package README

Every package `README.md` covers four things, under any heading names:

1. what the package is and what it is for;
2. its files (or the main ones);
3. how to run it;
4. how to test it.

## 11. The check

`make lint` (and so the CI `host` job) runs `scripts/check-doc-refs.sh` on every tracked `.md` file except
`.github/ISSUE_TEMPLATE/`. It prints `file:line: text` for each issue or PR reference and fails if it finds any.
Section anchors (`#1-set-up`) are not references.

Run it alone on some files: `scripts/check-doc-refs.sh docs/guides/ci.md`.

The check covers [§3](#3-no-issue-or-pr-references) only. The other rules need a person to read the document: the PR
template has a box for it.
