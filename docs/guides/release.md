# Guide: releasing braco

When to cut a release, what number to give it, and how to publish it. A release is a git tag on `main` plus a
GitHub Release with notes. Nothing is pushed to a registry: users build from the tagged source with `make setup` and
`make up` (and `make jetson-image` on the Jetson).

Team decisions (2026-11-04, [#108](https://github.com/SEAME-pt/SEA-ME_Team2_braco/issues/108)):

- braco is at **`v0.1.0`**: `v1.0.0` was renumbered and deleted. A clone from before 2026-11-04 has to run
  `git tag -d v1.0.0`, see [§4.4](#44-the-v100--v010-rollback).
- **Every sprint has its own release**: sprint N ends with `v0.N.0`, see [§3](#3-timing).

## 1. Why release at all

`main` moves every day. A release is a fixed point that people can name, return to and compare against:

- **Demos and evaluations**: "the sprint 2 demo ran `v0.2.0`" can be checked out and run again later.
- **The real arm**: the Jetson should run a version that passed the [safety checks](safety.md), not whatever
  `main` happens to be.
- **Bisecting**: "it worked in `v0.2.0`, broke in `v0.3.0`" narrows a regression to one range of PRs.
- **Communication**: the notes tell the team, instructors and future teams what changed, without them reading every
  PR.

## 2. Deciding to release

The end of every sprint is a release, always ([§3](#3-timing)). Between sprints, release only when one is true:

| Trigger | Example |
|---|---|
| A fix someone is waiting for | A driver fix the Jetson needs now (PATCH: `v0.2.1`) |
| A demo or evaluation mid-sprint | Release candidate of the next sprint release (`v0.3.0-rc.1`) |
| A breaking change is about to merge | Tag the last version before it (PATCH or RC), so the old behaviour stays reachable |

Do **not** release when:

- CI on `main` is red, or the last `main` run was skipped or cancelled.
- A change that moves the real arm has not passed the [first-motion checklist](safety.md#9-first-moveit-2-motion-on-the-real-arm).
- A feature is half merged (for example the ADR is accepted but only one of its PRs is in). For the sprint release,
  wait for it or leave it out and say so under **Known issues**.

A sprint release with little in it still ships: it marks what the sprint delivered, and the notes say so.

## 3. Timing

- **Cadence**: one release per sprint, sprint N → `v0.N.0` (a *release train*: the date is fixed and whatever is
  ready ships). Fixes between sprint releases are PATCH releases (`v0.N.1`, `v0.N.2`, ...).
- **Lead time before the demo**: tag the sprint release at least one working day before demo day (see the sprint
  issue's dates). The tagged commit is then deployed to the Jetson and rehearsed. A release cut an hour before a
  demo was never tested as a release. If the demo goes ahead with fixes merged after the tag, release a PATCH.
- **Freeze**: from the moment the release is decided until the tag is pushed, merge only fixes into `main`. Say so
  in the team channel. Keep it short: hours, not days.
- **Avoid**: releasing just before nobody is around to fix it (the end of the day before a weekend or a holiday).
- **Who**: one person per release, named in the sprint issue. They own it until it is verified on the Jetson.

## 4. Version numbers

braco uses [Semantic Versioning 2.0.0](https://semver.org): `vMAJOR.MINOR.PATCH`. It is in the **0.x phase**: the
interfaces are still changing, and SemVer allows anything to change in `0.y.z`.

### 4.1 What counts as the public interface

SemVer is defined by what *users* rely on. For braco that is:

- ROS 2 messages, services and actions in `braco_interfaces`
- Topic, service and action names, node names and their parameters
- Launch files and their arguments
- `make` targets and the variables they accept (`SIM=1`, `COUNT=`, ...)
- `docker/.env` variables and the compose services
- The web server's HTTP API
- Physical behaviour: zero pose, park pose, rest mode, overheat limits

Internal code (class names, private functions, file layout inside a package) is not public. Changing it alone is
not a breaking change.

### 4.2 Which number to bump

While braco is in `0.x`:

| Bump | When | braco examples |
|---|---|---|
| MINOR (`0.3.0`) | The sprint release. The only release that may carry new capabilities and breaking changes | Sprint 3's work: a new action in `braco_interfaces`, a topic renamed, MoveIt 2 planning added |
| PATCH (`0.2.1`) | Between sprints: bug fixes only, no new interface, nothing breaking | Driver timeout fixed; wrong joint limit in the URDF corrected; a docs fix that matters to users |

From `1.0.0` on, the usual SemVer rules apply instead:

| Bump | When | braco examples |
|---|---|---|
| MAJOR (`2.0.0`) | Something in [§4.1](#41-what-counts-as-the-public-interface) changes in a way that breaks existing users, launches or scripts | A field removed or renamed in a `braco_interfaces` message; a topic renamed; `make up` needs a new mandatory variable; the park pose changes |
| MINOR (`1.3.0`) | New capability, backwards compatible | A new action in `braco_interfaces`; a new `make` target; a new optional launch argument |
| PATCH (`1.2.1`) | Bug fixes only, no new interface | As above |

Rules:

- When MINOR goes up, PATCH resets to 0 (`0.2.3` → `0.3.0`).
- A breaking change never goes in a PATCH. It waits for the next sprint release, and the notes list it under
  **Breaking changes** with what users have to change.
- A change in physical behaviour of the arm goes in the notes' **Safety** section even if the code change is small.
- Release candidates: `v0.3.0-rc.1`, `v0.3.0-rc.2`, ... then `v0.3.0`. Use them before a demo when the release
  needs a rehearsal on the Jetson first. Mark them as *pre-release* on GitHub.
- **`1.0.0`** is a team decision, not the next number after a sprint: it means the interfaces in
  [§4.1](#41-what-counts-as-the-public-interface) are stable. Take it when the team agrees they are, for example at
  the final milestone. Record it in an ADR.

### 4.3 Where the version lives

- **Git tag**: `v0.2.0`, with the `v` prefix. This is the source of truth.
- **`package.xml`**: every package in `ros2_ws/src` has the same `<version>`, without the `v`: `0.2.0`.
  [REP 149](https://ros.org/reps/rep-0149.html) only allows three integers, so a release candidate keeps the final
  number there (`0.3.0` for `v0.3.0-rc.1`).
- Today all packages still say `0.0.0`. The sprint 2 release (`v0.2.0`) brings them in line.

### 4.4 The `v1.0.0` → `v0.1.0` rollback

The sprint 1 release was tagged `v1.0.0`. In SemVer, `1.0.0` means "the public interface is stable", which braco's
is not. The team decided on 2026-11-04 to renumber it `v0.1.0`: same commit, new name.

This is the **only exception** to "never delete a published tag" ([§8](#8-when-a-release-is-broken)), done once and
written down here. It was accepted because `v1.0.0` had existed for one day and nothing depends on it. Leaving it
would also break version ordering: `v1.0.0` sorts above every later `v0.N.0`.

**Done on 2026-11-04**: [`v0.1.0`](https://github.com/SEAME-pt/SEA-ME_Team2_braco/releases/tag/v0.1.0) is an
annotated tag on `1957c67` (the merge of PR #105, where `v1.0.0` was), published with the `v1.0.0` notes. The
`v1.0.0` release and tag are deleted from GitHub.

**Every clone made before that date** still has the old tag, because `git fetch` never deletes a tag that is gone
from the remote. Remove it once:

```bash
git tag -l v1.0.0      # prints v1.0.0: the clone still has it
git tag -d v1.0.0
git fetch --tags       # gets v0.1.0
```

For the record, the rollback ran:

```bash
git tag -a v0.1.0 -m "braco v0.1.0 (sprint 1)" v1.0.0^{commit}
git push origin v0.1.0
gh release view v1.0.0 --json body -q .body > /tmp/notes-v0.1.0.md
gh release create v0.1.0 --verify-tag --title "braco v0.1.0" --notes-file /tmp/notes-v0.1.0.md
gh release delete v1.0.0 --cleanup-tag --yes        # deletes the release and the remote tag
```

## 5. Pre-release checklist

All boxes ticked before tagging:

- [ ] CI is green on the exact `main` commit to tag (all of `host`, `ros` and `coverage`, none skipped)
- [ ] `make jetson-image` builds (CI does not build the arm64 image, see [ci.md](ci.md))
- [ ] Simulation smoke test on the PC: `make up SIM=1`, the arm loads and moves in Gazebo
- [ ] On the real arm, following [safety.md](safety.md): `make zero`, `make park`, `make bench`; plus the
      [first-motion checklist](safety.md#9-first-moveit-2-motion-on-the-real-arm) if motion code changed
- [ ] Docs match the behaviour: README, the guides, the ADRs that changed status
- [ ] Every breaking change is listed for the notes, with what users have to change
- [ ] Version number follows [§4.2](#42-which-number-to-bump): `v0.N.0` for sprint N, PATCH between sprints

## 6. Cutting a release

Example: the sprint 2 release, `v0.2.0`.

1. **Bump the package versions** in a normal PR (branch, PR, review, CI, merge):

   ```bash
   sed -i 's|<version>[^<]*</version>|<version>0.2.0</version>|' ros2_ws/src/*/package.xml
   grep -h '<version>' ros2_ws/src/*/package.xml | sort -u   # one line: 0.2.0
   git add ros2_ws/src/*/package.xml
   git commit -m "chore: bump version to 0.2.0"
   ```

2. **Tag the merged commit on `main`**, with an *annotated* tag (it stores who tagged, when, and a message; a
   lightweight tag stores none of that):

   ```bash
   git switch main && git pull --ff-only
   git log -1 --oneline                     # the commit CI checked green
   git tag -a v0.2.0 -m "braco v0.2.0 (sprint 2)"
   git push origin v0.2.0
   ```

3. **Publish the GitHub Release** from the tag. `--verify-tag` refuses to create a missing tag by accident;
   `--generate-notes` lists the merged PRs since the previous tag:

   ```bash
   gh release create v0.2.0 --verify-tag --generate-notes --title "braco v0.2.0"
   # release candidate: add --prerelease
   ```

4. **Edit the notes** on GitHub: add the curated sections from [§7](#7-release-notes) above the generated PR list.

5. **Deploy and verify** on the Jetson: check out the tag, `make jetson-image`, `make up`, then `make zero` and
   `make park`. The release is done when this works.

6. **Announce** in the team channel with the link and the highlights, and link the release in the sprint issue's
   retrospective.

## 7. Release notes

The GitHub Releases are braco's changelog. The generated PR list is complete but hard to read, so put a short
curated part above it, following [Keep a Changelog](https://keepachangelog.com):

```markdown
## Highlights
- One or two sentences on what users can do now that they couldn't before.

## Breaking changes
- `/braco/arm_state` renamed to `/braco/arm/state`: update subscribers and the web UI config.

## Safety
- Park pose moved 5° lower: check the work area clearance (safety.md §3).

## Fixed
- Driver no longer hangs when the serial port drops.

## Known issues
- MoveIt 2 planning is slow on the Jetson (#123).
```

- Write for the person running the arm, not for the reviewer of the PR: what changed for them, and what they have
  to do.
- Leave out empty sections. Always keep **Breaking changes** and **Safety** when they have content.
- Clear PR titles make the generated list readable. A title like `feat: add park pose action` reads better than
  `86 adr 0004`.

## 8. When a release is broken

- **Never move, delete or re-push a published tag.** Someone may already have checked it out, and git does not
  update existing tags on `fetch`, so two people would have different code under the same name. The one exception
  so far is the [`v1.0.0` rollback](#44-the-v100--v010-rollback), decided by the whole team.
- Fix forward: merge the fix, release a PATCH (`v0.2.1`).
- Mark the broken release: edit its notes with a warning at the top (`⚠ Do not use: the driver crashes on start,
  use v0.2.1`). Do not delete it.
- **Hotfix when `main` has moved on** with changes not ready to ship: branch from the tag, fix, tag from that
  branch, then bring the fix to `main` too:

  ```bash
  git switch -c release/v0.2.x v0.2.0
  # fix, PR into release/v0.2.x, CI green
  git tag -a v0.2.1 -m "braco v0.2.1" && git push origin v0.2.1
  # then a second PR (or cherry-pick) bringing the fix to main
  ```

## 9. Industry practices behind this guide

| Practice | What it means here | Reference |
|---|---|---|
| Semantic Versioning | The number says what kind of change it is; `0.x` while the interfaces are still changing ([§4](#4-version-numbers)) | [semver.org](https://semver.org) |
| Immutable releases | Published tags are never rewritten; broken releases are fixed forward ([§8](#8-when-a-release-is-broken)) | [git-tag: on re-tagging](https://git-scm.com/docs/git-tag#_on_re_tagging) |
| Release from a verified commit | Only a commit CI already checked green is tagged; nothing is built or changed between CI and the tag | [Google SRE book, ch. 8](https://sre.google/sre-book/release-engineering/) |
| Annotated tags | Records who released, when, and why | [git-tag](https://git-scm.com/docs/git-tag) |
| Release train | One release per sprint on a fixed date; features that miss it take the next one ([§3](#3-timing)) | |
| Release candidates | Rehearse on the target hardware before the final tag | [semver.org §9](https://semver.org/#spec-item-9) |
| Human-readable changelog | Curated notes above the generated list ([§7](#7-release-notes)) | [keepachangelog.com](https://keepachangelog.com) |
| Conventional commit types | `fix` fits a PATCH; `feat` and `type!:` / `BREAKING CHANGE:` wait for the sprint release (MAJOR from `1.0.0` on) | [conventionalcommits.org](https://www.conventionalcommits.org) |
| One version for the workspace | All packages share one number, like the ROS 2 core repositories | [REP 149](https://ros.org/reps/rep-0149.html) |
| Hardware sign-off | A robot release is verified on the robot, not only in CI and simulation ([§5](#5-pre-release-checklist)) | [safety.md](safety.md) |

Possible later steps, once releases are routine: a CI workflow that creates the GitHub Release when a `v*` tag is
pushed, signed tags (`git tag -s`), and publishing the Jetson image to a registry pinned by digest so the Jetson
does not have to build it.
