# Guide: releasing braco

When to cut a release, what number to give it, and how to publish it. A release is a git tag on `main` plus a
GitHub Release with notes. Nothing is pushed to a registry: users build from the tagged source with `make setup` and
`make up` (and `make jetson-image` on the Jetson).

## 1. Why release at all

`main` moves every day. A release is a fixed point that people can name, return to and compare against:

- **Demos and evaluations**: "the sprint 2 demo ran `v1.2.0`" can be checked out and run again later.
- **The real arm**: the Jetson should run a version that passed the [safety checks](safety.md), not whatever
  `main` happens to be.
- **Bisecting**: "it worked in `v1.1.0`, broke in `v1.2.0`" narrows a regression to one range of PRs.
- **Communication**: the notes tell the team, instructors and future teams what changed, without them reading every
  PR.

## 2. Deciding to release

Release when at least one is true:

| Trigger | Example |
|---|---|
| End of a sprint | Sprint review: tag what was delivered |
| Before a demo or evaluation | Freeze the version that will be shown |
| A user-visible capability lands | First MoveIt 2 trajectory on the real arm |
| A fix someone is waiting for | A driver fix the Jetson needs now (patch release) |
| A breaking change is about to merge | Tag the last version before it, so the old behaviour stays reachable |

Do **not** release when:

- CI on `main` is red, or the last `main` run was skipped or cancelled.
- A change that moves the real arm has not passed the [first-motion checklist](safety.md#9-first-moveit-2-motion-on-the-real-arm).
- A feature is half merged (for example the ADR is accepted but only one of its PRs is in). Wait, or release
  without announcing it as a feature.
- The only reason is "it's been a while". With no user-visible change, there is nothing to release.

## 3. Timing

- **Cadence**: one release per sprint is the baseline (a *release train*: the date is fixed and whatever is ready
  ships). Patch releases go out whenever needed.
- **Lead time before a demo**: tag at least one working day before. The tagged commit is then deployed to the
  Jetson and rehearsed. A release cut an hour before a demo was never tested as a release.
- **Freeze**: from the moment the release is decided until the tag is pushed, merge only fixes into `main`. Say so
  in the team channel. Keep it short: hours, not days.
- **Avoid**: releasing just before nobody is around to fix it (the end of the day before a weekend or a holiday).
- **Who**: whoever cuts the release owns it until it is verified on the Jetson. One person per release.

## 4. Version numbers

braco uses [Semantic Versioning 2.0.0](https://semver.org): `vMAJOR.MINOR.PATCH`, for example `v1.4.2`.

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

| Bump | When | braco examples |
|---|---|---|
| MAJOR (`2.0.0`) | Something above changes in a way that breaks existing users, launches or scripts | A field removed or renamed in a `braco_interfaces` message; a topic renamed; `make up` needs a new mandatory variable; the park pose changes |
| MINOR (`1.3.0`) | New capability, backwards compatible | A new action in `braco_interfaces`; a new `make` target; MoveIt 2 planning added; a new optional launch argument |
| PATCH (`1.2.1`) | Bug fixes only, no new interface | Driver timeout fixed; wrong joint limit in the URDF corrected; a docs fix that matters to users |

Rules:

- When MINOR goes up, PATCH resets to 0. When MAJOR goes up, both reset (`1.4.2` → `2.0.0`).
- If in doubt between two levels, pick the higher one. An unnecessary MINOR bump costs nothing. A breaking change
  shipped as a PATCH breaks someone's setup without warning.
- A change in physical behaviour of the arm is at least MINOR, and goes in the notes' **Safety** section even if
  the code change is small.
- Release candidates: `v1.3.0-rc.1`, `v1.3.0-rc.2`, ... then `v1.3.0`. Use them before a demo when the release
  needs a rehearsal on the Jetson first. Mark them as *pre-release* on GitHub.

### 4.3 Where the version lives

- **Git tag**: `v1.3.0`, with the `v` prefix. This is the source of truth.
- **`package.xml`**: every package in `ros2_ws/src` has the same `<version>`, without the `v`: `1.3.0`.
  [REP 149](https://ros.org/reps/rep-0149.html) only allows three integers, so a release candidate keeps the final
  number there (`1.3.0` for `v1.3.0-rc.1`).
- Today all packages still say `0.0.0` while `v1.0.0` is tagged. The next release brings them in line.

### 4.4 About `v1.0.0`

In SemVer, `1.0.0` means "the public interface is stable", and `0.y.z` means "anything may change". braco tagged
`v1.0.0` while its interfaces are still moving. That is fine, but it has a consequence: from now on every breaking
change is a MAJOR bump. Do not delete or reuse `v1.0.0` to go back to `0.x`. A published tag is never rewritten
(see [§8](#8-when-a-release-is-broken)). If MAJOR bumps get frequent, that is a sign to stabilise the interfaces
in [§4.1](#41-what-counts-as-the-public-interface) before changing them, for example with an ADR.

## 5. Pre-release checklist

All boxes ticked before tagging:

- [ ] CI is green on the exact `main` commit to tag (all of `host`, `ros` and `coverage`, none skipped)
- [ ] `make jetson-image` builds (CI does not build the arm64 image, see [ci.md](ci.md))
- [ ] Simulation smoke test on the PC: `make up SIM=1`, the arm loads and moves in Gazebo
- [ ] On the real arm, following [safety.md](safety.md): `make zero`, `make park`, `make bench`; plus the
      [first-motion checklist](safety.md#9-first-moveit-2-motion-on-the-real-arm) if motion code changed
- [ ] Docs match the behaviour: README, the guides, the ADRs that changed status
- [ ] Every breaking change is listed for the notes, with what users have to change
- [ ] Version number chosen with [§4.2](#42-which-number-to-bump), agreed with the team

## 6. Cutting a release

Example: `v1.1.0`.

1. **Bump the package versions** in a normal PR (branch, PR, review, CI, merge):

   ```bash
   sed -i 's|<version>[^<]*</version>|<version>1.1.0</version>|' ros2_ws/src/*/package.xml
   grep -h '<version>' ros2_ws/src/*/package.xml | sort -u   # one line: 1.1.0
   git add ros2_ws/src/*/package.xml
   git commit -m "chore: bump version to 1.1.0"
   ```

2. **Tag the merged commit on `main`**, with an *annotated* tag (it stores who tagged, when, and a message; a
   lightweight tag stores none of that):

   ```bash
   git switch main && git pull --ff-only
   git log -1 --oneline                     # the commit CI checked green
   git tag -a v1.1.0 -m "braco v1.1.0"
   git push origin v1.1.0
   ```

3. **Publish the GitHub Release** from the tag. `--verify-tag` refuses to create a missing tag by accident;
   `--generate-notes` lists the merged PRs since the previous tag:

   ```bash
   gh release create v1.1.0 --verify-tag --generate-notes --title "braco v1.1.0"
   # release candidate: add --prerelease
   ```

4. **Edit the notes** on GitHub: add the curated sections from [§7](#7-release-notes) above the generated PR list.

5. **Deploy and verify** on the Jetson: check out the tag, `make jetson-image`, `make up`, then `make zero` and
   `make park`. The release is done when this works.

6. **Announce** in the team channel with the link and the highlights.

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
  update existing tags on `fetch`, so two people would have different code under the same name.
- Fix forward: merge the fix, release a PATCH (`v1.1.1`).
- Mark the broken release: edit its notes with a warning at the top (`⚠ Do not use: the driver crashes on start,
  use v1.1.1`). Do not delete it.
- **Hotfix when `main` has moved on** with changes not ready to ship: branch from the tag, fix, tag from that
  branch, then bring the fix to `main` too:

  ```bash
  git switch -c release/v1.1.x v1.1.0
  # fix, PR into release/v1.1.x, CI green
  git tag -a v1.1.1 -m "braco v1.1.1" && git push origin v1.1.1
  # then a second PR (or cherry-pick) bringing the fix to main
  ```

## 9. Industry practices behind this guide

| Practice | What it means here | Reference |
|---|---|---|
| Semantic Versioning | The number says what kind of change it is ([§4](#4-version-numbers)) | [semver.org](https://semver.org) |
| Immutable releases | Published tags are never rewritten; broken releases are fixed forward ([§8](#8-when-a-release-is-broken)) | [git-tag: on re-tagging](https://git-scm.com/docs/git-tag#_on_re_tagging) |
| Release from a verified commit | Only a commit CI already checked green is tagged; nothing is built or changed between CI and the tag | [Google SRE book, ch. 8](https://sre.google/sre-book/release-engineering/) |
| Annotated tags | Records who released, when, and why | [git-tag](https://git-scm.com/docs/git-tag) |
| Release train | Fixed cadence (per sprint); features that miss the train take the next one | |
| Release candidates | Rehearse on the target hardware before the final tag | [semver.org §9](https://semver.org/#spec-item-9) |
| Human-readable changelog | Curated notes above the generated list ([§7](#7-release-notes)) | [keepachangelog.com](https://keepachangelog.com) |
| Conventional commit types | `feat` hints at MINOR, `fix` at PATCH, `type!:` or `BREAKING CHANGE:` at MAJOR | [conventionalcommits.org](https://www.conventionalcommits.org) |
| One version for the workspace | All packages share one number, like the ROS 2 core repositories | [REP 149](https://ros.org/reps/rep-0149.html) |
| Hardware sign-off | A robot release is verified on the robot, not only in CI and simulation ([§5](#5-pre-release-checklist)) | [safety.md](safety.md) |

Possible later steps, once releases are routine: a CI workflow that creates the GitHub Release when a `v*` tag is
pushed, signed tags (`git tag -s`), and publishing the Jetson image to a registry pinned by digest so the Jetson
does not have to build it.
