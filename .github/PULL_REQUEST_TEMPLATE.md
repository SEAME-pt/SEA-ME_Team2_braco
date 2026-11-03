## Task Description
<!--
Link the issue, give the goal of this PR in one sentence, then what changes and why.
Two or three lines are enough: the issue holds the detail.
-->
Closes #


---

## Estimate Review
<!--
The estimate lives on the issue. Now that the work is done, look back at it.
Fibonacci points: 1, 2, 3 or 5. If it turned out 8 or more, the issue should have been split.
-->
| Issue estimate | Actual |
|----------------|--------|
|                |        |

**Why it differed:** <!-- e.g. "Matched", or "Driver API undocumented, took 2 extra days" -->


---

## Definition of Done (DoD)

### Manual Tests (Mark N/A if not applicable)
<!--
List the commands you ran and their result, and say where: "sim", "real arm" or "CI only".
e.g. `make ci`, `make ci-ros`, `make coverage`.
-->


### Quality & Documentation
<!--
Indicate CI status and any documentation updates (e.g., README, ADR, wiring/pinout diagrams, or ROS topic definitions).
-->
- [ ] unit test (coverage ≥ 80%), or N/A
- [ ] CI green (`host`, `ros`, `coverage`)
- [ ] `docs/architecture/architecture.md` updated, or N/A <!-- sections listed in "Keeping this document current"; N/A when there is no change to nodes, topics, packages, folders, `make` commands or deployment -->
- [ ] Package README and ADR (`docs/decisions/`) updated, or N/A

### Safety (Mark N/A if the real arm does not move)
<!--
Required when this PR can move the real arm (driver, controllers, MoveIt 2, launch files, joint limits).
-->
- [ ] `docs/guides/safety.md` checklist followed
- [ ] Velocity scaling set low for the first run on the real arm
- [ ] STOP checked: it halts the arm during the move
