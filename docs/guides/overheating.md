# Guide: arm overheating and rest mode

**Goal:** cool the motors down and get back to work safely.

## 1. When the arm is hot

| Case | Temperature | What the arm does | What you do |
|---|---|---|---|
| **Warm** | 55 °C to 64 °C | Keeps working; the LCD warns | Park it yourself (step 1) |
| **Hot** | 65 °C or more | **Parks by itself** and enters rest; the LCD shows `REST: SERVO HOT` | Skip step 1 |

- **Rest mode:** the arm does not move, but it keeps reading the temperatures, so you can watch it cool down.
- **The arm never leaves rest by itself:** you decide when it is cool (step 3).

## 2. Before you start

- The system is on (`make up`).
- You are on the Jetson, in the repo root.
- Nothing within 350 mm of the arm: it will move.

## 3. Commands

| Command | What it does | The arm moves? |
|---|---|---|
| `make park` | Folds the arm, turns the motors off, enters rest | **Yes** |
| `make temps` | Shows the temperature of the six motors | No |
| `make zero` | Leaves rest, turns the motors on, moves the arm to zero | **Yes** |
| `make resume` | Leaves rest, but the motors stay off: the arm is loose | No |

## 4. Steps

1. **Warm only:** `make park`
2. **Wait,** and run `make temps` (or look at the LCD) every few minutes.
3. **Every joint below 55 °C:** `make zero`, and back to work.

## 5. Check

- The joystick and the web page move the arm again.

## 6. Problems

| Problem | Fix |
|---|---|
| The joystick or the web page does nothing | The arm is in rest: run `make zero` |
| A joint is still at 55 °C or more | Keep waiting, do not run `make zero` yet |
| The arm is loose after `make resume` | `make resume` does not turn the motors on: use `make zero` |
| It parks again soon after `make zero` | It was not cool enough: back to step 2, and wait longer |

Why it works this way: [ADR-0003](../decisions/ADR-0003-rest-per-temp.md).
