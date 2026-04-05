# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Core Rule

Do exactly what is asked. Nothing more, nothing less.
If something is unclear, ask before starting.

---

## Development Rules

**Rule 1: Always read first**
Before taking any action, always read:
- `CLAUDE.md`
- `project_specs.md`

If either file doesn't exist, create it before doing anything else.

**Rule 2: Define before you build (new features only)**
Before writing code for a **new feature or structural change**:
1. Update `project_specs.md` with what's being added and what "done" looks like
2. Show the updated spec
3. Wait for approval

For **bug fixes, tweaks, and small changes** — skip the spec update. Just fix it, validate, and explain what changed.

**Rule 3: Look before you create**
Always read existing files before creating or modifying them. Don't start building until you understand what's already there. If anything is unclear, ask first.

**Rule 4: Test before you respond**
After making any code change:
1. Run the validation command and capture the output
2. Check for errors or warnings
3. **Show the validation output in your response**

Never say "done" if the code is untested or the output contains errors.

---

## How to Respond

Explain like you're talking to a 15-year-old with no coding background.

For every response, include:
- **What I did** — plain English, no jargon
- **What you need to do** — step by step, assume they've never done it before
- **Why** — one sentence on what it does or why it matters
- **Next step** — one clear action
- **Errors** — if something went wrong, explain it simply and say how to fix it

When a task involves hardware or technical tools (Arduino IDE, serial monitor, localhost, etc.):
- Walk through exactly where to find what they need
- Describe what each key setting does in one plain sentence
- Be concise. Less is more.

---

## File Structure

- `cable_rig_v3/cable_rig_v3.ino` → Arduino firmware (WiFi + WebSocket + Serial)
- `cable_rig_v3/wifi_config.h` → WiFi SSID and password (edit before uploading)
- `cable_rig_v3/html_content.h` → Auto-generated from cable_rig_v3.html (do not edit directly)
- `cable_rig_v3.html` → Web control panel source (edit this, then regenerate html_content.h)
- `CLAUDE.md` → Instructions for Claude Code
- `project_specs.md` → What this project does and what needs to be built

Don't create new top-level folders without asking first.

---

## Code Style

- Write simple, readable code — clarity over cleverness
- Make one change at a time
- Don't change code that isn't related to the current task
- Don't over-engineer — build exactly what's needed
- If a big structural change is needed, explain why before making it

---

## Build & Validation

**IMPORTANT: After every code change, validate the build succeeds and show the output.**

### Required Libraries

Install via Arduino IDE Library Manager (Sketch → Include Library → Manage Libraries):
- **TMCStepper** — TMC2209 stepper driver control
- **WiFiNINA** — WiFi connectivity for Nano RP2040 Connect

### Firmware (Arduino CLI)

```bash
arduino-cli compile --fqbn arduino:mbed_nano:nanorp2040connect cable_rig_v3/cable_rig_v3.ino
```

Expected success output:
```
Sketch uses XXXXX bytes (X%) of program storage space
Global variables use XXXXX bytes (X%) of dynamic memory
Used library TMCStepper at version X.X.X
```

If you see compilation errors, fix them before responding.

### HTML Control Panel

After editing `cable_rig_v3.html`, regenerate the PROGMEM header before compiling:

```bash
{ echo 'const char HTML_CONTENT[] PROGMEM = R"rawliteral('; cat cable_rig_v3.html; echo ')rawliteral";'; } > cable_rig_v3/html_content.h
```

After changes, confirm:
- File is valid HTML (no unclosed tags, no syntax errors in `<script>` blocks)
- No references to missing functions or undefined variables
- All command strings match the firmware's protocol exactly
- html_content.h has been regenerated

---

## Testing Checklist

Before marking any task as done:
- Run the compile command and confirm zero errors
- Trace the change end-to-end: if you changed a command, verify the firmware parses it AND the HTML sends it correctly
- Verify existing behaviour wasn't broken (check that unrelated commands still match between firmware and HTML)
- For motion-related changes: confirm no blocking delays were introduced in the `loop()` path

Never say "done" if:
- The build errors out
- Any step produces unexpected output
- The full path (HTML → serial command → firmware handler → response) hasn't been traced

---

## Scope

Only build what is described in `project_specs.md`.
If anything is unclear, ask before starting.

---

## Project Overview

Cable rig controller for a dual-motor system using TMC2209 stepper drivers. Two motors: Left (vertical movement) and Right (horizontal/tension). The system consists of two files:

- **`cable_rig_v3/cable_rig_v3.ino`** — Arduino firmware (Nano RP2040 Connect) that drives two TMC2209 stepper motors via shared UART (Serial1). Handles non-blocking motion, coordinated moves, continuous wind/unwind, direction inversion, coupled-mode motion, and waypoint playback.
- **`cable_rig_v3.html`** — Browser-based control UI using the Web Serial API (Chrome/Edge only). Provides jog controls, waypoint management, a 2D position canvas, keyboard shortcuts (W/S for Left, arrows for Right, Space for stop), and coupled-mode settings.

## Hardware

- Arduino Nano RP2040 Connect
- Two TMC2209 drivers sharing Serial1 TX via 1kΩ resistors (driver A addr=0, driver B addr=1)
- Motor A (Left/vertical): D2=STEP, D3=DIR, D4=EN
- Motor B (Right/horizontal): D5=STEP, D6=DIR, D7=EN
- 12V PSU to VM, Nano 3.3V to VIO, shared GND

## Serial Protocol

All commands are newline-terminated, 115200 baud.

**Commands:** `JL+100`, `JR-200`, `JL+100 C` (coupled), `MOVE L1000 R500 S800` (coordinated), `WIND L`, `UNWIND R`, `STOP`, `HOME`, `GOHOME`, `POS`, `STATUS`, `SPEED 600`, `CURRENT 400`, `MICRO 16`, `INVL 1/0`, `INVR 1/0`, `COUPLE 0.5`, `STEALTHCHOP 1/0`, `ENABLE`, `DISABLE`, `PING`.

**Responses:** Prefixed with `OK`, `ERR`, `POS`, `DONE`, `DONEL`, `DONER`, or `PONG`.

## Architecture Notes

- Motion is entirely non-blocking — `loop()` uses `micros()` timing to interleave step pulses with serial command processing. **Never add blocking delays in the motion path.**
- Coordinated moves scale per-axis speed proportionally so both axes arrive simultaneously.
- The HTML UI tracks position state client-side by parsing `POS L:n R:n` responses. The firmware is the source of truth for position.
- Coupled mode makes Right compensate during Left jogs (UI sends `JL+n C`, firmware applies `coupleRatio`).
- Keep the firmware version string and filename in sync when updating either.
