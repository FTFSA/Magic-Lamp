# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Cable rig controller for a dual-motor system using TMC2209 stepper drivers. The rig has two motors: Left (vertical movement) and Right (horizontal/tension). The system consists of two files that work together:

- **`cable_rig_v3.ino`** — Arduino firmware (Nano RP2040 Connect) that drives two TMC2209 stepper motors via shared UART (Serial1). Handles non-blocking motion, coordinated moves, continuous wind/unwind, direction inversion, coupled-mode motion, and waypoint playback. Communicates over 115200 baud serial with a newline-terminated text protocol.
- **`cable_rig_v3.html`** — Browser-based control UI using the Web Serial API (Chrome/Edge only). Provides jog controls, waypoint management, a 2D position canvas, keyboard shortcuts (W/S for Left, arrows for Right, Space for stop), and coupled-mode settings.

## Hardware

- Arduino Nano RP2040 Connect
- Two TMC2209 drivers sharing Serial1 TX via 1kOhm resistors (driver A addr=0, driver B addr=1)
- Motor A (Left/vertical): D2=STEP, D3=DIR, D4=EN
- Motor B (Right/horizontal): D5=STEP, D6=DIR, D7=EN
- 12V PSU to VM, Nano 3.3V to VIO, shared GND

## Build & Upload

Requires the **TMCStepper** library (install via Arduino Library Manager). Compile and upload `cable_rig_v3.ino` targeting **Arduino Nano RP2040 Connect**.

The HTML file is standalone — open directly in Chrome/Edge. No build step needed.

## Serial Protocol

All commands are newline-terminated, 115200 baud. Key commands: `JL+100`, `JR-200`, `JL+100 C` (coupled), `MOVE L1000 R500 S800` (coordinated), `WIND L`, `UNWIND R`, `STOP`, `HOME`, `GOHOME`, `POS`, `STATUS`, `SPEED 600`, `CURRENT 400`, `MICRO 16`, `INVL 1/0`, `INVR 1/0`, `COUPLE 0.5`, `STEALTHCHOP 1/0`, `ENABLE`, `DISABLE`, `PING`. Responses prefixed with `OK`, `ERR`, `POS`, `DONE`, `DONEL`, `DONER`, or `PONG`.

## Architecture Notes

- Motion is entirely non-blocking — the main `loop()` uses `micros()` timing to interleave step pulses with serial command processing. Never add blocking delays in the motion path.
- Coordinated moves scale per-axis speed proportionally so both axes arrive simultaneously.
- The HTML UI tracks position state client-side by parsing `POS L:n R:n` responses. The firmware is the source of truth.
- Coupled mode makes Right compensate during Left jogs (UI sends `JL+n C`, firmware applies `coupleRatio`). This is for maintaining string tension during vertical moves.
