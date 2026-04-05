# Project Specs — Cable Rig Controller v3

## What This Project Does

Controls a cable-driven positioning rig with two NEMA 17 stepper motors (TMC2209 drivers) via an Arduino Nano RP2040 Connect. Provides a browser-based control panel served over WiFi.

## Components

### Firmware (`cable_rig_v3/cable_rig_v3.ino`)
- Drives two TMC2209 steppers via shared UART
- Non-blocking motion loop (jog, coordinated moves, continuous wind/unwind)
- WiFi connectivity via WiFiNINA (serves HTML + WebSocket)
- mDNS hostname advertisement so the controller can be reached by hostname instead of only numeric IP
- Accepts commands over both USB Serial and WebSocket
- Pushes real-time position updates to all connected WebSocket clients

### Web Control Panel (`cable_rig_v3.html`)
- Served by Arduino over WiFi (also works as standalone file)
- Connects to Arduino via WebSocket for commands and live status
- Motor jog controls, waypoint teach/playback, coupled mode, direction inversion
- 2D position canvas visualization
- Mobile-responsive layout (works on phone and desktop)
- Keyboard shortcuts: W/S (Left motor), arrows (Right motor), Space (stop)

### Config Files
- `cable_rig_v3/wifi_config.h` — WiFi SSID and password
- `cable_rig_v3/html_content.h` — Auto-generated PROGMEM HTML (do not edit directly)

## Features

| Feature | Status |
|---------|--------|
| Dual TMC2209 stepper control | Done |
| Non-blocking motion loop | Done |
| Jog, absolute move, coordinated move | Done |
| Continuous wind/unwind | Done |
| Coupled motor mode | Done |
| Direction inversion | Done |
| Waypoint teach and playback | Done |
| WiFi connectivity (WiFiNINA) | Building |
| mDNS hostname discovery (`<hostname>.local`) | Planned |
| WebSocket server (bidirectional) | Building |
| Arduino serves HTML over WiFi | Building |
| Mobile-responsive UI | Building |
| Multi-client support (phone + laptop) | Building |
| Real-time status push | Building |

## Serial Protocol

115200 baud, newline-terminated. Commands: `JL+100`, `JR-200`, `JL+100 C`, `MOVE L1000 R500 S800`, `WIND L`, `UNWIND R`, `STOP`, `HOME`, `GOHOME`, `POS`, `STATUS`, `SPEED 600`, `CURRENT 400`, `MICRO 16`, `INVL 1/0`, `INVR 1/0`, `COUPLE 0.5`, `STEALTHCHOP 1/0`, `ENABLE`, `DISABLE`, `PING`.

Same protocol used over WebSocket (no newline needed — messages are framed).

## "Done" Criteria for Current Work

- Firmware compiles with zero errors
- Arduino connects to WiFi and prints IP on Serial
- Arduino advertises a stable mDNS hostname on the local network after joining WiFi
- Browsing to `http://<hostname>.local` loads the control panel on mDNS-capable devices
- Browsing to `http://<IP>` loads the control panel
- Commands work over WebSocket (jog, stop, home, wind, etc.)
- Position updates stream to all connected clients in real time
- USB Serial still accepts commands
- UI is usable on phone (large buttons, stacked layout)
- Two clients can connect simultaneously

## Planned Addition: mDNS Hostname Discovery

Add local hostname discovery so the rig can be opened by a human-readable address such as `magic-lamp.local` instead of requiring the current DHCP IP address.

### Scope
- Set a readable device hostname during WiFi startup
- Start an mDNS responder after WiFi connects successfully
- Keep the existing direct-IP workflow working as a fallback
- Print both the assigned IP and the advertised hostname to Serial for debugging
- Update the web UI connection defaults so hostname-based access works when the page is opened via `.local`

### Done Criteria for This Feature
- Sketch compiles with the required mDNS dependency installed
- Serial output shows the advertised hostname after WiFi connection succeeds
- `http://<hostname>.local` serves the same control panel as the raw IP
- WebSocket connection works when the page is opened via the mDNS hostname
- If mDNS startup fails, the sketch keeps serving over IP and logs the failure instead of breaking WiFi control
