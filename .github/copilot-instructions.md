<!-- Copilot / AI agent instructions for SmartSolarRouter -->
# SmartSolarRouter — Copilot Instructions

Purpose
- Quick orientation for AI coding agents working on this PlatformIO ESP8266 project.

Big picture
- Platform: ESP8266 (board `d1_mini`), framework: Arduino (see `platformio.ini`).
- Runtime structure: `src/main.cpp` wires global components and the TaskScheduler.
- Major components:
  - `Dimmer` (`include/Dimmer.h`, `src/Dimmer.cpp`): core algorithm, PID control, and SSR timing. ISR-safe method: `IRAM_ATTR updateChannelsOutput()`.
  - `Shelly` (`include/Shelly.h`, `src/Shelly.cpp`): HTTP client that reads active power from a Shelly device.
  - `Network`, `Logger`, `Utils` (static utility classes in `include/`): Wi‑Fi/mdns, logging/teleplot over UDP, and hardware timer helpers.
  - Web UI / config / updater / dashboard: referenced in `main.cpp` via `GUI`, `configManager`, `updater`, and `dash` (likely from the esp8266-iot-framework dependency).

Data flow & runtime loop (short)
- Periodic tasks scheduled by TaskScheduler in `main.cpp`:
  - `GetShellyPowerCbk` fetches power via `Shelly::updateMeasures()` -> `getActivePower()`.
  - `PidFilterCbk` computes routed power via `Dimmer::update(gridPower)`.
  - `Dimmer_ISR` (hardware timer) calls `Dimmer::updateChannelsOutput()` to toggle SSR pins.

Codebase conventions & patterns
- Globals: many modules are instantiated as globals in `main.cpp` (e.g., `m_shelly`, `m_dimmer`, `m_udp`). Prefer matching this pattern for new hardware/service singletons.
- ISR safety: functions called from ISRs must be marked `IRAM_ATTR`, be short, avoid heap allocation, and not call blocking APIs.
- Static utility classes: `Logger`, `Network`, and `Utils` are non-instantiable classes with static APIs.
- Pin configuration and constants: hardware pins and per-board config live in `include/hwConfig.h` and `include/routerConfig.h`.
- PID and timing: PID sample cadence and measure periods are configured via macros in `routerConfig.h` and passed into `Dimmer` constructor.
- External framework: `platformio.ini` pulls an `esp8266-iot-framework` fork — many modules (web server, configManager, GUI) are provided by that library; search `webServer.h` and similar includes to find integration points.

Build / run / debug
- Build: `pio run -e debug` or `platformio run -e debug`.
- Build release: `pio run -e release` (release flags set in `platformio.ini`).
- Upload/flash: `pio run -t upload -e <debug|release>` (upload speed: 921600 configured).
- Serial monitor: `pio device monitor -e debug -b 115200` (monitor speed from `platformio.ini`).

What to watch for when editing
- Do not move or rename `IRAM_ATTR` functions; ensure code called from ISR is deterministic and small.
- When changing pin mappings, update `hwConfig.h` and ensure `m_dimmer.mapChannelToPin()` calls in `main.cpp` remain consistent.
- Changes to `lib_deps` or includes may alter how `GUI`, `configManager`, or `updater` are resolved — verify the dependency `https://github.com/Didifred/esp8266-iot-framework`.

Useful file references
- `platformio.ini` — platform, board, flags, and `lib_deps`.
- `src/main.cpp` — glue code, scheduler tasks, global instances, and overall flow.
- `include/Dimmer.h`, `src/Dimmer.cpp` — control algorithm and ISR interface.
- `include/Shelly.h`, `src/Shelly.cpp` — external device integration.
- `include/hwConfig.h`, `include/routerConfig.h` — hardware constants and timing macros.

If something is unclear
- Ask for which area to expand (build matrix, test strategy, or more code examples).

---
Please review and tell me what to clarify or expand. I can add snippets (example `pio` commands, ISR rules, or more file links).
