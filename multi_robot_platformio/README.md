# Multi-Robot PlatformIO Template

## Layout

```
platformio.ini          <- one [env:robotN] per robot, shared [env] defaults
lib/
  MotorDriver/           <- shared module, auto-detected, no lib_deps needed
  RobotComms/            <- shared module, auto-detected, no lib_deps needed
src/
  robot1/main.cpp
  robot2/main.cpp
  robot3/main.cpp
  robot4/main.cpp
  robot5/main.cpp
  robot6/main.cpp
```

Each robot has its own `src/robotN/` folder with its own `main.cpp`.
PlatformIO builds only the `src_dir` set for the environment you target,
so robots never see each other's `main.cpp`. Everything in `lib/` is
available to every robot automatically.

## Common commands

Build + upload a specific robot:
```
platformio run -e robot1 --target upload
```

Just build (no upload), useful for checking compile errors on all robots:
```
platformio run -e robot1
platformio run -e robot2
```

Open the serial monitor for a robot:
```
platformio device monitor -e robot1
```

List connected boards/ports:
```
platformio device list
```

## Adding a new robot (robot5 / robot6 already stubbed)

1. In `platformio.ini`, uncomment the `[env:robot5]` (or 6) block.
2. Set `board` to match its actual chip (`esp32dev`, `esp32-s3-devkitc-1`, etc.)
   — confirm with `esptool chip_id` if unsure which chip is physically connected.
3. Set `upload_port` to its COM port (`platformio device list` to check).
4. Add any robot-specific libraries under that env's `lib_deps`
   (they stack on top of `${env.lib_deps}` from the shared `[env]` block).
5. Write its logic in `src/robot5/main.cpp`.

## Adding a 7th+ robot

Copy an existing `[env:robotN]` block, rename it, and create a matching
`src/robotN/` folder with its own `main.cpp`.

## Adding a new shared module

Create a new folder under `lib/`, e.g. `lib/SensorFusion/`, with a
`.h`/`.cpp` pair. Any robot can then `#include <SensorFusion.h>` —
no config changes needed, PlatformIO's Library Dependency Finder
picks it up automatically.

## Two kinds of shared code in this project

- **`lib/`** — modules *you* write and maintain (MotorDriver, RobotComms).
  These are version-controlled with the project and are what you'll
  probably be iterating on most.

- **Arduino sketchbook folder (via `lib_extra_dirs`)** — third-party
  libraries you installed through the Arduino IDE's Library Manager
  (Smorphi, etc.). Set once in the shared `[env]` block:

  ```ini
  lib_extra_dirs = C:/Users/User/Documents/Arduino/libraries
  ```

  This makes every library in that folder visible to every robot
  environment. PlatformIO only actually compiles what a robot's
  `main.cpp` (or its includes) reference, via the Library Dependency
  Finder — so having unused libraries in that folder costs nothing
  at build time.

  **Caveat:** this folder holds one copy of each library on disk, so
  there's no per-robot version pinning the way `lib_deps` supports
  (e.g. `bodmer/TFT_eSPI @ ^2.5.43`). If you update a library through
  Arduino's Library Manager, every robot using it picks up that same
  new version at once. If you ever need robot A on one Smorphi version
  and robot B on another, you'd need to either vendor a copy into that
  robot's own folder or switch that specific library to `lib_deps`
  pointed at a pinned GitHub tag/commit instead.

  If a library on this path is nonstandard (missing `library.properties`,
  unusual folder layout) and PlatformIO's LDF fails to detect it, the
  fallback is to copy just that library folder directly into `lib/`.

## Limitations

| Area | Limitation | Workaround / Mitigation |
|------|------------|-------------------------|
| **Shared 3rd-party libs via `lib_extra_dirs`** | Single version on disk — all robots get the same version of Arduino-sketchbook libraries (e.g., Smorphi). No per-robot version pinning. | Vendor a copy into `lib/` for the robot that needs a different version, or switch that specific lib to `lib_deps` with a pinned Git tag/commit. |
| **Hardcoded `upload_port`** | Ports (COM5–COM10) are fixed in `platformio.ini`. If a robot enumerates on a different port, upload fails. | Use `platformio device list` before upload; on Linux/macOS use glob patterns (`/dev/ttyUSB*`). For Windows, a pre-upload script that matches by USB serial number / VID:PID is more robust. |
| **Build-time only isolation** | `build_src_filter` isolates source at compile time. Runtime communication (MQTT, ESP-NOW, UART) is not managed — robots can still interfere if they share topics/channels. | Define a clear message schema (your `RobotComms` JSON) and unique per-robot topics/client IDs. Consider a fleet config service for runtime coordination. |
| **No per-robot `board_build` overrides in shared `[env]`** | All robots inherit the same `framework = arduino` and `monitor_speed`. If one robot needs a different framework (e.g., ESP-IDF) or monitor speed, it must override explicitly. | Add `framework = espidf` or `monitor_speed = 921600` in that robot's `[env:robotN]` section — it will override the shared `[env]` value. |
| **Single `platformio.ini` grows with robot count** | Beyond ~10–15 robots, the ini file becomes long and repetitive. | Extract common blocks into a generated file (Python/Jinja2 template) or split into multiple `.ini` files with `include` (PlatformIO 6.1+), though this adds complexity. |
| **No built-in OTA / fleet upload** | `platformio run -e robotN --target upload` is wired (USB). No native OTA or parallel multi-robot upload. | Implement OTA in firmware (ArduinoOTA, ESP-NOW, HTTP) and trigger via a separate script / CI job. |
| **LDF quirks with `lib_extra_dirs`** | Libraries in the Arduino sketchbook folder without `library.properties` or with nonstandard layouts may not be detected. | Copy problematic libs into `lib/` (vendored) so PlatformIO's LDF sees them as project-local libraries. |
| **Windows COM port limit** | Traditional COM ports only go up to COM256. With many robots, you may hit this. | Use USB hubs with stable enumeration, or switch to network-based upload (OTA) for large fleets. |
