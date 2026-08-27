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
