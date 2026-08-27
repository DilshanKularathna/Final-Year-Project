# Compile & Upload Commands

Run these from the project root (folder containing `platformio.ini`).

## Compile only (check for errors, no upload)
```
platformio run -e robot1
platformio run -e robot2
platformio run -e robot3
platformio run -e robot4
```

## Compile + Upload
```
platformio run -e robot1 --target upload
platformio run -e robot2 --target upload
platformio run -e robot3 --target upload
platformio run -e robot4 --target upload
```

## Open Serial Monitor
```
platformio device monitor -e robot1
```
Press `Ctrl+C` to exit.

## List connected boards / COM ports
```
platformio device list
```
Use this to confirm which port a robot is on before setting `upload_port` in `platformio.ini`.

## Clean build files (if something seems stuck/cached)
```
platformio run -e robot1 --target clean
```

## Notes
- `-e robotN` picks which `[env:robotN]` from `platformio.ini` to use.
- Only plug in **one robot at a time** when uploading, unless each has a confirmed unique `upload_port`.
- If upload fails with a chip mismatch error, run `platformio device list` and re-check `board` / `upload_port` in `platformio.ini` for that robot.
