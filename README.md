# ESP32 DOOM Raycaster

A DOOM-inspired raycaster written in pure C for ESP32 using ESP-IDF.
This project runs on an SH1106 128x64 OLED and uses a 4x4 button matrix for player input.
It combines a simple DDA raycasting renderer with basic enemy AI and melee/fireball combat.

## Features
- 3D raycasting engine with wall distance shading and floor/ceiling rendering
- Player movement, turning, sprinting, and melee fist attack
- Enemy AI for Imp, Trooper, Sergeant, Pinky, and Baron with chase, attack, pain, and death states
- Imp fireball projectile attack with wall collision and player damage
- Sprite rendering with z-buffer occlusion for enemies and projectiles
- Toggleable automap showing player position, facing direction, and enemy markers
- HUD output on a second LCD showing HP, armor, ammo, and FPS

## Hardware
- ESP32 DevKit
- SH1106 128x64 OLED display on I2C
- 4x4 button matrix for controls
- Optional character LCD module for HUD output

## Controls
- Forward: Row1 / Col2
- Backward: Row2 / Col2
- Turn left: Row2 / Col1
- Turn right: Row2 / Col3
- Sprint: Row1 / Col1
- Shoot: Row1 / Col4
- Toggle automap: Row4 / Col4

## Build & Flash
```bash
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

## Project structure
- `main/main.c` — game logic, rendering, input, enemies, and display handling
- `main/lcd.c`, `main/lcd.h` — LCD HUD support
- `sdkconfig` — ESP-IDF configuration

## Notes
- Built as part of a Grade 12 Electronics project to demonstrate embedded graphics, input scanning, and game logic on ESP32
- Combines a compact SH1106 OLED renderer with a 4x4 keypad matrix and secondary LCD HUD output
- Uses `SH1106` OLED commands and I2C driver from ESP-IDF
- The 3D renderer uses a simple DDA raycast and a z-buffer for sprite occlusion

## License
See the `LICENSE` file for license details.
