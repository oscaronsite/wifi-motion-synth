# WiFi Motion Synth

**Turn the invisible into sight and sound.** Every WiFi signal in a room bounces off walls, furniture and people. When someone moves, those reflections change. WiFi Motion Synth catches those changes with a tiny ESP32-S3 board, draws them as a live 3D landscape in your browser, and plays them as music.

No camera, no microphone, no wearable. Just the WiFi you already have.

[image: the 3D terrain in the browser, calm blue with a yellow/red ripple from someone walking by]

---

## What it does

- **Senses movement through WiFi.** The board measures the WiFi signal on about 50 frequencies at once (WiFi CSI, Channel State Information), up to 100 times per second.
- **Draws it as a living terrain.** The current signal is the front line; the last seconds slide back into a 3D landscape. Blue means calm. Yellow, orange and red mean something moved.
- **Plays it as sound.** Movement becomes a drone, a groove, a Geiger counter or a beat that evolves with what happens in the room.
- **Shows it on the board itself.** The board's own screen displays a hi-fi style spectrum analyser, coloured the same way.

[image: the board's screen showing the coloured spectrum bars]

---

## The sound

The heart of the project. The room plays the instrument: stand still and it stays quiet or steady, walk through and it reacts.

| Mode | What you hear |
|---|---|
| **Drone** | A soft chord, silent when the room is calm, louder and brighter with movement. The notes come from the shape of the signal. |
| **Groove** | A steady drum pattern with an 8-note melody read from the signal. A calm room loops; movement makes it mutate. |
| **Geiger** | Hits whose speed follows the amount of movement, as a Geiger tick, musical rain, or a proper bass beat. |
| **Evolving groove** | A steady kick. Hats, snare, percussion and melody join in as movement grows, always on the beat. |

Everything is synthesized live in the browser, no samples. You can shape it with:

- Volume and a mixer (drone / drums / melody)
- Tempo, pattern, scale, root note
- Melody sound, drone tone, brightness, glide, fade-out, reverb
- Stereo by frequency: low WiFi frequencies on the left, high on the right
- **Record** straight to a WAV file
- **Mute** with the button on the board

[image: the sound panel]

[image or video: short clip of someone walking past, with the terrain reacting and the sound playing]

---

## The page

- **Sensitivity** and **noise filter** sliders, calibrated so colour, sound and the board's screen all agree
- **Readings per second:** 25 / 50 / 100
- **Quality:** Auto / High / Medium / Low, so it runs smoothly on older laptops too
- **Clean view** (double-click the terrain, or H) and **full screen** (F) for a pure visual
- **Board WiFi:** change the board's network from the page. If the new one fails, the board falls back to the previous one by itself.
- Drag the terrain to turn it; a strip at the bottom shows the last minute of movement

[image: full-screen clean view]

---

## Hardware

| What | Notes |
|---|---|
| LilyGo T-Display S3 (ESP32-S3) | The sensor and the spectrum screen. The firmware is written for this board (pins, screen, buttons). |
| A WiFi router (2.4 GHz) | The one you already have. No settings to change. |
| A laptop or PC with Python 3 | Runs the page. Tested on Windows 11. |

The board only needs USB power (a charger or powerbank works) and can sit in another room.

[image: the board next to the router]

---

## How it works

1. The laptop sends the board small "poke" packets over WiFi.
2. The router has to deliver each packet to the board. The board measures how that transmission arrived on each frequency: one reading per packet.
3. The board sends each reading back to the laptop over the same WiFi, and draws its own spectrum.
4. The page learns what the "calm" room looks like, compares every new reading against it, and turns the difference into colour and sound.

One board and one router see one line through the space: great for sensing movement, not for locating people.

---

## Quick start

1. **Flash the board** with `app_main.c` using ESP-IDF v5.5.6 and Espressif's esp-csi (`examples/get-started/csi_recv_router`). Put your WiFi name and password in `sdkconfig.defaults.esp32s3` first (only used on a blank board).
2. **Start the page** on a laptop on the same WiFi:
   ```
   python csi_viz.py
   ```
   (or double-click `Start WiFi Motion.bat` on Windows). The browser opens `http://localhost:8000`.
3. Click **Sound**, and walk around.

No board yet? Try it with simulated data: `python csi_viz.py --demo`

The full step-by-step guide (setup, buttons, troubleshooting, moving to another laptop) is in [docs/user-guide.md](docs/user-guide.md).

---

## Files

| File | What it is |
|---|---|
| `csi_viz.py` | The laptop program: finds the board, serves the page, draws the terrain, makes the sound. One file. |
| `app_main.c` | The board firmware. |
| `sdkconfig.defaults.esp32s3` | Build settings template (WiFi name and password placeholders). |
| `Start WiFi Motion.bat` | Windows: start the page with a double-click. |
| `Flash board.bat` | Windows: flash the firmware with a double-click. |
| `docs/user-guide.md` | The complete user guide. |

---

## Credits

- WiFi CSI capture is built on [Espressif esp-csi](https://github.com/espressif/esp-csi). The board firmware started from its `csi_recv_router` example.
- Board pin and display settings follow LilyGo's T-Display S3 documentation.

## Author

Created by **oscaronsite**, 2026.

## License

Apache License 2.0, see [LICENSE](LICENSE) and [NOTICE](NOTICE). You are free to use and modify it; please keep the copyright notice and credit the original project.
