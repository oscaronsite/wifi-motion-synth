# WiFi Motion Synth - User guide

A live view of how people moving in a room disturb the WiFi signal (CSI, Channel State Information), with sound. A LilyGo T-Display S3 board measures the signal from the WiFi router; the laptop shows it as a 3D terrain in the browser, and the board's own screen shows it as a hi-fi style spectrum.

Nothing in this project costs money or needs payment details.

Versions this was built and tested with: ESP-IDF v5.5.6, Espressif esp-csi commit 8633d67, board LilyGo T-Display S3 (ESP32-S3), Windows 11.

---

## 1. What's in this folder

| File | What it is |
|---|---|
| `README.md` | Project overview. This guide is in `docs/`. |
| `Start WiFi Motion.bat` | Double-click to start the page. |
| `csi_viz.py` | The program behind the page (one file, everything inside). |
| `Flash board.bat` | Double-click to put `app_main.c` on the board (only needed after a firmware change). |
| `app_main.c` | The board's firmware (source code). Already on the board, keep it as a backup. |
| `sdkconfig.defaults.esp32s3` | WiFi name and password used only the very first time a blank board is flashed. |

Keep this folder somewhere simple, for example `C:\WiFiMotion`.

---

## 2. Daily use

1. **Board:** plug it into any USB power (phone charger, laptop, powerbank). It joins the WiFi by itself. Its screen shows "Waiting for the page on the laptop" and its IP address at the top.
2. **Laptop:** must be on the **same WiFi network** as the board.
3. **Start:** double-click `Start WiFi Motion.bat`. A black window opens and the browser opens `http://localhost:8000`.
4. **Stop:** click in the black window and press **Ctrl+C**, then **Y**. Or just close the black window.

If double-clicking does not work, start it by hand. Open **PowerShell (no admin)** and run, one at a time:

```
cd C:\WiFiMotion
```
```
python csi_viz.py
```

(If `python` is not found, use the ESP-IDF Python: `C:\Espressif\tools\python\v5.5.6\venv\Scripts\python.exe csi_viz.py`)

Useful options, added at the end of the command:

- `--esp-ip 192.168.1.102` if the board is not found automatically (use the IP shown on the board's screen)
- `--demo` simulated data, to try the page without the board
- `--http-port 8010` if port 8000 is already used by something else

If only `py` works and not `python` (common after a fresh Python install), use `py csi_viz.py` instead. The .bat file already does that by itself.

---

## 3. The page

**Top right controls**

- **Readings per second (25 / 50 / 100):** how often the laptop asks the board for a reading. More = smoother surface, more WiFi traffic. The scrolling speed does not change.
- **Quality (Auto / High / Medium / Low):** how detailed the terrain is drawn. Lower = fewer, bigger tiles, smoother on slower laptops. **Auto** measures how fast the laptop draws and picks the level by itself (shown as "Auto (medium)" etc.); each laptop remembers its own.
- **Sensitivity:** how small a change it takes for a spot to turn yellow. The colours, the "Still / Movement" word, the bar, the sound and the board's screen all follow it. Middle = default. Far right = even background noise turns yellow.
- **Noise filter:** smooths the calm (blue) parts only; anything yellow, orange or red is always shown as measured. 80 % settles in about 1 s, 90 % in about 3 s, 100 % in about 10 s. Also applies to the board's screen.
- **Relearn the calm level:** click after moving the board or rearranging the room.
- **Sound:** turns sound on and off. **Sound settings** next to it opens or closes the sound panel; the sound keeps playing when the panel is closed.
- **Board WiFi:** change the board's WiFi network (see section 5).
- **Clean view:** hides everything except the terrain and a small corner line (movement word, MUTED). Shortcut: **double-click the graph** or press **H**. Same again to bring the controls back. Remembered for next time.
- **Full screen:** clean view filling the screen. Shortcut: **F**. **Esc** (or F) to leave. Move the mouse to bring the controls back; they fade after 3 seconds without movement. On a slower laptop, full screen works best with Quality on Auto, Medium or Low.

**What you see**

- The bright front line = the WiFi signal right now, split into about 50 frequencies (left to right).
- The terrain behind it = the last few seconds, sliding back.
- Blue = calm. Yellow, orange, red = the signal changed, something is moving between the board and the router.
- Drag the terrain with the mouse (or the arrow keys) to turn it.
- The strip at the bottom = movement over the last minute.
- Settings are remembered by the browser.

**Sound panel**

Browsers only allow sound after a click on the page, so click **Sound** once each time you open the page.

| Mode | What it does |
|---|---|
| Drone | Soft chord, silent when calm, louder and brighter with movement. Notes come from the curve's shape. |
| Groove | Steady drum pattern + an 8-note melody read from the curve. Calm room = a stable loop; movement makes it mutate. |
| Geiger | Hits whose speed follows the movement. Click sound: Geiger tick, Musical rain, or Bass beat (the Groove kick drum). |
| Evolving groove | Steady kick; hats, snare, percussion and melody fill in with movement, on the beat. |

Other settings: Volume, Mix (Drone / Drums / Melody), Tempo, Pattern, Beat plays (always or only with movement), Melody sound, Scale, Root note, Drone tone, Brightness, Glide, Fade-out, Reverb, Stereo by frequency (low frequencies left, high right).

- **Play 5 seconds:** hear the current sound without having to move.
- **Record:** saves what you hear as a WAV file in your Downloads folder (max 10 minutes). Click again to stop and save.
- Keep the page's tab visible while a beat plays. In a hidden or minimized tab the browser slows its timers and the beat stutters.

---

## 4. The board

**Screen**

- Top left: the board's IP address. Top right: WiFi signal (dBm) and readings per second. Top middle: **MUTED** when the sound is muted.
- Bars: the signal across the frequencies, coloured like the page (blue calm, yellow/orange/red movement). The thin line at the bottom = amount of movement.
- The bars only move while the page runs on the laptop: the readings exist because the laptop keeps asking for them.

**Buttons** (USB connector on the left)

- **BOOT button:** mute / unmute the page's sound (works once Sound was switched on in the page).
- **Other button:** short press = screen on/off. Hold 3 seconds = setup mode (section 5).

**Messages on the screen**

| Message | Meaning |
|---|---|
| Connecting + network name | Joining the WiFi. |
| Waiting for the page on the laptop | Connected, the page is not running (or the laptop is on another network). |
| Switching to + name | New WiFi settings received, restarting. |
| Setup mode | No known network works: join the WiFi "CSI-Setup" with the laptop. |

---

## 5. Changing the board's WiFi

**From the page (normal way):** click **Board WiFi**, type the network name and password, click **Send to the board**. The board restarts and joins it. If it can't, it goes back to the previous network by itself after about 20 seconds.

- The laptop must be able to reach the board on the new network. A second access point plugged into the same router by cable is the same network; a separate router is not, so the laptop must join it too.
- Only 2.4 GHz networks work (the board has no 5 GHz).

**Setup mode (new place, or nothing works):** if the board can't join any known network, or if you hold the other button 3 seconds, it creates its own WiFi called **CSI-Setup** (no password). Connect the laptop to CSI-Setup, start the page, click Board WiFi and send the settings.

The WiFi settings are stored on the board itself and survive power cuts and reflashing. The file `sdkconfig.defaults.esp32s3` only matters for a completely blank board.

---

## 6. Updating the board's firmware

Only needed if `app_main.c` changes. The laptop needs ESP-IDF (section 8).

**Easy way:** plug the board in with USB, double-click `Flash board.bat`. When it ends with "Hard resetting", unplug and replug the board.

**By hand:** open **PowerShell (no admin)** and run one at a time:

```
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
```
(type `Y` if asked)
```
. "C:\Espressif\tools\Microsoft.v5.5.6.PowerShell_profile.ps1"
```
```
cd C:\esp-csi\examples\get-started\csi_recv_router
```
```
Copy-Item C:\WiFiMotion\app_main.c .\main\app_main.c -Force
```
```
idf.py flash
```

Then unplug and replug the board. The first two lines turn the window into an "ESP-IDF PowerShell"; they are needed in every new window.

If it says **"Failed to connect"**: unplug the board, hold **BOOT**, plug it in, release, run `idf.py flash` again, then unplug and replug.

To see the board's own messages: `idf.py monitor`. To quit the monitor on an AZERTY keyboard: **Ctrl+T** then **Ctrl+X** (Ctrl+] does not work), or close the window.

---

## 7. Troubleshooting

| Problem | What to do |
|---|---|
| Page stuck on "Looking for the board" | Laptop and board on the same network? Board screen shows an IP? Start with `--esp-ip` and that IP. |
| Windows firewall question about Python | Allow it for private networks. |
| Readings per second drop a lot | Weak signal (below about -80 dBm on the board's screen) or a running microwave. Move the board closer to the router or higher up. |
| Readings stop after a while | The router may have given the board a new IP. The page searches again by itself; otherwise restart it. |
| No sound | Click Sound on the page (needed after every page load). Check the board shows no MUTED. Check the Mix faders. |
| Deep bass hard to hear | Laptop speakers can't play deep bass; use headphones or a speaker. |
| Beat stutters | Keep the page's tab visible. |
| Terrain choppy on a slower laptop | Quality on Auto or Low; plug the laptop into power (Windows throttles on battery); make the browser window smaller. |
| Controls disappeared | You're in clean view: double-click the graph or press H. |
| "Cannot start the web page on port 8000" | Another copy is still running: close it, or add `--http-port 8010`. |
| `eim` or `idf.py` "not recognized" | You're not in an ESP-IDF PowerShell: run the two activation lines from section 6. |

---

## 8. Moving to another laptop

**To just use it (most cases):** the board already has its firmware and WiFi settings. The new laptop only needs:

1. **Python 3**, free from python.org (tick "Add python.exe to PATH" during install).
2. This folder.
3. Optional, only to read the board over a USB cable: in **PowerShell (no admin)** run `py -m pip install pyserial`. Not needed over WiFi (the black window then just says the USB connection is not available, which is fine).

Quickest way to get Python: in **PowerShell (no admin)** run `winget install -e --id Python.Python.3.14`, close the window, open a new one, check with `py --version`. (The "Python install manager" from python.org works too: answer **N** to the long-paths question, **Y** to installing Python.)

Then connect the laptop to the board's network and double-click `Start WiFi Motion.bat`. No ESP-IDF needed.

**Setting up flashing on a new laptop** (only if you want to change the firmware). Takes about 1 hour, mostly downloads (several GB).

1. **PowerShell (no admin):** `winget install Espressif.EIM-CLI`
2. Close that window, open a **new PowerShell (no admin)**, go to your home folder first (`cd $HOME`; running it from `C:\Windows\System32` makes it fail to save its settings), then: `eim install -i v5.5.6 -a true`
3. Get the project, in the same window:
   ```
   cd C:\
   ```
   ```
   git clone https://github.com/espressif/esp-csi.git
   ```
   ```
   cd C:\esp-csi
   ```
   ```
   git checkout 8633d67
   ```
   (The "detached HEAD" message is normal.)
4. Copy `sdkconfig.defaults.esp32s3` from this folder into `C:\esp-csi\examples\get-started\csi_recv_router\` and put your WiFi name and password in it (only used if the board is blank).
5. Activate ESP-IDF (the two lines from section 6), go to the project folder, then once: `idf.py set-target esp32s3`
6. From then on, flash as in section 6.

If `eim shell` complains "no activation script set", ignore it and use the activation lines from section 6 instead: the script is at `C:\Espressif\tools\Microsoft.v5.5.6.PowerShell_profile.ps1`.

**A brand-new board:** first flash erases whatever is on it. For a completely clean start run `idf.py erase-flash` before `idf.py flash` (this also erases saved WiFi settings, so the ones from `sdkconfig.defaults.esp32s3` are used).

---

## 9. How it works (short)

- The laptop sends the board about 50 tiny packets per second over WiFi (UDP port 5005). The router has to transmit each one to the board, and the board measures how that transmission arrived on each frequency: one reading per packet.
- The board sends each reading back to the laptop over the same WiFi, and draws its own spectrum.
- A person moving changes how the signal bounces around the room, so the shape of the curve changes. The page compares the current shape with the "calm" shape and colours the difference.
- One board and one router see one "line" through the space: good for detecting movement, not for locating people.
- Settings: page settings are stored by the browser; WiFi settings on the board (they survive reflashing). Sensitivity, noise filter and mute are sent to the board every few seconds by the page.
