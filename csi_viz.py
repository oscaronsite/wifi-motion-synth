#!/usr/bin/env python3
# WiFi Motion Synth - https://github.com/oscaronsite/wifi-motion-synth
# Copyright 2026 oscaronsite
# SPDX-License-Identifier: Apache-2.0
"""
WiFi motion visualizer for an ESP32 that reports WiFi CSI readings.

What it does:
  1. Finds the board, over WiFi (wireless firmware) and/or over USB.
  2. Sends the board small "poke" packets over WiFi. Each poke is transmitted
     to the board by your WiFi router, and each transmission gives one reading.
  3. Serves a web page on this computer only (http://localhost:8000) that draws
     the readings as a live 3D terrain which ripples when something moves
     between the board and the router.
  4. Optional sound (button on the page): a soft drone, silent when the room is
     still, louder with movement, with notes taken from the shape of the curve.
  5. "Board WiFi" button on the page: send the board a new network name and
     password.
  6. "Noise filter" slider: smooths the calm (blue) parts of the terrain and of
     the board's screen; anything orange or red is always shown as measured.
  7. Sound modes: Drone, Groove, Geiger, Evolving groove; recording to WAV;
     the board's BOOT button mutes and unmutes.
  8. "Quality" setting: fewer, larger tiles for slower laptops (Auto picks).
  9. Clean view (double-click the graph, or H) and full screen (F).

How to run (PowerShell, no admin; use "py" instead of "python" if needed):
    python csi_viz.py                          -> finds the board by itself
    python csi_viz.py --esp-ip 192.168.1.102   -> if it is not found automatically
    python csi_viz.py --demo                   -> simulated data, no board needed

Needs only Python 3. "pyserial" is optional (only to read the board over USB).
Stop it with Ctrl+C.
"""

import argparse
import json
import math
import queue
import random
import socket
import struct
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BAUD = 921600           # ignored by the board's native USB port, kept for USB-UART boards
ESPRESSIF_USB_ID = 0x303A
TICK_SECONDS = 0.05     # the page gets a new slice of data 20 times per second
BOARD_UDP_PORT = 5005   # must match UDP_PORT in the board firmware
RATES = (25, 50, 100)   # readings per second offered on the page


# --------------------------------------------------------------------------
# Shared state between the reader thread and the web server
# --------------------------------------------------------------------------

class Hub:
    """Collects readings and hands them to every open browser tab."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frames = []          # readings received since the last tick
        self.rssi = None
        self.source = "live"      # "live" or "demo"
        self.link = ""            # how the readings arrive, shown on the page
        self.note = "Looking for the board on WiFi and USB..."
        self.rate = 50            # pokes (so readings) per second, set from the page
        self.board_ip = ""
        self.board_ssid = None    # None = unknown, "" = older firmware that does not say
        self.wifi_status = ""
        self.pending_ssid = ""    # network the board was asked to join
        self.pending_since = 0.0
        self.smooth_ms = 100      # noise filter settling time sent to the board, 0..10000 ms
        self.sens = 100           # Sensitivity slider sent to the board, 0..200 (100 = middle)
        self.muted = False        # toggled by the board's BOOT button
        self.count = 0            # total readings, used to measure readings per second
        self.clients = []

    def add_frame(self, amplitudes, rssi, link=None):
        with self.lock:
            self.frames.append(amplitudes)
            self.rssi = rssi
            self.count += 1
            if link:
                self.link = link

    def set_note(self, text):
        with self.lock:
            changed = text != self.note
            self.note = text
        if changed and text:
            print(text, flush=True)

    def subscribe(self):
        q = queue.Queue(maxsize=200)
        with self.lock:
            self.clients.append(q)
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    def publish(self, payload):
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(payload)
            except queue.Full:
                pass  # a stalled tab just misses a slice


HUB = Hub()
LINK = None   # the WifiLink, once started


# --------------------------------------------------------------------------
# Reading the ESP32
# --------------------------------------------------------------------------

def parse_csi_line(line):
    """
    Turn one 'CSI_DATA,...,"[1,2,3,...]"' line into (amplitudes, rssi).
    Returns None for anything that is not a clean CSI line.

    The numbers in the brackets come in pairs: imaginary part, then real part,
    one pair per sub-frequency. The strength of each sub-frequency is the
    length of that pair: sqrt(imag^2 + real^2).
    """
    start = line.find("CSI_DATA")
    if start < 0:
        return None
    lb = line.find("[", start)
    rb = line.find("]", lb)
    if lb < 0 or rb < 0:
        return None
    try:
        values = [int(v) for v in line[lb + 1:rb].split(",")]
    except ValueError:
        return None
    if len(values) < 64 or len(values) % 2:
        return None

    head = line[start:lb].split(",")
    rssi = None
    first_word_invalid = True
    try:
        rssi = int(head[3])
        declared_len = int(head[-3])
        first_word_invalid = head[-2].strip() != "0"
        if declared_len != len(values):
            return None  # line got cut or mixed with another one
    except (ValueError, IndexError):
        pass

    return to_amplitudes(values, first_word_invalid), rssi


def to_amplitudes(values, first_word_invalid):
    amplitudes = []
    for i in range(0, len(values) - 1, 2):
        amplitudes.append(round(math.hypot(values[i], values[i + 1]), 2))
    if first_word_invalid:
        # The chip marks the first 4 numbers (2 sub-frequencies) as garbage.
        amplitudes[0] = 0.0
        amplitudes[1] = 0.0
    return amplitudes


def parse_csi_packet(data):
    """
    One reading sent by the wireless firmware:
    "CSI1", counter (4 bytes), signal strength (1), first-word flag (1),
    length (2), then the raw numbers, one signed byte each.
    """
    if len(data) < 12 + 64 or data[:4] != b"CSI1":
        return None
    _seq, rssi, first_word_invalid, length = struct.unpack_from("<IbBH", data, 4)
    if length < 64 or length % 2 or len(data) < 12 + length:
        return None
    values = struct.unpack_from("%db" % length, data, 12)
    return to_amplitudes(values, bool(first_word_invalid)), rssi


def find_port(list_ports):
    ports = list(list_ports.comports())
    espressif = [p for p in ports if p.vid == ESPRESSIF_USB_ID]
    if espressif:
        return espressif[0].device
    return None


def serial_reader(port_arg):
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        # Fine on a laptop that only talks to the board over WiFi.
        print("(USB connection to the board not available: pyserial is not installed. "
              "WiFi works normally.)", flush=True)
        return

    while True:
        port = port_arg or find_port(list_ports)
        if not port:
            time.sleep(2)   # no board on USB: fine, it may be on WiFi only
            continue

        ser = serial.Serial()
        ser.port = port
        ser.baudrate = BAUD
        ser.timeout = 1
        ser.dtr = False   # keep both control lines low so opening the port
        ser.rts = False   # does not reset the board into download mode
        try:
            ser.open()
        except (serial.SerialException, OSError) as err:
            HUB.set_note("Cannot open %s (%s). If the ESP-IDF monitor is still running, "
                         "close it with Ctrl+] and this will retry by itself." % (port, err))
            time.sleep(2)
            continue

        print("Board connected on USB (%s)." % port, flush=True)
        got_data = False
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", "ignore").strip()
                parsed = parse_csi_line(line)
                if parsed:
                    HUB.add_frame(parsed[0], parsed[1], "USB " + port)
                    if not got_data:
                        got_data = True
                        HUB.set_note("Receiving readings over USB (%s)." % port)
                elif line and "CSI_DATA" not in line:
                    # Normal log lines from the board (WiFi connecting, errors...)
                    print("[ESP32] " + line[:200], flush=True)
        except (serial.SerialException, OSError) as err:
            print("USB connection to %s closed (%s)." % (port, err), flush=True)
        finally:
            try:
                ser.close()
            except Exception:
                pass
        time.sleep(1)


def demo_generator():
    """Simulated readings, clearly labelled as such on the page."""
    HUB.source = "demo"
    HUB.set_note("DEMO MODE: simulated data, not your room.")
    n = 64
    shape = []
    for k in range(n):
        if k < 6 or k > 58 or k == 32:
            shape.append(0.0)
        else:
            x = (k - 6) / 52.0
            shape.append(16 + 7 * math.sin(x * 5.3 + 0.6) + 4 * math.sin(x * 13.0))
    t0 = time.time()
    while True:
        t = time.time() - t0
        # Someone "walks by" for about 4 seconds, every 12 seconds.
        phase = t % 12.0
        walk = math.sin(math.pi * (phase - 5.0) / 4.0) if 5.0 < phase < 9.0 else 0.0
        gain = 1.0 + 0.08 * random.uniform(-1, 1)
        amps = []
        for k in range(n):
            base = shape[k]
            if base == 0.0:
                amps.append(0.0)
                continue
            wobble = walk * 0.35 * math.sin(k * 0.22 + t * 5.0)
            value = base * (1.0 + wobble) * gain + random.uniform(-0.7, 0.7)
            amps.append(round(max(value, 0.0), 2))
        HUB.add_frame(amps, -48 + int(3 * walk), "simulated")
        time.sleep(1.0 / HUB.rate)


def local_broadcast_addresses():
    """255.255.255.255 plus the usual home-network broadcast address (x.y.z.255)."""
    addresses = ["255.255.255.255"]
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("192.0.2.1", 9))      # no packet is sent, this only picks the network card
        mine = probe.getsockname()[0]
        probe.close()
        addresses.append(mine.rsplit(".", 1)[0] + ".255")
    except OSError:
        pass
    return addresses


class WifiLink:
    """
    Talks to the board over WiFi.
      - asks "CSI?" on the local network until a board answers "CSI!"
      - then pokes it HUB.rate times per second
      - readings come back from the board as "CSI1" packets (wireless firmware)
    With the older USB firmware the pokes still trigger readings; they then
    arrive over USB instead.
    """

    def __init__(self, esp_ip):
        self.fixed_ip = esp_ip
        self.board_ip = esp_ip
        self.last_packet = 0.0
        self.need_hello = True    # keep asking "CSI?" until the board answers with its network
        self.last_btn = None      # number of the last BOOT button press handled
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", 0))
        self.sock.settimeout(1.0)

    def receiver(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                # Windows reports "connection reset" here when a poke found no
                # listener on the board (older firmware). Harmless.
                time.sleep(0.01)
                continue
            if data[:4] == b"CSI!":
                ssid = data[4:].decode("utf-8", "replace")
                self.need_hello = False
                if self.board_ip != addr[0] and not self.fixed_ip:
                    self.board_ip = addr[0]
                    HUB.set_note("Found the board on WiFi at %s. Waiting for readings..." % addr[0])
                with HUB.lock:
                    HUB.board_ip = addr[0]
                    HUB.board_ssid = ssid
                    if HUB.pending_ssid and ssid == HUB.pending_ssid:
                        HUB.wifi_status = "Done: the board is now on \"%s\"." % ssid
                        HUB.pending_ssid = ""
                continue
            if data[:4] == b"BTN0" and len(data) >= 5:
                if data[4] != self.last_btn:          # the board sends each press 3 times
                    self.last_btn = data[4]
                    HUB.muted = not HUB.muted
                    print("Board button: sound %s." % ("muted" if HUB.muted else "unmuted"), flush=True)
                continue
            if data[:7] == b"SETW-OK":
                self.need_hello = True
                with HUB.lock:
                    HUB.wifi_status = ("The board saved the settings and is restarting to join \"%s\". "
                                       "This takes up to 30 seconds." % HUB.pending_ssid)
                print("Board accepted the new WiFi settings and is restarting.", flush=True)
                continue
            if data[:8] == b"SETW-ERR":
                reason = data[8:].decode("utf-8", "replace").strip()
                with HUB.lock:
                    HUB.wifi_status = "The board refused the settings: %s." % reason
                    HUB.pending_ssid = ""
                continue
            parsed = parse_csi_packet(data)
            if parsed:
                if not self.last_packet:
                    HUB.set_note("Receiving readings over WiFi from %s." % addr[0])
                self.last_packet = time.time()
                if not self.board_ip:
                    self.board_ip = addr[0]
                with HUB.lock:
                    HUB.board_ip = addr[0]
                HUB.add_frame(parsed[0], parsed[1], "WiFi " + addr[0])

    def sender(self):
        next_poke = time.perf_counter()
        last_search = 0.0
        last_settings_sent = 0.0
        settings_sent = None
        while True:
            now = time.time()
            # Ask "is there a board?" whenever no readings arrived for a few seconds:
            # at start, after the board restarts, or when its address may have changed.
            if (now - self.last_packet > 4 or self.need_hello) and now - last_search > 1.5:
                last_search = now
                targets = [] if self.fixed_ip else local_broadcast_addresses()
                if self.board_ip:
                    targets.append(self.board_ip)
                for address in targets:
                    try:
                        self.sock.sendto(b"CSI?", (address, BOARD_UDP_PORT))
                    except OSError:
                        pass
            if not self.board_ip:
                time.sleep(0.2)
                next_poke = time.perf_counter()
                continue
            # Tell the board the noise filter and Sensitivity settings when they change,
            # and every 5 s in case the board restarted.
            settings = (HUB.smooth_ms, HUB.sens, HUB.muted)
            if settings != settings_sent or now - last_settings_sent > 5:
                try:
                    ms = settings[0]
                    self.sock.sendto(b"SMT2" + bytes([ms & 0xFF, ms >> 8]), (self.board_ip, BOARD_UDP_PORT))
                    self.sock.sendto(b"SENS" + bytes([settings[1]]), (self.board_ip, BOARD_UDP_PORT))
                    self.sock.sendto(b"MUTE" + bytes([1 if settings[2] else 0]), (self.board_ip, BOARD_UDP_PORT))
                    settings_sent = settings
                    last_settings_sent = now
                except OSError:
                    pass
            try:
                self.sock.sendto(b"csi", (self.board_ip, BOARD_UDP_PORT))
            except OSError:
                time.sleep(0.5)
            next_poke += 1.0 / HUB.rate
            delay = next_poke - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.5:
                next_poke = time.perf_counter()   # we fell behind (laptop was asleep?), start fresh

    def send_wifi(self, ssid, password):
        """Ask the board to join another network. Returns an error text, or "" when sent."""
        if not self.board_ip:
            return "The board has not been found yet."
        s = ssid.encode("utf-8")
        p = password.encode("utf-8")
        packet = b"SETW" + bytes([len(s)]) + s + bytes([len(p)]) + p
        try:
            for _ in range(2):      # twice, in case one copy gets lost
                self.sock.sendto(packet, (self.board_ip, BOARD_UDP_PORT))
                time.sleep(0.05)
        except OSError as err:
            return "Could not reach the board (%s)." % err
        return ""

    def start(self):
        threading.Thread(target=self.receiver, daemon=True).start()
        threading.Thread(target=self.sender, daemon=True).start()


def ticker():
    """20 times per second, send everything received since last time to the page."""
    last_count = 0
    last_rate_time = time.time()
    rate = 0.0
    last_sent = 0.0
    last_print = time.time()
    last_print_count = 0
    while True:
        time.sleep(TICK_SECONDS)
        now = time.time()
        with HUB.lock:
            frames = HUB.frames
            HUB.frames = []
            count = HUB.count
            if HUB.pending_ssid and now - HUB.pending_since > 60:
                HUB.wifi_status = ("The board has not come back yet. If \"%s\" is a different network "
                                   "from this laptop's, connect the laptop to it too. If the name or "
                                   "password was wrong, the board goes back to its previous network "
                                   "by itself within a minute." % HUB.pending_ssid)
                HUB.pending_ssid = ""
            info = {"rssi": HUB.rssi, "src": HUB.source, "link": HUB.link, "note": HUB.note,
                    "rate": HUB.rate, "bip": HUB.board_ip, "bssid": HUB.board_ssid,
                    "wifi": HUB.wifi_status, "muted": HUB.muted}
        if now - last_rate_time >= 1.0:
            rate = (count - last_count) / (now - last_rate_time)
            last_count = count
            last_rate_time = now
        if now - last_print >= 10.0:
            print("Readings per second: %d" % round((count - last_print_count) / (now - last_print)), flush=True)
            last_print = now
            last_print_count = count
        if not frames and now - last_sent < 1.0:
            continue  # nothing new; a heartbeat once per second is enough
        last_sent = now
        info["rows"] = frames
        info["pps"] = round(rate)
        HUB.publish(json.dumps(info).encode("utf-8"))


# --------------------------------------------------------------------------
# Web server (this computer only)
# --------------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep the PowerShell window readable

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            q = HUB.subscribe()
            try:
                while True:
                    try:
                        payload = q.get(timeout=5)
                        self.wfile.write(b"data: " + payload + b"\n\n")
                    except queue.Empty:
                        self.wfile.write(b": still here\n\n")
                    self.wfile.flush()
            except OSError:
                pass  # the tab was closed
            finally:
                HUB.unsubscribe(q)
        else:
            self.send_error(404)

    def reply_json(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == "/wifi":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                data = json.loads(self.rfile.read(min(length, 1024)).decode("utf-8"))
                ssid = str(data.get("ssid", ""))
                password = str(data.get("password", ""))
            except (ValueError, UnicodeDecodeError, AttributeError):
                self.reply_json(400, {"ok": False, "msg": "Bad request."})
                return
            if not 1 <= len(ssid.encode("utf-8")) <= 32:
                self.reply_json(200, {"ok": False, "msg": "The network name must be 1 to 32 characters."})
                return
            plen = len(password.encode("utf-8"))
            if plen and not 8 <= plen <= 63:
                self.reply_json(200, {"ok": False, "msg": "The password must be empty (open network) or 8 to 63 characters."})
                return
            if LINK is None:
                self.reply_json(200, {"ok": False, "msg": "Not available in demo mode."})
                return
            with HUB.lock:
                HUB.pending_ssid = ssid
                HUB.pending_since = time.time()
                HUB.wifi_status = "Sending to the board..."
            err = LINK.send_wifi(ssid, password)
            if err:
                with HUB.lock:
                    HUB.pending_ssid = ""
                    HUB.wifi_status = err
                self.reply_json(200, {"ok": False, "msg": err})
                return
            print("Sent new WiFi settings (network \"%s\") to the board." % ssid, flush=True)
            self.reply_json(200, {"ok": True, "msg": "Sent."})
            return
        if self.path in ("/smooth", "/sens"):
            try:
                length = int(self.headers.get("Content-Length", "0"))
                value = int(self.rfile.read(min(length, 16)).decode("ascii").strip())
            except (ValueError, UnicodeDecodeError):
                value = -1
            if self.path == "/smooth" and 0 <= value <= 10000:
                HUB.smooth_ms = value
                self.send_response(204)
            elif self.path == "/sens" and 0 <= value <= 200:
                HUB.sens = value
                self.send_response(204)
            else:
                self.send_response(400)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        if self.path != "/rate":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            value = int(self.rfile.read(min(length, 16)).decode("ascii").strip())
        except (ValueError, UnicodeDecodeError):
            value = 0
        if value in RATES:
            HUB.rate = value
            print("Readings per second set to %d from the page." % value, flush=True)
            self.send_response(204)
        else:
            self.send_response(400)
        self.send_header("Content-Length", "0")
        self.end_headers()


def main():
    parser = argparse.ArgumentParser(description="Live WiFi motion view from an ESP32 (CSI).")
    parser.add_argument("--port", help="serial port of the ESP32, for example COM5")
    parser.add_argument("--demo", action="store_true", help="simulated data, no board needed")
    parser.add_argument("--esp-ip", help="IP address of the board, if it is not found automatically")
    parser.add_argument("--rate", type=int, choices=RATES, default=50,
                        help="readings per second at start (default 50, also changeable on the page)")
    parser.add_argument("--http-port", type=int, default=8000, help="web page port (default 8000)")
    parser.add_argument("--no-browser", action="store_true", help="do not open the browser")
    args = parser.parse_args()

    HUB.rate = args.rate
    if args.demo:
        threading.Thread(target=demo_generator, daemon=True).start()
    else:
        threading.Thread(target=serial_reader, args=(args.port,), daemon=True).start()
        global LINK
        LINK = WifiLink(args.esp_ip)
        LINK.start()
        print(HUB.note, flush=True)
    threading.Thread(target=ticker, daemon=True).start()

    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    except OSError as err:
        print("Cannot start the web page on port %d (%s)." % (args.http_port, err))
        print("Try another one by adding:  --http-port 8010")
        return
    server.daemon_threads = True
    url = "http://localhost:%d" % args.http_port
    print("Page: %s   (stop with Ctrl+C)" % url, flush=True)
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


# --------------------------------------------------------------------------
# The page
# --------------------------------------------------------------------------

PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WiFi motion</title>
<style>
  :root {
    --sea: #101833;
    --deep: #080d20;
    --calm: #7cc3cf;
    --warm: #ffb454;
    --hot: #ff5a6e;
    --ink: #ece8dc;
    --dim: #97a1c0;
  }
  * { box-sizing: border-box; }
  html, body {
    height: 100%; margin: 0; overflow: hidden;
    background: var(--sea); color: var(--ink);
    font-family: Bahnschrift, "DIN Alternate", "Segoe UI", system-ui, sans-serif;
    font-size: 16px; line-height: 1.4;
  }
  canvas { position: fixed; inset: 0; width: 100%; height: 100%; cursor: grab; touch-action: none; }
  canvas:active { cursor: grabbing; }
  .corner { position: fixed; pointer-events: none; }
  #head { top: 22px; left: 28px; max-width: min(34rem, 60vw); }
  h1 { margin: 0; font-size: 1.9rem; font-weight: 600; letter-spacing: 0.01em; }
  #status { margin: 4px 0 0; color: var(--dim); font-size: 0.95rem; }
  #demo {
    display: none; margin-top: 10px; padding: 6px 10px; width: fit-content;
    background: var(--warm); color: #1a1405; font-weight: 600; border-radius: 3px;
  }
  #meter { left: 28px; bottom: 118px; width: min(24rem, 70vw); }
  #word { font-size: 2.6rem; font-weight: 600; line-height: 1.1; }
  #track { margin-top: 10px; height: 8px; background: rgba(236, 232, 220, 0.14); border-radius: 4px; overflow: hidden; }
  #fill { height: 100%; width: 0%; background: var(--calm); border-radius: 4px; }
  #hint { margin: 8px 0 0; color: var(--dim); font-size: 0.9rem; }
  #strip-label { left: 28px; bottom: 76px; color: var(--dim); font-size: 0.85rem; }
  #controls {
    top: 22px; right: 28px; pointer-events: auto;
    display: flex; flex-direction: column; align-items: flex-end; gap: 10px;
    font-size: 0.9rem; color: var(--dim);
  }
  #controls label { display: flex; align-items: center; gap: 10px; }
  input[type=range] { width: 150px; accent-color: var(--calm); }
  select {
    font: inherit; color: var(--ink); background: var(--sea);
    border: 1px solid rgba(236, 232, 220, 0.4); border-radius: 3px; padding: 4px 8px; cursor: pointer;
  }
  select:focus-visible { outline: 2px solid var(--warm); outline-offset: 3px; }
  button[aria-pressed="true"] { background: var(--calm); color: #08212a; border-color: var(--calm); }
  #soundPanel {
    display: grid; grid-template-columns: auto 150px; gap: 8px 10px; align-items: center; justify-items: end;
    padding: 12px 14px; border: 1px solid rgba(236, 232, 220, 0.22); border-radius: 4px;
    background: rgba(8, 13, 32, 0.78);
  }
  #soundPanel[hidden] { display: none; }
  #soundPanel { max-height: calc(100vh - 250px); overflow-y: auto; }
  #soundPanel select, #soundPanel input { width: 150px; }
  #soundPanel input.check { width: auto; justify-self: start; }
  #soundPanel .sec { text-align: left; color: var(--ink); font-size: 0.8rem; letter-spacing: 0.08em;
                     text-transform: uppercase; margin-top: 4px; border-top: 1px solid rgba(236,232,220,0.15); padding-top: 6px; }
  #soundPanel .off { display: none; }
  #soundPanel .wide { grid-column: 1 / -1; justify-self: stretch; text-align: right; }
  #soundPanel .wide.sec { text-align: left; }
  #soundPanel button.wide { text-align: center; }
  #notes { color: var(--ink); min-height: 1.4em; }
  #wifiPanel {
    width: 300px; padding: 12px 14px; border: 1px solid rgba(236, 232, 220, 0.22); border-radius: 4px;
    background: rgba(8, 13, 32, 0.9); display: flex; flex-direction: column; gap: 8px; align-items: stretch;
  }
  #wifiPanel[hidden] { display: none; }
  #wifiPanel label { display: block; }
  #wifiPanel input[type=text], #wifiPanel input[type=password] {
    width: 100%; margin-top: 3px; font: inherit; color: var(--ink); background: var(--sea);
    border: 1px solid rgba(236, 232, 220, 0.4); border-radius: 3px; padding: 5px 8px;
  }
  #wifiPanel input:focus-visible { outline: 2px solid var(--warm); outline-offset: 1px; }
  #wifiPanel .row { display: flex; align-items: center; gap: 8px; justify-content: flex-start; }
  #wifiNow { color: var(--ink); }
  #wifiStatus { color: var(--warm); min-height: 1.2em; }
  #wifiPanel .small { font-size: 0.82rem; }
  button {
    font: inherit; color: var(--ink); background: transparent;
    border: 1px solid rgba(236, 232, 220, 0.4); border-radius: 3px; padding: 6px 12px; cursor: pointer;
  }
  button:hover { border-color: var(--ink); }
  button:focus-visible, input:focus-visible { outline: 2px solid var(--warm); outline-offset: 3px; }
  .btnrow { display: flex; gap: 8px; justify-content: flex-end; }
  #mini {
    top: 18px; left: 22px; display: none; font-size: 1rem; color: var(--dim);
  }
  #miniWord { color: var(--ink); font-weight: 600; }
  #miniMuted { color: var(--warm); font-weight: 600; margin-left: 12px; }
  #miniHint { margin-top: 4px; font-size: 0.85rem; transition: opacity 0.6s; }
  /* clean view: only the terrain and a small corner line */
  body.clean #head, body.clean #meter, body.clean #strip-label, body.clean #controls { display: none; }
  body.clean #mini { display: block; }
  /* full screen: controls come back while the mouse moves */
  body.clean.peek #controls { display: flex; }
  body.clean.idle canvas { cursor: none; }
  #empty {
    position: fixed; inset: 0; display: flex; align-items: center; justify-content: center;
    pointer-events: none; text-align: center; padding: 0 10vw; color: var(--dim); font-size: 1.15rem;
  }
</style>
</head>
<body>
<canvas id="view"></canvas>

<div class="corner" id="head">
  <h1>WiFi motion</h1>
  <p id="status">Starting...</p>
  <div id="demo">Demo mode: simulated data, not your room</div>
</div>

<div class="corner" id="controls">
  <label>Readings per second
    <select id="rate"><option value="25">25</option><option value="50" selected>50</option><option value="100">100</option></select>
  </label>
  <label title="Lower quality draws fewer, larger tiles: smoother on slower laptops. Auto picks for you.">Quality
    <select id="quality"><option value="auto">Auto</option><option value="high">High</option><option value="medium">Medium</option><option value="low">Low</option></select>
  </label>
  <label title="How small a change turns yellow. The colours, the word, the bar and the sound all follow it.">Sensitivity <input id="sens" type="range" min="-1" max="1" step="0.05" value="0"></label>
  <label title="Smooths the calm (blue) parts only. Anything orange or red is always shown as measured.">Noise filter <input id="smooth" type="range" min="0" max="100" step="1" value="60"></label>
  <button id="relearn" type="button">Relearn the calm level</button>
  <div class="btnrow">
    <button id="soundBtn" type="button" aria-pressed="false">Sound: off</button>
    <button id="soundSetBtn" type="button" aria-expanded="false" hidden>Sound settings</button>
  </div>
  <div id="soundPanel" hidden>
    <label for="sMode">Mode</label><select id="sMode"></select>
    <label for="sVolume">Volume</label><input id="sVolume" type="range" min="0" max="1" step="0.01">
    <div class="wide sec">Mix</div>
    <label for="sMixDrone">Drone</label><input id="sMixDrone" type="range" min="0" max="1" step="0.01">
    <label for="sMixDrums" data-modes="Groove|Geiger|Evolving groove" id="lMixDrums">Drums / clicks</label><input id="sMixDrums" type="range" min="0" max="1" step="0.01" data-modes="Groove|Geiger|Evolving groove">
    <label for="sMixMel" data-modes="Groove|Evolving groove">Melody</label><input id="sMixMel" type="range" min="0" max="1" step="0.01" data-modes="Groove|Evolving groove">
    <div class="wide sec" data-modes="Groove|Evolving groove">Rhythm</div>
    <label for="sBpm" data-modes="Groove|Evolving groove" id="lBpm">Tempo</label><input id="sBpm" type="range" min="60" max="150" step="1" data-modes="Groove|Evolving groove">
    <label for="sPattern" data-modes="Groove">Pattern</label><select id="sPattern" data-modes="Groove"></select>
    <label for="sBeatWhen" data-modes="Groove|Evolving groove">Beat plays</label>
    <select id="sBeatWhen" data-modes="Groove|Evolving groove"><option>Always</option><option>Only with movement</option></select>
    <label for="sMel" data-modes="Groove|Evolving groove">Melody sound</label>
    <select id="sMel" data-modes="Groove|Evolving groove"><option>Pluck</option><option>Soft keys</option><option>Bell</option></select>
    <label for="sClick" data-modes="Geiger">Click sound</label>
    <select id="sClick" data-modes="Geiger"><option value="Tick">Geiger tick</option><option value="Notes">Musical rain</option><option value="Bass">Bass beat</option></select>
    <div class="wide sec">Tone</div>
    <label for="sScale">Scale</label><select id="sScale"></select>
    <label for="sRoot">Root note</label><select id="sRoot"></select>
    <label for="sWave">Drone tone</label>
    <select id="sWave"><option value="sine">Soft (sine)</option><option value="triangle">Warm (triangle)</option><option value="sawtooth">Rich (saw)</option></select>
    <label for="sBright">Brightness</label><input id="sBright" type="range" min="250" max="4000" step="10">
    <label for="sGlide">Glide</label><input id="sGlide" type="range" min="0.05" max="3" step="0.05">
    <label for="sTail">Fade-out</label><input id="sTail" type="range" min="0.3" max="8" step="0.1">
    <label for="sSpace">Reverb</label><input id="sSpace" type="range" min="0" max="1" step="0.01">
    <label for="sStereo" title="Changes in the low frequencies on the left, high frequencies on the right">Stereo by frequency</label>
    <input id="sStereo" type="checkbox" class="check">
    <div class="wide" id="notes"></div>
    <button class="wide" id="preview" type="button">Play 5 seconds</button>
    <button class="wide" id="recBtn" type="button" aria-pressed="false">Record</button>
  </div>
  <button id="wifiBtn" type="button" aria-expanded="false">Board WiFi</button>
  <div class="btnrow">
    <button id="cleanBtn" type="button" title="Shortcut: H, or double-click the graph">Clean view</button>
    <button id="fsBtn" type="button" title="Shortcut: F. Esc to leave">Full screen</button>
  </div>
  <div id="wifiPanel" hidden>
    <div id="wifiNow">Looking for the board...</div>
    <label>Network name<input id="wName" type="text" maxlength="32" autocomplete="off" spellcheck="false"></label>
    <label>Password<input id="wPass" type="password" maxlength="63" autocomplete="off"></label>
    <label class="row"><input id="wShow" type="checkbox"> Show password</label>
    <button id="wSend" type="button">Send to the board</button>
    <div id="wifiStatus" role="status"></div>
    <div class="small">The board restarts and joins this network. If it can't, it goes back to the previous one by itself.</div>
    <div class="small">If it can't join any network, it opens its own WiFi called CSI-Setup: connect this laptop to it, keep this page open, and send the settings from here.</div>
  </div>
</div>

<div class="corner" id="meter">
  <div id="word">...</div>
  <div id="track"><div id="fill"></div></div>
  <p id="hint"></p>
</div>
<div class="corner" id="strip-label">Movement over the last minute</div>

<div class="corner" id="mini"><span id="miniWord"></span><span id="miniMuted"></span>
  <div id="miniHint">Double-click or press H to show the controls</div></div>

<div id="empty">Waiting for readings from the board...</div>

<script>
(function () {
  "use strict";

  // ---- settings ----------------------------------------------------------
  var ROWS = 90;            // slices kept on screen (about 4.5 seconds)
  var WINDOW = 20;          // slices used to judge movement (about 1 second)
  var HISTORY = 1200;       // points in the bottom strip (about 1 minute)

  // ---- state -------------------------------------------------------------
  var curLen = 0, altLen = 0, altCount = 0;
  var warm = [], idx = null, n = 0;
  var acc = null, accN = 0;
  var rows = [], devs = [], win = [];
  var base = null, floorLevel = 0, level = 0, ratio = 1;
  var activity = 0, hotEnv = 0;   // activity drives the word, the bar, the strip and the sound
  var hist = [];
  var meta = { src: "live", pps: 0, rssi: null, link: "", note: "", rate: 50 };
  var lastRowTime = 0, lastMsgTime = 0, streamDown = false;
  var sens = 1;                    // 1/3 .. 3, middle of the slider = 1
  // The change needed before a spot turns yellow. Everything (colours, word, bar,
  // sound, noise filter) uses this one threshold, so they always agree.
  function colourGate() { return 2.5 * floorLevel / sens; }
  function colourOf(dv, g) { return clamp((dv - g) / (4.8 * g), 0, 1); }
  var smoothPct = 60, calmRow = null;
  // Noise filter curve: how long the calm parts take to settle.
  // 80 % = about 1 s, 90 % = about 3 s, 100 % = about 10 s (deliberately a lot).
  function settleSeconds(pct) { return 10 * Math.pow(10, (pct - 100) / 20); }
  function keepPerStep(pct, stepsPerSecond) {
    return pct <= 0 ? 0 : Math.exp(-1 / (stepsPerSecond * settleSeconds(pct)));
  }
  try { var sp = parseInt(localStorage.getItem("csi_smooth"), 10); if (sp >= 0 && sp <= 100) smoothPct = sp; } catch (e) {}
  var yaw = -0.5, pitch = 0.62;

  function resetAll() {
    warm = []; idx = null; n = 0; acc = null; accN = 0;
    rows = []; devs = []; win = []; base = null;
    floorLevel = 0; level = 0; ratio = 1; activity = 0; hotEnv = 0;
  }
  function relearn() { base = null; floorLevel = 0; win = []; level = 0; ratio = 1; activity = 0; hotEnv = 0; }

  // ---- turning raw readings into terrain slices ---------------------------
  function buildMask() {
    var L = curLen, mean = new Array(L), k, i;
    for (k = 0; k < L; k++) {
      var s = 0;
      for (i = 0; i < warm.length; i++) s += warm[i][k];
      mean[k] = s / warm.length;
    }
    var nz = mean.filter(function (v) { return v > 0.01; }).sort(function (a, b) { return a - b; });
    if (nz.length < 8) { warm = []; return; }
    var median = nz[Math.floor(nz.length / 2)];
    idx = [];
    for (k = 0; k < L; k++) if (mean[k] > 0.3 * median) idx.push(k);   // drop the empty edge frequencies
    if (idx.length < 8) { idx = null; warm = []; return; }
    n = idx.length;
    acc = new Float32Array(n); accN = 0;
    warm = [];
  }

  function addFrame(a) {
    if (a.length !== curLen) {
      if (a.length === altLen) altCount++; else { altLen = a.length; altCount = 1; }
      if (curLen !== 0 && altCount <= 30) return;      // ignore the odd packet of another size
      curLen = a.length; altCount = 0; resetAll();
    } else {
      altCount = 0;
    }
    if (!idx) { warm.push(a); if (warm.length >= 40) buildMask(); return; }
    var s = 0, i;
    for (i = 0; i < n; i++) s += a[idx[i]];
    s /= n;
    if (!(s > 0)) return;
    // Divide by the average so that only the SHAPE of the signal counts,
    // not its overall loudness (the chip changes its own volume all the time).
    for (i = 0; i < n; i++) acc[i] += a[idx[i]] / s;
    accN++;
  }

  function pushRow() {
    var row = new Float32Array(n), dev = new Float32Array(n), i, j;
    for (i = 0; i < n; i++) row[i] = acc[i] / accN;
    acc.fill(0); accN = 0;
    if (!base) base = Float32Array.from(row);
    for (i = 0; i < n; i++) dev[i] = row[i] - base[i];

    // Noise filter: a point that is calm (would be drawn blue) is smoothed over time;
    // if it or a neighbour shows any change (orange or red), it is shown exactly as measured.
    var shown = row;
    if (floorLevel && smoothPct > 0) {
      if (!calmRow || calmRow.length !== n) calmRow = Float32Array.from(row);
      var keep = keepPerStep(smoothPct, 20), gate = colourGate();
      shown = new Float32Array(n);
      for (i = 0; i < n; i++) {
        var near = Math.max(Math.abs(dev[i]), i > 0 ? Math.abs(dev[i - 1]) : 0, i < n - 1 ? Math.abs(dev[i + 1]) : 0);
        if (near > gate) calmRow[i] = row[i];
        else calmRow[i] += (row[i] - calmRow[i]) * (1 - keep);
        shown[i] = calmRow[i];
      }
    } else {
      calmRow = null;
    }
    rows.push(shown); devs.push(dev); sliceCount++;
    if (rows.length > ROWS) { rows.shift(); devs.shift(); }
    win.push(row);
    if (win.length > WINDOW) win.shift();

    // How much did each frequency wobble during the last second?
    var m = 0;
    if (win.length >= 8) {
      for (i = 0; i < n; i++) {
        var mean = 0, v = 0;
        for (j = 0; j < win.length; j++) mean += win[j][i];
        mean /= win.length;
        for (j = 0; j < win.length; j++) { var d = win[j][i] - mean; v += d * d; }
        m += Math.sqrt(v / win.length);
      }
      m /= n;
    }
    if (win.length >= WINDOW) {
      // "floorLevel" is the wobble of an empty room. It follows quickly downwards
      // and only creeps upwards while things are calm.
      if (floorLevel === 0 || m < floorLevel) floorLevel = floorLevel === 0 ? m : floorLevel + (m - floorLevel) * 0.2;
      else if (m < floorLevel * 1.6) floorLevel += (m - floorLevel) * 0.01;
      else floorLevel += (m - floorLevel) * 0.0002;
      floorLevel = Math.max(floorLevel, 1e-4);
      ratio = m / floorLevel;
      level = 1 - Math.exp(-Math.max(0, ratio - 1.6) / 5);   // only used to pace the calm picture
    }
    // Activity = how much colour the newest slice of the terrain shows, computed the
    // same way the terrain is coloured. Blue bumps count for nothing; faint yellow
    // counts a little; large red areas count fully.
    var amount = 0;
    if (floorLevel && devs.length >= 2) {
      var g = colourGate(), dA = devs[devs.length - 1], dB = devs[devs.length - 2], ts = [];
      for (i = 0; i < n - 1; i++) {
        var dvc = (Math.abs(dA[i]) + Math.abs(dA[i + 1]) + Math.abs(dB[i]) + Math.abs(dB[i + 1])) * 0.25;
        var tc = colourOf(dvc, g);
        ts.push(tc >= 0.1 ? tc : 0);              // below 0.1 the spot still looks blue
      }
      lastHeat = ts.slice();                      // kept in order: used for stereo and pitch
      ts.sort(function (x, y) { return y - x; });
      var top = Math.max(3, Math.round(ts.length * 0.15)), hs = 0;
      for (i = 0; i < top; i++) hs += ts[i];
      amount = hs / top;
    }
    hotEnv = Math.max(amount, hotEnv * 0.85);      // holds a fraction of a second, no stutter
    activity = floorLevel && win.length >= WINDOW ? hotEnv : 0;

    // The "calm picture" follows the signal quickly when still, very slowly during movement.
    var alpha = level < 0.12 ? 0.03 : 0.003;
    for (i = 0; i < n; i++) base[i] += (row[i] - base[i]) * alpha;

    hist.push(activity);
    if (hist.length > HISTORY) hist.shift();
    lastRowTime = performance.now();

    // A calmer copy of the newest slice, used to pick the notes.
    if (!smoothRow || smoothRow.length !== n) smoothRow = Float32Array.from(row);
    else for (i = 0; i < n; i++) smoothRow[i] += (row[i] - smoothRow[i]) * 0.2;
    updateSound();
  }

  // ---- sound ----------------------------------------------------------------
  // Four modes, all driven by the same "activity" (how much colour the terrain shows):
  //   Drone           three soft voices, silent when still, louder with movement
  //   Groove          steady drum pattern; a melody whose notes come from the curve's shape
  //   Geiger          random clicks, denser with more movement
  //   Evolving groove steady kick; hats and percussion fill in with movement, on the beat
  // Notes are snapped to a scale so every combination sounds consonant.
  var SCALES = {
    "Minor pentatonic": [0, 3, 5, 7, 10],
    "Major pentatonic": [0, 2, 4, 7, 9],
    "Dorian": [0, 2, 3, 5, 7, 9, 10],
    "Lydian": [0, 2, 4, 6, 7, 9, 11]
  };
  var ROOTS = { "C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9 };
  var NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"];
  var MODES = ["Drone", "Groove", "Geiger", "Evolving groove"];
  // 16 steps: x = hit, o = soft hit, . = rest
  var PATTERNS = {
    "Four on the floor": { k: "x...x...x...x...", s: "....x.......x...", h: "..x...x...x...x." },
    "Lo-fi broken":      { k: "x......x..x.....", s: "....x.......x..o", h: "x.o.x.o.x.o.xoo." },
    "Minimal":           { k: "x.......x.......", s: "............x...", h: "....o.......o..." }
  };
  var snd = { on: false, volume: 0.7, mode: "Drone", scale: "Minor pentatonic", root: "A", wave: "triangle",
              bright: 1200, glide: 0.8, tail: 3, space: 0.5,
              bpm: 96, pattern: "Four on the floor", beatWhen: "Always", mel: "Pluck", click: "Tick",
              mixDrone: 0.6, mixDrums: 0.8, mixMel: 0.7, stereo: true };
  try {
    var saved = JSON.parse(localStorage.getItem("csi_sound") || "{}");
    for (var key in snd) if (key !== "on" && saved[key] !== undefined && typeof saved[key] === typeof snd[key]) snd[key] = saved[key];
    if (!SCALES[snd.scale]) snd.scale = "Minor pentatonic";
    if (ROOTS[snd.root] === undefined) snd.root = "A";
    if (MODES.indexOf(snd.mode) < 0) snd.mode = "Drone";
    if (!PATTERNS[snd.pattern]) snd.pattern = "Four on the floor";
  } catch (e) {}
  function saveSound() { try { localStorage.setItem("csi_sound", JSON.stringify(snd)); } catch (e) {} }

  var smoothRow = null, audio = null, previewUntil = 0, lastTarget = 0, boardMuted = false;
  var noteIdx = [null, null, null], noteMidi = [0, 0, 0];
  var lastHeat = null;                         // colour of each spot on the newest slice (0..1)
  var riffIdx = [], riffMidi = [], riffPos = [];
  var beatEnv = 0, clicksPerSec = 0;

  function midiHz(m) { return 440 * Math.pow(2, (m - 69) / 12); }
  function degreeMidi(deg, base) {
    var sc = SCALES[snd.scale];
    return base + ROOTS[snd.root] + sc[deg % sc.length] + 12 * Math.floor(deg / sc.length);
  }
  function currentLevel() {
    var lv = floorLevel && performance.now() - lastRowTime < 2000 ? activity : 0;
    if (performance.now() < previewUntil) lv = Math.max(lv, 0.6);
    return snd.on ? lv : 0;
  }

  function buildAudio() {
    var AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return null;
    var ctx = new AC(), a = { ctx: ctx, voices: [] };
    a.master = ctx.createGain();
    a.mute = ctx.createGain();
    a.limiter = ctx.createDynamicsCompressor();
    // Only a safety limiter, so loud and soft keep their difference.
    a.limiter.threshold.value = -4; a.limiter.knee.value = 4; a.limiter.ratio.value = 8;
    a.limiter.attack.value = 0.005; a.limiter.release.value = 0.3;
    a.master.connect(a.mute); a.mute.connect(a.limiter); a.limiter.connect(ctx.destination);

    // Reverb: three seconds of fading noise as the "room".
    var len = Math.floor(ctx.sampleRate * 3.2), buf = ctx.createBuffer(2, len, ctx.sampleRate);
    for (var ch = 0; ch < 2; ch++) {
      var d = buf.getChannelData(ch);
      for (var i = 0; i < len; i++) d[i] = (Math.random() * 2 - 1) * Math.pow(1 - i / len, 2.5);
    }
    a.conv = ctx.createConvolver(); a.conv.buffer = buf;
    a.wet = ctx.createGain(); a.conv.connect(a.wet); a.wet.connect(a.master);
    // One second of white noise, reused by the drums and clicks.
    a.noise = ctx.createBuffer(1, ctx.sampleRate, ctx.sampleRate);
    var nd = a.noise.getChannelData(0);
    for (i = 0; i < nd.length; i++) nd[i] = Math.random() * 2 - 1;

    // Layers: each has its own fader, a dry path and a reverb send.
    function layer(sendAmount) {
      var g = ctx.createGain(), dry = ctx.createGain(), send = ctx.createGain();
      g.connect(dry); dry.connect(a.master); g.connect(send); send.connect(a.conv);
      return { g: g, dry: dry, send: send, sendAmount: sendAmount };
    }
    a.drone = layer(1); a.mel = layer(0.8); a.drums = layer(0.15);

    // The drone: three voices, two slightly detuned oscillators each.
    a.filter = ctx.createBiquadFilter(); a.filter.type = "lowpass"; a.filter.Q.value = 0.6; a.filter.frequency.value = snd.bright;
    a.env = ctx.createGain(); a.env.gain.value = 0;
    var levels = [0.5, 0.32, 0];
    for (var v = 0; v < 3; v++) {
      var g = ctx.createGain(); g.gain.value = levels[v];
      var pan = ctx.createStereoPanner();
      var o1 = ctx.createOscillator(), o2 = ctx.createOscillator();
      o1.type = o2.type = snd.wave; o1.detune.value = -6; o2.detune.value = 6;
      o1.frequency.value = o2.frequency.value = 110 * (v + 1);
      o1.connect(g); o2.connect(g); g.connect(pan); pan.connect(a.filter); o1.start(); o2.start();
      a.voices.push({ g: g, o: [o1, o2], pan: pan });
    }
    a.filter.connect(a.env); a.env.connect(a.drone.g);
    a.mute.gain.value = boardMuted ? 0 : 1;
    return a;
  }

  // ---- one-shot instruments (all synthesized, no samples) ----
  function out(layerName, pos) {
    // A short-lived panner per note, so its position in the stereo field follows the spectrum.
    var ctx = audio.ctx, p = ctx.createStereoPanner();
    p.pan.value = snd.stereo && pos !== undefined ? clamp((pos - 0.5) * 1.6, -0.9, 0.9) : 0;
    p.connect(audio[layerName].g);
    return p;
  }
  function envGain(t, peak, attack, decay) {
    var g = audio.ctx.createGain();
    g.gain.setValueAtTime(0.0001, t);
    g.gain.exponentialRampToValueAtTime(Math.max(peak, 0.0002), t + attack);
    g.gain.exponentialRampToValueAtTime(0.0001, t + attack + decay);
    return g;
  }
  function noiseBurst(t, dur) {
    var s = audio.ctx.createBufferSource();
    s.buffer = audio.noise;
    s.start(t, Math.random() * 0.5, dur + 0.05);
    return s;
  }
  function kick(t, vel, decay) {
    var ctx = audio.ctx, o = ctx.createOscillator(), g = envGain(t, vel, 0.002, decay || 0.42);
    o.frequency.setValueAtTime(150, t); o.frequency.exponentialRampToValueAtTime(42, t + 0.12);
    o.connect(g); g.connect(out("drums")); o.start(t); o.stop(t + 0.5);
  }
  function snare(t, vel) {
    var ctx = audio.ctx, n = noiseBurst(t, 0.22), bp = ctx.createBiquadFilter(), g = envGain(t, vel * 0.55, 0.002, 0.18);
    bp.type = "bandpass"; bp.frequency.value = 1900; bp.Q.value = 0.7;
    n.connect(bp); bp.connect(g); g.connect(out("drums"));
    var o = ctx.createOscillator(), g2 = envGain(t, vel * 0.3, 0.002, 0.09);
    o.type = "triangle"; o.frequency.value = 190; o.connect(g2); g2.connect(out("drums")); o.start(t); o.stop(t + 0.15);
  }
  function hat(t, vel, open, pos) {
    var ctx = audio.ctx, n = noiseBurst(t, open ? 0.35 : 0.07), hp = ctx.createBiquadFilter();
    var g = envGain(t, vel * 0.28, 0.001, open ? 0.3 : 0.045);
    hp.type = "highpass"; hp.frequency.value = 7000;
    n.connect(hp); hp.connect(g); g.connect(out("drums", pos));
  }
  function perc(t, vel, pos) {          // a small tuned tom; pitch follows where the change is
    var ctx = audio.ctx, o = ctx.createOscillator(), g = envGain(t, vel * 0.4, 0.002, 0.14);
    var f = 220 + pos * 440;
    o.frequency.setValueAtTime(f * 1.4, t); o.frequency.exponentialRampToValueAtTime(f, t + 0.05);
    o.connect(g); g.connect(out("drums", pos)); o.start(t); o.stop(t + 0.2);
  }
  function tick(t, vel, pos) {          // the classic Geiger click
    var ctx = audio.ctx, n = noiseBurst(t, 0.006), hp = ctx.createBiquadFilter(), g = envGain(t, vel * 0.7, 0.0005, 0.004);
    hp.type = "highpass"; hp.frequency.value = 2500;
    n.connect(hp); hp.connect(g); g.connect(out("drums", pos));
  }
  function note(midi, t, vel, pos, kind, layerName) {
    var ctx = audio.ctx, f = midiHz(midi), dest = out(layerName || "mel", pos);
    if (kind === "Soft keys") {
      var g = envGain(t, vel * 0.32, 0.012, 0.8);
      [[1, 1], [2, 0.25]].forEach(function (h) {
        var o = ctx.createOscillator(), hg = ctx.createGain();
        o.frequency.value = f * h[0]; hg.gain.value = h[1];
        o.connect(hg); hg.connect(g); o.start(t); o.stop(t + 0.9);
      });
      g.connect(dest);
    } else if (kind === "Bell") {
      var c = ctx.createOscillator(), m = ctx.createOscillator(), mg = ctx.createGain(), bg = envGain(t, vel * 0.26, 0.003, 1.4);
      c.frequency.value = f; m.frequency.value = f * 3.5;
      mg.gain.setValueAtTime(f * 2.5 * (0.4 + vel), t); mg.gain.exponentialRampToValueAtTime(1, t + 0.7);
      m.connect(mg); mg.connect(c.frequency); c.connect(bg); bg.connect(dest);
      m.start(t); c.start(t); m.stop(t + 1.5); c.stop(t + 1.5);
    } else {                             // Pluck
      var short = kind === "Short pluck";
      var o2 = ctx.createOscillator(), lp = ctx.createBiquadFilter(), pg = envGain(t, vel * 0.3, 0.004, short ? 0.12 : 0.38);
      o2.type = "sawtooth"; o2.frequency.value = f;
      lp.type = "lowpass"; lp.Q.value = 2;
      lp.frequency.setValueAtTime(Math.min(9000, snd.bright * (1.2 + 2.5 * vel)), t);
      lp.frequency.exponentialRampToValueAtTime(320, t + (short ? 0.12 : 0.3));
      o2.connect(lp); lp.connect(pg); pg.connect(dest); o2.start(t); o2.stop(t + 0.5);
    }
  }

  // ---- the melody: 8 notes read from the shape of the curve ----
  // The curve is sampled at 8 places from left to right; the height at each place picks a
  // note of the scale. A still room gives a stable curve, so the melody becomes a loop;
  // movement reshapes the curve and the loop mutates.
  function updateRiff() {
    if (!smoothRow) return;
    var r = smoothRow, m = r.length, lo = Infinity, hi = -Infinity, i;
    for (i = 0; i < m; i++) { lo = Math.min(lo, r[i]); hi = Math.max(hi, r[i]); }
    var span = Math.max(hi - lo, 1e-6), steps = SCALES[snd.scale].length * 2;
    for (var k = 0; k < 8; k++) {
      var pos = (k + 0.5) / 8, u = (r[Math.round(pos * (m - 1))] - lo) / span;
      var want = u * steps;
      if (riffIdx[k] === undefined || riffIdx[k] > steps || Math.abs(want - riffIdx[k]) > 0.65) riffIdx[k] = Math.round(want);
      riffMidi[k] = degreeMidi(riffIdx[k], 48);
      riffPos[k] = pos;
    }
  }
  function hotPosition() {               // a random spot, chosen more often where it is redder
    if (!lastHeat) return Math.random();
    var total = 0, i;
    for (i = 0; i < lastHeat.length; i++) total += lastHeat[i];
    if (total <= 0) return Math.random();
    var r = Math.random() * total;
    for (i = 0; i < lastHeat.length; i++) { r -= lastHeat[i]; if (r <= 0) break; }
    return Math.min(i, lastHeat.length - 1) / Math.max(lastHeat.length - 1, 1);
  }
  function hit(pat, s) { var c = pat.charAt(s); return c === "x" ? 1 : (c === "o" ? 0.55 : 0); }

  // ---- the beat clock: plans notes a little ahead, so timing stays tight ----
  var seq = { next: 0, step: 0, timer: null, geigerNext: 0 };
  function playStep(s, t, lv) {
    var beatGain = snd.beatWhen === "Always" ? 1 : beatEnv;
    var melVel = 0.45 + 0.55 * lv, r;
    if (snd.mode === "Groove") {
      var p = PATTERNS[snd.pattern];
      if (beatGain > 0.02) {
        if ((r = hit(p.k, s))) kick(t, r * beatGain);
        if ((r = hit(p.s, s))) snare(t, r * beatGain);
        if ((r = hit(p.h, s))) hat(t, r * beatGain * (0.8 + 0.4 * lv), false, 0.5 + (Math.random() - 0.5) * 0.3);
      }
      // melody on every 8th; with strong movement also on the 16ths in between
      if (s % 2 === 0 || lv > 0.35) {
        var k = s % 2 === 0 ? (s / 2) % 8 : s % 8;
        if (riffMidi[k]) note(riffMidi[k], t, melVel, riffPos[k], snd.mel);
      }
    } else if (snd.mode === "Evolving groove") {
      if (s % 4 === 0) kick(t, beatGain);                       // the steady pulse
      if (beatGain > 0.02) {
        if ((s === 4 || s === 12) && lv > 0.1) snare(t, (0.4 + 0.6 * lv) * beatGain);
        var hatChance = (s % 4 === 2 ? 0.15 : 0) + (lv < 0.04 ? 0 : 0.1 + lv);
        if (Math.random() < hatChance) hat(t, (0.5 + 0.5 * lv) * beatGain, false, hotPosition());
        if (s === 14 && lv > 0.6) hat(t, 0.8 * beatGain, true, hotPosition());
        if (s % 2 === 1 && Math.random() < lv * 0.6) { var hp = hotPosition(); perc(t, (0.5 + 0.5 * lv) * beatGain, hp); }
      }
      // melody grows from silence to quarter notes, 8ths, then 16ths
      var every = lv < 0.04 ? 0 : (lv < 0.25 ? 4 : (lv < 0.55 ? 2 : 1));
      if (every && s % every === 0) {
        var k2 = (s / every) % 8;
        if (riffMidi[k2]) note(riffMidi[k2], t, melVel, riffPos[k2], snd.mel);
      }
    }
  }
  function scheduleGeiger(ahead, lv) {
    var ctx = audio.ctx, now = ctx.currentTime;
    var bass = snd.click === "Bass";
    var u = lv < 0.04 ? 0 : Math.pow((lv - 0.04) / 0.96, bass ? 1.2 : 1.6);
    // Bass beat: the kick drum from the rhythm modes, from about one per second when
    // faintly yellow up to 12 per second (a drum roll) in full red.
    clicksPerSec = lv < 0.04 ? 0 : (bass ? 1 + 11 * u : 1.5 + 45 * u);
    if (clicksPerSec <= 0) { seq.geigerNext = 0; return; }
    // seq.geigerNext is the time of the next hit: play it, then plan the one after.
    // First hit right away when movement starts; if movement suddenly grows, don't keep
    // waiting for a hit that was planned for a slower pace.
    if (seq.geigerNext === 0 || seq.geigerNext < now) seq.geigerNext = now + 0.02;
    if (seq.geigerNext - now > 1.25 / clicksPerSec) seq.geigerNext = now + 0.75 / clicksPerSec;
    while (seq.geigerNext < ahead) {
      var pos = hotPosition(), vel = 0.4 + 0.6 * lv;
      if (bass) {
        // shorter kicks when they come fast, so a drum roll stays punchy instead of a rumble
        kick(seq.geigerNext, 0.55 + 0.45 * lv, clamp(1.2 / clicksPerSec, 0.08, 0.42));
      } else if (snd.click === "Notes") {
        var deg = Math.round(pos * SCALES[snd.scale].length * 2);
        note(degreeMidi(deg, 60), seq.geigerNext, vel, pos, "Short pluck", "drums");
      } else {
        tick(seq.geigerNext, vel, pos);
      }
      if (bass) seq.geigerNext += (0.75 + 0.5 * Math.random()) / clicksPerSec;   // loosely regular, still alive
      else seq.geigerNext += -Math.log(1 - Math.random()) / clicksPerSec;      // random spacing, like real decay
    }
  }
  function schedule() {
    if (!audio || !snd.on || audio.ctx.state !== "running") return;
    var ctx = audio.ctx, ahead = ctx.currentTime + 0.12, lv = currentLevel();
    if (snd.mode === "Geiger") { scheduleGeiger(ahead, lv); return; }
    if (snd.mode === "Drone") { seq.next = 0; return; }
    if (seq.next < ctx.currentTime - 0.2 || seq.next === 0) seq.next = ctx.currentTime + 0.03;  // start, or after a pause
    var stepDur = 60 / snd.bpm / 4;
    while (seq.next < ahead) {
      playStep(seq.step % 16, seq.next, lv);
      seq.next += stepDur;
      seq.step++;
    }
  }

  function curveFeatures(r) {
    var m = r.length, sum = 0, ws = 0, i, m1 = 0, m2 = -1;
    for (i = 0; i < m; i++) { sum += r[i]; ws += r[i] * i; if (r[i] > r[m1]) m1 = i; }
    var gap = Math.max(2, Math.floor(m / 6));
    for (i = 0; i < m; i++) if (Math.abs(i - m1) >= gap && (m2 < 0 || r[i] > r[m2])) m2 = i;
    var centre = sum > 0 ? ws / sum / (m - 1) : 0.5;
    return [clamp((centre - 0.5) * 5 + 0.5, 0, 1), m1 / (m - 1), (m2 < 0 ? m1 : m2) / (m - 1)];
  }

  function updateSound() {
    if (!audio) return;
    var ctx = audio.ctx, t = ctx.currentTime, v;
    var lv = currentLevel();

    // Drone loudness: silent while there is no colour ("Still"); faint yellow is quiet,
    // big red is loud (about 12 times louder, around 20 dB).
    var target = lv < 0.04 ? 0 : 0.06 + 0.94 * Math.pow((lv - 0.04) / 0.96, 1.4);
    audio.env.gain.setTargetAtTime(target, t, target > lastTarget ? 0.25 : snd.tail / 3);
    lastTarget = target;
    beatEnv += ((lv >= 0.04 ? 1 : 0) - beatEnv) * (lv >= 0.04 ? 0.3 : 0.03);

    // Mixer: the drone always follows its fader; beat and melody only play in the rhythm modes.
    audio.master.gain.setTargetAtTime(snd.volume * 0.5, t, 0.05);
    audio.drone.g.gain.setTargetAtTime(snd.mixDrone, t, 0.1);
    audio.drums.g.gain.setTargetAtTime(snd.mode === "Drone" ? 0 : snd.mixDrums, t, 0.05);
    audio.mel.g.gain.setTargetAtTime(snd.mode === "Drone" || snd.mode === "Geiger" ? 0 : snd.mixMel, t, 0.05);
    ["drone", "mel", "drums"].forEach(function (name) {
      var L = audio[name];
      L.dry.gain.setTargetAtTime(1 - 0.5 * snd.space * L.sendAmount, t, 0.1);
      L.send.gain.setTargetAtTime(snd.space * L.sendAmount, t, 0.1);
    });
    audio.wet.gain.setTargetAtTime(0.9, t, 0.1);
    audio.filter.frequency.setTargetAtTime(snd.bright * (1 + 1.5 * lv), t, 0.3);
    audio.voices[2].g.gain.setTargetAtTime(lv > 0.3 ? 0.22 : 0, t, 0.8);

    var f = smoothRow ? curveFeatures(smoothRow) : [0.5, 0.5, 0.5];
    var scale = SCALES[snd.scale];
    var steps = [scale.length + 1, Math.round(scale.length * 1.4) + 1, scale.length + 1];
    for (v = 0; v < 3; v++) {
      var want = f[v] * (steps[v] - 1);
      // Only move to another note when the curve has clearly shifted, so notes do not flutter.
      if (noteIdx[v] === null || noteIdx[v] > steps[v] - 1 || Math.abs(want - noteIdx[v]) > 0.65) noteIdx[v] = Math.round(want);
      var semis = scale[noteIdx[v] % scale.length] + 12 * Math.floor(noteIdx[v] / scale.length);
      var midi = 36 + ROOTS[snd.root] + 12 * v + semis;
      noteMidi[v] = midi;
      for (var k = 0; k < 2; k++) {
        audio.voices[v].o[k].type = snd.wave;
        audio.voices[v].o[k].frequency.setTargetAtTime(midiHz(midi), t, Math.max(snd.glide / 3, 0.01));
      }
      audio.voices[v].pan.pan.setTargetAtTime(snd.stereo ? (f[v] - 0.5) * 1.4 : 0, t, 0.5);
    }
    updateRiff();
    window.__csiSound = { target: target, midi: noteMidi.slice(), riff: riffMidi.slice(), state: ctx.state,
                          mode: snd.mode, clicks: clicksPerSec, beat: beatEnv };
  }

  function setBoardMuted(m) {
    boardMuted = !!m;
    if (audio) audio.mute.gain.setTargetAtTime(boardMuted ? 0 : 1, audio.ctx.currentTime, 0.03);
  }

  // ---- recording to a WAV file (what you hear, after the volume) ----
  var REC_CODE = "class Rec extends AudioWorkletProcessor{process(i){var c=i[0];" +
    "if(c&&c.length){this.port.postMessage([c[0].slice(0),(c[1]||c[0]).slice(0)]);}return true;}}" +
    "registerProcessor('csi-rec',Rec);";
  var rec = { on: false, chunks: [], frames: 0, node: null, start: 0, ready: false };
  function startRecording(done) {
    var ctx = audio.ctx;
    function begin() {
      rec.chunks = []; rec.frames = 0; rec.on = true; rec.start = performance.now();
      if (!rec.node) {
        rec.node = new AudioWorkletNode(ctx, "csi-rec", { numberOfInputs: 1, numberOfOutputs: 1, outputChannelCount: [2] });
        rec.node.port.onmessage = function (e) {
          if (!rec.on) return;
          rec.chunks.push(e.data); rec.frames += e.data[0].length;
          if (rec.frames > ctx.sampleRate * 600) stopRecording();      // 10 minutes at most
        };
        var silent = ctx.createGain(); silent.gain.value = 0;
        audio.limiter.connect(rec.node); rec.node.connect(silent); silent.connect(ctx.destination);
      }
      done(true);
    }
    if (rec.ready) { begin(); return; }
    var url = URL.createObjectURL(new Blob([REC_CODE], { type: "application/javascript" }));
    ctx.audioWorklet.addModule(url).then(function () { rec.ready = true; begin(); })
      .catch(function () { done(false); });
  }
  function stopRecording() {
    if (!rec.on) return;
    rec.on = false;
    var n = rec.frames, rate = audio.ctx.sampleRate, buf = new ArrayBuffer(44 + n * 4), dv = new DataView(buf);
    function str(o, s) { for (var i = 0; i < s.length; i++) dv.setUint8(o + i, s.charCodeAt(i)); }
    str(0, "RIFF"); dv.setUint32(4, 36 + n * 4, true); str(8, "WAVE"); str(12, "fmt ");
    dv.setUint32(16, 16, true); dv.setUint16(20, 1, true); dv.setUint16(22, 2, true);
    dv.setUint32(24, rate, true); dv.setUint32(28, rate * 4, true); dv.setUint16(32, 4, true); dv.setUint16(34, 16, true);
    str(36, "data"); dv.setUint32(40, n * 4, true);
    var o = 44;
    rec.chunks.forEach(function (c) {
      for (var i = 0; i < c[0].length; i++) {
        dv.setInt16(o, clamp(c[0][i], -1, 1) * 32767, true);
        dv.setInt16(o + 2, clamp(c[1][i], -1, 1) * 32767, true);
        o += 4;
      }
    });
    rec.chunks = [];
    var d = new Date(), pad = function (x) { return (x < 10 ? "0" : "") + x; };
    var name = "wifi-sound-" + d.getFullYear() + pad(d.getMonth() + 1) + pad(d.getDate()) + "-" +
               pad(d.getHours()) + pad(d.getMinutes()) + pad(d.getSeconds()) + ".wav";
    var a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([buf], { type: "audio/wav" }));
    a.download = name;
    document.body.appendChild(a); a.click(); a.remove();
    updateRecButton();
  }
  function updateRecButton() {
    var el = document.getElementById("recBtn");
    if (!el) return;
    if (rec.on) {
      var s = Math.floor((performance.now() - rec.start) / 1000);
      el.textContent = "Stop and save (" + Math.floor(s / 60) + ":" + (s % 60 < 10 ? "0" : "") + (s % 60) + ")";
      el.setAttribute("aria-pressed", "true");
    } else {
      el.textContent = "Record";
      el.setAttribute("aria-pressed", "false");
    }
  }

  function noteName(midi) { return NOTE_NAMES[midi % 12] + (Math.floor(midi / 12) - 1); }

  // ---- data connection ----------------------------------------------------
  var es = new EventSource("/events");
  es.onmessage = function (ev) {
    var msg;
    try { msg = JSON.parse(ev.data); } catch (e) { return; }
    streamDown = false; lastMsgTime = performance.now();
    meta = msg;
    if (msg.muted !== undefined && !!msg.muted !== boardMuted) {
      setBoardMuted(msg.muted);
      if (snd.on) elSoundBtn.textContent = boardMuted ? "Sound: on (muted from the board)" : "Sound: on";
    }
    if (msg.rate && document.activeElement !== elRate && String(msg.rate) !== elRate.value) elRate.value = String(msg.rate);
    var list = msg.rows || [];
    for (var i = 0; i < list.length; i++) addFrame(list[i]);
    if (idx && accN > 0) pushRow();
  };
  es.onerror = function () { streamDown = true; };

  // ---- controls -----------------------------------------------------------
  var canvas = document.getElementById("view"), ctx = canvas.getContext("2d");
  var elStatus = document.getElementById("status"), elDemo = document.getElementById("demo");
  var elWord = document.getElementById("word"), elFill = document.getElementById("fill");
  var elHint = document.getElementById("hint"), elEmpty = document.getElementById("empty");

  var elRate = document.getElementById("rate");
  elRate.addEventListener("change", function () {
    fetch("/rate", { method: "POST", body: elRate.value }).catch(function () {});
    relearn();   // fewer or more readings changes the noise level, so learn "calm" again
  });
  var elSens = document.getElementById("sens");
  try { var sv = parseFloat(localStorage.getItem("csi_sens")); if (sv >= -1 && sv <= 1) elSens.value = String(sv); } catch (e) {}
  function applySens() {
    var v = parseFloat(elSens.value);
    sens = Math.pow(3, v);
    try { localStorage.setItem("csi_sens", String(v)); } catch (e) {}
    fetch("/sens", { method: "POST", body: String(Math.round(v * 100 + 100)) }).catch(function () {});
  }
  elSens.addEventListener("input", applySens);
  applySens();
  var elQuality = document.getElementById("quality");
  elQuality.value = qualityChoice;
  updateQualityLabel();
  elQuality.addEventListener("change", function () {
    qualityChoice = elQuality.value;
    try { localStorage.setItem("csi_quality", qualityChoice); } catch (e) {}
    setQualityLevel(qualityChoice === "auto" ? qualityLevel : qualityChoice);
    updateQualityLabel();
  });
  var elSmooth = document.getElementById("smooth");
  elSmooth.value = String(smoothPct);
  // The board gets the same settling time, so its screen matches the page.
  function sendSmooth() {
    var ms = smoothPct <= 0 ? 0 : Math.min(10000, Math.round(settleSeconds(smoothPct) * 1000));
    fetch("/smooth", { method: "POST", body: String(ms) }).catch(function () {});
  }
  elSmooth.addEventListener("input", function () {
    smoothPct = parseInt(elSmooth.value, 10);
    try { localStorage.setItem("csi_smooth", String(smoothPct)); } catch (e) {}
    sendSmooth();
  });
  sendSmooth();
  document.getElementById("relearn").addEventListener("click", relearn);

  var elSoundBtn = document.getElementById("soundBtn"), elPanel = document.getElementById("soundPanel");
  var elSoundSet = document.getElementById("soundSetBtn");
  // The sound settings panel can be closed while the sound keeps playing.
  var soundPanelOpen = true;
  try { if (localStorage.getItem("csi_sound_panel") === "closed") soundPanelOpen = false; } catch (e) {}
  function showSoundPanel(open) {
    elPanel.hidden = !open;
    elSoundSet.setAttribute("aria-expanded", open ? "true" : "false");
    elSoundSet.textContent = open ? "Hide sound settings" : "Sound settings";
  }
  elSoundSet.addEventListener("click", function () {
    soundPanelOpen = elPanel.hidden;
    try { localStorage.setItem("csi_sound_panel", soundPanelOpen ? "open" : "closed"); } catch (e) {}
    if (soundPanelOpen) { elWifiPanel.hidden = true; elWifiBtn.setAttribute("aria-expanded", "false"); }
    showSoundPanel(soundPanelOpen);
  });
  var elNotes = document.getElementById("notes");
  var elScale = document.getElementById("sScale"), elRoot = document.getElementById("sRoot");
  Object.keys(SCALES).forEach(function (name) { elScale.add(new Option(name, name)); });
  Object.keys(ROOTS).forEach(function (name) { elRoot.add(new Option(name, name)); });
  MODES.forEach(function (name) { document.getElementById("sMode").add(new Option(name, name)); });
  Object.keys(PATTERNS).forEach(function (name) { document.getElementById("sPattern").add(new Option(name, name)); });
  var soundInputs = { sVolume: "volume", sMode: "mode", sScale: "scale", sRoot: "root", sWave: "wave",
                      sBright: "bright", sGlide: "glide", sTail: "tail", sSpace: "space",
                      sBpm: "bpm", sPattern: "pattern", sBeatWhen: "beatWhen", sMel: "mel", sClick: "click",
                      sMixDrone: "mixDrone", sMixDrums: "mixDrums", sMixMel: "mixMel", sStereo: "stereo" };
  function showModeRows() {
    // show only the settings that matter for the chosen mode
    Array.prototype.forEach.call(document.querySelectorAll("#soundPanel [data-modes]"), function (el) {
      el.classList.toggle("off", el.getAttribute("data-modes").split("|").indexOf(snd.mode) < 0);
    });
    document.getElementById("lBpm").textContent = "Tempo " + snd.bpm + " BPM";
    document.getElementById("lMixDrums").textContent = snd.mode === "Geiger" ? "Clicks" : "Drums";
  }
  Object.keys(soundInputs).forEach(function (id) {
    var el = document.getElementById(id), key = soundInputs[id];
    if (el.type === "checkbox") el.checked = !!snd[key]; else el.value = String(snd[key]);
    el.addEventListener(el.tagName === "SELECT" || el.type === "checkbox" ? "change" : "input", function () {
      if (el.type === "checkbox") snd[key] = el.checked;
      else snd[key] = typeof snd[key] === "number" ? parseFloat(el.value) : el.value;
      if (key === "scale" || key === "root") { noteIdx = [null, null, null]; riffIdx = []; }
      if (key === "mode") seq.next = 0;
      showModeRows(); saveSound(); updateSound();
    });
  });
  showModeRows();
  setInterval(function () { schedule(); if (rec.on) updateRecButton(); }, 25);
  document.getElementById("recBtn").addEventListener("click", function () {
    if (!audio || !snd.on) return;
    if (rec.on) { stopRecording(); return; }
    startRecording(function (ok) {
      if (!ok) document.getElementById("recBtn").textContent = "Recording is not available in this browser";
      else updateRecButton();
    });
  });
  elSoundBtn.addEventListener("click", function () {
    elWifiPanel.hidden = true; elWifiBtn.setAttribute("aria-expanded", "false");
    snd.on = !snd.on;
    if (snd.on && !audio) audio = buildAudio();
    if (snd.on && !audio) { snd.on = false; elSoundBtn.textContent = "Sound is not available in this browser"; return; }
    if (snd.on && audio.ctx.state === "suspended") audio.ctx.resume();
    if (!snd.on && rec.on) stopRecording();
    elSoundBtn.textContent = snd.on ? (boardMuted ? "Sound: on (muted from the board)" : "Sound: on") : "Sound: off";
    elSoundBtn.setAttribute("aria-pressed", snd.on ? "true" : "false");
    elSoundSet.hidden = !snd.on;
    showSoundPanel(snd.on && soundPanelOpen);
    updateSound();
  });
  document.getElementById("preview").addEventListener("click", function () {
    previewUntil = performance.now() + 5000;
    updateSound();
    setTimeout(updateSound, 5100);     // let it fade out again even if no readings arrive
  });

  var elWifiBtn = document.getElementById("wifiBtn"), elWifiPanel = document.getElementById("wifiPanel");
  var elWifiNow = document.getElementById("wifiNow"), elWifiStatus = document.getElementById("wifiStatus");
  var elWName = document.getElementById("wName"), elWPass = document.getElementById("wPass");
  var localWifiMsg = "";
  elWifiBtn.addEventListener("click", function () {
    var open = elWifiPanel.hidden;
    elWifiPanel.hidden = !open;
    elWifiBtn.setAttribute("aria-expanded", open ? "true" : "false");
    if (open) { showSoundPanel(false); elWName.focus(); }
  });
  document.getElementById("wShow").addEventListener("change", function (e) {
    elWPass.type = e.target.checked ? "text" : "password";
  });
  document.getElementById("wSend").addEventListener("click", function () {
    var name = elWName.value.trim();
    if (!name) { localWifiMsg = "Type the network name first."; elWifiStatus.textContent = localWifiMsg; return; }
    localWifiMsg = "";
    fetch("/wifi", { method: "POST", headers: { "Content-Type": "application/json" },
                     body: JSON.stringify({ ssid: name, password: elWPass.value }) })
      .then(function (r) { return r.json(); })
      .then(function (res) {
        if (!res.ok) { localWifiMsg = res.msg; elWifiStatus.textContent = res.msg; }
        else { elWPass.value = ""; }
      })
      .catch(function () { localWifiMsg = "The Python script is not responding."; elWifiStatus.textContent = localWifiMsg; });
  });

  // ---- clean view and full screen ----
  var view = { clean: false, cleanBeforeFs: false, hintTimer: 0, idleTimer: 0 };
  var elMini = document.getElementById("mini"), elMiniHint = document.getElementById("miniHint");
  var elMiniWord = document.getElementById("miniWord"), elMiniMuted = document.getElementById("miniMuted");
  var elCleanBtn = document.getElementById("cleanBtn"), elFsBtn = document.getElementById("fsBtn");
  function isFullscreen() { return !!(document.fullscreenElement || document.webkitFullscreenElement); }
  function setClean(on, remember) {
    view.clean = !!on;
    document.body.classList.toggle("clean", view.clean);
    document.body.classList.remove("peek");
    if (remember !== false) { try { localStorage.setItem("csi_clean", view.clean ? "1" : "0"); } catch (e) {} }
    if (view.clean) {                       // show the hint for a few seconds
      elMiniHint.style.opacity = "1";
      elMiniHint.textContent = isFullscreen() ? "Move the mouse for the controls. Esc to leave full screen"
                                               : "Double-click or press H to show the controls";
      clearTimeout(view.hintTimer);
      view.hintTimer = setTimeout(function () { elMiniHint.style.opacity = "0"; }, 4000);
    }
  }
  function toggleFullscreen() {
    if (isFullscreen()) {
      (document.exitFullscreen || document.webkitExitFullscreen).call(document);
    } else {
      var el = document.documentElement, req = el.requestFullscreen || el.webkitRequestFullscreen;
      if (req) { var pr = req.call(el); if (pr && pr.catch) pr.catch(function () {}); }
    }
  }
  function onFullscreenChange() {
    if (isFullscreen()) {
      view.cleanBeforeFs = view.clean;
      setClean(true, false);
      elFsBtn.textContent = "Leave full screen";
    } else {
      setClean(view.cleanBeforeFs, false);
      document.body.classList.remove("idle");
      elFsBtn.textContent = "Full screen";
    }
    setTimeout(resize, 50);
  }
  document.addEventListener("fullscreenchange", onFullscreenChange);
  document.addEventListener("webkitfullscreenchange", onFullscreenChange);
  elCleanBtn.addEventListener("click", function () { setClean(!view.clean); });
  elFsBtn.addEventListener("click", toggleFullscreen);
  canvas.addEventListener("dblclick", function (e) {
    e.preventDefault();
    if (isFullscreen()) { document.body.classList.toggle("peek"); return; }
    setClean(!view.clean);
  });
  function hideWhenIdle() {
    var ctl = document.getElementById("controls"), act = document.activeElement;
    if (ctl.matches(":hover") || (act && act !== document.body && ctl.contains(act) && act.tagName !== "BUTTON")) {
      view.idleTimer = setTimeout(hideWhenIdle, 1000);   // keep them while you use them
      return;
    }
    document.body.classList.remove("peek");
    document.body.classList.add("idle");
  }
  // In full screen the controls appear while the mouse moves, and fade after 3 s.
  window.addEventListener("mousemove", function (e) {
    if (!view.clean || !isFullscreen()) return;
    document.body.classList.add("peek");
    document.body.classList.remove("idle");
    clearTimeout(view.idleTimer);
    view.idleTimer = setTimeout(hideWhenIdle, 3000);
  });
  try { if (localStorage.getItem("csi_clean") === "1") setClean(true, false); } catch (e) {}

  var drag = null;
  canvas.addEventListener("pointerdown", function (e) { drag = { x: e.clientX, y: e.clientY }; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener("pointermove", function (e) {
    if (!drag) return;
    yaw = clamp(yaw + (e.clientX - drag.x) * 0.005, -1.25, 1.25);
    pitch = clamp(pitch + (e.clientY - drag.y) * 0.004, 0.2, 1.2);
    drag = { x: e.clientX, y: e.clientY };
  });
  canvas.addEventListener("pointerup", function () { drag = null; });
  canvas.addEventListener("pointercancel", function () { drag = null; });
  window.addEventListener("keydown", function (e) {
    if (e.target && (e.target.tagName === "INPUT" || e.target.tagName === "SELECT")) return;
    if (e.key === "h" || e.key === "H") { if (!isFullscreen()) setClean(!view.clean); return; }
    if (e.key === "f" || e.key === "F") { toggleFullscreen(); return; }
    if (e.key === "ArrowLeft") yaw = clamp(yaw - 0.08, -1.25, 1.25);
    if (e.key === "ArrowRight") yaw = clamp(yaw + 0.08, -1.25, 1.25);
    if (e.key === "ArrowUp") pitch = clamp(pitch + 0.06, 0.2, 1.2);
    if (e.key === "ArrowDown") pitch = clamp(pitch - 0.06, 0.2, 1.2);
  });

  // Quality levels. rowStep/colStep: draw every Nth slice / frequency (bigger tiles).
  // sharp: use the screen's full sharpness. ms: time between pictures.
  var QUALITY = {
    high:   { rowStep: 1, colStep: 1, sharp: true,  ms: 30, outline: true },
    medium: { rowStep: 2, colStep: 1, sharp: false, ms: 30, outline: true },
    low:    { rowStep: 2, colStep: 2, sharp: false, ms: 45, outline: false }
  };
  var qualityChoice = "auto", qualityLevel = "high";
  try {
    var qs = localStorage.getItem("csi_quality");
    if (qs === "auto" || QUALITY[qs]) qualityChoice = qs;
    var ql = localStorage.getItem("csi_quality_auto");
    if (qualityChoice === "auto" && QUALITY[ql]) qualityLevel = ql;   // start where Auto ended last time
  } catch (e) {}
  if (qualityChoice !== "auto") qualityLevel = qualityChoice;
  var drawTimes = [], lastAutoChange = 0;

  var W = 0, H = 0, dpr = 1;
  function resize() {
    dpr = QUALITY[qualityLevel].sharp ? Math.min(window.devicePixelRatio || 1, 2) : 1;
    W = window.innerWidth; H = window.innerHeight;
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  }
  window.addEventListener("resize", resize);
  resize();

  // ---- drawing ------------------------------------------------------------
  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  var SEA = [16, 24, 51], CALM = [124, 195, 207], WARM = [255, 180, 84], HOT = [255, 90, 110], INK = [236, 232, 220];
  function heat(t) {                    // calm -> warm -> hot
    var a = t < 0.5 ? CALM : WARM, b = t < 0.5 ? WARM : HOT, u = t < 0.5 ? t * 2 : (t - 0.5) * 2;
    return [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u];
  }
  function rgb(c, light, fade) {        // light: shading, fade: 1 = near, 0 = melts into the background
    var r = SEA[0] + (c[0] * light - SEA[0]) * fade;
    var g = SEA[1] + (c[1] * light - SEA[1]) * fade;
    var b = SEA[2] + (c[2] * light - SEA[2]) * fade;
    return "rgb(" + (r | 0) + "," + (g | 0) + "," + (b | 0) + ")";
  }

  var lastDraw = 0, px = [], py = [], sliceCount = 0;
  function frame(now) {
    requestAnimationFrame(frame);
    if (now - lastDraw < QUALITY[qualityLevel].ms) return;
    lastDraw = now;
    updateText(now);

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    var bg = ctx.createLinearGradient(0, 0, 0, H);
    bg.addColorStop(0, "#080d20"); bg.addColorStop(1, "#101833");
    ctx.fillStyle = bg; ctx.fillRect(0, 0, W, H);

    var R = rows.length;
    elEmpty.style.display = R < 2 ? "flex" : "none";
    var t0 = performance.now();
    if (R >= 2) drawTerrain(R);
    drawStrip();
    autoQuality(performance.now() - t0, R);
  }

  // Auto: if drawing takes too long, step down one level; if there is a lot of
  // headroom, step back up. Waits a few seconds between changes.
  function setQualityLevel(lv) {
    if (lv === qualityLevel) return;
    qualityLevel = lv;
    drawTimes = [];
    lastAutoChange = performance.now();
    try { if (qualityChoice === "auto") localStorage.setItem("csi_quality_auto", lv); } catch (e) {}
    resize();
    updateQualityLabel();
  }
  function autoQuality(ms, R) {
    if (qualityChoice !== "auto" || R < 10) return;
    drawTimes.push(ms);
    if (drawTimes.length < 40) return;
    if (drawTimes.length > 40) drawTimes.shift();
    if (performance.now() - lastAutoChange < 4000) return;
    var avg = drawTimes.reduce(function (x, y) { return x + y; }, 0) / drawTimes.length;
    var order = ["high", "medium", "low"], k = order.indexOf(qualityLevel);
    if (avg > 20 && k < 2) setQualityLevel(order[k + 1]);
    else if (avg < 5 && k > 0) setQualityLevel(order[k - 1]);
  }
  function updateQualityLabel() {
    var el = document.getElementById("quality");
    if (!el) return;
    el.options[0].text = qualityChoice === "auto" ? "Auto (" + qualityLevel + ")" : "Auto";
  }

  function drawTerrain(R) {
    var cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
    var scale = Math.min(W * 0.25, H * 0.40), cx = W * 0.55, cyScreen = H * 0.5;
    var DEPTH = 2.2, CAM = 7, HEIGHT = 0.42;

    function project(x, y, z, out) {
      var X = x * cy - z * sy, Z = x * sy + z * cy;
      var Y = y * cp + Z * sp, Z2 = -y * sp + Z * cp;
      var f = CAM / (CAM + Z2);
      out[0] = cx + X * f * scale; out[1] = cyScreen - Y * f * scale;
    }

    var r, i, tmp = [0, 0];
    for (r = 0; r < R; r++) {
      if (!px[r] || px[r].length !== n) { px[r] = new Float32Array(n); py[r] = new Float32Array(n); }
      var age = R - 1 - r;                                  // 0 = newest slice, at the front
      var z = -DEPTH / 2 + (age / (ROWS - 1)) * DEPTH;
      var row = rows[r];
      for (i = 0; i < n; i++) {
        project(-1 + 2 * i / (n - 1), Math.max(row[i] - 0.2, 0.02) * HEIGHT, z, tmp);
        px[r][i] = tmp[0]; py[r][i] = tmp[1];
      }
    }

    var cGate = floorLevel ? colourGate() : 0;
    var Q = QUALITY[qualityLevel], RS = Q.rowStep, CS = Q.colStep;
    // Which slices and frequencies to draw; the newest slice and both edges are always included.
    var rs = [], cols = [], k, j;
    // Tiles stay attached to the same slices as they slide back, so nothing flickers.
    for (r = 0; r < R - 1; r++) if ((sliceCount - R + r) % RS === 0) rs.push(r);
    rs.push(R - 1);
    for (i = 0; i < n - 1; i += CS) cols.push(i);
    cols.push(n - 1);
    var j0 = sy > 0 ? cols.length - 2 : 0, j1 = sy > 0 ? -1 : cols.length - 1, jstep = sy > 0 ? -1 : 1;
    ctx.lineWidth = 0.7; ctx.lineJoin = "round";
    for (k = 0; k < rs.length - 1; k++) {                   // far slices first, near ones on top
      var ra = rs[k], rb = rs[k + 1];
      var fade = 0.3 + 0.7 * (1 - (R - 1 - rb) / ROWS);
      var a = rows[rb], d = devs[rb], d0 = devs[ra];
      for (j = j0; j !== j1; j += jstep) {
        var i2, ia = cols[j], ib = cols[j + 1], dv;
        if (CS === 1 && RS === 1) {
          dv = (Math.abs(d[ia]) + Math.abs(d[ib]) + Math.abs(d0[ia]) + Math.abs(d0[ib])) * 0.25;
        } else {
          // a bigger tile takes the strongest change it covers, so no colour is lost
          dv = 0;
          for (var rr = ra + 1; rr <= rb; rr++) {
            var dd = devs[rr], dp = devs[rr - 1];
            for (i2 = ia; i2 < ib; i2++) {
              dv = Math.max(dv, (Math.abs(dd[i2]) + Math.abs(dd[i2 + 1]) + Math.abs(dp[i2]) + Math.abs(dp[i2 + 1])) * 0.25);
            }
          }
        }
        var t = cGate ? colourOf(dv, cGate) : 0;
        var light = clamp(0.82 + (a[ib] - a[ia]) / (ib - ia) * 3 * (sy > 0 ? -1 : 1), 0.55, 1.1);
        var col = rgb(heat(t), light, fade);
        ctx.fillStyle = col; if (Q.outline) ctx.strokeStyle = col;
        ctx.beginPath();
        ctx.moveTo(px[ra][ia], py[ra][ia]); ctx.lineTo(px[ra][ib], py[ra][ib]);
        ctx.lineTo(px[rb][ib], py[rb][ib]); ctx.lineTo(px[rb][ia], py[rb][ia]);
        ctx.closePath(); ctx.fill(); if (Q.outline) ctx.stroke();
      }
    }

    // Front wall and the bright "right now" line.
    var zf = -DEPTH / 2, f = R - 1, p = [0, 0];
    ctx.beginPath();
    project(-1, 0, zf, p); ctx.moveTo(p[0], p[1]);
    for (i = 0; i < n; i++) ctx.lineTo(px[f][i], py[f][i]);
    project(1, 0, zf, p); ctx.lineTo(p[0], p[1]);
    ctx.closePath(); ctx.fillStyle = "rgba(8,13,32,0.85)"; ctx.fill();
    ctx.beginPath();
    for (i = 0; i < n; i++) { if (i) ctx.lineTo(px[f][i], py[f][i]); else ctx.moveTo(px[f][i], py[f][i]); }
    ctx.strokeStyle = "rgb(" + INK.join(",") + ")"; ctx.lineWidth = 1.6; ctx.stroke();

    // Plain-language axis labels.
    if (view.clean) return;
    ctx.font = "14px Bahnschrift, 'Segoe UI', system-ui, sans-serif";
    ctx.fillStyle = "rgba(151,161,192,0.95)";
    ctx.textBaseline = "top";
    project(-1, 0, zf, p); ctx.textAlign = "left"; ctx.fillText("lower frequencies", p[0], p[1] + 10);
    project(1, 0, zf, p); ctx.textAlign = "right"; ctx.fillText("higher frequencies", p[0], p[1] + 10);
    ctx.textBaseline = "middle";
    var side = sy > 0 ? -1 : 1;
    ctx.textAlign = side > 0 ? "left" : "right";
    project(side * 1.06, 0, zf, p); ctx.fillText("now", p[0], p[1]);
    project(side * 1.06, 0, DEPTH / 2, p); ctx.fillText("a few seconds ago", p[0], p[1]);
  }

  function drawStrip() {
    if (view.clean) return;
    var x0 = 28, x1 = W - 28, yb = H - 22, h = 44, L = hist.length;
    ctx.strokeStyle = "rgba(236,232,220,0.18)"; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x0, yb + 0.5); ctx.lineTo(x1, yb + 0.5); ctx.stroke();
    if (L < 2) return;
    var stepX = (x1 - x0) / (HISTORY - 1), start = x1 - (L - 1) * stepX, i;
    for (i = 0; i < L; i++) {
      var v = hist[i];
      if (v < 0.01) continue;
      ctx.fillStyle = rgb(heat(v), 1, 0.95);
      ctx.fillRect(start + i * stepX, yb - v * h, Math.max(stepX, 1), v * h);
    }
  }

  var lastText = 0;
  function updateText(now) {
    if (now - lastText < 200) return;
    lastText = now;
    elDemo.style.display = meta.src === "demo" ? "block" : "none";

    var quiet = now - lastRowTime > 2000;
    if (streamDown) {
      elStatus.textContent = "Lost contact with the Python script. Start it again; this page reconnects by itself.";
    } else if (!lastMsgTime) {
      elStatus.textContent = "Connecting to the Python script...";
    } else if (quiet) {
      elStatus.textContent = meta.note || "No readings from the board. Check the PowerShell window for messages.";
    } else {
      var where = meta.src === "demo" ? "Simulated" : "Live from the board" + (meta.link ? " over " + meta.link : "");
      var sig = meta.rssi === null || meta.rssi === undefined ? "" : ", signal " + meta.rssi + " dBm";
      elStatus.textContent = where + ", " + meta.pps + " readings per second" + sig;
    }

    var word, hint = "";
    if (rows.length < 2 || quiet) { word = "No signal yet"; }
    else if (!floorLevel) { word = "Learning..."; hint = "Stay still for a couple of seconds."; }
    else if (activity < 0.04) { word = "Still"; hint = "Drag the terrain to turn it."; }
    else if (activity < 0.45) { word = "Movement"; }
    else { word = "Strong movement"; }
    elWord.textContent = word; elHint.textContent = hint;
    if (view.clean) {
      elMiniWord.textContent = word;
      elMiniMuted.textContent = boardMuted && snd.on ? "MUTED" : "";
    }
    if (!elWifiPanel.hidden) {
      if (meta.src === "demo") elWifiNow.textContent = "Not available in demo mode.";
      else if (!meta.bip) elWifiNow.textContent = "Looking for the board...";
      else if (meta.bssid === null || meta.bssid === undefined) elWifiNow.textContent = "Board found at " + meta.bip + ".";
      else if (meta.bssid === "") elWifiNow.textContent = "Board at " + meta.bip + " (older firmware: flash the new one to use this).";
      else elWifiNow.textContent = "Board is on \u201c" + meta.bssid + "\u201d at " + meta.bip + ".";
      var serverMsg = meta.wifi || "";
      if (serverMsg) localWifiMsg = "";
      elWifiStatus.textContent = localWifiMsg || serverMsg;
    }
    if (snd.on && audio) {
      var lvNow = currentLevel();
      if (boardMuted) elNotes.textContent = "Muted with the board's BOOT button";
      else if (snd.mode === "Geiger") elNotes.textContent = lvNow < 0.04 ? "Silent (room is still)" : Math.round(clicksPerSec) + (snd.click === "Bass" ? " beats per second" : " clicks per second");
      else if (snd.mode === "Drone") elNotes.textContent = lastTarget > 0 || now < previewUntil
        ? "Playing " + noteName(noteMidi[0]) + ", " + noteName(noteMidi[1]) + (activity > 0.3 ? ", " + noteName(noteMidi[2]) : "")
        : "Silent (room is still)";
      else elNotes.textContent = riffMidi.length ? "Melody: " + riffMidi.map(noteName).join(" ") : "Waiting for the curve...";
    }
    var shown = floorLevel && !quiet ? activity : 0;
    elFill.style.width = Math.round(shown * 100) + "%";
    elFill.style.background = rgb(heat(shown), 1, 1);
  }

  requestAnimationFrame(frame);
})();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
