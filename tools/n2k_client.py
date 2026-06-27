#!/usr/bin/env python3
"""Minimal SeaTalkng/NMEA2000 JSON client.

Connects to the gateway (the host simulator now, or the ESP32 over WiFi later),
reads newline-delimited JSON, and pretty-prints a live instrument panel.

    python3 n2k_client.py [host] [port]
    python3 n2k_client.py 192.168.1.50 2000     # ESP32 over WiFi

The wire format is identical for the simulator and the ESP32, so software you
build against this works unchanged on the real boat.
"""
import json
import socket
import sys
import time

def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 2000
    max_msgs = int(sys.argv[3]) if len(sys.argv) > 3 else 0  # 0 = run forever

    print(f"connecting to {host}:{port} ...")
    sock = socket.create_connection((host, port), timeout=5)
    print("connected. streaming:\n")

    state = {}          # latest value per message type
    buf = b""
    count = 0
    sock.settimeout(5)
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                print("\nconnection closed by server")
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue
                state[msg.get("type", msg.get("pgn"))] = msg
                count += 1
                render(state)
                if max_msgs and count >= max_msgs:
                    return
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

def render(state):
    w = state.get("wind", {})
    d = state.get("depth", {})
    s = state.get("stw", {})
    h = state.get("heading", {})
    p = state.get("position", {})
    parts = []
    if h: parts.append(f"HDG {h['heading_deg']:5.1f}°{h['ref'][0]}")
    if s: parts.append(f"STW {s['stw_kn']:5.2f}kn")
    if w: parts.append(f"WIND {w['speed_kn']:5.2f}kn@{w['angle_deg']:5.1f}° {w['ref']}")
    if d: parts.append(f"DEPTH {d['depth_m']:5.2f}m")
    if p: parts.append(f"POS {p['lat']:.5f},{p['lon']:.5f}")
    # single updating line
    sys.stdout.write("\r" + "  ".join(parts) + " " * 8)
    sys.stdout.flush()

if __name__ == "__main__":
    main()
