# Pharma Rover: Dedicated Network Setup & IP Routing Guide

> [!IMPORTANT]
> **Use a Phone Hotspot or Dedicated 2.4 GHz Router:**
> Do NOT connect the robot devices to shared university / campus Wi-Fi networks. Campus networks frequently enable **Client Isolation** (which prevents the laptop from reaching the ESP32 or phone camera) and randomize DHCP IP leases.

---

## 1. Network Topology & Static DHCP Reservations

Set up a dedicated 2.4 GHz Wi-Fi hotspot with the following static IP allocations:

| Device | Role | Fixed IP Address | Listening Port | Protocol | Bandwidth Estimate |
|---|---|---|---|---|---|
| **Laptop Server** | Mission Orchestrator & Web Server | `192.168.4.100` | **8000** (Web Dashboard / REST) | HTTP / MJPEG | Server Host |
| **Car ESP32** | Drive motor controller | `192.168.4.1` | **5000** (Framed `0xAA..0x55`) | TCP Stream | ~5 KB/s |
| **Arm ESP32** | Arm joints, gripper, grab authorization | `192.168.4.2` | **5001** (Framed `0xAA..0x55`) | TCP Stream | ~5 KB/s |
| **Display Relay** | Optional serial bridge to Uno D0 | `192.168.4.1:8080` only if hosted by car ESP32; otherwise reserve a separate IP | **8080** (Text lines) | TCP Text Lines | ~2 KB/s |
| **Overhead Phone Camera** | Top-down floor tracking | `192.168.4.16` | **8080** (MJPEG stream) | HTTP Video | **~1.5 - 2.5 MB/s** |
| **ESP32-CAM Hand Camera** | Box verification (ArUco/Barcode) | `192.168.4.15` | **81** (Stream) or **POST 8000** | HTTP Stream / POST | **~0.8 - 1.2 MB/s** |

### Total Wi-Fi Bandwidth Requirement:
- Peak bandwidth: **~3.5 MB/s (~28 Mbps)**.
- Easily accommodated on a standard 2.4 GHz 802.11n Wi-Fi channel.
- If packet drops occur, reduce overhead camera resolution to `1280x720 @ 15fps` in `config.yaml`.

---

## 2. Windows Firewall Port Authorization

Only the laptop web server accepts inbound connections. The laptop initiates TCP connections to the ESP32s, so do not open ports 5000 or 5001 inbound on Windows. To open the dashboard to other devices on the LAN, allow port 8000:

```powershell
New-NetFirewallRule -DisplayName "PharmaRoverServer" -Direction Inbound -LocalPort 8000 -Protocol TCP -Action Allow
```

## 3. Connect the Devices

- Reserve separate addresses for the car ESP32 and arm ESP32. The IPs above are examples; match them to the firmware and router reservations.
- The laptop server opens independent connections to `car_esp32.ip:tcp_port` and `arm_esp32.ip:tcp_port` in `server/config.yaml`. The server coordinates the boards; the ESP32s do not connect directly to each other.
- The phone must expose a **live** MJPEG or RTSP feed. Set its URL in `devices.cameras.overhead_phone.source`. A saved video file is not a live tracker feed.
- To show the laptop webcam as a second camera, set `devices.cameras.laptop_webcam.enabled: true` and select the correct OpenCV device index, commonly `0` or `1`.
- Floor ArUco tags 0–3 calibrate the room; the car tag is ID 5 and the optional start tag is ID 6. Keep these IDs unique and make the printed tags match `server/config.yaml` and `data/room.json`.
- Browser PTT uploads audio to the laptop's `/api/audio/transcribe` endpoint. Faster-Whisper runs on the laptop, and its model downloads on first use. Phone browsers generally require HTTPS for microphone access; laptop `localhost` is allowed.

## 4. Firmware

The ESP32 firmware is one PlatformIO project, `firmware/` (envs `esp32_main`, `esp32_cam`); the Uno TFT sketch is in `arduino/uno_tft_display/` (Arduino IDE).
Both find the laptop automatically (UDP broadcast on port 4210), so DHCP addresses from a phone hotspot are fine.
If your hotspot blocks broadcasts, set `devices.controller_esp32.fallback_ip` in `server/config.yaml` and
`SERVER_IP` in `firmware/src/esp32_cam/config.h`. See SETUP_GUIDE.md.
