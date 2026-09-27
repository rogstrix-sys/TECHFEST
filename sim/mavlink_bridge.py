"""
sim/mavlink_bridge.py: Lightweight MAVLink v2 Telemetry Bridge for QGroundControl / SITL.

Encodes real-time UAV swarm telemetry into standard MAVLink v2 binary frames
and broadcasts them over UDP port 14550 (or user-configured port).
Enables direct connection to:
- QGroundControl (QGC)
- Mission Planner
- Physical Pixhawk / PX4 / ArduPilot autopilots via SITL bridge
Zero external dependencies (pure Python implementation of MAVLink v2 framing & X.25 CRC).
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time
from typing import Any, Dict, List, Optional, Tuple


def crc16_accumulate(buf: bytes, crc: int = 0xFFFF) -> int:
    """Calculates standard MAVLink X.25 / MCRF4XX 16-bit CRC checksum."""
    for byte in buf:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc


class MAVLinkV2Frame:
    """
    Pure Python MAVLink v2 Frame Encoder.
    Format:
    [0xFD, len, incompat_flags, compat_flags, seq, sys_id, comp_id, msg_id_0, msg_id_1, msg_id_2, payload..., crc_low, crc_high]
    """
    MAGIC = 0xFD

    # MAVLink CRC Extra bytes for message integrity
    CRC_EXTRA = {
        0: 50,    # HEARTBEAT
        1: 124,   # SYS_STATUS
        30: 39,   # ATTITUDE
        33: 104,  # GLOBAL_POSITION_INT
    }

    def __init__(self, sys_id: int = 1, comp_id: int = 1) -> None:
        self.sys_id = sys_id
        self.comp_id = comp_id
        self.seq = 0

    def encode(self, msg_id: int, payload: bytes) -> bytes:
        """Encode binary MAVLink v2 packet with header and X.25 CRC."""
        payload_len = len(payload)
        incompat_flags = 0x00
        compat_flags = 0x00
        seq = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF

        msg_id_bytes = struct.pack("<I", msg_id)[:3]

        header = struct.pack(
            "<BBBBBB",
            self.MAGIC,
            payload_len,
            incompat_flags,
            compat_flags,
            seq,
            self.sys_id,
        ) + struct.pack("<B", self.comp_id) + msg_id_bytes

        # Calculate CRC over header (excluding magic 0xFD) + payload + CRC_EXTRA
        crc_data = header[1:] + payload
        crc = crc16_accumulate(crc_data, 0xFFFF)
        crc_extra = self.CRC_EXTRA.get(msg_id, 0)
        crc = crc16_accumulate(bytes([crc_extra]), crc)

        checksum = struct.pack("<H", crc)
        return header + payload + checksum

    def encode_heartbeat(self, base_mode: int = 128, custom_mode: int = 4, system_status: int = 4) -> bytes:
        """
        HEARTBEAT (msg_id 0):
        uint32_t custom_mode, uint8_t type (2=quadrotor), uint8_t autopilot (3=ArduPilot/12=PX4),
        uint8_t base_mode, uint8_t system_status, uint8_t mavlink_version (3)
        """
        # Type: 2 (MAV_TYPE_QUADROTOR), Autopilot: 12 (MAV_AUTOPILOT_PX4)
        payload = struct.pack("<IBBBBB", custom_mode, 2, 12, base_mode, system_status, 3)
        return self.encode(0, payload)

    def encode_sys_status(self, voltage_mv: int = 15200, current_ca: int = 1450, battery_remaining: int = 85) -> bytes:
        """
        SYS_STATUS (msg_id 1):
        onboard_control_sensors_present (uint32), enabled (uint32), health (uint32),
        load (uint16), voltage_battery (uint16 mV), current_battery (int16 cA),
        battery_remaining (int8 %), drop_rate_comm (uint16), errors_comm (uint16),
        errors_count1 (uint16), errors_count2 (uint16), errors_count3 (uint16), errors_count4 (uint16)
        """
        sensors = 0b0011111111111111
        payload = struct.pack(
            "<IIIHHhHHHHHHb",
            sensors, sensors, sensors,
            150, voltage_mv, current_ca,
            0, 0, 0, 0, 0, 0,
            max(0, min(100, int(battery_remaining)))
        )
        return self.encode(1, payload)

    def encode_attitude(self, time_boot_ms: int, roll_rad: float, pitch_rad: float, yaw_rad: float,
                        rollspeed: float = 0.0, pitchspeed: float = 0.0, yawspeed: float = 0.0) -> bytes:
        """
        ATTITUDE (msg_id 30):
        uint32_t time_boot_ms, float roll, float pitch, float yaw,
        float rollspeed, float pitchspeed, float yawspeed
        """
        payload = struct.pack("<Iffffff", time_boot_ms, float(roll_rad), float(pitch_rad), float(yaw_rad),
                               float(rollspeed), float(pitchspeed), float(yawspeed))
        return self.encode(30, payload)

    def encode_global_position_int(self, time_boot_ms: int, lat_deg: float, lon_deg: float,
                                   alt_m: float, relative_alt_m: float,
                                   vx_mps: float, vy_mps: float, vz_mps: float,
                                   heading_deg: float) -> bytes:
        """
        GLOBAL_POSITION_INT (msg_id 33):
        uint32_t time_boot_ms, int32_t lat (degE7), int32_t lon (degE7),
        int32_t alt (mm), int32_t relative_alt (mm),
        int16_t vx (cm/s), int16_t vy (cm/s), int16_t vz (cm/s),
        uint16_t hdg (cdeg)
        """
        lat_e7 = int(lat_deg * 1e7)
        lon_e7 = int(lon_deg * 1e7)
        alt_mm = int(alt_m * 1000.0)
        rel_alt_mm = int(relative_alt_m * 1000.0)
        vx_cms = int(vx_mps * 100.0)
        vy_cms = int(vy_mps * 100.0)
        vz_cms = int(vz_mps * 100.0)
        hdg_cdeg = int((heading_deg % 360.0) * 100.0)

        payload = struct.pack(
            "<IiiiihhhH",
            time_boot_ms, lat_e7, lon_e7, alt_mm, rel_alt_mm,
            vx_cms, vy_cms, vz_cms, hdg_cdeg
        )
        return self.encode(33, payload)


class MAVLinkBridge:
    """
    UDP Broadcast Server streaming MAVLink v2 packets to QGroundControl / Autopilots.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 14550,
        ref_lat: float = 19.0760,  # Reference latitude (e.g. Mumbai / IIT Bombay)
        ref_lon: float = 72.8777,  # Reference longitude
    ) -> None:
        self.host = host
        self.port = port
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.encoders: Dict[str, MAVLinkV2Frame] = {}

    def get_encoder(self, drone_id: str, default_sys_id: int = 1) -> MAVLinkV2Frame:
        if drone_id not in self.encoders:
            # Map UAV_1 -> 1, SCOUT_1 -> 1, RELAY_1 -> 5, etc.
            sys_id = default_sys_id
            try:
                digits = "".join(ch for ch in drone_id if ch.isdigit())
                if digits:
                    sys_id = max(1, min(250, int(digits)))
            except Exception:
                pass
            self.encoders[drone_id] = MAVLinkV2Frame(sys_id=sys_id, comp_id=1)
        return self.encoders[drone_id]

    def broadcast_telemetry(self, telemetry_frame: Dict[str, Any]) -> int:
        """
        Encodes and broadcasts MAVLink packets for all active drones in frame.
        Returns number of packets successfully transmitted.
        """
        packets_sent = 0
        sim_time = float(telemetry_frame.get("sim_time", 0.0))
        time_boot_ms = int(sim_time * 1000.0)
        drones = telemetry_frame.get("drones", [])

        # Conversion: 1 degree latitude ~ 111,000 meters; longitude scaled by cos(lat)
        meters_per_deg_lat = 111000.0
        meters_per_deg_lon = 111000.0 * math.cos(math.radians(self.ref_lat))

        for idx, d in enumerate(drones, start=1):
            d_id = d.get("id", f"UAV_{idx}")
            enc = self.get_encoder(d_id, default_sys_id=idx)

            pos = d.get("position", [0.0, 0.0, 0.0])
            vel = d.get("velocity", [0.0, 0.0, 0.0])
            att = d.get("attitude", [0.0, 0.0, 0.0])
            bat_pct = float(d.get("battery_pct", 100.0))

            # Geographic mapping from simulation coordinates (x: East, y: North, z: Up)
            lat = self.ref_lat + (pos[1] / meters_per_deg_lat)
            lon = self.ref_lon + (pos[0] / meters_per_deg_lon)
            alt = float(pos[2])

            heading_deg = math.degrees(att[2]) % 360.0

            # 1. HEARTBEAT
            hb_pkt = enc.encode_heartbeat()
            self.sock.sendto(hb_pkt, (self.host, self.port))
            packets_sent += 1

            # 2. SYS_STATUS
            sys_pkt = enc.encode_sys_status(
                voltage_mv=int(14800 + (bat_pct / 100.0) * 2000),
                current_ca=1400,
                battery_remaining=int(bat_pct),
            )
            self.sock.sendto(sys_pkt, (self.host, self.port))
            packets_sent += 1

            # 3. GLOBAL_POSITION_INT
            pos_pkt = enc.encode_global_position_int(
                time_boot_ms=time_boot_ms,
                lat_deg=lat,
                lon_deg=lon,
                alt_m=alt + 15.0,  # MSL
                relative_alt_m=alt,
                vx_mps=vel[0],
                vy_mps=vel[1],
                vz_mps=vel[2],
                heading_deg=heading_deg,
            )
            self.sock.sendto(pos_pkt, (self.host, self.port))
            packets_sent += 1

            # 4. ATTITUDE
            att_pkt = enc.encode_attitude(
                time_boot_ms=time_boot_ms,
                roll_rad=att[0],
                pitch_rad=att[1],
                yaw_rad=att[2],
            )
            self.sock.sendto(att_pkt, (self.host, self.port))
            packets_sent += 1

        return packets_sent

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="UAV-X MAVLink Telemetry Bridge")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Target UDP IP (e.g. 127.0.0.1)")
    parser.add_argument("--port", type=int, default=14550, help="Target UDP port (e.g. 14550 for QGC)")
    args = parser.parse_args()

    print(f"[*] Starting UAV-X MAVLink Bridge broadcasting to {args.host}:{args.port}...")
    bridge = MAVLinkBridge(host=args.host, port=args.port)
    try:
        while True:
            # Probe simulation server
            import urllib.request
            import json
            try:
                req = urllib.request.Request(f"http://127.0.0.1:8000/api/telemetry")
                with urllib.request.urlopen(req, timeout=1.0) as resp:
                    frame = json.loads(resp.read().decode())
                    sent = bridge.broadcast_telemetry(frame)
                    sys.stdout.write(f"\r[+] MAVLink stream active: {sent} packets broadcast to {args.host}:{args.port}")
                    sys.stdout.flush()
            except Exception as err:
                sys.stdout.write(f"\r[*] Waiting for simulation server at http://127.0.0.1:8000/...")
                sys.stdout.flush()
            time.sleep(0.1)  # 10 Hz MAVLink stream
    except KeyboardInterrupt:
        print("\n[*] MAVLink Bridge stopped.")
    finally:
        bridge.close()
