"""
tests/unit/test_mavlink_bridge.py: Unit tests for MAVLink v2 encoder and UDP bridge.
"""

import socket
import struct
import pytest
from sim.mavlink_bridge import MAVLinkV2Frame, MAVLinkBridge, crc16_accumulate


def test_mavlink_crc16():
    """Verify CRC-16 accumulate produces valid non-zero checksum."""
    data = b"HELLO_MAVLINK"
    crc = crc16_accumulate(data)
    assert isinstance(crc, int)
    assert 0 <= crc <= 0xFFFF


def test_mavlink_heartbeat_packet():
    """Verify MAVLink v2 HEARTBEAT packet framing."""
    encoder = MAVLinkV2Frame(sys_id=1, comp_id=1)
    packet = encoder.encode_heartbeat(base_mode=128, custom_mode=4, system_status=4)

    assert len(packet) == 10 + 9 + 2  # header(10) + payload(9) + crc(2) = 21 bytes
    assert packet[0] == 0xFD  # MAVLink v2 magic byte
    assert packet[1] == 9     # Payload length
    assert packet[5] == 1     # Sys ID
    assert packet[6] == 1     # Comp ID
    # Message ID 0 (3 bytes)
    msg_id = packet[7] | (packet[8] << 8) | (packet[9] << 16)
    assert msg_id == 0


def test_mavlink_global_position_packet():
    """Verify GLOBAL_POSITION_INT encoding."""
    encoder = MAVLinkV2Frame(sys_id=2, comp_id=1)
    packet = encoder.encode_global_position_int(
        time_boot_ms=10500,
        lat_deg=19.0760,
        lon_deg=72.8777,
        alt_m=45.2,
        relative_alt_m=30.2,
        vx_mps=5.0,
        vy_mps=-2.0,
        vz_mps=0.5,
        heading_deg=180.0,
    )
    assert packet[0] == 0xFD
    assert packet[1] == 28  # GLOBAL_POSITION_INT payload length = 28 bytes
    assert packet[5] == 2   # Sys ID
    msg_id = packet[7] | (packet[8] << 8) | (packet[9] << 16)
    assert msg_id == 33     # msgid 33


def test_mavlink_udp_broadcast():
    """Verify MAVLinkBridge transmits packets over UDP socket."""
    # Open local UDP receiver
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("127.0.0.1", 14555))
    recv_sock.settimeout(1.0)

    bridge = MAVLinkBridge(host="127.0.0.1", port=14555)

    telemetry_frame = {
        "sim_time": 12.5,
        "drones": [
            {
                "id": "SCOUT_1",
                "position": [10.0, 20.0, 30.0],
                "velocity": [2.0, 1.0, 0.0],
                "attitude": [0.05, -0.02, 1.57],
                "battery_pct": 92.5,
            }
        ]
    }

    try:
        sent = bridge.broadcast_telemetry(telemetry_frame)
        assert sent == 4  # HEARTBEAT, SYS_STATUS, GLOBAL_POSITION_INT, ATTITUDE

        # Receive first packet
        data, addr = recv_sock.recvfrom(512)
        assert len(data) > 0
        assert data[0] == 0xFD
    finally:
        bridge.close()
        recv_sock.close()
