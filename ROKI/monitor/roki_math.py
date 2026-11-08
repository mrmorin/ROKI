"""Pure geometry and timing helpers shared by the ROKI monitor and tests.

There are no GUI or serial dependencies here, so the physical conventions can
be tested on a headless machine.  Do not change the confirmed FRD/N/E/Up yaw
sign without an independent hardware test.
"""

import math
from datetime import datetime


def frd_to_neu(point, roll, pitch, yaw):
    """Rotate a body-FRD vector into world North/East/Up."""
    x, y, z = point
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    north = cp * cy * x + (sr * sp * cy - cr * sy) * y + (cr * sp * cy + sr * sy) * z
    east = cp * sy * x + (sr * sp * sy + cr * cy) * y + (cr * sp * sy - sr * cy) * z
    down = -sp * x + sr * cp * y + cr * cp * z
    return north, east, -down


def project_neu(point, azimuth, elevation):
    """Screen-right, screen-up and depth for a North/East/Up point."""
    north, east, up = point
    right = math.cos(azimuth) * east - math.sin(azimuth) * north
    depth = math.sin(azimuth) * east + math.cos(azimuth) * north
    vertical = math.cos(elevation) * up + math.sin(elevation) * depth
    camera_depth = math.cos(elevation) * depth - math.sin(elevation) * up
    return right, vertical, camera_depth


def build_playback_timeline(rows):
    """Elapsed seconds based primarily on transmitter capture timestamps.

    USB delivery can batch multiple samples under identical host timestamps.
    tx_ms preserves the original sensor cadence and packet loss gaps.  Fall
    back to host time on a transmitter restart or absent/broken tx_ms; keep
    the timeline monotonic even if the host clock jumps backward.
    """
    times = []
    prev_tx = prev_host = None
    elapsed = 0.0
    for row in rows:
        try:
            tx = int(row['tx_ms']) & 0xFFFFFFFF
        except (KeyError, TypeError, ValueError):
            tx = None
        try:
            host = datetime.fromisoformat(row['received_at'])
        except (KeyError, TypeError, ValueError):
            host = None
        if times:
            dt = None
            if tx is not None and prev_tx is not None:
                delta_ms = (tx - prev_tx) & 0xFFFFFFFF
                if 0 < delta_ms <= 30000:
                    dt = delta_ms / 1000.0
            if dt is None and host is not None and prev_host is not None:
                try:
                    host_dt = (host - prev_host).total_seconds()
                    if 0 < host_dt <= 300:
                        dt = host_dt
                except TypeError:
                    pass
            elapsed += dt if dt is not None else 0.02
        times.append(elapsed)
        prev_tx, prev_host = tx, host
    return times
