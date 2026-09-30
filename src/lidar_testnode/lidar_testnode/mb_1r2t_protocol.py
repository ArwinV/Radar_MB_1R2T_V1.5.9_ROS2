"""
Packet parsing and scan assembly for the Radar_MB_1R2T lidar.

This module has no ROS dependencies so it can be unit tested on its own.

The device streams serial data at 153600 baud (8N1). Packets look like:

    [0xAA] [0x55]                 sync bytes
    [type]                        0x40 for measurement packets
    [count]                       number of samples in the packet
    [start_angleL] [start_angleH] start angle in 1/128 degree
    [stop_angleL] [stop_angleH]   stop angle in 1/128 degree
    [checkL] [checkH]             unconfirmed, probably a checksum
    count x {[quality] [distanceL] [distanceH]}  distance in mm

The meaning of the two bytes after the stop angle has not been confirmed.
They are assumed to be a YDLIDAR-style checksum: the XOR of all 16-bit
little-endian words in the packet, with each quality byte counted as its
own word. See packet_checksum().
"""

from dataclasses import dataclass
import math

SYNC = b'\xaa\x55'
HEADER_LENGTH = 10  # sync bytes included
SAMPLE_LENGTH = 3
TYPE_MEASUREMENT = 0x40
FULL_CIRCLE = 360.0
ANGLE_SCALE = 128.0


@dataclass
class Packet:
    """One decoded measurement packet."""

    start_angle: float  # degrees
    stop_angle: float  # degrees
    qualities: list  # raw quality bytes
    distances: list  # metres, 0.0 means no return
    checksum: int  # value from the header
    computed_checksum: int  # value computed from the packet contents

    @property
    def checksum_ok(self):
        return self.checksum == self.computed_checksum

    def sample_angles(self):
        """Return the angle in degrees of each sample, not wrapped to 360."""
        count = len(self.distances)
        if self.stop_angle > self.start_angle:
            span = self.stop_angle - self.start_angle
        else:
            span = FULL_CIRCLE - self.start_angle + self.stop_angle
        step = span / (count - 1)
        return [self.start_angle + i * step for i in range(count)]


def packet_checksum(header, payload):
    """
    Compute the assumed checksum of a packet.

    header is the 10 header bytes including sync, payload the sample bytes.
    """
    checksum = 0
    # Sync word, type/count word, start angle and stop angle
    for i in (0, 2, 4, 6):
        checksum ^= header[i] | (header[i + 1] << 8)
    for i in range(0, len(payload), SAMPLE_LENGTH):
        checksum ^= payload[i]
        checksum ^= payload[i + 1] | (payload[i + 2] << 8)
    return checksum


def parse_packets(buffer):
    """
    Decode every complete packet at the front of buffer.

    buffer is a bytearray. Bytes that have been decoded or skipped are
    removed from it; an incomplete packet at the end is left in place so it
    can be finished once more data arrives. Returns a list of Packets.
    """
    packets = []
    while True:
        start = buffer.find(SYNC)
        if start < 0:
            # Keep a trailing 0xAA, it may be the first half of a sync
            keep = 1 if buffer[-1:] == SYNC[:1] else 0
            del buffer[:len(buffer) - keep]
            return packets
        del buffer[:start]
        if len(buffer) < HEADER_LENGTH:
            return packets

        packet_type = buffer[2]
        count = buffer[3]
        start_raw = buffer[4] | (buffer[5] << 8)
        stop_raw = buffer[6] | (buffer[7] << 8)
        max_raw = FULL_CIRCLE * ANGLE_SCALE
        if (packet_type != TYPE_MEASUREMENT or count < 2
                or start_raw >= max_raw or stop_raw >= max_raw):
            # Not a measurement packet, or a false sync. Skip the sync
            # bytes and search again.
            del buffer[:len(SYNC)]
            continue

        length = HEADER_LENGTH + count * SAMPLE_LENGTH
        if len(buffer) < length:
            return packets

        header = bytes(buffer[:HEADER_LENGTH])
        payload = bytes(buffer[HEADER_LENGTH:length])
        del buffer[:length]
        packets.append(Packet(
            start_angle=start_raw / ANGLE_SCALE,
            stop_angle=stop_raw / ANGLE_SCALE,
            qualities=list(payload[0::SAMPLE_LENGTH]),
            distances=[
                (payload[i + 1] | (payload[i + 2] << 8)) / 1000.0
                for i in range(0, len(payload), SAMPLE_LENGTH)
            ],
            checksum=header[8] | (header[9] << 8),
            computed_checksum=packet_checksum(header, payload),
        ))


@dataclass
class Scan:
    """One full revolution, binned into equal angular steps from 0 degrees."""

    ranges: list  # metres, math.inf where there was no return
    intensities: list

    @property
    def angle_increment(self):
        """Angle between bins in radians."""
        return 2.0 * math.pi / len(self.ranges)


class ScanAssembler:
    """Collect packets and emit a Scan every time the lidar completes a turn."""

    def __init__(self):
        self._current = []  # (angle, distance, quality) for this revolution
        self._next = []  # samples from a wrapping packet that belong to the next one
        self._last_start_angle = None
        self._first_turn = True  # the first revolution is incomplete, drop it

    def add_packet(self, packet):
        """Add a packet. Returns a Scan when a revolution completed, else None."""
        scan = None
        if (self._last_start_angle is not None
                and packet.start_angle < self._last_start_angle):
            if not self._first_turn:
                scan = self._build_scan(self._current)
            self._first_turn = False
            self._current = self._next
            self._next = []
        self._last_start_angle = packet.start_angle

        for angle, distance, quality in zip(
                packet.sample_angles(), packet.distances, packet.qualities):
            if angle >= FULL_CIRCLE:
                self._next.append((angle - FULL_CIRCLE, distance, quality))
            else:
                self._current.append((angle, distance, quality))
        return scan

    @staticmethod
    def _build_scan(samples):
        if not samples:
            return None
        bins = len(samples)
        ranges = [math.inf] * bins
        intensities = [0.0] * bins
        for angle, distance, quality in samples:
            index = int(round(angle / FULL_CIRCLE * bins)) % bins
            if distance > 0.0 and distance < ranges[index]:
                ranges[index] = distance
                intensities[index] = float(quality)
        return Scan(ranges=ranges, intensities=intensities)
