import math

from lidar_testnode.mb_1r2t_protocol import packet_checksum, parse_packets, ScanAssembler


def make_packet(start_angle, stop_angle, distances_mm, qualities=None,
                packet_type=0x40, checksum=None):
    count = len(distances_mm)
    qualities = qualities or [100] * count
    start_raw = int(start_angle * 128)
    stop_raw = int(stop_angle * 128)
    header = bytearray([
        0xaa, 0x55, packet_type, count,
        start_raw & 0xff, start_raw >> 8,
        stop_raw & 0xff, stop_raw >> 8,
        0, 0])
    payload = bytearray()
    for quality, distance in zip(qualities, distances_mm):
        payload += bytes([quality, distance & 0xff, distance >> 8])
    if checksum is None:
        checksum = packet_checksum(header, payload)
    header[8] = checksum & 0xff
    header[9] = checksum >> 8
    return bytes(header + payload)


def test_parses_packet():
    buffer = bytearray(make_packet(10.0, 20.0, [1000, 2000, 3000], [1, 2, 3]))
    packets = parse_packets(buffer)
    assert len(packets) == 1
    packet = packets[0]
    assert packet.start_angle == 10.0
    assert packet.stop_angle == 20.0
    assert packet.distances == [1.0, 2.0, 3.0]
    assert packet.qualities == [1, 2, 3]
    assert packet.checksum_ok
    assert packet.sample_angles() == [10.0, 15.0, 20.0]
    assert buffer == bytearray()


def test_detects_bad_checksum():
    buffer = bytearray(make_packet(10.0, 20.0, [1000, 2000], checksum=0x1234))
    assert not parse_packets(buffer)[0].checksum_ok


def test_waits_for_incomplete_packet():
    data = make_packet(10.0, 20.0, [1000, 2000, 3000])
    buffer = bytearray(data[:-1])
    assert parse_packets(buffer) == []
    assert buffer == bytearray(data[:-1])
    buffer += data[-1:]
    assert len(parse_packets(buffer)) == 1


def test_waits_for_incomplete_header():
    data = make_packet(10.0, 20.0, [1000, 2000])
    buffer = bytearray(data[:5])
    assert parse_packets(buffer) == []
    buffer += data[5:]
    assert len(parse_packets(buffer)) == 1


def test_resyncs_after_garbage():
    # 0xaa 0xaa 0x55 used to make the old reader miss the sync
    buffer = bytearray(b'\x01\x02\xaa' + make_packet(10.0, 20.0, [1000, 2000]))
    assert len(parse_packets(buffer)) == 1


def test_keeps_trailing_half_sync():
    buffer = bytearray(b'\x01\x02\xaa')
    parse_packets(buffer)
    assert buffer == bytearray(b'\xaa')


def test_skips_non_measurement_and_false_sync():
    other_type = make_packet(10.0, 20.0, [1000, 2000], packet_type=0x28)
    bad_angle = b'\xaa\x55\x40\x02\xff\xff\x00\x00\x00\x00'
    good = make_packet(30.0, 40.0, [1000, 2000])
    buffer = bytearray(other_type + bad_angle + good)
    packets = parse_packets(buffer)
    assert [p.start_angle for p in packets] == [30.0]


def test_wrapping_packet_angles():
    packet = parse_packets(bytearray(make_packet(350.0, 10.0, [1000] * 5)))[0]
    assert packet.sample_angles() == [350.0, 355.0, 360.0, 365.0, 370.0]


def feed(assembler, start, stop, distances_mm):
    packet = parse_packets(bytearray(make_packet(start, stop, distances_mm)))[0]
    return assembler.add_packet(packet)


def test_assembles_full_revolution():
    assembler = ScanAssembler()
    # First, partial, revolution is dropped
    assert feed(assembler, 300.0, 330.0, [500] * 4) is None
    scans = []
    for turn in range(3):
        for start in range(0, 360, 40):
            # 4 samples 10 degrees apart covering [start, start + 30]
            scan = feed(assembler, float(start), float(start + 30), [1000 + start] * 4)
            if scan is not None:
                scans.append(scan)
    # The partial turn is dropped, then each wrap to 0 degrees completes one
    assert len(scans) == 2
    scan = scans[-1]
    assert len(scan.ranges) == 36
    assert math.isclose(scan.angle_increment, math.radians(10.0))
    # Sample at 40 degrees is in bin 4
    assert scan.ranges[4] == 1.04
    assert scan.intensities[4] == 100.0


def test_samples_past_360_go_to_next_revolution():
    assembler = ScanAssembler()
    feed(assembler, 0.0, 30.0, [5000] * 4)  # partial first turn
    feed(assembler, 340.0, 10.0, [2000] * 4)  # 340, 350 now; 360, 370 next turn
    assert feed(assembler, 0.0, 30.0, [5000] * 4) is None  # dropped first turn
    feed(assembler, 340.0, 350.0, [3000] * 2)
    scan = feed(assembler, 0.0, 30.0, [5000] * 4)
    # 2 wrapped samples + 4 + 2 samples
    assert len(scan.ranges) == 8
    # The wrapped 2 m samples are the closest ones near 0 degrees
    assert scan.ranges[0] == 2.0


def test_zero_distance_is_no_return():
    assembler = ScanAssembler()
    feed(assembler, 0.0, 90.0, [1000, 0])
    feed(assembler, 180.0, 270.0, [1000, 1000])
    feed(assembler, 0.0, 90.0, [0, 1000])
    feed(assembler, 180.0, 270.0, [1000, 1000])
    scan = feed(assembler, 0.0, 90.0, [1000, 1000])
    assert scan.ranges == [math.inf, 1.0, 1.0, 1.0]
    assert scan.intensities == [0.0, 100.0, 100.0, 100.0]
