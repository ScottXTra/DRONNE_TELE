import serial
import time

PORT = "COM6"
BAUD = 420000

CRSF_GPS = 0x02
CRSF_LINK_STATISTICS = 0x14
CRSF_FLIGHT_MODE = 0x21

# Addresses/sync bytes commonly seen in CRSF streams.
CRSF_SYNC_BYTES = {
    0x00,  # broadcast
    0xC8,  # flight controller / serial sync
    0xEA,  # remote control
    0xEC,  # receiver
    0xEE,  # transmitter module
}


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def hexstr(data: bytes) -> str:
    return " ".join(f"{b:02x}" for b in data)


def parse_gps(payload: bytes):
    # Spec GPS payload is 15 bytes.
    # Ignore extra bytes if a newer sender appends fields.
    if len(payload) < 15:
        return None

    payload = payload[:15]

    lat_raw = int.from_bytes(payload[0:4], "big", signed=True)
    lon_raw = int.from_bytes(payload[4:8], "big", signed=True)
    speed_raw = int.from_bytes(payload[8:10], "big", signed=False)
    heading_raw = int.from_bytes(payload[10:12], "big", signed=False)
    altitude_raw = int.from_bytes(payload[12:14], "big", signed=False)
    sats = payload[14]

    return {
        "lat_raw": lat_raw,
        "lon_raw": lon_raw,
        "lat": lat_raw / 10_000_000,
        "lon": lon_raw / 10_000_000,
        # There is some doc disagreement, so print both.
        "speed_kmh_div10": speed_raw / 10,
        "speed_kmh_div100": speed_raw / 100,
        "heading_deg": heading_raw / 100,
        "altitude_m": altitude_raw - 1000,
        "sats": sats,
    }


def parse_link_stats(payload: bytes):
    # CRSF 0x14 is usually 10 bytes.
    if len(payload) < 10:
        return None

    return {
        "uplink_rssi_1": -payload[0],
        "uplink_rssi_2": -payload[1],
        "uplink_lq": payload[2],
        "uplink_snr": int.from_bytes(payload[3:4], "big", signed=True),
        "active_antenna": payload[4],
        "rf_mode": payload[5],
        "uplink_tx_power": payload[6],
        "downlink_rssi": -payload[7],
        "downlink_lq": payload[8],
        "downlink_snr": int.from_bytes(payload[9:10], "big", signed=True),
    }


def parse_flight_mode(payload: bytes):
    # Betaflight often sends ASCII mode text, sometimes null-terminated.
    return payload.rstrip(b"\x00").decode("ascii", errors="replace")


def process_frame(frame: bytes):
    address = frame[0]
    length = frame[1]
    frame_type = frame[2]
    payload = frame[3:-1]
    crc_rx = frame[-1]

    crc_calc = crc8_dvb_s2(frame[2:-1])
    if crc_rx != crc_calc:
        return False

    if frame_type == CRSF_GPS:
        gps = parse_gps(payload)
        if gps:
            print()
            print("GPS FRAME")
            print(f"  raw:       {hexstr(frame)}")
            print(f"  sats:      {gps['sats']}")
            print(f"  lat:       {gps['lat']:.7f}")
            print(f"  lon:       {gps['lon']:.7f}")
            print(f"  altitude:  {gps['altitude_m']} m")
            print(f"  heading:   {gps['heading_deg']:.2f} deg")
            print(f"  speed/10:  {gps['speed_kmh_div10']:.2f} km/h")
            print(f"  speed/100: {gps['speed_kmh_div100']:.2f} km/h")

            if gps["lat_raw"] != 0 or gps["lon_raw"] != 0:
                print(f"  maps:      https://maps.google.com/?q={gps['lat']:.7f},{gps['lon']:.7f}")

    elif frame_type == CRSF_LINK_STATISTICS:
        stats = parse_link_stats(payload)
        if stats:
            # Print occasionally, not every single frame.
            now = time.time()
            if now - process_frame.last_link_print > 2.0:
                process_frame.last_link_print = now
                print()
                print("LINK STATS")
                print(f"  uplink LQ:   {stats['uplink_lq']}%")
                print(f"  downlink LQ: {stats['downlink_lq']}%")
                print(f"  uplink RSSI: {stats['uplink_rssi_1']} / {stats['uplink_rssi_2']} dBm")
                print(f"  down RSSI:   {stats['downlink_rssi']} dBm")
                print(f"  RF mode:     {stats['rf_mode']}")

    elif frame_type == CRSF_FLIGHT_MODE:
        mode = parse_flight_mode(payload)
        if mode:
            now = time.time()
            if now - process_frame.last_mode_print > 2.0:
                process_frame.last_mode_print = now
                print()
                print("FLIGHT MODE")
                print(f"  {mode}")

    return True


process_frame.last_link_print = 0.0
process_frame.last_mode_print = 0.0


def find_and_process_frames(buffer: bytearray):
    good_frames = 0

    while len(buffer) >= 4:
        if buffer[0] not in CRSF_SYNC_BYTES:
            del buffer[0]
            continue

        length = buffer[1]

        # CRSF valid length range: 2..62 according to the spec.
        if length < 2 or length > 62:
            del buffer[0]
            continue

        total_len = length + 2

        if len(buffer) < total_len:
            break

        frame = bytes(buffer[:total_len])

        # If CRC is bad, do not delete the whole candidate frame.
        # Delete one byte and resync, because we may have started mid-stream.
        crc_rx = frame[-1]
        crc_calc = crc8_dvb_s2(frame[2:-1])

        if crc_rx != crc_calc:
            del buffer[0]
            continue

        del buffer[:total_len]
        process_frame(frame)
        good_frames += 1

    return good_frames


def main():
    print(f"Opening {PORT} at {BAUD}...")
    buffer = bytearray()

    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        print("Opened.")
        print("Decoding CRSF telemetry. Press Ctrl+C to stop.")

        last_status = time.time()
        total_bytes = 0
        total_frames = 0

        while True:
            data = ser.read(512)
            if data:
                total_bytes += len(data)
                buffer.extend(data)
                total_frames += find_and_process_frames(buffer)

            if time.time() - last_status > 5.0:
                last_status = time.time()
                print()
                print(f"STATUS: bytes={total_bytes}, frames={total_frames}, buffer={len(buffer)}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
