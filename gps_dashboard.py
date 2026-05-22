import threading
import time
from collections import deque

import meshtastic.serial_interface
import serial
import serial.tools.list_ports

from dash import Dash, html, dcc
from dash.dependencies import Input, Output, State
import dash_leaflet as dl


PORT = "COM6"
BAUD = 420000
MESHTASTIC_PORT = "COM5"
MESHTASTIC_CHANNEL_INDEX = 0
SEND_DELAY_OPTIONS = [2, 5, 10, 15, 30, 60]
DEFAULT_MESSAGE_TEMPLATE = (
    "GPS {GPSLat},{GPSLon} | Alt {AltitudeM}m | Speed {SpeedKmh}kmh | "
    "Heading {HeadingDeg}deg | Sats {Satellites} | Mode {FlightMode} | "
    "Battery {BatteryVolts}V ({BatteryPercent}%) | Cell {CellVolts}V x{CellCount} | "
    "Current {CurrentA}A | Used {MahUsed}mAh | UpLQ {UplinkLQ} | DownLQ {DownlinkLQ} | "
    "Serial {SerialConnected} | Age {LastGpsAge}"
)

CRSF_GPS = 0x02
CRSF_BATTERY_SENSOR = 0x08
CRSF_LINK_STATISTICS = 0x14
CRSF_FLIGHT_MODE = 0x21

CRSF_SYNC_BYTES = {
    0x00,
    0xC8,
    0xEA,
    0xEC,
    0xEE,
}

latest = {
    "connected": False,
    "last_update": None,
    "raw_frame": "",
    "lat": 0.0,
    "lon": 0.0,
    "sats": 0,
    "altitude_m": 0,
    "speed_kmh": 0.0,
    "heading_deg": 0.0,
    "link_uplink_lq": None,
    "link_downlink_lq": None,
    "flight_mode": "",
    "battery_volts": 0.0,
    "current_a": 0.0,
    "mah_used": 0,
    "battery_percent": None,
    "cell_count": None,
    "cell_volts": 0.0,
    "frames": 0,
    "bytes": 0,
    "error": "",
    "meshtastic_enabled": False,
    "meshtastic_delay_s": 10,
    "meshtastic_last_sent": None,
    "meshtastic_last_attempt": None,
    "meshtastic_last_message": "",
    "meshtastic_send_count": 0,
    "meshtastic_error": "",
    "meshtastic_last_result": "",
    "meshtastic_template": DEFAULT_MESSAGE_TEMPLATE,
}

recent_points = deque(maxlen=500)
latest_lock = threading.Lock()
meshtastic_lock = threading.Lock()
meshtastic_interface = None


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
    if len(payload) < 15:
        return None

    payload = payload[:15]

    lat_raw = int.from_bytes(payload[0:4], "big", signed=True)
    lon_raw = int.from_bytes(payload[4:8], "big", signed=True)
    speed_raw = int.from_bytes(payload[8:10], "big", signed=False)
    heading_raw = int.from_bytes(payload[10:12], "big", signed=False)
    altitude_raw = int.from_bytes(payload[12:14], "big", signed=False)
    sats = payload[14]

    lat = lat_raw / 10_000_000
    lon = lon_raw / 10_000_000

    # Most CRSF GPS decoders treat speed as km/h * 10.
    speed_kmh = speed_raw / 10
    heading_deg = heading_raw / 100
    altitude_m = altitude_raw - 1000

    return {
        "lat": lat,
        "lon": lon,
        "lat_raw": lat_raw,
        "lon_raw": lon_raw,
        "speed_kmh": speed_kmh,
        "heading_deg": heading_deg,
        "altitude_m": altitude_m,
        "sats": sats,
    }


def parse_link_stats(payload: bytes):
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
    return payload.rstrip(b"\x00").decode("ascii", errors="replace")


def infer_cell_count(pack_voltage: float):
    if pack_voltage <= 0:
        return None

    inferred = max(1, min(12, round(pack_voltage / 4.2)))
    return inferred


def parse_battery_sensor(payload: bytes):
    if len(payload) < 8:
        return None

    voltage_raw = int.from_bytes(payload[0:2], "big", signed=False)
    current_raw = int.from_bytes(payload[2:4], "big", signed=False)
    mah_used = int.from_bytes(payload[4:7], "big", signed=False)
    battery_percent = payload[7]
    battery_volts = voltage_raw / 100
    current_a = current_raw / 100
    cell_count = infer_cell_count(battery_volts)
    cell_volts = battery_volts / cell_count if cell_count else 0.0

    return {
        "battery_volts": battery_volts,
        "current_a": current_a,
        "mah_used": mah_used,
        "battery_percent": battery_percent,
        "cell_count": cell_count,
        "cell_volts": cell_volts,
    }


def process_frame(frame: bytes):
    frame_type = frame[2]
    payload = frame[3:-1]

    with latest_lock:
        latest["frames"] += 1
        latest["raw_frame"] = hexstr(frame)

    if frame_type == CRSF_GPS:
        gps = parse_gps(payload)
        if not gps:
            return

        with latest_lock:
            latest["last_update"] = time.time()
            latest["lat"] = gps["lat"]
            latest["lon"] = gps["lon"]
            latest["sats"] = gps["sats"]
            latest["altitude_m"] = gps["altitude_m"]
            latest["speed_kmh"] = gps["speed_kmh"]
            latest["heading_deg"] = gps["heading_deg"]

        # Only add real nonzero fixes to the trail.
        if gps["lat_raw"] != 0 or gps["lon_raw"] != 0:
            recent_points.append((gps["lat"], gps["lon"]))

    elif frame_type == CRSF_LINK_STATISTICS:
        stats = parse_link_stats(payload)
        if stats:
            with latest_lock:
                latest["link_uplink_lq"] = stats["uplink_lq"]
                latest["link_downlink_lq"] = stats["downlink_lq"]

    elif frame_type == CRSF_BATTERY_SENSOR:
        battery = parse_battery_sensor(payload)
        if battery:
            with latest_lock:
                latest["battery_volts"] = battery["battery_volts"]
                latest["current_a"] = battery["current_a"]
                latest["mah_used"] = battery["mah_used"]
                latest["battery_percent"] = battery["battery_percent"]
                latest["cell_count"] = battery["cell_count"]
                latest["cell_volts"] = battery["cell_volts"]

    elif frame_type == CRSF_FLIGHT_MODE:
        with latest_lock:
            latest["flight_mode"] = parse_flight_mode(payload)


def build_meshtastic_message():
    now = time.time()
    last_update = latest["last_update"]
    age_text = "never"
    if last_update is not None:
        age_text = f"{(now - last_update):.1f}s"

    template = latest["meshtastic_template"].strip() or DEFAULT_MESSAGE_TEMPLATE
    template_vars = {
        "GPSLat": f"{latest['lat']:.7f}",
        "GPSLon": f"{latest['lon']:.7f}",
        "Satellites": str(latest["sats"]),
        "AltitudeM": str(latest["altitude_m"]),
        "SpeedKmh": f"{latest['speed_kmh']:.1f}",
        "HeadingDeg": f"{latest['heading_deg']:.1f}",
        "FlightMode": latest["flight_mode"] or "NONE",
        "BatteryVolts": f"{latest['battery_volts']:.2f}",
        "CurrentA": f"{latest['current_a']:.2f}",
        "MahUsed": str(latest["mah_used"]),
        "BatteryPercent": "N/A" if latest["battery_percent"] is None else str(latest["battery_percent"]),
        "CellCount": "N/A" if latest["cell_count"] is None else str(latest["cell_count"]),
        "CellVolts": f"{latest['cell_volts']:.2f}",
        "UplinkLQ": "N/A" if latest["link_uplink_lq"] is None else str(latest["link_uplink_lq"]),
        "DownlinkLQ": "N/A" if latest["link_downlink_lq"] is None else str(latest["link_downlink_lq"]),
        "SerialConnected": "YES" if latest["connected"] else "NO",
        "FramesDecoded": str(latest["frames"]),
        "BytesReceived": str(latest["bytes"]),
        "LastGpsAge": age_text,
        "RawFrame": latest["raw_frame"] or "NONE",
        "TelemetryError": latest["error"] or "NONE",
        "MeshSendCount": str(latest["meshtastic_send_count"]),
    }

    return template.format_map(_TemplateVars(template_vars))


class _TemplateVars(dict):
    def __missing__(self, key):
        return "{" + key + "}"


def close_meshtastic_interface():
    global meshtastic_interface

    if meshtastic_interface is None:
        return

    try:
        meshtastic_interface.close()
    except Exception:
        pass

    meshtastic_interface = None


def get_meshtastic_interface():
    global meshtastic_interface

    if meshtastic_interface is None:
        meshtastic_interface = meshtastic.serial_interface.SerialInterface(
            devPath=MESHTASTIC_PORT
        )

    return meshtastic_interface


def send_meshtastic_text(message: str):
    with meshtastic_lock:
        try:
            interface = get_meshtastic_interface()
            packet = interface.sendText(message, channelIndex=MESHTASTIC_CHANNEL_INDEX)
            packet_id = packet.get("id") if isinstance(packet, dict) else None
            if packet_id is None:
                return "sent"
            return f"sent packet id {packet_id}"
        except Exception:
            close_meshtastic_interface()
            raise


def meshtastic_sender_worker():
    while True:
        time.sleep(0.25)

        with latest_lock:
            enabled = latest["meshtastic_enabled"]
            delay_s = latest["meshtastic_delay_s"]
            last_sent = latest["meshtastic_last_sent"]
            message = build_meshtastic_message()

            if not enabled:
                continue

            now = time.time()
            if last_sent is not None and now - last_sent < delay_s:
                continue

            latest["meshtastic_last_attempt"] = now

        try:
            result_text = send_meshtastic_text(message)

            with latest_lock:
                latest["meshtastic_last_sent"] = now
                latest["meshtastic_last_message"] = message
                latest["meshtastic_send_count"] += 1
                latest["meshtastic_error"] = ""
                latest["meshtastic_last_result"] = result_text

        except Exception as e:
            error_text = repr(e)

            with latest_lock:
                latest["meshtastic_error"] = error_text


def send_debug_meshtastic_message():
    now = time.time()
    with latest_lock:
        message = build_meshtastic_message()

    with latest_lock:
        latest["meshtastic_last_attempt"] = now

    try:
        result_text = send_meshtastic_text(message)
        with latest_lock:
            latest["meshtastic_last_sent"] = now
            latest["meshtastic_last_message"] = message
            latest["meshtastic_send_count"] += 1
            latest["meshtastic_error"] = ""
            latest["meshtastic_last_result"] = result_text
    except Exception as e:
        error_text = repr(e)

        with latest_lock:
            latest["meshtastic_error"] = error_text


def find_and_process_frames(buffer: bytearray):
    while len(buffer) >= 4:
        if buffer[0] not in CRSF_SYNC_BYTES:
            del buffer[0]
            continue

        length = buffer[1]

        if length < 2 or length > 62:
            del buffer[0]
            continue

        total_len = length + 2

        if len(buffer) < total_len:
            break

        frame = bytes(buffer[:total_len])

        crc_rx = frame[-1]
        crc_calc = crc8_dvb_s2(frame[2:-1])

        if crc_rx != crc_calc:
            del buffer[0]
            continue

        del buffer[:total_len]
        process_frame(frame)


def serial_worker():
    buffer = bytearray()

    while True:
        try:
            with latest_lock:
                latest["error"] = ""
                latest["connected"] = False

            print(f"Opening {PORT} at {BAUD}...")
            with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
                print("Serial opened.")
                with latest_lock:
                    latest["connected"] = True

                while True:
                    data = ser.read(512)
                    if data:
                        with latest_lock:
                            latest["bytes"] += len(data)
                        buffer.extend(data)
                        find_and_process_frames(buffer)

        except Exception as e:
            with latest_lock:
                latest["connected"] = False
                latest["error"] = repr(e)
            print("Serial error:", repr(e))
            time.sleep(2)


def status_card(title, value):
    return html.Div(
        [
            html.Div(title, style={"fontSize": "14px", "color": "#666"}),
            html.Div(value, style={"fontSize": "24px", "fontWeight": "bold"}),
        ],
        style={
            "padding": "14px",
            "border": "1px solid #ddd",
            "borderRadius": "12px",
            "background": "white",
            "boxShadow": "0 1px 4px rgba(0,0,0,0.08)",
        },
    )


def meshtastic_switch_style(enabled):
    return {
        "display": "inline-flex",
        "alignItems": "center",
        "gap": "12px",
        "padding": "8px 12px",
        "border": "1px solid #bbb",
        "borderRadius": "999px",
        "background": "#eefaf2" if enabled else "#f4f4f4",
        "cursor": "pointer",
    }


def meshtastic_switch_track_style(enabled):
    return {
        "width": "44px",
        "height": "24px",
        "borderRadius": "999px",
        "background": "#22c55e" if enabled else "#9ca3af",
        "position": "relative",
        "transition": "background 0.15s ease",
        "flexShrink": 0,
    }


def meshtastic_switch_handle_style(enabled):
    return {
        "position": "absolute",
        "top": "3px",
        "left": "23px" if enabled else "3px",
        "width": "18px",
        "height": "18px",
        "borderRadius": "50%",
        "background": "white",
        "boxShadow": "0 1px 2px rgba(0,0,0,0.2)",
        "transition": "left 0.15s ease",
    }


app = Dash(__name__)

default_center = [43.0, -79.0]

app.layout = html.Div(
    [
        html.H1("Drone GPS Telemetry Dashboard"),

        dcc.Store(id="meshtastic-settings"),

        html.Div(
            [
                html.H3("Meshtastic Relay"),
                html.Button(
                    [
                        html.Span(
                            html.Span(style=meshtastic_switch_handle_style(False)),
                            id="meshtastic-enabled-track",
                            style=meshtastic_switch_track_style(False),
                        ),
                        html.Span("Repeat telemetry send: OFF", id="meshtastic-enabled-label"),
                    ],
                    id="meshtastic-enabled",
                    n_clicks=0,
                    title="Toggle repeated Meshtastic telemetry sending",
                    style=meshtastic_switch_style(False),
                ),
                html.Div(
                    [
                        html.Label("Delay between messages while repeat send is ON"),
                        dcc.Dropdown(
                            id="meshtastic-delay",
                            options=[
                                {"label": f"{delay} seconds", "value": delay}
                                for delay in SEND_DELAY_OPTIONS
                            ],
                            value=10,
                            clearable=False,
                            style={"width": "220px"},
                        ),
                    ],
                    style={"display": "flex", "gap": "10px", "alignItems": "center"},
                ),
                html.Div(
                    f"Library target: Meshtastic SerialInterface({MESHTASTIC_PORT}) channel {MESHTASTIC_CHANNEL_INDEX}",
                    style={"fontSize": "13px", "color": "#666"},
                ),
                html.Div("Message template", style={"fontWeight": "bold", "marginTop": "8px"}),
                dcc.Textarea(
                    id="meshtastic-template",
                    value=DEFAULT_MESSAGE_TEMPLATE,
                    style={
                        "width": "100%",
                        "height": "110px",
                        "fontFamily": "Consolas, monospace",
                        "fontSize": "13px",
                    },
                ),
                html.Div(
                    "Available vars: {GPSLat} {GPSLon} {Satellites} {AltitudeM} {SpeedKmh} {HeadingDeg} {FlightMode} {BatteryVolts} {CurrentA} {MahUsed} {BatteryPercent} {CellCount} {CellVolts} {UplinkLQ} {DownlinkLQ} {SerialConnected} {FramesDecoded} {BytesReceived} {LastGpsAge} {RawFrame} {TelemetryError} {MeshSendCount}",
                    style={"fontSize": "13px", "color": "#666"},
                ),
                html.Button(
                    "Send debug test message",
                    id="meshtastic-debug-send",
                    n_clicks=0,
                    style={
                        "width": "240px",
                        "padding": "10px 12px",
                        "borderRadius": "8px",
                        "border": "1px solid #bbb",
                        "background": "#f7f7f7",
                        "cursor": "pointer",
                    },
                ),
            ],
            style={
                "padding": "14px",
                "border": "1px solid #ddd",
                "borderRadius": "12px",
                "background": "white",
                "boxShadow": "0 1px 4px rgba(0,0,0,0.08)",
                "marginBottom": "12px",
            },
        ),

        html.Div(
            id="status-row",
            style={
                "display": "grid",
                "gridTemplateColumns": "repeat(4, 1fr)",
                "gap": "12px",
                "marginBottom": "12px",
            },
        ),

        html.Div(
            [
                dl.Map(
                    id="map",
                    center=default_center,
                    zoom=13,
                    children=[
                        dl.TileLayer(),
                        dl.Marker(
                            id="drone-marker",
                            position=default_center,
                            children=[
                                dl.Tooltip("Drone"),
                                dl.Popup(id="marker-popup"),
                            ],
                        ),
                        dl.Polyline(id="trail", positions=[]),
                    ],
                    style={"height": "600px", "width": "100%", "borderRadius": "12px"},
                )
            ],
            style={"border": "1px solid #ddd", "borderRadius": "12px", "overflow": "hidden"},
        ),

        html.H3("Raw / Debug"),
        html.Pre(
            id="debug",
            style={
                "background": "#111",
                "color": "#0f0",
                "padding": "12px",
                "borderRadius": "8px",
                "whiteSpace": "pre-wrap",
                "fontSize": "13px",
            },
        ),

        dcc.Interval(id="tick", interval=500, n_intervals=0),
    ],
    style={
        "fontFamily": "Arial, sans-serif",
        "padding": "20px",
        "background": "#f5f5f5",
    },
)


@app.callback(
    Output("meshtastic-enabled-label", "children"),
    Output("meshtastic-enabled", "style"),
    Output("meshtastic-enabled-track", "style"),
    Output("meshtastic-enabled-track", "children"),
    Output("meshtastic-delay", "value"),
    Input("tick", "n_intervals"),
)
def sync_legacy_meshtastic_controls(_):
    with latest_lock:
        enabled = latest["meshtastic_enabled"]
        delay_s = latest["meshtastic_delay_s"]
    label = f"Repeat telemetry send: {'ON' if enabled else 'OFF'}"
    return (
        label,
        meshtastic_switch_style(enabled),
        meshtastic_switch_track_style(enabled),
        html.Span(style=meshtastic_switch_handle_style(enabled)),
        delay_s,
    )


@app.callback(
    Output("meshtastic-settings", "data", allow_duplicate=True),
    Input("meshtastic-enabled", "n_clicks"),
    State("meshtastic-delay", "value"),
    State("meshtastic-template", "value"),
    prevent_initial_call=True,
)
def toggle_meshtastic_enabled(_, delay_s, template_text):
    with latest_lock:
        latest["meshtastic_enabled"] = not latest["meshtastic_enabled"]
        if delay_s in SEND_DELAY_OPTIONS:
            latest["meshtastic_delay_s"] = delay_s
        latest["meshtastic_template"] = template_text or DEFAULT_MESSAGE_TEMPLATE
        return {
            "enabled": latest["meshtastic_enabled"],
            "delay_s": latest["meshtastic_delay_s"],
            "template": latest["meshtastic_template"],
        }


@app.callback(
    Output("meshtastic-settings", "data"),
    Input("meshtastic-delay", "value"),
    Input("meshtastic-template", "value"),
    prevent_initial_call=False,
)
def update_meshtastic_settings(delay_s, template_text):
    with latest_lock:
        if delay_s in SEND_DELAY_OPTIONS:
            latest["meshtastic_delay_s"] = delay_s
        latest["meshtastic_template"] = template_text or DEFAULT_MESSAGE_TEMPLATE
        return {
            "enabled": latest["meshtastic_enabled"],
            "delay_s": latest["meshtastic_delay_s"],
            "template": latest["meshtastic_template"],
        }


@app.callback(
    Output("meshtastic-debug-send", "children"),
    Input("meshtastic-debug-send", "n_clicks"),
    prevent_initial_call=True,
)
def trigger_debug_send(n_clicks):
    send_debug_meshtastic_message()
    return f"Send debug test message ({n_clicks})"


@app.callback(
    Output("status-row", "children"),
    Output("map", "center"),
    Output("drone-marker", "position"),
    Output("marker-popup", "children"),
    Output("trail", "positions"),
    Output("debug", "children"),
    Input("tick", "n_intervals"),
)
def update_dashboard(_):
    with latest_lock:
        snapshot = dict(latest)

    lat = snapshot["lat"]
    lon = snapshot["lon"]
    sats = snapshot["sats"]

    has_fix = not (lat == 0.0 and lon == 0.0)

    if has_fix:
        center = [lat, lon]
        marker_pos = [lat, lon]
    else:
        center = default_center
        marker_pos = default_center

    age = None
    if snapshot["last_update"]:
        age = time.time() - snapshot["last_update"]

    connected_text = "YES" if snapshot["connected"] else "NO"
    fix_text = "YES" if has_fix else "NO / INDOORS"
    relay_text = "ON" if snapshot["meshtastic_enabled"] else "OFF"

    cards = [
        status_card("Serial connected", connected_text),
        status_card("Mesh relay", relay_text),
        status_card("GPS fix", fix_text),
        status_card("Satellites", str(sats)),
        status_card("Altitude", f"{snapshot['altitude_m']} m"),
        status_card("Latitude", f"{lat:.7f}"),
        status_card("Longitude", f"{lon:.7f}"),
        status_card("Speed", f"{snapshot['speed_kmh']:.1f} km/h"),
        status_card("Heading", f"{snapshot['heading_deg']:.1f}°"),
        status_card("Battery", f"{snapshot['battery_volts']:.2f} V"),
        status_card("Cell voltage", f"{snapshot['cell_volts']:.2f} V"),
        status_card("Current", f"{snapshot['current_a']:.2f} A"),
        status_card(
            "Battery %",
            "N/A" if snapshot["battery_percent"] is None else f"{snapshot['battery_percent']}%",
        ),
        status_card("Used", f"{snapshot['mah_used']} mAh"),
        status_card(
            "Cells",
            "N/A" if snapshot["cell_count"] is None else str(snapshot["cell_count"]),
        ),
    ]

    popup = html.Div(
        [
            html.B("Drone"),
            html.Br(),
            f"Lat: {lat:.7f}",
            html.Br(),
            f"Lon: {lon:.7f}",
            html.Br(),
            f"Sats: {sats}",
            html.Br(),
            f"Alt: {snapshot['altitude_m']} m",
        ]
    )

    debug = (
        f"PORT: {PORT}\n"
        f"BAUD: {BAUD}\n"
        f"Connected: {snapshot['connected']}\n"
        f"Frames decoded: {snapshot['frames']}\n"
        f"Bytes received: {snapshot['bytes']}\n"
        f"Last GPS age: {age:.1f}s\n" if age is not None else
        f"PORT: {PORT}\n"
        f"BAUD: {BAUD}\n"
        f"Connected: {snapshot['connected']}\n"
        f"Frames decoded: {snapshot['frames']}\n"
        f"Bytes received: {snapshot['bytes']}\n"
        f"Last GPS age: never\n"
    )

    debug += (
        f"Flight mode: {snapshot['flight_mode']}\n"
        f"Battery volts: {snapshot['battery_volts']:.2f}\n"
        f"Cell volts: {snapshot['cell_volts']:.2f}\n"
        f"Cell count: {snapshot['cell_count']}\n"
        f"Current: {snapshot['current_a']:.2f}\n"
        f"mAh used: {snapshot['mah_used']}\n"
        f"Battery percent: {snapshot['battery_percent']}\n"
        f"Uplink LQ: {snapshot['link_uplink_lq']}\n"
        f"Downlink LQ: {snapshot['link_downlink_lq']}\n"
        f"Meshtastic enabled: {snapshot['meshtastic_enabled']}\n"
        f"Meshtastic delay: {snapshot['meshtastic_delay_s']} s\n"
        f"Meshtastic template: {snapshot['meshtastic_template']}\n"
        f"Meshtastic sends: {snapshot['meshtastic_send_count']}\n"
        f"Meshtastic last attempt: {snapshot['meshtastic_last_attempt']}\n"
        f"Meshtastic last sent: {snapshot['meshtastic_last_sent']}\n"
        f"Meshtastic last message: {snapshot['meshtastic_last_message']}\n"
        f"Meshtastic last result: {snapshot['meshtastic_last_result']}\n"
        f"Meshtastic error: {snapshot['meshtastic_error']}\n"
        f"Error: {snapshot['error']}\n"
        f"Last raw frame:\n{snapshot['raw_frame']}\n"
    )

    return cards, center, marker_pos, popup, list(recent_points), debug


if __name__ == "__main__":
    t = threading.Thread(target=serial_worker, daemon=True)
    t.start()

    sender = threading.Thread(target=meshtastic_sender_worker, daemon=True)
    sender.start()

    app.run(debug=False, host="127.0.0.1", port=8050)
