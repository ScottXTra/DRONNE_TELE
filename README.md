# DRONNE_TELE

Live CRSF telemetry dashboard for an FPV drone/controller link, with optional Meshtastic relay output.

Primary script:
- `gps_dashboard.py`

R&D / helper scripts:
- `RandD\list_ports.py`
- `RandD\live_decode_radio.py`
- `RandD\read_radio.py`

## What It Does

Telemetry flow:

`Drone -> ELRS/CRSF -> USB serial -> Computer -> Meshtastic USB node -> Mesh`

The dashboard can:
- read CRSF telemetry from your radio/controller over USB
- show GPS and link data in a web dashboard
- decode battery telemetry
- send formatted telemetry messages out through a Meshtastic node connected to the PC

## Features

- live GPS map
- GPS trail
- link quality display
- flight mode display
- battery telemetry display
- Meshtastic relay enable/disable toggle
- selectable send delay
- customizable outgoing message template
- debug button to manually send the current template output

## Requirements

- Windows
- Python 3.10+ recommended
- FPV controller/radio connected by USB
- Meshtastic node connected by USB

Python packages are listed in `requirements.txt`.

## Setup

Create and activate a virtual environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

Install dependencies:

```powershell
pip install -r requirements.txt
```

## Configure Serial Ports

Open `gps_dashboard.py` and set these values near the top of the file:

```python
PORT = "COM6"
BAUD = 420000
MESHTASTIC_PORT = "COM7"
MESHTASTIC_CHANNEL_INDEX = 0
```

Meaning:
- `PORT`: your FPV controller / CRSF telemetry serial port
- `BAUD`: CRSF serial baud rate
- `MESHTASTIC_PORT`: the USB serial port for your Meshtastic node
- `MESHTASTIC_CHANNEL_INDEX`: Meshtastic channel index to send on

If you are not sure which COM port is which, you can use:

```powershell
python .\RandD\list_ports.py
```

## Run

Start the dashboard:

```powershell
python .\gps_dashboard.py
```

Then open:

```text
http://127.0.0.1:8050
```

## Using The Dashboard

### Telemetry Input

The dashboard reads CRSF telemetry from the serial port defined by `PORT`.

Currently supported telemetry includes:
- GPS
- link statistics
- flight mode
- battery sensor

### Meshtastic Relay

In the `Meshtastic Relay` panel you can:
- enable or disable sending
- choose the send interval
- edit the outgoing message template
- click the debug send button to send one message immediately

The debug send button uses the same message template as the automatic relay.

### Message Template Variables

You can customize the Meshtastic message using placeholders like:

- `{GPSLat}`
- `{GPSLon}`
- `{Satellites}`
- `{AltitudeM}`
- `{SpeedKmh}`
- `{HeadingDeg}`
- `{FlightMode}`
- `{BatteryVolts}`
- `{CurrentA}`
- `{MahUsed}`
- `{BatteryPercent}`
- `{CellCount}`
- `{CellVolts}`
- `{UplinkLQ}`
- `{DownlinkLQ}`
- `{SerialConnected}`
- `{FramesDecoded}`
- `{BytesReceived}`
- `{LastGpsAge}`
- `{RawFrame}`
- `{TelemetryError}`
- `{MeshSendCount}`

Example template:

```text
GPS {GPSLat},{GPSLon} | Alt {AltitudeM}m | Battery {BatteryVolts}V | Cell {CellVolts}V | Mode {FlightMode}
```

If telemetry is missing, disconnected, or still zero, the relay can still send using the current template values.

## Troubleshooting

### `could not open port 'COMx'`

The selected COM port is wrong or the device is not connected.

Check:
- the controller is plugged in
- the Meshtastic node is plugged in
- Windows Device Manager shows the expected COM ports
- `PORT` and `MESHTASTIC_PORT` match the real device ports

### Dashboard opens but no telemetry updates

Check:
- the correct CRSF serial port is selected
- the baud rate is correct
- the controller is actually outputting telemetry over USB

### Meshtastic messages are not sending

Check:
- `MESHTASTIC_PORT` is correct
- the Meshtastic node is connected and working
- the chosen `MESHTASTIC_CHANNEL_INDEX` is valid
- the debug panel for `Meshtastic error`

### Browser shows old callback errors

If you updated the script while the page was already open:
- stop the app
- restart it
- hard refresh the browser tab

## Notes

- `gps_dashboard.py` is the intended main script.
- The scripts in `RandD\` are just helper / testing tools.
