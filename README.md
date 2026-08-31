Serial-driven autopilot and telemetry stack for an autonomous surface vehicle (USV) operating on Lake Garda. The system fuses a dual-antenna RTK GPS for heading, drives a waypoint-following autopilot over a motor-control serial link, ingests water-quality and sonar telemetry from an onboard PIC controller, and serves a live monitoring dashboard over the network.

## Features

- **Waypoint autopilot** — steers the boat through a fixed list of GPS targets using a turn-then-straight state machine, computing heading and distance error each control loop iteration (`seriali/serial_motori.py`).
- **Dual-antenna RTK heading** — decodes the u-blox `UBX_NAV_RELPOSNED` binary message from a two-antenna GPS setup (rover + base) for a stable compass heading, alongside standard NMEA `$GNGGA` position fixes (`seriali/serial_doppia_antenna.py`, `classes/classes_externals.py`).
- **Sensor telemetry ingestion** — parses PIC-controller messages (`$AUTO_`, `$PWR_`, `$H2O_`, `$IMU_`, `$GPS_`, `$INFO_`, `$ADC_`, `$SON_`) covering battery voltage/current, water pH/TDS/temperature/oxygen, sonar depth, and IMU orientation (`seriali/serial_segnali.py`, `functions/Serials_functions/functions_external.py`).
- **Live web dashboard** — a Flask server exposes a `/data` JSON endpoint and a Plotly-powered dashboard (`templates/index.html`) that streams pH, temperature, oxygen, sonar, and conductivity readings in real time.
- **CSV flight logging** — every autopilot control-loop iteration is logged to `boat_log.csv` (timestamp, GPS position, motor command, temperature, sonar reading) for post-mission analysis.

## Architecture

The system runs as four concurrent threads launched from `scripts/main.py`, each owning one physical serial connection:

| Thread | Module | Serial connector | Baud rate | Role |
|---|---|---|---|---|
| `read_thread` | `seriali/serial_doppia_antenna.py` | GPS (dual antenna) | 115200 | Heading (UBX binary) + position (NMEA) |
| `write_thread` | `seriali/serial_motori.py` | Motors | 57600 | Waypoint autopilot, motor commands, CSV logging |
| `signal_thread` | `seriali/serial_segnali.py` | PIC/sensors | 57600 | Battery, water quality, sonar, IMU telemetry |
| `flask_thread` | `scripts/main.py` | — | — | Web dashboard (`/` and `/data`) |

Shared state is held in two plain data objects, populated by the threads above and read by the dashboard:

- `all_data` (`Dati`) — current/target/initial GPS position, heading, and navigation error
- `signal_data` (`Signal_data`) — latest sensor and power telemetry

Supporting modules:

- `functions/Serials_functions/hex_functions.py` — decoders for u-blox UBX binary field types (`U1`/`I1`/`X1`/`U2`/`I2`/`X2`/`U4`/`I4`/`X4`), used to parse the binary GPS heading message.
- `functions/Serials_functions/functions_external.py` — command parsing, coordinate conversion (NMEA DDM → decimal degrees), bearing/distance math (haversine, initial bearing), and the PIC motor-command/checksum protocol.

See `Documentation/` (not tracked in this repository — see [Documentation](#documentation)) for the full hardware and protocol reference, including serial pinouts, message formats, and the u-blox interface spec.

## Prerequisites

- Python 3.9+
- Physical (or emulated) serial access to three connectors:
  - Dual-antenna GPS module (u-blox ZED-F9P, RTK rover + base)
  - Motor controller (PIC-based ESC interface)
  - PIC/sensor telemetry bus (battery, H2O sensors, sonar, IMU)

## Installation

```bash
git clone git@github.com:Isla-lab/GardAI_demo.git
cd GardAI_demo
pip install flask pyserial numpy
```

> **Note:** this project does not yet ship a `requirements.txt` — the above covers the current runtime dependencies (`flask`, `pyserial`, `numpy`).

## Usage

1. Connect the three serial devices and confirm their device paths.
2. Update the device paths in `scripts/main.py` (`/dev/GPS`, `/dev/MOTORI`, `/dev/Dati`) to match your system — these are Linux-style paths from the original deployment machine and will need to be changed (e.g. to `COM3`, `COM4`, ...) on Windows.
3. Update `TEMPLATE_PATH` in `scripts/main.py`, which is currently hardcoded to the original deployment machine's filesystem path.
4. Run the system:

```bash
python scripts/main.py
```

5. Open the dashboard in a browser at the host/port configured in `scripts/main.py` (default: `http://192.168.95.130:5000`).

Autopilot waypoints are currently hardcoded as a list of NMEA DDM coordinate strings in `seriali/serial_motori.py` (`target = [...]`) — edit this list to change the mission route.

## Project Structure

```
.
├── classes/
│   └── classes_externals.py         # DoppiaAntenna, signal_data, all_data — shared state objects
├── functions/
│   └── Serials_functions/
│       ├── functions_external.py    # command parsing, coordinate & bearing math, checksums
│       └── hex_functions.py         # UBX binary field decoders
├── scripts/
│   └── main.py                      # entry point: serial setup, thread orchestration, Flask app
├── seriali/
│   ├── serial_doppia_antenna.py     # GPS heading + position reader
│   ├── serial_motori.py             # waypoint autopilot + motor command writer
│   └── serial_segnali.py            # PIC/sensor telemetry reader
├── templates/
│   └── index.html                   # live Plotly dashboard
└── boat_log.csv                     # generated at runtime, gitignored
```

## Documentation

Hardware architecture, wiring, and serial protocol details (power system, GPS setup, PIC firmware message formats, radio/LoRa configuration, and onboard device credentials) are kept in `Documentation/Barca-2024.pdf`. This folder is intentionally excluded from version control (see `.gitignore`) because it contains device credentials — request it directly from the project maintainers if you need hardware-level reference material.

## License

No license file is currently included in this repository. Contact the project maintainers ([Isla-lab](https://github.com/Isla-lab)) for usage terms.
