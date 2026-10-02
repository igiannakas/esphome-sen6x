# `sen6x` — Sensirion SEN62 / SEN63C / SEN65 / SEN66 / SEN68 / SEN69C for ESPHome

ESPHome's built-in `sen6x` component with the full data sheet implemented, plus VOC
algorithm state persistence. Loading it as an external component overrides the built-in one
(ESPHome logs `External components are overriding built-in components`), so nothing else in a
config changes.

On top of the core component this carries:

| Change | Origin |
|---|---|
| CO2 options (`automatic_self_calibration`, `altitude_compensation`, `ambient_pressure_compensation`, `…_source`) | upstream PR #18780 |
| `temperature_compensation`, `temperature_acceleration`, `startup_delay` | upstream PR #18781 |
| PM number-concentration sensors `pmc_0_5` … `pmc_10_0` (command 0x0316) | upstream PR #18782 |
| Actions `start_measurement`, `stop_measurement`, `start_fan_cleaning`, `activate_sht_heater` | upstream PR #18783 |
| Device-status binary sensors (`fan_error` … `pm_error`, read-and-clear 0xD210) | upstream PR #18784 |
| Action wait windows no longer break after 24.8 days of uptime (`millis()` wrap) | this repo |
| **VOC algorithm state** save / restore / reset (0x6181, 0xD304), with a boot-time restore issued in idle mode as the datasheet requires | this repo |
| Raw VOC signal sensor `raw_voc` (0x0405, 0x0455) | this repo |

Once the upstream PRs land in a release, everything but the last three rows is in core.

## Installation

```yaml
external_components:
  - source: github://igiannakas/esphome-sen6x
    components: [sen6x]
```

## Which sensor has what

| | SEN62 | SEN63C | SEN65 | SEN66 | SEN68 | SEN69C |
|---|:-:|:-:|:-:|:-:|:-:|:-:|
| PM mass (`pm_*`) and number (`pmc_*`) | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `temperature`, `humidity` | | ✓ | ✓ | ✓ | ✓ | ✓ |
| `voc_index`, `nox_index`, `raw_voc` | | | ✓ | ✓ | ✓ | ✓ |
| `co2` | | ✓ | | ✓ | | ✓ |
| `formaldehyde` | | | | | ✓ | ✓ |

## Configuration

```yaml
i2c:
  sda: GPIO22
  scl: GPIO23

sensor:
  - platform: sen6x
    id: sen6x_dev
    address: 0x6B                     # default
    update_interval: 60s              # default
    type: SEN66                       # optional; the part is detected from its product name
    startup_delay: 60s                # readings are held back this long after start (max 1h)
    restore_voc_state_on_boot: true   # default; hand a saved VOC state back during setup
    on_voc_state_stored:              # the stored VOC state changed; see VOC algorithm state
      then:
        - logger.log: VOC state stored
    on_voc_state_update:              # the latest VOC state changed (reads too); same section
      then:
        - logger.log: VOC state updated
    temperature_compensation:         # self-heating correction, datasheet 4.8.13
      offset: -0.3                    # °C, -163.84..163.835
      normalized_offset_slope: 0      # -3.2768..3.2767
      time_constant: 0                # s, 0..65535
    temperature_acceleration:         # datasheet 4.8.14; defaults shown
      k: 20.0
      p: 20.0
      t1: 100.0
      t2: 300.0
    pm_1_0:
      name: PM1
    pm_2_5:
      name: PM2.5
    pm_4_0:
      name: PM4
    pm_10_0:
      name: PM10
    # number concentrations, #/cm³
    pmc_0_5:
      name: PM0.5 count
    pmc_1_0:
      name: PM1 count
    pmc_2_5:
      name: PM2.5 count
    pmc_4_0:
      name: PM4 count
    pmc_10_0:
      name: PM10 count
    temperature:
      name: Temperature
    humidity:
      name: Humidity
    voc_index:
      name: VOC Index
      algorithm_tuning:               # all optional; defaults shown
        index_offset: 100             # 1..250
        learning_time_offset_hours: 12   # 1..1000
        learning_time_gain_hours: 12     # 1..1000
        gating_max_duration_minutes: 180 # 0..3000
        std_initial: 50               # 10..5000 (VOC only)
        gain_factor: 230              # 1..1000
    nox_index:
      name: NOx Index
      algorithm_tuning:               # same keys as VOC except std_initial; defaults shown
        index_offset: 1
        learning_time_offset_hours: 12
        learning_time_gain_hours: 12
        gating_max_duration_minutes: 720
        gain_factor: 230
    raw_voc:                          # raw VOC signal, ticks
      name: VOC Raw
      filters:
        - lambda: return 65535.0 - x;  # optional; makes it rise with the VOC Index
    co2:
      name: CO2
      automatic_self_calibration: true
      altitude_compensation: 100      # m, 0..3000
      ambient_pressure_compensation: 1013            # hPa, 700..1200
      ambient_pressure_compensation_source: my_bmp   # a pressure sensor id; written when it changes
    formaldehyde:                     # ppb
      name: Formaldehyde
```

`voc:` and `nox:` still work as the old names of `voc_index:` / `nox_index:` (removed in 2027.2).

`raw_voc` is the VOC sensor's raw signal (SRAW_VOC, 0–65535 ticks, datasheet 4.8.11 / 4.8.12),
the value the VOC Index is worked out from. It is read in the same update as the index, so it
publishes at the same rate. It falls as VOCs rise; `65535.0 - x` mirrors it within its range so a
graph of it moves the same way as the index and stays positive. The example node measures it
from a zero you set instead; see [VOC Raw](#voc-raw).

### Device status

```yaml
binary_sensor:
  - platform: sen6x
    sen6x_id: sen6x_dev
    fan_error:
      name: Fan Error
    fan_speed_warning:
      name: Fan Speed Warning
    rht_error:
      name: RH&T Error
    gas_error:
      name: Gas Error
    co2_error:
      name: CO2 Error
    hcho_error:
      name: HCHO Error
    pm_error:
      name: PM Error
```

The status register is read *and cleared* at the start of every update, so each flag means "this
happened since the last update", not "this is happening now" — a one-cycle blip is real and shows.
Flags the detected part cannot report are disabled at setup.

### Buttons and actions

```yaml
button:
  - platform: sen6x
    sen6x_id: sen6x_dev
    save_voc_state:
      name: VOC Baseline Save
    restore_voc_state:
      name: VOC Baseline Restore
    reset_voc_algorithm:
      name: VOC Baseline Reset
```

| Action | What it does |
|---|---|
| `sen6x.start_measurement` / `sen6x.stop_measurement` | measurement on/off; a start re-arms `startup_delay` |
| `sen6x.start_fan_cleaning` | ~10 s fan burst; idle mode only (datasheet 4.8.22), warns and does nothing while measuring |
| `sen6x.activate_sht_heater` | RH&T heater; idle mode only (4.8.23) |
| `sen6x.save_voc_state` | read the VOC algorithm state and write it to flash (works while measuring) |
| `sen6x.read_voc_state` | read the VOC algorithm state without storing it |
| `sen6x.restore_voc_state` | stop, hand the stored state back, start again |
| `sen6x.reset_voc_algorithm` | device reset of the VOC engine and drop the stored copy |

All take the component id (`sen6x.save_voc_state: sen6x_dev`). Every command opens a wait window
during which the next one is refused with "Device busy"; a fan clean needs stop → 2 s → clean →
11 s → start. From a lambda: `id(sen6x_dev).save_voc_state()`, `restore_voc_state()`,
`reset_voc_algorithm()`, `read_voc_state()`, `has_voc_state()` (true once a state is in flash),
`get_voc_state()` (its four words), and `has_latest_voc_state()` / `get_latest_voc_state()` (the
most recent state read or saved).

## VOC algorithm state

The VOC Index is relative: the sensor scores the air against what it has learned is typical, and
that learning lives in eight opaque bytes it hands over on request (0x6181) and takes back in idle
mode. Without persistence every reboot re-anchors the baseline for ~45 minutes. This component
never saves on its own; the config decides when. The pattern that works:

```yaml
ota:
  - platform: esphome
    on_begin:
      then:
        - sen6x.save_voc_state: sen6x_dev   # the reboot the state exists to bridge

interval:
  - interval: 6h                              # insurance against a power cut
    then:
      - sen6x.save_voc_state: sen6x_dev
```

`on_shutdown` is too late — the save completes on a 20 ms timer that a shutting-down loop never
runs. A save is one 8-byte NVS write; at 6 h that is ~1,500 a year. With
`restore_voc_state_on_boot: true` (the default) the stored state is written back during setup,
while the sensor is still idle and before the measurement starts — the only window in which it is
accepted. NOx has no equivalent command and starts from scratch on every boot.

`on_voc_state_stored` runs whenever the stored state changes: after every save, once during setup
when it is read back from flash, and when a reset clears it. `get_voc_state()` returns the four
words. The datasheet treats them as opaque; the example node decodes them as Sensirion's gas index
algorithm stores its state, into the learned mean and standard deviation of the raw VOC signal.

`sen6x.read_voc_state` fetches the state without writing flash, so it can run far more often than a
save (the datasheet allows it every measurement interval). A restore still hands back the stored
copy. `on_voc_state_update` runs whenever `get_latest_voc_state()` changes: after every read or
save, once during setup from the stored copy, and when a reset clears it.

## Examples

`examples/air-quality-xiao-esp32c6-sen66.yaml` is a complete air-quality node: a XIAO ESP32-C6
with a SEN66 and a VEML7700 light sensor. The `-sen65` variant is the same node on a part without
CO2. Wi-Fi credentials come from a `secrets.yaml` next to the file (`secrets.yaml.example`).

Each node is two files. The node file holds the settings: its `substitutions:` and, on a SEN66,
the CO2 block. The logic both nodes share is in `common/air-quality-core.yaml`, which the node
file fetches from this repository on GitHub under `packages:` every time it is built. The node
file and `secrets.yaml` are all you need locally.

What the node does:

- **Status LED** — green, amber or red for the worst of CO2, PM, VOC and NOx against the levels in
  the node file, flashing for "ventilate" and sensor faults, breathing while a window is open.
  With LED - Ambient Light Control on, it dims with the room and stays off in the dark.
- **Open-window detection** — Open Window Detected comes on when the room cools quickly and goes
  off when it warms back up. Detection pauses while sun is warming the board.
- **VOC baseline** — saved to flash every `voc_state_save_interval` and on SEN - VOC Baseline Save,
  and handed back at boot. SEN - VOC Mean, Std Dev and Sensitivity show what the sensor has
  learned, refreshed every `voc_state_read_interval`.
- **VOC Raw** — the raw VOC signal measured from a zero you set; see below.

ESP - Uptime, ESP - Temperature, ESP - WiFi Signal and LED - Brightness start disabled in Home
Assistant; enable them there to see them.

### VOC Raw

VOC Raw is SEN - VOC Raw Zero Offset minus the raw VOC reading averaged over 30 s, so it reads 0
at the zero and rises as VOCs rise. SEN - VOC Raw Value (pre-offset) is that average before the
zero is taken off; compare it with the offset to see how far the signal has drifted.

SEN - VOC Raw Set Zero sets the zero to the current reading, or type one into SEN - VOC Raw Zero
Offset. The zero is kept across restarts. SEN - VOC Raw Zero Offset Sync decides how it moves on
its own:

| Option | The zero |
|---|---|
| Cleanest air | the cleanest reading seen; it only moves towards cleaner air |
| VOC Mean | follows SEN - VOC Mean, the air the sensor counts as normal |
| Cleanest air (N h) | the cleanest reading of the last `voc_raw_zero_window_hours` hours |
| Cleanest air (N h) + airing reset | as Cleanest air (N h), and when a window is opened and the air gets cleaner than just before, the zero and its history start again from the airing (default) |
| Off | moves only when you set it |

On the Cleanest air options VOC Raw stays at 0 or above. For 10 minutes after a restart, a fan
clean or a VOC baseline restore or reset, readings are not used while the sensor settles, and VOC
Raw can dip below 0. On the two N h options, Set Zero also clears the history and starts it again
from the current reading.

SEN - VOC Raw Zero Offset Log shows the last thing that happened to the zero, with the time: each
move, from and to, and what made it. Home Assistant's history keeps the earlier entries.

### Node settings

Each block in the node file's `substitutions:` is described where it is set. Values ending
`_default` only set a starting value in Home Assistant; once the node has booted, change them
there.

| Block | Settings | Example |
|---|---|---|
| SENSOR MODEL | `sen_model`, `co2_state`, `co2_fault` | SEN66 |
| DEVICE | `device_name`, `friendly_name` | |
| NETWORK | `wifi_ssid`, `wifi_password` | from `secrets.yaml` |
| PINS | `pin_sda`, `pin_scl`, `pin_status_led` | D4, D5, D1 |
| SEN6x | `sen6x_sample_interval`, `sen6x_temp_offset`, `sen6x_timeout_ms` | 10s, -0.30, 60000 |
| AIR QUALITY LEVELS | `co2_l1` … `nox_l3` | |
| VOC LEARNING | `voc_gating_max_duration_minutes`, `voc_learning_time_offset_hours`, `voc_learning_time_gain_hours` | 720, 24, 24 |
| OPEN WINDOW DETECTION | `temp_rate_interval_s`, `temp_rate_window` | 30, 6 |
| | `window_open_rate_default`, `window_close_rate_default` | -0.07, 0.02 °C/min |
| | `window_open_samples`, `window_close_samples`, `window_max_hold_default_min` | 2, 4, 60 |
| SUN LOCKOUT | `sun_lockout_lux_default`, `sun_lockout_minutes_default`, `heat_spike_rate_default`, `window_settle_samples` | 1000, 45, 0.30, 3 |
| VOC BASELINE SAVE | `voc_state_save_interval` | 6h |
| VOC CALIBRATION READ | `voc_state_read_interval` | 10min |
| VOC RAW ZERO WINDOW | `voc_raw_zero_window_hours`, 1 to 255 | 48 |
| LIGHT SENSOR | `lux_sample_interval`, `lux_publish_min_gap`, `lux_publish_delta`, `lux_publish_heartbeat`, `dark_room_threshold_default` | 2s, 4s, 10%, 300s, 1 |
| STATUS LED | `led_brightness_max_default`, `led_brightness_min_default`, `led_brightness_max_lux_default` | 100, 20, 400 |
| | `led_flash_on`, `led_flash_off`, `led_breathe_step`, `led_breathe_fade` | 200ms, 800ms, 2000ms, 1800ms |
| DIAGNOSTICS | `diag_interval` | 60s |

`voc_state_read_interval`, `voc_raw_zero_window_hours` and the three `led_brightness_*_default`
values may be left out; the core falls back to the values shown.

## Tests

`tests/` validates every option on ESP32 (esp-idf), ESP8266 and RP2040 with `esphome config`,
plus the partial `algorithm_tuning` case; CI runs them and the examples on every push.

## Repository layout

| Path | Contents |
|---|---|
| `components/sen6x/` | the component (what `external_components` fetches) |
| `common/` | the air-quality logic the example nodes fetch under `packages:` |
| `examples/` | complete configs |
| `tests/` | config-validation tests |

## License

GPL-3.0 — see `LICENSE`.
