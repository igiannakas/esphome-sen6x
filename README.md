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

Once the upstream PRs land in a release, everything but the last two rows is in core.

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
| `voc_index`, `nox_index` | | | ✓ | ✓ | ✓ | ✓ |
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
| `sen6x.restore_voc_state` | stop, hand the stored state back, start again |
| `sen6x.reset_voc_algorithm` | device reset of the VOC engine and drop the stored copy |

All take the component id (`sen6x.save_voc_state: sen6x_dev`). Every command opens a wait window
during which the next one is refused with "Device busy"; a fan clean needs stop → 2 s → clean →
11 s → start. From a lambda: `id(sen6x_dev).save_voc_state()`, `restore_voc_state()`,
`reset_voc_algorithm()` and `has_voc_state()` (true once a state is in flash).

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

## Examples

`examples/air-quality-xiao-esp32c6-sen66.yaml` is a complete air-quality node (XIAO ESP32-C6,
SEN66 + VEML7700, status LED, open-window detection, VOC state persistence); the `-sen65` variant
is the same node on a part without CO2. `examples/sen65-veml7700-minimal.yaml` is the bare
sensor. Wi-Fi credentials come from a `secrets.yaml` next to the file (`secrets.yaml.example`).

Each air-quality node is two files. The node file holds the settings: its `substitutions:` and,
on a SEN66, the CO2 block. The logic both nodes share is in `examples/common/air-quality-core.yaml`,
which the node file pulls in under `packages:`. Copy the `common/` folder along with the node file.

## Tests

`tests/` validates every option on ESP32 (esp-idf), ESP8266 and RP2040 with `esphome config`,
plus the partial `algorithm_tuning` case; CI runs them and the examples on every push.

## Repository layout

| Path | Contents |
|---|---|
| `components/sen6x/` | the component (what `external_components` fetches) |
| `examples/` | complete configs |
| `tests/` | config-validation tests |

## License

GPL-3.0 — see `LICENSE`.
