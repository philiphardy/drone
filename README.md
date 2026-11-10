**Project Overview**

This repository contains a small flight-control program for the Raspberry Pi Pico (RP2040). It reads an IMU (MPU6050), runs a simple flight-control loop, and drives motor outputs (PWM). It's intended as a hobby / learning project.

**Prerequisites**

- **Toolchain**: `arm-none-eabi-gcc` (ARM embedded toolchain)
- **Build tools**: `cmake`, `ninja`
- **Pico SDK**: either installed locally or available via `PICO_SDK_PATH` (see project `CMakeLists.txt`)
- **Flashing tools**: use the Pico UF2 bootloader (drag-and-drop) or the included VS Code tasks which call `picotool` / `openocd`.

**Quick Build (cmd.exe)**

1. Create and configure the build directory:

```
cmake -S . -B build -G Ninja -DPICO_SDK_PATH="%USERPROFILE%\\.pico-sdk\\pico-sdk"
```

2. Build the firmware:

```
ninja -C build
```

Alternatively, use the provided VS Code tasks: `Compile Project` builds with the project `build` directory.

**Flash / Run**

- Using UF2: put the Pico into BOOTSEL mode and copy `build\drone.uf2` to the mounted drive.
- Using the provided tasks: run the `Run Project` or `Flash` tasks in VS Code (they call `picotool` and `openocd` respectively).

**Hardware**

- 1 x 7.6v 2S LiPo Power Supply
- 1 x RP2040 (MCU)
- 1 x MPU6050 (IMU)
TODO

**Wiring**

```mermaid
%%{init: {"themeVariables": {"fontSize": "48px"}, "themeCSS": ".edgeLabel, .edgeLabel p, .edgeLabel span {font-size: 32px !important;}", "flowchart": {"wrappingWidth": 2400}}}%%
flowchart TB
    BAT["<img src='Battery.png' width='600' height='232'/><br/><b>2S LiPo</b><br/>7.4 V nominal / 8.4 V full"]

    BUCK["<img src='voltage%20buck.png' width='150'/><br/><b>LM2596 buck converter</b><br/>set to 5 V"]
    DIV["<b>Battery divider</b><br/>20k (top) / 10k (bottom), x3"]
    PICO["<img src='Pico%20Pinout.png' width='2188' height='1257'/><br/><b>Raspberry Pi Pico (RP2040)</b>"]
    ESC["<img src='ESC%20board.png' width='676' height='736'/><br/><b>AERO SELFIE 30A 4-in-1 ESC</b>"]
    IMU["<img src='MPU-6050.png' width='150'/><br/><b>MPU6050 (GY-521)</b><br/>IMU, I2C addr 0x68"]

    M0["<img src='Motor.png' width='217' height='248'/><br/>Motor GP0 (-X)"]
    M1["<img src='Motor.png' width='217' height='248'/><br/>Motor GP1 (+Y)"]
    M2["<img src='Motor.png' width='217' height='248'/><br/>Motor GP2 (-Y)"]
    M3["<img src='Motor.png' width='217' height='248'/><br/>Motor GP3 (+X)"]

    %% Power
    BAT -- "BAT+ / BAT-" --> ESC
    BAT -- "BAT+ / BAT-" --> BUCK
    BUCK -- "OUT+ (5 V) to VSYS, pin 39<br/>OUT- to GND, pin 38" --> PICO
    PICO -- "3V3(OUT), pin 36 to VCC<br/>GND, pin 38 to GND" --> IMU

    %% Signals
    PICO <-- "GP26 (pin 31) SDA<br/>GP27 (pin 32) SCL<br/>I2C1, pull-ups on GP26/GP27" --> IMU
    PICO -- "GP0 (pin 1) to signal 1<br/>GP1 (pin 2) to signal 2<br/>GP2 (pin 4) to signal 3<br/>GP3 (pin 5) to signal 4<br/>DSHOT via PIO<br/>GND, pin 3 to ESC connector -" --> ESC

    %% Battery sense
    BAT -- "BAT+" --> DIV
    DIV -- "midpoint to GP28 (pin 34, ADC2)<br/>bottom leg to GND (pin 35, AGND)" --> PICO

    %% Motors
    ESC -- "1A / 1B / 1C" --> M0
    ESC -- "2A / 2B / 2C" --> M1
    ESC -- "3A / 3B / 3C" --> M2
    ESC -- "4A / 4B / 4C" --> M3
```

Notes:

- All grounds (battery, ESC, buck, Pico, IMU, divider) are common.
- The ESC `+` pin on its signal connector is battery voltage and is left unconnected, as is the current-sense pin `C`; the ESC has no BEC.
- Motor GPn uses ESC channel n+1 for both signal and outputs: GP0 to signal 1 and 1A-1C, through GP3 to signal 4 and 4A-4C.
- Set the buck converter to 5 V before connecting it to VSYS.
- The battery divider is 20k from BAT+ to GP28 and 10k from GP28 to GND (x3 ratio), which keeps GP28 at or below 2.8 V on a full 8.4 V pack.

**Source Layout**

- `main.c` — entry point: system init, ESC arming, selects flight controller, motor test (`RUN_MOTOR_TEST`), IMU level calibration (`RUN_IMU_CAL`) or battery voltage test (`RUN_BATTERY_TEST`)
- `flight_controller.c` / `.h` — fixed-rate stabilization loop (roll/pitch level-hold, yaw-rate PIDs)
- `flight_plan.c` / `.h` — scripted flight standing in for an RC link (spool up, climb, hold, descend by climb-rate setpoint), with hover feedforward scaled to battery voltage
- `attitude.c` / `.h` — attitude estimator: IMU sampling, gyro bias calibration, Kalman-filtered roll/pitch
- `mixer.c` / `.h` — + configuration motor mixer and motor layout
- `failsafe.c` / `.h` — stop motors, dump the flight log and halt
- `flight_log.c` / `.h` — fixed-point RAM flight log, dumped as CSV over USB
- `battery.c` / `.h` — 2S pack voltage sensing (ADC, GP28 through a divider)
- `battery_test.c` / `.h` — bench test that prints the battery voltage over USB
- `motor_test.c` / `.h` — per-pair bench motor test that logs IMU rates
- `imu_cal.c` / `.h` — IMU level calibration that prints the mounting offsets for `attitude.c`
- `pid.c` / `.h` — PID controller
- `kalman.c` / `.h` — per-axis Kalman filter
- `motor.c` / `.h` — DSHOT motor output (PIO)
- `dshot_tx.pio` — PIO program for the DSHOT transmitter
- `mpu6050.c` / `mpu6050.h` — IMU driver (I2C)
- `vector_3d.h` — small math helpers for 3D vectors
- `download_flight_log.py` — saves the flight log from the Pico over USB serial to `flight logs/flight_log_X.csv` (needs `pyserial`)
- `CMakeLists.txt` — project build configuration
