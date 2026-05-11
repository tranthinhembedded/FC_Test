# STM32 Flight Controller - FC_F411

Firmware flight controller cho STM32F411, thiết kế theo kiến trúc module, tập trung vào điều khiển quadcopter thời gian thực, sensor fusion, PID cascade, RC failsafe, optical-flow position hold và telemetry qua UART DMA.

> Cảnh báo an toàn: firmware này điều khiển ESC và motor thật. Luôn test khi đã tháo propeller, cấp nguồn qua current-limited bench supply nếu có, kiểm tra đúng thứ tự motor/chiều quay trước khi arm.

---

## Mục lục

- [Tổng quan](#tổng-quan)
- [Tính năng chính](#tính-năng-chính)
- [Phần cứng mục tiêu](#phần-cứng-mục-tiêu)
- [Sơ đồ ngoại vi và pinout](#sơ-đồ-ngoại-vi-và-pinout)
- [Cấu trúc source code](#cấu-trúc-source-code)
- [Luồng khởi động firmware](#luồng-khởi-động-firmware)
- [Main loop thời gian thực](#main-loop-thời-gian-thực)
- [Flight modes](#flight-modes)
- [RC receiver iBUS](#rc-receiver-ibus)
- [Optical Flow và Position Hold](#optical-flow-và-position-hold)
- [PID tuning runtime](#pid-tuning-runtime)
- [Telemetry](#telemetry)
- [Sensor system](#sensor-system)
- [Failsafe và arming logic](#failsafe-và-arming-logic)
- [Build và flash](#build-và-flash)
- [Checklist test an toàn](#checklist-test-an-toàn)
- [Debug nhanh](#debug-nhanh)
- [Giới hạn hiện tại](#giới-hạn-hiện-tại)
- [Roadmap đề xuất](#roadmap-đề-xuất)

---

## Tổng quan

`FC_F411` là firmware flight controller viết bằng C cho STM32F411. Dự án dùng STM32 HAL, CMSIS DSP (`arm_math.h`) và tổ chức source theo từng lớp chức năng:

```text
Input / Communication
        ↓
Sensor drivers
        ↓
Sensor fusion / state estimation
        ↓
Flight control / PID cascade
        ↓
Motor mixer
        ↓
PWM output to ESCs
```

Firmware hiện tại ưu tiên:

- Vòng điều khiển 1 kHz.
- Đọc RC iBUS bằng USART2 DMA circular ring buffer.
- Đọc optical flow trực tiếp bằng USART6 DMA circular ring buffer.
- Điều khiển attitude/rate bằng cascaded PID.
- Position Hold bằng optical flow, yaw compensation và PID vị trí/vận tốc.
- Health check cảm biến, pre-arm check, RC loss failsafe và critical disarm.
- Telemetry JSON 10 Hz qua USART1 DMA TX.
- PID tuning runtime qua UART command.

---

## Tính năng chính

### Flight control

- Angle/Hover mode tự cân bằng.
- Rate/Acro mode điều khiển tốc độ góc.
- Cascaded PID cho attitude và angular rate.
- Motor mixing cho quad X.
- Giới hạn góc mục tiêu, giới hạn output PID và anti-windup cơ bản.
- Reset PID khi đổi mode, disarm hoặc failsafe.

### Position hold bằng optical flow

- Hỗ trợ optical-flow velocity input.
- Tích phân velocity để ước lượng vị trí tương đối trong earth frame.
- Capture hold position khi stick roll/pitch ở gần center.
- Position PID tạo velocity target.
- Velocity PID tạo tilt correction.
- Tự invalidate khi optical data stale, chất lượng thấp, altitude không hợp lệ hoặc velocity vượt giới hạn.

### Communication

- USART2: RC iBUS DMA RX.
- USART1: telemetry JSON và PID tuning command.
- USART6: optical flow direct parser.
- DMA circular buffer cho RX để giảm blocking trong loop 1 kHz.
- UART error recovery có stop/restart DMA.

### Sensors

- IMU: ICM20602 qua SPI1.
- Magnetometer: HMC5883L hoặc QMC5883L qua I2C.
- Barometer: BMP280 hoặc BME280 qua I2C.
- GPS/NMEA và Kalman 1D đã có module, nhưng chưa phải đường xử lý chính trong `main.c` hiện tại.

### Safety

- Preflight sensor check.
- IMU calibration khi boot nếu đủ cảm biến.
- Arming interlock: RC link, sensor health, calibration, fusion OK, tilt OK.
- Optical-flow arming check khi PosHold được bật.
- RC loss failsafe landing.
- Critical disarm khi lỗi IMU/fusion/calibration hoặc tilt quá lớn.

---

## Phần cứng mục tiêu

| Thành phần | Giá trị hiện tại |
|---|---|
| MCU | STM32F411C(C-E)Ux / STM32F411CEU6 Blackpill |
| Package | UFQFPN48 |
| HSE | 25 MHz |
| SYSCLK | 96 MHz |
| HAL | STM32F4 HAL |
| DSP | CMSIS DSP / `arm_math.h` |
| Timebase chính | TIM2 1 us |
| PWM motor | TIM3 + TIM4, 400 Hz, pulse 1000-2000 us |
| Project config | `FC_F411.ioc` |

---

## Sơ đồ ngoại vi và pinout

### UART

| UART | Pin | Chức năng | Baud trong source |
|---|---|---|---|
| USART1 TX/RX | PA9 / PA10 | Telemetry + PID tuning | 115200 8N1 |
| USART2 TX/RX | PA2 / PA3 | RC iBUS RX | 115200 8N1 |
| USART6 TX/RX | PA11 / PA12 | Optical flow RX | 115200 8N1 |

> Lưu ý: nếu regenerate code từ CubeMX, hãy kiểm tra lại baudrate trong `.ioc` và `Core/Src/platform/usart.c` để tránh lệch cấu hình.

### SPI1 - IMU ICM20602

| Pin | Chức năng |
|---|---|
| PA4 | IMU CS |
| PA5 | SPI1 SCK |
| PA6 | SPI1 MISO |
| PA7 | SPI1 MOSI |

SPI1 đang cấu hình master mode, CPOL high, CPHA second edge, baud prescaler 8.

### I2C

| Bus | Pin | Chức năng |
|---|---|---|
| I2C1 | PB8 / PB9 | Magnetometer, barometer |
| I2C2 | PB10 / PB3 | Barometer fallback / extra I2C bus |

I2C driver có bus recovery: đổi pin sang open-drain GPIO, tạo clock unstuck, generate STOP và re-init peripheral.

### PWM motor output

Timer motor chạy với prescaler `96-1`, period `2500-1`, tương đương timer tick 1 MHz và PWM khoảng 400 Hz.

| Pin | Timer channel | Source trong `Control_Motor()` | Gợi ý motor |
|---|---|---|---|
| PB4 | TIM3 CH1 | `PWM_MOTOR[1]` | M1 front-right |
| PB5 | TIM3 CH2 | `PWM_MOTOR[0]` | M2 back-right |
| PB6 | TIM4 CH1 | `PWM_MOTOR[3]` | M3 back-left |
| PB7 | TIM4 CH2 | `PWM_MOTOR[2]` | M4 front-left |

Luôn xác minh thứ tự motor thực tế bằng bench test trước khi gắn propeller.

### LED / buzzer

| Pin | Vai trò |
|---|---|
| PC13 | LED heartbeat / startup indication |
| PB13 | Buzzer |
| PB12 | GPIO output dự phòng |

---

## Cấu trúc source code

```text
Core/
├── Inc/
│   ├── comm/          # Optical packet, direct parser, telemetry, PID tuning
│   ├── control/       # Flight controller và PID definitions
│   ├── input/         # RC receiver iBUS
│   ├── platform/      # HAL peripheral init, delay, I2C recovery, system check
│   └── sensor/        # IMU, mag, baro, GPS, filters, calibration
│
├── Src/
│   ├── comm/
│   ├── control/
│   ├── input/
│   ├── platform/
│   └── sensor/
│
├── Startup/
│   └── startup_stm32f411ceux.s
│
├── FC_F411.ioc
└── README.md
```

### Module chính

| Module | Vai trò |
|---|---|
| `Core/Src/main.c` | Khởi tạo hệ thống, scheduler 1 kHz, background task, UART callbacks |
| `control/flight_control.c` | Arming, flight modes, failsafe, PID cascade, motor mix, optical hold |
| `control/pid.c` | PID position/rate, derivative filter, clamp output/integral |
| `input/rc_input.c` | iBUS parser, DMA ring buffer, link timeout, switch debounce |
| `comm/optical_direct.c` | MSP v2 và MiCoLink parser trên USART6 |
| `comm/optical_input.c` | Optical payload abstraction, cập nhật velocity vào flight controller |
| `comm/pid_tuning.c` | UART command parser cho PID/MAGCAL/OFLOW |
| `comm/telemetry.c` | JSON telemetry stream qua USART1 DMA |
| `sensor/imu_config.c` | ICM20602 SPI driver, calibration, raw-to-physical conversion |
| `sensor/magnetometer_sensor.c` | HMC5883L/QMC5883L driver và recovery |
| `sensor/bmp280_sensor.c` | BMP280/BME280 driver, compensated pressure/temp |
| `sensor/complementary_filter.c` | Attitude prediction/update |
| `sensor/mahony.c` | Mahony AHRS implementation |
| `platform/system_check.c` | Startup probe, health aggregation, heartbeat, buzzer |
| `platform/i2c.c` | I2C init + bus recovery wrappers |

---

## Luồng khởi động firmware

`main()` thực hiện các bước chính:

1. `HAL_Init()` và `SystemClock_Config()`.
2. Init GPIO, DMA, TIM2/TIM3/TIM4, SPI1, USART2, USART1, USART6, I2C1, I2C2.
3. Start motor PWM ở mức idle 1000 us để ESC ổn định.
4. Blink LED startup.
5. `SystemCheck_Init()`:
   - Chờ nguồn sensor ổn định.
   - Recover I2C bus.
   - Probe magnetometer/barometer/IMU.
   - Calibrate barometer ground pressure nếu baro sẵn sàng.
   - Calibrate IMU nếu đủ sensor.
6. `RcReceiver_Init()` cho USART2 DMA iBUS.
7. Start TIM2 làm microsecond timebase.
8. `Magnetometer_Init()`.
9. Reset complementary filter và toàn bộ PID.
10. `PidTuning_Init()` cho USART1 command RX.
11. `OpticalDirect_Init()` cho USART6 optical RX.
12. Đặt trạng thái an toàn ban đầu: motor disabled, disarmed, throttle 1000 us.

---

## Main loop thời gian thực

Firmware chạy vòng điều khiển chính mỗi 1000 us.

```text
1 kHz control loop
├── Drain RC DMA ring buffer
├── Update RC link status
├── Drain optical UART6 DMA parser
├── Read IMU
├── Complementary filter predict
├── Magnetometer update theo divider
├── Flight control MPC/PID/motor mix
├── Barometer background update
├── Sensor service/recovery
├── System health update
├── PID tuning command parser
├── Measure loop execution time
└── Telemetry 10 Hz
```

Các tần số quan trọng:

| Tác vụ | Tần số |
|---|---:|
| Main control loop | 1 kHz |
| Telemetry | 10 Hz |
| Barometer process | 10 Hz |
| Magnetometer correction | khoảng 25 Hz theo `MAG_UPDATE_DIV = 40` |
| RC iBUS | theo frame receiver, parser được drain trong loop và idle slack |
| Optical UART6 | drain trong loop và idle slack |

Loop có diagnostic variables:

| Biến | Ý nghĩa |
|---|---|
| `loop_dt_us` | Khoảng thời gian giữa hai vòng loop |
| `loop_dt_max_us` | Max dt trong cửa sổ debug |
| `loop_exec_us` | Thời gian thực thi loop, không tính telemetry formatting |
| `loop_exec_max_us` | Max execution time |
| `loop_overrun_count` | Số lần loop vượt tolerance |

---

## Flight modes

### HOVER / Angle mode

Điều kiện chọn:

```c
MPC_Status = HOVER;
```

Hoặc RC mode switch thấp hơn/equal 1500 us.

Đặc điểm:

- Stick roll/pitch tạo angle target.
- Stick yaw tạo yaw angle step.
- Attitude PID tạo rate target.
- Rate PID tạo moment.
- Motor mixer phân phối moment vào 4 motor.

### RATE_MODE / Acro mode

Điều kiện chọn:

```c
RC_Raw_SW_Mode > 1500U
```

Đặc điểm:

- Stick roll/pitch/yaw tạo rate target trực tiếp.
- Không tự cân bằng về góc 0.
- Dùng rate PID trực tiếp trên gyro feedback.

### Position Hold

Điều kiện chọn:

```c
RC_Raw_SW_PosHold > 1500U
```

Chỉ active khi:

- Optical flow healthy.
- Stick roll/pitch gần center để capture hold position.
- PosHold switch đang bật.
- Data age chưa stale.

---

## RC receiver iBUS

`input/rc_input.c` đọc iBUS frame qua USART2 DMA circular buffer.

Thông số chính:

| Tham số | Giá trị |
|---|---:|
| DMA buffer | 512 bytes |
| iBUS frame size | 32 bytes |
| Header | `0x20 0x40` |
| Checksum | `0xFFFF - sum(frame[0..29])` |
| Link timeout | 500 ms |
| Channel valid range | 900-2100 us |
| Switch low/high threshold | 1300 / 1700 us |
| Switch debounce | 5 valid frames |
| RC smoothing | PT1 15 Hz |

Channel map hiện tại:

| iBUS channel | Dữ liệu | Ghi chú |
|---|---|---|
| CH1 | Roll | bytes 2-3 |
| CH2 | Pitch | inverted bằng `3000 - raw` |
| CH3 | Throttle | bytes 6-7 |
| CH4 | Yaw | bytes 8-9 |
| CH5 | Arm switch | bytes 10-11 |
| CH6 | Mode switch | bytes 12-13 |
| CH8 | Position Hold switch | bytes 16-17 |

Khi mất link, firmware set:

```text
Roll      = 1500
Pitch     = 1500
Yaw       = 1500
Throttle  = 1000
Arm       = 1000
Mode      = 1000
PosHold   = 1000
```

---

## Optical Flow và Position Hold

### Các protocol optical đang hỗ trợ

Firmware có 2 đường optical:

1. Custom optical packet parser trong `comm/optical_packet.h`.
2. Direct parser trong `comm/optical_direct.c` cho MSP v2 và MiCoLink.

### Custom optical packet format

Định nghĩa trong `Core/Inc/comm/optical_packet.h`:

| Field | Size | Giá trị |
|---|---:|---|
| Sync 1 | 1 | `0xAA` |
| Sync 2 | 1 | `0x55` |
| Version | 1 | `1` |
| Type | 1 | `0x10` |
| Payload length | 1 | `20` |
| Sequence | 1 | incrementing seq |
| Payload | 20 | `optical_payload_t` |
| CRC | 1 | CRC8 DVB-S2 |

Tổng frame length: 27 bytes.

Payload:

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint16_t flags;
    int32_t distance_mm;
    int32_t flow_x_raw;
    int32_t flow_y_raw;
    uint8_t range_quality;
    uint8_t flow_quality;
} optical_payload_t;
```

Flags:

| Flag | Ý nghĩa |
|---|---|
| `OPTICAL_FLAG_DISTANCE_VALID` | distance hợp lệ |
| `OPTICAL_FLAG_FLOW_VALID` | flow hợp lệ |
| `OPTICAL_FLAG_RANGE_QUALITY_VALID` | range quality có giá trị |
| `OPTICAL_FLAG_FLOW_QUALITY_VALID` | flow quality có giá trị |
| `OPTICAL_FLAG_DISTANCE_OUT_RANGE` | range sensor báo ngoài tầm |
| `OPTICAL_FLAG_PROTOCOL_MSP` | payload đến từ MSP |
| `OPTICAL_FLAG_PROTOCOL_MICOLINK` | payload đến từ MiCoLink |

### MSP v2 direct parser

MSP v2 parser nhận các command:

| Command | ID | Ý nghĩa |
|---|---:|---|
| `MSP2_SENSOR_RANGEFINDER` | `0x1F01` | distance + range quality |
| `MSP2_SENSOR_OPTIC_FLOW` | `0x1F02` | flow x/y + flow quality |

CRC dùng CRC8 DVB-S2 polynomial `0xD5`.

### MiCoLink direct parser

MiCoLink parser nhận:

| Field | Giá trị |
|---|---|
| Header | `0xEF` |
| Message ID | `0x51` |
| Payload length tối thiểu | 20 bytes |
| Checksum | Sum 8-bit toàn frame trừ checksum |

Dữ liệu được convert sang `optical_payload_t` rồi gọi:

```c
OpticalInput_ApplyPayload(seq, &payload);
```

### Position Hold pipeline

```text
Optical payload
    ↓
OpticalInput_ApplyPayload()
    ↓
FlightController_UpdateOpticalFlowVelocity()
    ↓
Body velocity LPF
    ↓
Yaw transform body frame → earth frame
    ↓
Integrate earth velocity → relative position
    ↓
Position PID → velocity target
    ↓
Velocity PID → tilt correction
    ↓
Add correction to roll/pitch angle target
```

Giới hạn optical flow hiện tại:

| Tham số | Giá trị |
|---|---:|
| Stale timeout | 0.20 s |
| Min normalized quality | 0.10 |
| Min altitude | 0.01 m |
| Max altitude | 10.0 m |
| Max body velocity | 4.0 m/s |
| Max position error | 1.5 m |
| Velocity LPF gain | 0.35 |

---

## PID tuning runtime

`comm/pid_tuning.c` nhận command ASCII qua USART1 RX DMA. Mỗi command kết thúc bằng `\n`.

### Attitude/rate PID

```text
PID:RR:kp:ki:kd    # Rate Roll
PID:RP:kp:ki:kd    # Rate Pitch
PID:RY:kp:ki:kd    # Rate Yaw
PID:AR:kp:ki:kd    # Angle Roll
PID:AP:kp:ki:kd    # Angle Pitch
PID:AY:kp:ki:kd    # Angle Yaw
```

Ví dụ:

```text
PID:RR:0.70:1.26:0.11
PID:AP:1.70:0.007:1.80
```

### Optical-flow PID

```text
PID:VX:kp:ki:kd    # Velocity hold X
PID:VY:kp:ki:kd    # Velocity hold Y
PID:PX:kp:ki:kd    # Position hold X
PID:PY:kp:ki:kd    # Position hold Y
```

Ví dụ:

```text
PID:VX:5.20:1.15:0.04
PID:PX:1.10:0.08:0.00
```

### Optical-flow injection test

```text
OFLOW:vx:vy:alt:quality:dt
```

Ví dụ:

```text
OFLOW:0.10:-0.05:1.20:200:0.02
```

### Reset optical hold

```text
OFRESET
```

### Magnetometer calibration

```text
MAGCAL
```

Command này chuyển `MagCal.state` sang `MAG_CAL_START` và đặt `samples_target = 1000`.

---

## Telemetry

Telemetry được gửi qua USART1 bằng DMA TX ở 10 Hz. Format là JSON một dòng.

Ví dụ rút gọn:

```json
{
  "ang": [roll, pitch, yaw],
  "angT": [roll_target, pitch_target, yaw_target],
  "rate": [p, q, r],
  "rateT": [p_target, q_target, r_target],
  "flow": [vx_body, vy_body],
  "rc": [roll, pitch, yaw, throttle],
  "mot": [m0, m1, m2, m3],
  "sys": [arm, optical_block_reason, mode, rc_link, failsafe, fault_mask, optical_healthy, magcal_state, magcal_samples],
  "mag": [offset_x, offset_y, offset_z, scale_x, scale_y, scale_z],
  "v": 11.1,
  "hz": 1000,
  "exec": 320
}
```

Ý nghĩa `sys`:

| Index | Ý nghĩa |
|---:|---|
| 0 | `armStatus`: 1 nếu armed |
| 1 | `flight_optical_arm_block_reason` |
| 2 | flight mode: 0 rate, 1 hover, 2 poshold switch on |
| 3 | `rc_link_ok` |
| 4 | `flight_failsafe_state` |
| 5 | `system_sensor_fault_mask` |
| 6 | optical flow healthy |
| 7 | `MagCal.state` |
| 8 | `MagCal.samples` |

> `vbat` hiện đang hard-code `11.1f`. Cần nối ADC thật nếu muốn battery telemetry chính xác.

---

## Sensor system

### IMU - ICM20602

File: `sensor/imu_config.c`

- Giao tiếp SPI1.
- WHO_AM_I expected: `0x12`.
- Burst read 14 bytes từ accel/gyro registers.
- Gyro sensitivity: `131.0f`.
- Accel sensitivity: `16384.0f`.
- Calibration samples: 1000.
- Có sign mapping riêng cho trục drone.

### Magnetometer - HMC5883L/QMC5883L

File: `sensor/magnetometer_sensor.c`

- HMC5883L address: `0x1E` 7-bit.
- QMC5883L address: `0x0D` 7-bit.
- Tự detect HMC/QMC.
- Hỗ trợ recovery khi đọc lỗi liên tiếp.
- QMC cấu hình continuous 200 Hz, 8G.

### Barometer - BMP280/BME280

File: `sensor/bmp280_sensor.c`

- Address supported: `0x76` hoặc `0x77`.
- BMP280 chip ID: `0x58`.
- BME280 chip ID: `0x60`.
- Có service interval và recovery cooldown.
- `BARO_PROCESS()` cập nhật pressure, temperature và altitude tương đối sau ground calibration.

### Complementary filter

File: `sensor/complementary_filter.c`

- Predict bằng gyro integration.
- Update yaw/heading bằng magnetometer.
- Xuất Euler angle degree/radian và quaternion state.

### Mahony AHRS

File: `sensor/mahony.c`

- Có reset, set gains và update với IMU + optional magnetometer.
- Có stationary trim logic.
- Hiện tại source đã có module, nhưng flow chính trong `main.c` đang dùng complementary filter.

### GPS và Kalman

Files:

- `sensor/gps.c`
- `sensor/gps_kalman.c`

GPS parser hỗ trợ `$GPGGA` và `$GNGGA`. Kalman 1D hỗ trợ position/velocity estimate theo từng trục. Trong main loop hiện tại, USART2 đang dùng cho RC iBUS, nên nếu muốn dùng GPS đồng thời cần remap UART hoặc chuyển RC sang bus khác.

---

## Failsafe và arming logic

### Điều kiện arm

Arm chỉ được cho phép khi:

- RC link OK.
- Throttle thấp hơn 1150 us.
- Arm switch > 1500 us.
- Sensor runtime OK.
- Preflight ready.
- IMU đã calibrate.
- Fusion OK.
- Roll/pitch hiện tại nhỏ hơn `ARM_MAX_TILT_DEG = 25°`.
- Nếu PosHold switch bật, optical flow phải đạt arming condition.

### Optical arming check

Khi `ARM_REQUIRED_OPTICAL_FLOW = 1` và PosHold được bật, firmware kiểm tra:

| Điều kiện | Giá trị |
|---|---:|
| Có optical packet hợp lệ | bắt buộc |
| Optical frame không stale | age <= 300 ms |
| Flow valid | bắt buộc |
| Distance không out-of-range | bắt buộc |
| Distance range | 80-6000 mm |
| Flow quality min | 25 nếu field valid |
| Range quality min | 20 nếu field valid |

`flight_optical_arm_block_reason`:

| Code | Ý nghĩa |
|---:|---|
| 0 | OK |
| 1 | Chưa có optical packet |
| 2 | Chưa có optical timestamp |
| 3 | Optical frame stale |
| 4 | Flow invalid |
| 5 | Range out-of-range |
| 6 | Distance ngoài khoảng arm |
| 7 | Flow quality thấp |
| 8 | Range quality thấp |

### Failsafe states

```c
typedef enum {
    FLIGHT_FAILSAFE_INACTIVE = 0,
    FLIGHT_FAILSAFE_LANDING,
    FLIGHT_FAILSAFE_CRITICAL,
    FLIGHT_FAILSAFE_LANDED
} FlightFailsafeState_t;
```

### RC loss / interlock failsafe

Khi đang armed mà mất RC hoặc interlock không còn hợp lệ, firmware vào landing failsafe:

- Giữ attitude target về level.
- Dùng throttle tại thời điểm vào failsafe, clamp tối thiểu.
- Giảm throttle dần theo `FAILSAFE_LAND_DESCENT_US_PER_S = 80 us/s`.
- Disarm khi đã giữ min throttle đủ thời gian hoặc vượt max landing time.

Thông số:

| Tham số | Giá trị |
|---|---:|
| Min landing throttle | 1120 us |
| Descent rate | 80 us/s |
| Min hold time at min throttle | 2 s |
| Max landing time | 12 s |

### Critical failsafe

Critical disarm xảy ra khi:

- IMU fault.
- Fusion fault.
- Calibration fault.
- Roll/pitch vượt `CRITICAL_TILT_DISARM_DEG = 70°`.

---

## Build và flash

### Yêu cầu

- STM32CubeIDE.
- ST-Link hoặc probe tương thích.
- STM32CubeF4 HAL package.
- CMSIS DSP được enable để dùng `arm_math.h`.

### Cách build bằng STM32CubeIDE

1. Clone hoặc mở project source.
2. Mở `FC_F411.ioc` hoặc import project vào STM32CubeIDE.
3. Kiểm tra lại clock 96 MHz và peripheral mapping.
4. Build project.
5. Flash firmware vào STM32F411 bằng ST-Link.
6. Mở serial monitor USART1 ở 115200 8N1 để xem telemetry và gửi command PID tuning.

### Lưu ý khi regenerate code từ CubeMX

- Chỉ sửa code trong vùng `USER CODE BEGIN/END`.
- Sau khi regenerate, kiểm tra lại:
  - USART baudrate.
  - DMA circular mode cho USART1 RX, USART2 RX, USART6 RX.
  - TIM3/TIM4 PWM output pins.
  - Include path cho `Core/Inc` và CMSIS DSP.

---

## Checklist test an toàn

### Trước khi cấp nguồn motor

- [ ] Board flash thành công.
- [ ] PC13 blink/heartbeat hoạt động.
- [ ] IMU WHO_AM_I đúng `0x12`.
- [ ] Magnetometer được detect HMC/QMC.
- [ ] Barometer detect BMP/BME.
- [ ] RC iBUS frame count tăng.
- [ ] `rc_link_ok = 1` khi bật transmitter.
- [ ] Throttle thấp, arm switch low khi boot.
- [ ] Telemetry gửi ổn định 10 Hz.

### Test không gắn propeller

- [ ] Motor output ở 1000 us khi disarmed.
- [ ] Arm chỉ được phép khi throttle thấp.
- [ ] Đổi Hover/Rate mode reset PID.
- [ ] Roll/pitch/yaw response đúng chiều.
- [ ] Motor order đúng với frame.
- [ ] RC loss làm throttle về failsafe/disarm đúng kỳ vọng.

### Test optical flow

- [ ] `optical_rx_packet_valid = 1`.
- [ ] `optical_rx_frame_count` tăng đều.
- [ ] `optical_rx_flow_valid = 1`.
- [ ] Distance nằm trong 80-6000 mm nếu muốn arm PosHold.
- [ ] Flow/range quality đủ lớn.
- [ ] `optical_flow_state.healthy = 1` khi data hợp lệ.
- [ ] PosHold chỉ active khi stick gần center.

---

## Debug nhanh

### RC không nhận

Kiểm tra:

- USART2 RX pin PA3.
- Baudrate 115200.
- iBUS receiver output đúng serial iBUS, không phải PWM/SBUS.
- `rc_byte_count`, `rc_frame_sync_count`, `rc_checksum_error_count`.
- DMA1 Stream5 interrupt/config.

### Không arm được

Kiểm tra các biến:

```text
fc_preflight_ready
runtime_sensors_ok
rc_link_ok
is_calibrated
Complimentary_Filter.Fusion_OK
flight_optical_arm_ok
flight_optical_arm_block_reason
system_sensor_fault_mask
RC_Raw_Throttle
RC_Raw_SW_Arm
```

### Optical flow không chạy

Kiểm tra:

```text
optical_uart6_dma_running
optical_uart6_byte_count
optical_uart6_msp_frame_count
optical_uart6_micolink_frame_count
optical_uart6_msp_crc_error_count
optical_uart6_micolink_checksum_error_count
optical_rx_frame_count
optical_rx_recent_bytes
```

Nếu byte count tăng nhưng frame count không tăng, khả năng cao sai protocol, sai baudrate hoặc checksum/CRC.

### I2C sensor lỗi

Kiểm tra:

```text
i2c1_recovery_count
i2c2_recovery_count
i2c1_recovery_fail_count
i2c2_recovery_fail_count
i2c_sensor_ready[]
mag_sensor_type
baro_ready
baro_chip_id
```

### Loop bị overrun

Kiểm tra:

```text
loop_exec_us
loop_exec_max_us
loop_overrun_count
optical_uart6_pending_bytes
optical_uart6_process_limit_count
rc_max_pending_bytes
```

Telemetry dùng `snprintf`, nên hiện đã được đặt sau khi đo `loop_exec_us` để không làm sai timing diagnostic chính.

---

## Giới hạn hiện tại

- `vbat` trong telemetry đang hard-code `11.1f`, chưa đọc ADC thật.
- GPS module có sẵn nhưng USART2 hiện dùng cho RC iBUS trong main flow.
- Position Hold dựa trên optical flow integration tương đối, sẽ drift nếu flow scale/quality không ổn định.
- Optical flow scale trong `OpticalInput_ApplyPayload()` đang dùng hệ số đơn giản cho raw flow, cần calibrate theo sensor thực tế.
- Không có persistent storage cho PID/calibration; reboot sẽ quay về giá trị compile-time.
- Chưa có blackbox logging, MAVLink/MSP telemetry đầy đủ, DShot hay mixer config runtime.

---

## Roadmap đề xuất

- Lưu PID và calibration vào flash.
- Thêm ADC battery voltage/current sensing.
- Thêm blackbox logging qua UART/SPI flash.
- Hỗ trợ DShot ESC protocol.
- Thêm altitude hold bằng barometer + optical rangefinder.
- Tích hợp EKF nhẹ cho attitude/velocity/position.
- Tách GPS sang UART riêng và thêm GPS position hold.
- Thêm CLI cấu hình mixer/motor order.
- Thêm unit test cho packet parser, PID và iBUS checksum.
- Chuẩn hóa telemetry schema và tool ground station.

---

## License


---

## Credits

Developed by `tranthinhembedded`.

Firmware được tổ chức theo hướng modular flight stack cho STM32F411, phù hợp cho nghiên cứu embedded control, quadcopter stabilization và thử nghiệm optical-flow position hold.
