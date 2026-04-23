# STM32 Flight Controller (FC_F411)

A modular, high-performance Flight Controller firmware for STM32F411 microcontrollers, designed with a clean architecture and inspired by professional flight stacks like iNav and Betaflight.

---

## 🚀 Overview

This project is a custom Flight Controller implementation for quadcopters. It features a modular architecture that separates hardware abstraction, sensor processing, and control logic, allowing for easy maintenance and expansion.

### Key Features
- **1 kHz Main Loop**: Highly responsive control loop running at 1ms intervals.
- **Cascaded PID Control**: Dual-loop PID system for stable Attitude (Angle) and Rate (Acro) control.
- **Sensor Fusion**: Uses Complementary Filter and Mahony filter for accurate attitude estimation.
- **Modular Hardware Support**:
  - **IMU**: ICM20602 (SPI)
  - **Magnetometer**: HMC5883L/QMC5883L (I2C)
  - **Receiver**: iBUS (Flysky) via USART DMA
  - **Telemetry**: Custom telemetry and PID tuning protocol via USART.
- **Advanced Features**: Integrated support for Optical Flow position hold and Barometer altitude sensing.

---

## 🛠 Hardware Architecture

- **Microcontroller**: STM32F411CEU6 (Blackpill)
- **Timebase**: `TIM2` (1 µs resolution)
- **Actuators**: 4x PWM outputs for Brushless ESCs (TIM3 & TIM4)
- **Communication**:
  - `USART2`: RC Input (iBUS)
  - `USART1`: Telemetry & PID Tuning
  - `SPI1`: High-speed IMU data
  - `I2C1`: Magnetometer / Barometer

---

## 📂 Project Structure

```text
Core/Src/
├── control/        # PID controllers and flight mode logic
├── sensor/         # Drivers and fusion algorithms (IMU, Mag, GPS)
├── input/          # RC receiver handling (iBUS)
├── comm/           # Telemetry and tuning protocols
└── platform/       # HAL and peripheral initializations
```

---

## ⚙️ Control Modes

1. **HOVER (Angle Mode)**: Self-leveling mode using cascaded PID. Ideal for beginners or stable filming.
2. **RATE (Acro Mode)**: Direct control of rotation rates. Used for racing and freestyle maneuvers.
3. **Position Hold**: (Experimental) Uses Optical Flow data to maintain horizontal position.

---

## 🔧 Setup and Build

This project is developed using **STM32CubeIDE** and **VS Code**.

1. Clone the repository.
2. Open in STM32CubeIDE or use the provided `.vscode` configurations.
3. Build the project to generate the `.elf` or `.hex` file.
4. Flash to your STM32F411 board using ST-Link.

---

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

*Developed by [tranthinhembedded](https://github.com/tranthinhembedded)*
