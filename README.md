# ✈ ATABEY UAV Fixed-Wing Autopilot

Embedded fixed-wing flight control software developed for the Atabey UAV platform.

The project focuses on real-time flight stabilization, sensor fusion, actuator control, and autonomous flight infrastructure using STM32 microcontrollers.

The software architecture is designed to be modular, scalable, and hardware-oriented for embedded systems development and flight control experimentation.

---

## 🎯 Project Goals

- Real-time flight stabilization for fixed-wing UAVs
- Modular STM32-based autopilot architecture
- Low-level embedded systems development
- Sensor fusion and attitude estimation
- Deterministic scheduler-based execution
- Expandable autonomous flight infrastructure
- Educational and research-oriented development

---

## 🧠 Software Architecture

The autopilot software is divided into independent modules responsible for sensing, estimation, control, timing, and actuator management.

### Main Software Layers

- **IMU Layer**  
  MPU6050 communication and sensor abstraction

- **AHRS Layer**  
  Complementary filter based attitude estimation

- **RC Layer**  
  PWM receiver decoding and normalization

- **Control Layer**  
  Flight stabilization and control algorithms

- **Scheduler Layer**  
  Periodic task execution and timing management

- **Servo Layer**  
  PWM output generation for servos and ESCs

- **Utility Layer**  
  Common math and timing utilities

---

## 📁 Project Structure

```text
App/
│
├── ahrs/          # Attitude estimation and complementary filter
├── config/        # Global project configuration
├── control/       # Flight control algorithms
├── imu/           # MPU6050 driver and IMU interface
├── rc/            # RC receiver input processing
├── scheduler/     # Task scheduling infrastructure
├── servo/         # PWM actuator output
├── utils/         # Utility and helper functions
│
├── flight_app.c
└── flight_app.h
```

---

## ⚙ Current Features

### Implemented

- [x] Modular application structure
- [x] MPU6050 communication
- [x] IMU abstraction layer
- [x] Complementary filter prototype
- [x] PWM input capture
- [x] RC signal normalization
- [x] Servo PWM generation
- [x] PID stabilization loop

### In Progress

- [ ] Flight mode infrastructure
- [ ] Scheduler improvements
- [ ] Failsafe mechanisms

### Planned

- [ ] GPS integration
- [ ] Telemetry support
- [ ] Autonomous navigation
- [ ] Advanced sensor fusion
- [ ] Flight data logging

---

## 🔁 High-Level Execution Flow

1. MCU initialization
2. Peripheral configuration
3. Sensor startup sequence
4. Scheduler initialization
5. Periodic sensor acquisition
6. Attitude estimation update
7. Control loop execution
8. Servo/ESC output generation

---

## 🛠 Hardware Platform

### Current Hardware

- STM32F411CEU6 Microcontroller
- MPU6050 IMU
- BMM150 Magnetometer
- RC Receiver
- 2  Servos (Aileron)
- ESC Output (Thrust Motor)

---

## 🧪 Development Philosophy

This project prioritizes:

- Deterministic timing behavior
- Hardware abstraction
- Modular software architecture
- Readable and maintainable code
- Educational experimentation

---

## 📊 Future Work

Planned future improvements include:

- Kalman Filter for IMU
- GPS navigation
- Telemetry infrastructure
- Autonomous waypoint navigation
- Flight data recorder
- Ground control station integration

---

## 🛡 License

This project is developed for educational and research purposes.
