# PEARL — Physical Environment Aware Reasoning Layer

An AI-powered embedded hardware diagnostics system. An STM32 F401RE streams live sensor telemetry over UART to a Raspberry Pi 5, which feeds it through a hardware context engine and an LLM (Gemini 3.6 Flash) to provide real-time natural language fault diagnosis via a web dashboard.

Built as a 7th semester capstone project at VIT Vellore under Dr. Jacob Raglend I.

## Architecture

STM32 F401RE → UART → Raspberry Pi 5 → Context Engine (hardware.yaml) → LLM → Web Dashboard (WebSocket)

## Components

- firmware/ — Bare-metal STM32 firmware (PlatformIO). Reads MPU9250 (accel + magnetometer), ACS712 current sensor. Streams JSON telemetry at 5 Hz. Includes I2C bus auto-recovery.
- server/ — FastAPI + WebSocket server on the Pi. Parses telemetry, injects hardware context from hardware.yaml, queries Gemini, serves a live dashboard.
- docs/ — Wiring references and demo documentation.

## Demo Scenarios

1. Magnetic interference — magnet near MPU9250; magnetometer spikes while accel/gyro stay stable
2. I2C bus disconnect — pull SDA/SCL wire; firmware auto-detects and recovers
3. Physical disturbance — tap/shake the board; accelerometer spikes detected
4. Orientation change — tilt the board; gravity vector shift diagnosed
