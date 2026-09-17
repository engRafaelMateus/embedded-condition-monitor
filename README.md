# Embedded Condition Monitor

Embedded monitoring and telemetry platform developed with **ESP32, ESP-IDF, FreeRTOS and C**, focused on embedded systems, firmware architecture and industrial communication.

The project implements an embedded monitoring node capable of acquiring sensor data, processing measurements, classifying equipment operating conditions, generating local alarms and transmitting telemetry through multiple communication interfaces.

The firmware is developed using native **ESP-IDF APIs** and a multitask architecture based on **FreeRTOS**.

---

## Overview

The system currently integrates:

* ESP32
* ESP-IDF
* FreeRTOS
* MPU6050 via I2C
* ADC Oneshot
* GPIO interrupts
* PWM / LEDC
* UART
* CAN / TWAI
* Modbus RTU
* FreeRTOS Queues
* Task Notifications
* Wokwi
* Git / GitHub
* CMake

---

## System Architecture

```text
                 +----------------+
                 |    MPU6050     |
                 | Temp + Accel   |
                 +-------+--------+
                         |
                        I2C
                         |
                         v
                  +--------------+
                  | taskSensores |
                  +------+-------+
                         |
                    Sensor Queue
                         |
                         v
                  +--------------+
                  | taskControle |
                  +------+-------+
                         |
          +--------------+--------------+
          |              |              |
          v              v              v
     Status LEDs     PWM Buzzer    Telemetry Queue
                                         |
                                         v
                                  +---------------+
                                  | taskTelemetria|
                                  +-------+-------+
                                          |
                              +-----------+-----------+
                              |                       |
                              v                       v
                         CAN / TWAI             UART Telemetry
```

Operator alarm acknowledgement is handled independently:

```text
Push Button
    |
GPIO Interrupt
    |
    v
   ISR
    |
Task Notification
    |
    v
taskBotao
    |
Task Notification
    |
    v
taskControle
```

Modbus RTU uses an independent communication path:

```text
Modbus Master
    |
   UART1
    |
    v
Modbus RTU
    |
   UART2
    |
    v
Modbus Slave
    |
    v
Input Registers
```

---

## Implemented Features

### Sensor Acquisition

The firmware performs digital and analog data acquisition using native ESP-IDF drivers.

#### MPU6050

Communication with the MPU6050 is implemented using the ESP32 I2C master driver.

Acquired data:

* Temperature
* X-axis acceleration
* Y-axis acceleration
* Z-axis acceleration

Raw sensor values are converted into engineering values used by the control and telemetry layers.

#### ADC

Analog acquisition is implemented using the **ESP-IDF ADC Oneshot driver**.

The acquisition routine:

* Performs multiple ADC samples
* Accepts only valid readings
* Calculates the average of 10 valid samples
* Discards the acquisition cycle if enough valid samples cannot be obtained

This reduces the influence of isolated acquisition errors before the data reaches the control layer.

---

### FreeRTOS Architecture

The firmware uses multiple FreeRTOS tasks with separated responsibilities.

#### `taskSensores`

Responsible for:

* MPU6050 acquisition
* Temperature processing
* Acceleration processing
* ADC acquisition
* ADC averaging
* Sensor data validation

The resulting measurements are sent to the control layer through a **FreeRTOS Queue**.

#### `taskControle`

Responsible for:

* Receiving sensor measurements
* Evaluating equipment condition
* Updating status LEDs
* Managing the audible alarm
* Processing operator alarm acknowledgement
* Building the telemetry structure
* Sending telemetry to the communication layer

#### `taskTelemetria`

Responsible for receiving structured telemetry and forwarding data to the configured communication interfaces.

#### `taskBotao`

Responsible for processing operator alarm acknowledgement.

The GPIO ISR only generates an event, keeping interrupt execution short.

Debounce and event processing are performed outside interrupt context.

---

### FreeRTOS Communication

Structured data is transported between tasks using **FreeRTOS Queues**.

```text
taskSensores
     |
 sensor_data_t
     |
     v
fila_sensores
     |
     v
taskControle
     |
telemetry_data_t
     |
     v
fila_telemetria
     |
     v
taskTelemetria
```

Lightweight events are handled using **Task Notifications**.

```text
GPIO ISR
   |
   v
taskBotao
   |
   v
taskControle
```

---

### Equipment State Machine

The equipment operating condition is classified into three states:

```text
NORMAL
WARNING
CRITICAL
```

Current thresholds:

| Parameter   | NORMAL  | WARNING    | CRITICAL |
| ----------- | ------- | ---------- | -------- |
| Temperature | < 45 °C | 45–59.9 °C | >= 60 °C |
| ADC value   | < 2500  | 2500–3299  | >= 3300  |

A CRITICAL condition has priority over WARNING and NORMAL.

The state is evaluated from the latest valid sensor acquisition cycle.

---

### Local Alarm System

The monitoring node provides local visual and audible indication.

Status indication:

* Green LED: NORMAL
* Yellow LED: WARNING
* Red LED: CRITICAL

The buzzer is controlled through the ESP32 **LEDC PWM peripheral**.

When the system enters CRITICAL state, the audible alarm is activated.

The operator can acknowledge the alarm using a push button. Alarm acknowledgement silences the audible indication without changing the actual equipment condition.

When the system leaves CRITICAL state, the acknowledgement state is automatically reset.

---

### GPIO Interrupt Handling

The acknowledgement button uses GPIO interrupt processing.

The implementation includes:

* Positive-edge GPIO interrupt
* Short ISR execution
* `vTaskNotifyGiveFromISR()`
* `portYIELD_FROM_ISR()`
* Event handling outside the ISR
* Software debounce
* Task-to-task event forwarding

This keeps time-consuming processing outside interrupt context.

---

## Telemetry Architecture

Telemetry is represented internally using structured C data.

```c
typedef struct
{
    sensor_data_t dados;
    system_state_t estado;
    bool alarme_reconhecido;
} telemetry_data_t;
```

The telemetry structure contains:

* Temperature
* Three-axis acceleration
* ADC measurement
* Equipment operating state
* Alarm acknowledgement status

Application data is kept separate from protocol-specific encoding.

This allows the telemetry layer to support different communication interfaces without coupling sensor acquisition directly to the transport protocol.

---

## CAN / TWAI Communication

The project implements a CAN telemetry protocol using the ESP32 **TWAI controller**.

Telemetry is divided into dedicated CAN frames.

| CAN ID  | Data                             |
| ------- | -------------------------------- |
| `0x100` | Operating state and alarm status |
| `0x200` | Temperature                      |
| `0x201` | ADC measurement                  |
| `0x202` | X/Y/Z acceleration               |

Standard 11-bit CAN identifiers are used.

### CAN Data Encoding

Floating-point measurements are converted into fixed-point integer representations before transmission.

Temperature:

```text
temperature × 100
```

Example:

```text
24.00 °C
→ 2400
→ 0x0960
```

Acceleration:

```text
acceleration × 1000
```

Example:

```text
-0.250 g
→ -250
→ 0xFF06
```

Multi-byte values are serialized with the most significant byte first.

The receiver reconstructs the original signed value and restores the engineering scale.

### CAN Frame Validation

The receive-side parser validates:

* CAN identifier
* Standard/extended frame format
* RTR configuration
* Data Length Code
* Acquisition status
* Operating state range
* Alarm acknowledgement range

Unexpected frames are rejected before their payload is interpreted.

### CAN Execution Modes

The firmware supports two CAN execution paths.

#### Protocol Test Mode

```c
CAN_MODO_TESTE = 1
```

Frames are encoded, printed and processed locally.

This validates:

* Message layout
* Payload encoding
* Signed values
* Scaling
* Frame interpretation

#### TWAI Driver Mode

```c
CAN_MODO_TESTE = 0
```

The ESP32 TWAI driver is configured for:

```text
500 kbit/s
```

Transmission uses:

```c
twai_transmit()
```

Reception is handled by a dedicated task using:

```c
twai_receive()
```

Received frames are forwarded to the same CAN protocol parser used by the protocol test mode.

---

## UART Communication

The firmware uses multiple ESP32 UART controllers.

UART communication is separated from sensor acquisition and control logic.

UART1 can operate as the direct telemetry interface or as the Modbus Master interface when the Modbus loopback configuration is enabled.

UART2 is used by the Modbus RTU Slave.

---

## Modbus RTU

Modbus RTU communication is implemented using the **ESP-Modbus** component.

The firmware contains:

* Modbus RTU Slave
* Modbus RTU Master
* Input Register mapping
* Master parameter descriptor
* Modbus request/response processing

The Slave exposes an **Input Register** through the Modbus register area.

The Master performs a standard Modbus request using function code:

```text
0x04 — Read Input Registers
```

Communication parameters:

```text
Mode:       Modbus RTU
Baud rate:  115200
Data bits:  8
Parity:     None
Stop bits:  1
Slave ID:   1
```

### Modbus Architecture

```text
Modbus Master
   UART1
     |
     v
 Modbus RTU
     |
     v
   UART2
Modbus Slave
     |
     v
Input Register
```

The firmware includes a configurable loopback mode:

```c
MODBUS_LOOPBACK_TEST
```

When enabled, UART1 operates as the Modbus Master and UART2 operates as the Modbus Slave.

This configuration allows a complete Modbus request/response transaction to execute inside the project.

### ESP-Modbus Integration

The implementation uses ESP-Modbus controller APIs for:

* Serial Slave creation
* Serial Master creation
* Communication configuration
* Slave register descriptor configuration
* Master parameter descriptor configuration
* Slave startup
* Master startup
* Modbus request transmission
* Input Register reading

The implemented communication flow performs a Master request and receives the Input Register exposed by the Slave.

---

## Project Structure

The repository follows the standard ESP-IDF project structure.

```text
embedded-condition-monitor/
|
├── CMakeLists.txt
├── dependencies.lock
├── diagram.json
├── wokwi.toml
├── wokwi-project.txt
|
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml
    └── main.c
```

The project uses CMake through the native ESP-IDF build system.

External component dependencies are managed through the ESP-IDF Component Manager.

---

## Implementation Status

* [x] ESP32 firmware using native ESP-IDF APIs
* [x] MPU6050 integration via I2C
* [x] ADC acquisition and sample averaging
* [x] FreeRTOS multitask architecture
* [x] Queue-based sensor and telemetry pipeline
* [x] GPIO interrupts and Task Notifications
* [x] NORMAL / WARNING / CRITICAL state machine
* [x] LED and PWM buzzer alarm system
* [x] Structured telemetry architecture
* [x] UART communication
* [x] CAN/TWAI telemetry protocol
* [x] CAN frame encoding, decoding and validation
* [x] Modbus RTU Master and Slave
* [x] ESP-Modbus integration and register communication

---

## Development Approach

The project is developed incrementally using native ESP-IDF APIs, with emphasis on real-time architecture, clear separation of responsibilities, structured communication, error handling and maintainable firmware design.

Each subsystem is implemented and validated before being integrated into the complete monitoring platform.

---

## Author

**Rafael Lopes Mateus**

Computer Engineer

Embedded Systems | Hardware | Firmware

Araçatuba, SP - Brazil
