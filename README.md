# Embedded Condition Monitor

Embedded platform for **equipment condition monitoring, diagnostics and telemetry**, developed with **ESP32, ESP-IDF, FreeRTOS and C**, with progressive integration of C++ for the embedded Linux gateway.

The project implements an embedded monitoring node capable of acquiring sensor data, classifying equipment operating conditions, generating local alarms and transmitting telemetry to external devices.

The architecture is being progressively expanded toward industrial communication using **CAN/TWAI, RS-485 and an embedded Linux gateway**.

---

## Project Goals

The main objective of this project is to develop a complete embedded system covering:

- Sensor acquisition
- Real-time multitasking architecture
- Equipment condition classification
- Hardware control
- Alarm management
- Telemetry
- Device-to-device communication
- Fault detection and recovery
- Hardware validation
- Industrial communication
- Embedded Linux integration

---

## Current Architecture

```text
MPU6050
Temperature + Acceleration
        |
       I2C
        |
        v
   taskSensores
        |
        | FreeRTOS Queue
        v
   taskControle
        |
   +----+-------------------+
   |                        |
   v                        v
Status LEDs             PWM Buzzer
                            |
                       Alarm Control

Button
  |
GPIO Interrupt
  |
  v
taskBotao
  |
Task Notification
  |
  v
taskControle

taskControle
     |
     v
Telemetry
     |
    UART
     |
     v
External Device
```

---

## Implemented Features

### Sensor Acquisition

- MPU6050 integration using **I2C**
- Temperature acquisition
- Acceleration acquisition on X, Y and Z axes
- Analog signal acquisition using **ADC Oneshot**
- Multiple ADC sample averaging
- Validation of sensor acquisition cycles
- Invalid acquisition discard
- Initial peripheral error handling

### FreeRTOS Architecture

- Multiple independent tasks
- Sensor acquisition task
- Control task
- Button/event task
- Telemetry responsibility separated from sensor acquisition
- Inter-task communication using **FreeRTOS Queues**
- Event signaling using **Task Notifications**
- Blocking task synchronization
- GPIO interrupt handling

### Equipment State Classification

The system classifies operating conditions into three states:

```text
NORMAL
WARNING
CRITICAL
```

Current thresholds:

| Parameter | NORMAL | WARNING | CRITICAL |
| --- | --- | --- | --- |
| Temperature | < 45 °C | 45–59.9 °C | >= 60 °C |
| Analog Input | < 2500 | 2500–3299 | >= 3300 |

A **CRITICAL** condition has priority over WARNING and NORMAL states.

---

### Alarm System

- Green LED for NORMAL condition
- Yellow LED for WARNING condition
- Red LED for CRITICAL condition
- Passive buzzer controlled using **LEDC PWM**
- Operator alarm acknowledgement
- Alarm acknowledgement does not modify the actual machine condition
- Alarm acknowledgement automatically resets after leaving CRITICAL state

---

### GPIO and Interrupts

- GPIO input interrupt
- Positive-edge detection
- Short ISR execution
- FreeRTOS notification from ISR
- Context-switch request using `portYIELD_FROM_ISR`
- Software debounce outside the ISR
- Event forwarding between tasks using Task Notifications

---

### UART Communication

- UART driver configured using native ESP-IDF APIs
- UART TX and RX pins configured
- Bidirectional UART communication infrastructure
- Structured UART telemetry
- Transmission of sensor measurements and equipment state
- Separation between acquisition, control and communication responsibilities

---

## Telemetry

The telemetry layer is responsible for transporting operational measurements, equipment state and alarm information.

Example:

```text
TEMP=31.20
AX=0.04
AY=-0.01
AZ=1.00
ADC=2740
STATE=WARNING
ALARM_ACK=0
```

The architecture keeps sensor acquisition independent from communication, allowing the telemetry transport layer to evolve without directly affecting the acquisition logic.

Future telemetry versions will include:

- Device identification
- Message sequence counter
- Timestamp
- Heartbeat
- Communication timeout detection
- Message integrity validation
- Communication fault recovery

---

## In Development

### CAN / TWAI

The project is being expanded to support communication between embedded nodes using the ESP32 native **TWAI controller (CAN protocol)**.

Planned functionality includes:

- CAN/TWAI communication between ESP32 nodes
- Message identifiers
- Sensor telemetry frames
- Equipment status frames
- Alarm and diagnostic frames
- Node identification
- Communication timeout detection
- Communication fault handling
- Recovery after communication loss

---

### RS-485

An **RS-485 industrial communication layer** is also planned for communication between embedded devices and gateway systems.

Development scope includes:

- UART integration with RS-485 transceiver
- Half-duplex communication
- Device addressing
- Structured message protocol
- Command and telemetry exchange
- Communication timeout detection
- Fault recovery
- Future evaluation of Modbus RTU

---

### Firmware Robustness

The firmware is being progressively expanded with reliability and diagnostic features:

- Watchdog integration
- NVS configuration persistence
- Structured diagnostic logging using `ESP_LOG`
- Centralized error handling
- Automatic failure recovery
- Communication health monitoring
- Sensor failure detection
- Modular `.c/.h` architecture
- Integration tests
- Fault injection tests

---

## Embedded Linux Gateway

The next system layer consists of an **embedded Linux gateway** responsible for receiving telemetry from embedded nodes and forwarding data to external services.

```text
ESP32 Sensor Node
       |
 CAN / RS-485
       |
       v
Embedded Linux Gateway
       |
       +---- MQTT
       |
       +---- HTTP / REST
       |
       +---- TCP/IP
       |
       +---- UDP
       |
       v
Backend / Cloud Services
```

The gateway development roadmap includes:

- Modern C++
- Linux services and daemons
- Threads
- Mutexes
- Condition variables
- Resource management
- Error handling
- IPC mechanisms
- TCP/IP sockets
- UDP communication
- MQTT
- HTTP/REST
- External API integration
- Network reconnection
- Communication failure handling
- `systemd` services
- `systemd` timers
- journal logging

---

## Hardware Validation Roadmap

The project is also being evolved from simulation toward physical prototyping and hardware validation.

Planned activities include:

- Datasheet analysis
- Schematic reading
- Physical prototyping
- Power supply validation
- Multimeter measurements
- Oscilloscope analysis
- Logic analyzer debugging
- UART signal validation
- I2C signal validation
- PWM signal validation
- Hardware fault diagnosis
- Bring-up procedures
- Through-hole soldering
- SMD soldering practice
- PCB schematic capture
- PCB layout using **KiCad**

---

## Reliability and Testing

The project roadmap includes engineering practices focused on robustness and validation:

- Sensor failure simulation
- Communication failure simulation
- Timeout testing
- Fault reproduction
- Recovery testing
- Boundary-value testing
- Integration testing
- Watchdog validation
- Communication reconnection testing
- Technical test documentation
- Test result documentation
- Non-conformity recording

---

## Technologies

### Firmware

- C
- C++
- ESP32
- ESP-IDF
- FreeRTOS

### Microcontroller Peripherals

- GPIO
- ADC
- PWM / LEDC
- Interrupts
- I2C
- UART
- Timers

### RTOS Concepts

- Tasks
- Queues
- Task Notifications
- Interrupt Service Routines
- Blocking synchronization
- Inter-task communication

### Industrial Communication

- UART
- CAN / TWAI
- RS-485
- Modbus RTU - planned

### Network and Telemetry Roadmap

- MQTT
- HTTP/REST
- TCP/IP
- UDP
- External APIs

### Embedded Linux Roadmap

- Modern C++
- Threads
- Mutexes
- Condition Variables
- IPC
- Sockets
- systemd
- journal

### Hardware

- Sensors
- Analog acquisition
- Digital interfaces
- Schematic reading
- Datasheet analysis
- Hardware debugging
- PCB design
- Prototyping

### Tools

- Git
- GitHub
- ESP-IDF
- Wokwi
- CMake
- KiCad
- Oscilloscope
- Logic Analyzer
- Multimeter

---

## Development Approach

The project is developed incrementally, with each subsystem implemented, tested and versioned separately.

The development process emphasizes:

- Native ESP-IDF APIs
- Hardware/firmware integration
- Clear separation of responsibilities
- Real-time architecture
- Failure handling
- Code organization
- Technical documentation
- Testability
- Reliability
- Maintainability
- Progressive system integration

The goal is not only to implement functionality, but also to understand the complete behavior of the system from the hardware interface to the firmware architecture and external communication layers.

---

## Development Roadmap

### Current

- [x] MPU6050 I2C integration
- [x] Temperature acquisition
- [x] Accelerometer acquisition
- [x] ADC acquisition
- [x] ADC averaging
- [x] FreeRTOS sensor task
- [x] FreeRTOS control task
- [x] FreeRTOS Queue communication
- [x] NORMAL / WARNING / CRITICAL state machine
- [x] GPIO status LEDs
- [x] GPIO interrupt
- [x] Button debounce
- [x] Task Notifications
- [x] Alarm acknowledgement
- [x] PWM buzzer
- [x] UART configuration
- [x] UART telemetry

### Next Steps

- [ ] CAN/TWAI communication
- [ ] RS-485 communication
- [ ] Structured communication protocol
- [ ] Heartbeat and timeout detection
- [ ] Firmware modularization
- [ ] Watchdog
- [ ] NVS persistence
- [ ] Structured diagnostic logging
- [ ] Integration tests
- [ ] Fault injection tests
- [ ] Embedded Linux gateway
- [ ] C++ Linux service
- [ ] MQTT integration
- [ ] HTTP/REST integration
- [ ] TCP/IP and UDP communication
- [ ] Hardware prototype
- [ ] Oscilloscope and logic analyzer validation
- [ ] KiCad schematic
- [ ] PCB layout
- [ ] OTA update

---

## Project Status

**Active Development**

Current development priorities:

1. CAN/TWAI communication
2. RS-485 communication
3. Firmware modularization
4. Fault handling and watchdog
5. NVS and diagnostic logging
6. Embedded Linux gateway
7. MQTT and HTTP integration
8. Hardware validation
9. KiCad PCB development

---

## Author

**Rafael Lopes Mateus**

Computer Engineer

Embedded Systems, Hardware & Firmware Development

Araçatuba, SP - Brazil
