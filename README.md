# CAN Bus Logger Configuration & Documentation

This document provides the hardware wiring guidelines, bitrate calculation formulas for the TWAI (CAN Bus compatible) peripheral, and etc.

---

## 1. TWAI Pin Wiring

To establish a TWAI network, the ESP32 must be connected to a CAN transceiver (e.g.,TJA1050) which converts the logic-level signals to differential CAN signals (`CAN_H` and `CAN_L`).

### Hardware Connection
| ESP32 Pin | Transceiver Pin | Description |
| :--- | :--- | :--- |
| **GPIO 4**  | **TXD** / **CTX** | TWAI Transmit Data Output |
| **GPIO 5**  | **RXD** / **CRX** | TWAI Receive Data Input |
| **GND** | **GND** | Ground Reference |
| **5V** | **VCC** | Power Supply (Depends on Transceiver IC) |


> ⚠️ **Important Note on Termination:** Ensure a **120Ω termination resistor** is present between `CAN_H` and `CAN_L` at both ends of the CAN bus backbone.

### Connection Diagram

```text
  +-------------------+               +--------------------+
  |      ESP32S3      |               |   CAN Transceiver  |
  |                   |               |  (e.g., TJA1050)   |
  |       GPIO 4 (TX)X--------------->TXD                  |      CAN Bus
  |       GPIO 5 (RX)<---------------+RXD            CAN_H|========== X (Termination
  |                   |               |                    |              Resistor
  |                   |               |               CAN_L|========== X   120 ohms)
  |                   |               |                    |
  |                GNDX---------------XGND                 |
  +-------------------+               +--------------------+

```

## 2. Bitrate Calculation

TWAI timing parameters are derived from the ESP32 source clock ($f_{\text{CLK}}$, typically $80 \text{ MHz}$). The total bit time is divided into Time Quanta ($T_q$) and consists of three segments: **Sync_Seg**, **Bit_Seg1**, and **Bit_Seg2**.

### Mathematical Formulas

1. Time Quantum Length ($`T_q`$)

The duration of one Time Quantum is determined by the Bit Rate Prescaler ($`\text{BRP}`$):

$$T_q = \frac{\text{BRP}}{f_{\text{CLK}}}$$

2. Output Bitrate Formula (ESP-IDF Official)

Using the official ESP-IDF register definitions where $`T_{\text{seg1}} = \text{prop\_seg} + \text{tseg}_1`$ and $`T_{\text{seg2}} = \text{tseg}_2`$:

$$\text{Bitrate} = \frac{f_{\text{CLK}}}{\text{BRP} \times (1 + \text{prop}_{\text{seg}} + \text{tseg}_1 + \text{tseg}_2)}$$

3. Sample Point Formula

$$\text{Sample Point} = \frac{1 + \text{prop}_{\text{seg}} + \text{tseg}_1}{1 + \text{prop}_{\text{seg}} + \text{tseg}_1 + \text{tseg}_2}$$

Example Configuration: 250 kbps

Given $`f_{\text{CLK}} = 80 \text{ MHz}`$: 1. Prescaler Setup: Choose $`\text{BRP} = 16 \implies T_q = 200 \text{ ns}`$ 2. Quanta Allocation: Divide the single bit duration into $`20 \cdot T_q`$:

$`\text{prop\_seg} = 10`$

$`\text{tseg}_1 = 4`$

$`\text{tseg}_2 = 5`$

Verify Bitrate & Sample Point:

$$\text{Bitrate} = \frac{80,000,000}{16 \times (1 + 10 + 4 + 5)} = 250,000 \text{ bps} = 250 \text{ kbps}$$

$$\text{Sample Point} = \frac{1 + 10 + 4}{20} = \frac{15}{20} = 75\%$$

Example Configuration: 500 kbps

Given $`f_{\text{CLK}} = 80 \text{ MHz}`$: 1. Prescaler Setup: Choose $`\text{BRP} = 8 \implies T_q = 100 \text{ ns}`$ 2. Quanta Allocation: Divide the single bit duration into $`20 \cdot T_q`$:

$`\text{prop\_seg} = 10`$

$`\text{tseg}_1 = 4`$

$`\text{tseg}_2 = 5`$

Verify Bitrate & Sample Point:

$$\text{Bitrate} = \frac{80,000,000}{8 \times (1 + 10 + 4 + 5)} = 500,000 \text{ bps} = 500 \text{ kbps}$$

$$\text{Sample Point} = \frac{1 + 10 + 4}{20} = \frac{15}{20} = 75\% $$

## 3. System Status LED Indicators
To provide real-time visual feedback on the system's operational state without requiring an active serial debugger console, the ESP32 controls a single status LED using a dedicated FreeRTOS task (led_status_task).


| LED Status (`current_led_status`) | Trigger Condition (Global Logic) | Blink Pattern Description | Total Duration Per Cycle |
| :--- | :--- | :--- | :--- |
| **`LED_STATUS_SD_FAIL`** | `sd_card_not_found == true` | **SOS Pattern**:<br>• 3x short blinks (100ms ON, 100ms OFF)<br>• End-of-pattern delay 500ms | ~1100 ms |
| **`LED_STATUS_BUS_OFF`** | `bus_off_detected == true` | **Solid OFF**:<br>• LED is turned off continuously with a 1000ms delay | 1000 ms |
| **`LED_STATUS_ERROR`** | `write_errors > 0` | **Double Blink**:<br>• 2x blinks (200ms ON, 200ms OFF, 200ms ON, 200ms OFF)<br>• End-of-pattern delay 600ms | 1200 ms |
| **`LED_STATUS_LOGGING`** | `logging_active == true` | **Fast Heartbeat**:<br>• Fast blinking (50ms ON, 250ms OFF) | 300 ms |
| **`LED_STATUS_IDLE`** | Default condition (no active errors/tasks) | **Slow Blink**:<br>• Slow blink 1x per second (200ms ON, 800ms OFF) | 1000 ms |
| **`LED_STATUS_STARTUP`** | Outside the main `if-else` logic | **Solid ON**:<br>• LED stays ON continuously for 2000ms | 2000 ms |


## 4. Generating Test CAN Traffic Without a Vehicle (STM32 Simulator)

Performing live on-site testing directly on vehicle fleets or heavy machinery carries safety risks and is highly impractical during the early phases of firmware development. To overcome these constraints, we can build a standalone CAN Bus Simulator capable of precisely playing back real telemetry data captured from actual field units.

### Hardware Simulator Setup

This simulator is built using an STM32F407VET6 microcontroller connected to a TJA1050 transceiver and an SD Card Reader module (utilizing high-speed SDIO/SPI protocols).
```text

  +--------------------------------------------------------+
  |                   STM32F407VET6 BOARD                  |
  |                                                        |
  |  [ SD Card ] ---> Reads CSV Log Files from Field Units |
  |                                                        |
  |  CAN_TX (PB9) X----------> TXD [ TJA1050 Transceiver ] |
  |  CAN_RX (PB8) <----------  RXD [   (Simulator Node)  ] |
  +--------------------------------------------------------+
                                       |
                                CAN_H / CAN_L (Bus Link)
                                       |
  +--------------------------------------------------------+
  |                  ESP32 / ESP32-S3 BOARD                |
  |                                                        |
  |  GPIO 4 (TX)  X----------> TXD [  TJA1050 Transceiver] |
  |  GPIO 5 (RX)  <----------  RXD [   (Receiver Node)   ] |
  |                                                        |
  |  [ TWAI Driver ] ---> Receives & Validates Data Packets|
  +--------------------------------------------------------+
```

### Simulator Working Mechanism

1. SD Card Initialization: The STM32 initializes the high-speed SDIO/SPI connection to read the FAT32-formatted SD card and search for the designated raw CSV telemetry logs.

2. CSV Parsing: The simulator reads the data line-by-line, extracting essential packet properties:

3. Bitrate: Configures the initialization parameters of the STM32's internal bxCAN hardware module (250 kbps or 500 kbps).

4. CAN ID: The frame identifier (Standard 11-bit or Extended 29-bit).

5. DLC (Data Length Code): The number of data bytes in the frame (0 to 8 bytes).

6. Data Bytes: The actual sensor hexadecimal payloads exported from real machinery.

7. Payload Broadcasting: The STM32 transmits the reconstructed packets onto the physical network via the TJA1050 transceiver, allowing the receiving ESP32 node to read, parse, and process them as if it were connected directly to an active vehicle.

### Field Unit Telemetry Logging Profiles (SD Card Dataset)

The CSV files on the SD card contain real-world telemetry logs captured from the following machinery units:

1. VOLVO FMX420 (Heavy Duty Truck):
   * Characteristics: Runs on the J1939 protocol with a standard bitrate of 500 kbps (29-bit Extended ID).

   * Data Logs: Contains engine telemetry (RPM, torque parameters), engine coolant temperature, engine oil pressure, accelerator pedal position, odometer status, and real-time fuel consumption rates.

2. VOLVO ADT (Articulated Dump Truck):

   * Characteristics: Utilizes a multi-node CAN architecture at a 250 kbps bitrate.

    * Data Logs: Contains the status of the dump body articulation hydraulics, truck bed tilt angles, transmission temperature sensors, brake pressures, and differential gear lock statuses (diff_lock).

3. XCMG (Heavy Machinery - Excavator/Crane):

    * Characteristics: Employs customized CANopen/J1939 architectures running at 250 kbps.

    * Data Logs: Contains hook load-cell sensor measurements, main hydraulic pump pressures, boom/arm motion angles, and operational safety indicators (Limit Switches).

By leveraging this STM32-based simulator, testing the ESP32's TWAI data-parsing logic can be conducted safely and comprehensively on your workbench using authentic, production-grade field data.

