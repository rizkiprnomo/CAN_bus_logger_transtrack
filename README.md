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

$$\text{Sample Point} = \frac{1 + 10 + 4}{20} = \frac{15}{20} = 75\%$$