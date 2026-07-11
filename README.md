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

TWAI timing parameters are derived from the ESP32 source clock ($f_{CLK}$, typically $80 \text{ MHz}$). The total bit time is divided into Time Quanta ($T_q$) and consists of three segments: **Sync_Seg**, **Bit_Seg1**, and **Bit_Seg2**.

### Mathematical Formulas

1. **Time Quantum ($T_q$)**
   $$T_q = \frac{\text{BRP}}{f_{CLK}}$$
   *Where $\text{BRP}$ is the Bit Rate Prescaler (integer).*

2. **Total Number of Quanta per Bit ($N_{Tq}$)**
   $$N_{Tq} = \text{Sync\_Seg} + \text{Bit\_Seg1} + \text{Bit\_Seg2}$$
   *(Note: $\text{Sync\_Seg}$ is always fixed at $1 \cdot T_q$).*

3. **Final Bitrate Formula**
   $$\text{Bitrate} = \frac{1}{N_{Tq} \times T_q} = \frac{f_{CLK}}{\text{BRP} \times N_{Tq}}$$

---

### Example Configuration: 250 kbps
Given $f_{CLK} = 80 \text{ MHz}$:
* **Prescaler Selection:** Set $\text{BRP} = 16 \implies T_q = \frac{16}{80,000,000} = 200 \text{ ns}$
* **Quanta Allocation:** Set $N_{Tq} = 20$ 
  $$\text{Sync\_Seg} = 1, \quad \text{Bit\_Seg1} = 15, \quad \text{Bit\_Seg2} = 4$$
* **Calculation:**
  $$\text{Bitrate} = \frac{80,000,000}{16 \times 20} = 250,000 \text{ bps} = 250 \text{ kbps}$$

---

### Example Configuration: 500 kbps
Given $f_{CLK} = 80 \text{ MHz}$:
* **Prescaler Selection:** Set $\text{BRP} = 8 \implies T_q = \frac{8}{80,000,000} = 100 \text{ ns}$
* **Quanta Allocation:** Set $N_{Tq} = 20$ 
  $$\text{Sync\_Seg} = 1, \quad \text{Bit\_Seg1} = 15, \quad \text{Bit\_Seg2} = 4$$
* **Calculation:**
  $$\text{Bitrate} = \frac{80,000,000}{8 \times 20} = 500,000 \text{ bps} = 500 \text{ kbps}$$

