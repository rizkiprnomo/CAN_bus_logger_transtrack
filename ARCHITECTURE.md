# Architecture Documentation: CAN Bus Logger 

This document outlines the software architecture designed for the CAN Bus Logger system using ESP-IDF. The architecture ensures reliable data logging to local storage (SD Card) without degrading the real-time performance of data reception from the CAN Bus network.

---

## 1. Producer-Consumer Architecture (Multi-Tier Queue)

The system utilizes a **Producer-Consumer** model isolated through two FreeRTOS queues to guarantee a clean separation of concerns and prevent blocking at the hardware interrupt level.

### 🔄 Data Pipeline Flow
1. **[ CAN BUS Network ]**
   │ 
   ▼ *(Hardware Interrupt triggers)*
2. **[ ISR Callback: `twai_rx_cb` ]**
   │ ──► Copies raw frame to a flat, value-based structure
   ▼ *(`xQueueSendFromISR` — Non-blocking)*
3. **[ `can_rx_queue` ]** *(Capacity: 30 frames)*
   │ 
   ▼ *(`xQueueReceive` — Blocking / Fast Worker)*
4. **[ CAN RX Task ]** *(Core 0, Priority 6)*
   │ ──► Appends high-resolution microsecond timestamp (`esp_timer`)
   ▼ *(`xQueueSend` — Non-blocking, Timeout: 0)*
5. **[ `sd_log_queue` ]** *(Capacity: 100 log frames)*
   │ 
   ▼ *(`xQueueReceive` — Blocking with Timeout: 500ms)*
6. **[ SD Write Task ]** *(Core 1, Priority 4)*
   │ ──► Converts binary to Hex string & writes data lines
   ▼ *(Physical I/O block transfer)*
7. **[ Physical Storage: SD CARD ]**

---

### 🔹 Core Components:
* **ISR Context (`twai_rx_cb`):** Retrieves data directly from the hardware buffer as fast as possible and pushes it into the `can_rx_queue` as a flat structure (instead of relying on driver-internal pointers) to eliminate potential Bus Errors.
* **CAN RX Task:** Executes on **Core 0 (High Priority: 6)**. It pops data from the ISR queue, injects a high-resolution microsecond timestamp, and forwards the packet to the SD logging queue.
* **SD Write Task:** Executes on **Core 1 (Low Priority: 4)**. It converts binary arrays into hex string representations and manages slow storage I/O operations independently, completely insulating the CAN interface from write latency.

---

## 2. Threshold Safeguards & Overflow Strategy

SD Card write operations inherently possess non-deterministic latency due to internal physical sectors triggering garbage collection. When SD Card write speeds drop below the incoming CAN Bus frame rate, the system deploys the following mitigations:

* **Shock Absorber Buffer:** The `sd_log_queue` is allocated with a **100-frame capacity** to absorb temporary bursts of traffic while the SD Card is busy flushing blocks.
* **Non-Blocking Drop Policy:** Inside the `can_rx_task`, data transmission to the `sd_log_queue` uses a strict timeout parameter of `0`. If the queue is entirely full, incoming frames are **instantly dropped** to safeguard remaining heap memory from out-of-memory (OOM) risks.
* **Statistical Indicators:** Every time a frame is dropped due to a saturated queue, a thread-safe atomic global counter `frames_dropped` is incremented for runtime diagnostic visibility.

---

## 3. SD Card Failure Handling & Life-Cycle Management

To mitigate filesystem corruption on FATFS and prevent logs from being lost due to abrupt power cuts, the system implements dynamic file life-cycle management (Transient Fault Handling):

* **Periodic Open-Write-Close Pattern:** The target `.csv` file is never left permanently open. The system tracks successful transactions via a `sync_counter`. Every **50 frames**, the file descriptor is explicitly flushed and closed via `fclose()` to commit the File Allocation Table (FAT) cluster changes to physical media, before resetting the file pointer to `NULL`.
* **Intermittent Timeout Flush:** If the CAN Bus traffic drops or completely stops before reaching the 50-frame flush threshold, a **500 ms** timeout triggers within the `xQueueReceive` loop of the SD Write Task. This timeout forces any residual data lingering in the RAM buffer to safely commit to physical media by triggering a premature file close.
* **Media Loss Detection (Hot-Plug Recovery):** If `fwrite()` fails to return the expected written byte length (indicating the card was removed or is entirely full), the system forces an immediate `fclose()`, toggles a `sd_healthy = false` flag, and flags a `write_errors` counter. In subsequent loops, the task bypasses futile write attempts and switches into a periodic retry loop without blocking the rest of the MCU.
* **Unique Header Injection:** To prevent the CSV header from duplicating during periodic open/close cycles, the system attempts to open the file in read mode (`"r"`) first. The header string is only injected if the file is confirmed to be non-existent.

---

## 4. Bus-Off Detection & Autonomous Recovery

When the physical network experiences severe abnormalities (e.g., disconnected cables, short circuits, or missing 120-Ohm termination resistors), the hardware's Transmit Error Counter (TEC) will scale past 255, shifting the controller into a *Bus-Off* (offline) state. The system handles recovery seamlessly according to fault isolation standards:

* **Interrupt to Task Context Decoupling:** Hardware state transits are caught by the `.on_state_change` callback inside IRAM via string array status matching (`"bus_off"` and `"active"`). As per the technical requirements of the platform, recovery procedures **must not** run directly within the interrupt context. Instead, the ISR acts as a lightweight trigger that signals the `can_rx_task_handle` with a dedicated emergency bitmask (`0x01`) using the highly efficient `xTaskNotifyFromISR` mechanism.
* **Recovery Execution (Task Context):** Inside the `can_rx_task`, the non-blocking `xTaskNotifyWait` catches the notification. Upon reading the `0x01` bitmask, it invokes the official `twai_node_recover()` function safely from within the **Task Context**.
* **Autonomous Resynchronization Phase:** Once the recovery sequence is initiated, the internal CAN hardware controller independently monitors the bus lines and waits until it detects 128 consecutive sequences of 11 recessive bits to guarantee that network signals have completely stabilized.
* **Online Resumption Transition:** As soon as the hardware stabilization phase completes, the status callback detects the transition back to `"active"`. The controller re-enables the interface via `twai_node_enable()`, bringing the node back ONLINE to immediately resume draining pending transmit/receive queues without needing an exhaustive MCU reboot cycle.