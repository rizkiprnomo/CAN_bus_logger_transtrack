# AI Prompt Strategy: CAN Bus Logger

## 1. System Architecture & Requirements
* **CAN Interface (Listen-Only)**:
    * Utilize ESP-IDF TWAI driver.
    * Bitrate must be configurable (250/500 kbps) via Kconfig or NVS.
    * Implement robust Bus-Off/Error Passive recovery logic; document strategy in README.
* **Logging Pipeline**:
    * Log format: CSV (`timestamp_us`, `id`, `extended`, `rtr`, `dlc`, `data_hex`).
    * Data path: `can_rx_task` (Producer) -> `xQueue` -> `log_writer_task` (Consumer/SD Writer).
    * Logic: Implement drop counters and expose `frames_received`, `frames_dropped`, and `write_errors` for monitoring.
* **FreeRTOS Task Allocation**:
    * `can_rx_task`: High priority, non-blocking queue ingestion.
    * `log_writer_task`: Disk I/O operations, file rotation handling.
    * `monitor_task`: LED feedback, system health, UART command shell.

## 2. Technical Implementation Standards
* **Storage (FATFS)**:
    * Manage FAT partition mounting.
    * Implement file rotation (`can_001.csv`, `can_002.csv`, etc.).
    * Handle SD removal/mount failure gracefully without impacting CAN RX stability.
* **Control Interfaces**:
    * GPIO Button: Trigger logging toggle and manual log file rotation.
    * UART Command Shell:
        * Implement byte-by-byte polling via `uart_read_bytes`.
        * Perform robust input sanitization for noise/transmission artifacts.
        * Supported Commands: `start`, `stop`, `stats`.

## 3. Engineering & Debugging Protocol
* **Protocol Fidelity**: All CAN/J1939 parsing must strictly adhere to user-verified definitions (e.g., bit positions, byte lengths) established in session history.
* **Interface Robustness**: Maintain system stability under industrial environmental conditions (noise/interference).
* **Operational Transparency**: AI solutions must consistently track and expose key metrics (`frames_received`, `frames_dropped`, `write_errors`) in diagnostic feedback.

## 4. Interaction Workflow
* **Constraint Adherence**: When a hardware-level implementation is requested (e.g., TWAI filters), provide direct implementation and architectural rationale.
* **Diagnostic Prioritization**: In the event of implementation failure, default to Hex/ASCII level debugging before suggesting logic modifications.
* **Context Preservation**: Strictly maintain continuity with previously established technical specs and component-specific configurations (e.g., ESP32-S3, RP2040, industrial panel PCs).