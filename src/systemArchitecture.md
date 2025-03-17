## System Architecture

### Task Implementation

The synthesizer operates using a combination of **interrupts** and **thread-based tasks** to ensure real-time responsiveness and efficient execution.

| Task Name          | Implementation Method | Description |
|--------------------|----------------------|-------------|
| **scanKeysTask**   | Thread (FreeRTOS)    | Scans the keyboard matrix to detect key presses and releases. Runs every 20ms. |
| **displayUpdateTask** | Thread (FreeRTOS) | Updates the OLED display every 100ms with the current note and volume. |
| **decodeTask**     | Thread (FreeRTOS)    | Processes incoming CAN messages and updates the synthesizer state accordingly. |
| **CAN_TX_Task**    | Thread (FreeRTOS)    | Handles sending key press/release messages over the CAN bus. |
| **sampleISR**      | Interrupt (Timer ISR) | Generates the waveform output by updating the phase accumulator at 22kHz. |
| **CAN_RX_ISR_**    | Interrupt (CAN ISR)  | Handles reception of CAN messages and passes them to the decode task. |
| **CAN_TX_ISR_**    | Interrupt (CAN ISR)  | Signals completion of CAN transmission to allow sending new messages. |

### Task Execution and Scheduling

- **Threads**: Tasks are scheduled using FreeRTOS with predefined initiation intervals. Higher-frequency tasks, such as `scanKeysTask`, run more often than lower-priority tasks like `displayUpdateTask`.
- **Interrupts**: The **sampleISR** function runs at a fixed **22kHz rate** to ensure continuous sound generation without blocking the main task execution.  
- **Synchronization & Resource Management**: Shared resources such as `sysState` are protected using **mutexes** to prevent race conditions.

This architecture ensures that real-time constraints are met while efficiently handling input, output, and communication.
