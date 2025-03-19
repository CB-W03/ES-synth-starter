# Music Synthesiser Report

## Table of Contents
1. [Introduction](#introduction)  
2. [System Requirements Fulfillment](#system-requirements-fulfillment)  
   1. [Core Functional Specifications](#core-functional-specs)  
   2. [Non-Functional Specifications](#non-functional-specs)  
   4. [Advanced Features](#advanced-features)  
3. [System Architecture](#system-architecture)  
   1. [Tasks and ISRs](#tasks-and-isrs)  
   2. [Thread Scheduling and Timing](#thread-scheduling)
   3. [Shared Data Structures & Synchronization](#shared-data-and-sync)  
4. [Analysis of Real-Time Behavior](#analysis-of-real-time)  
   1. [Task Characterization](#task-characterization)  
   2. [Worst-Case Execution Times and Rates](#worst-case-execution-times)  
   3. [Critical Instant Analysis](#critical-instant-analysis)  
   4. [CPU Utilization](#cpu-utilization)  
   5. [Inter-Task Blocking and Deadlocks](#inter-task-blocking)  
---

## 1. Introduction <a id="introduction"></a>

This report features a multi-board **Music Synthesiser** system built on a microcontroller using FreeRTOS tasks and interrupt service routines (ISRs). The project meets all core and non-functional specifications. Where possible, advanced features enhance the core functionality (without changing its fundamental behavior).

---

## 2. System Requirements Fulfillment <a id="system-requirements-fulfillment"></a>

### 2.1 Core Functional Specifications <a id="core-functional-specs"></a>

1. **Play a musical tone as a sawtooth wave when a key is pressed**  
   -**Implemented** ✔: The code’s default waveform (`waveSel = 0`) is a sawtooth, generated in `sampleISR()` via `generateSample()`.

2. **No perceptible delay between pressing a key and the tone starting**  
   - **Implemented**✔ : Key scanning occurs every 20ms in `scanKeysTask()`. Once a key press is detected, the step size is updated immediately (either locally if in receiver mode or via CAN message to the receiver).

3. **Volume control with at least 8 increments**  
   - **Implemented**✔ : A knob object (`Knob volumeKnob`) ranges from `0..8`. Its updates happen in `scanKeysTask()` for the receiver board. It is on knob3.

4. **OLED display showing note name and volume**  
   - **Implemented**✔ : The display currently shows Volume, Position, and Waveform and the note name being played. Note names are defined in the array `noteNames[]`.

5. **OLED display refresh every 100ms and LED toggle**  
   - **Implemented**✔: `displayUpdateTask()` refreshes the display at 100ms intervals and toggles the built-in LED.

6. **Configurable as sender or receiver**  
   - **Implemented**: The code automatically determines role using handshake logic. The handshake sets `receiverMode` to `true` for the leftmost board (position 0) and this acts as the reciever.

7. **Sender sends CAN message on key press/release**  
   - **Implemented**✔: In `scanKeysTask()`, a pressed/released difference triggers a `'P'` or `'R'` CAN message to the bus.

8. **Receiver plays or stops note on receiving CAN messages**  
   - **Implemented**✔ : `decodeTask()` interprets `'P'` (press) or `'R'` (release) messages, updating `pressed[]` and `currentStepSize[]` accordingly.

### 2.2 Non-Functional Specifications <a id="non-functional-specs"></a>

9. **Use interrupts and threads**  
   - **Implemented**✔ : The system uses multiple tasks (`scanKeysTask()`, `displayUpdateTask()`, `decodeTask()`, `CAN_TX_Task()`) plus ISRs (`CAN_RX_ISR_()`, `CAN_TX_ISR_()`, `sampleISR()` using a hardware timer).

10. **Protect data/resources accessed by multiple tasks**  
   - **Implemented**✔: `pressed[]` is protected by `pressedMutex`; `sysState` and `msgInQ`/`msgOutQ` are accessed via semaphores and FreeRTOS queues.

11. **Well-structured and maintainable code**  
   - **Implemented**✔ : Code is modular, uses classes (`Knob`), and organizes functionality into tasks.

12. **Compile-time options for measuring execution time**  
   - **Implemented**✔ : The code has placeholders (like `#define TEST_*`) for running tasks in isolation and measuring maximum execution times.


### 2.3 Advanced Features <a id="advanced-features"></a>
- **Polyphony**: Multiple Keys can be pressed at the same time (up to 8)
- **Multiple Waveforms**: The code supports sawtooth, square, and triangle wave generation (`waveSel = 0,1,2`).
- **Joystick-based Waveform Selection**: The joystick’s Y-axis increments and decrements the waveform type if moved above or below thresholds.
- **Multi-Board Handshake**: The code automatically assigns position indexes to multiple boards on the CAN bus, enabling them to share up to 60 keys combined (they need to be reset together to sync).
- **Changable Octaves**: The code also has an octave knob (Knob2).
---

## 3. System Architecture <a id="system-architecture"></a>

### 3.1 Tasks and ISRs <a id="tasks-and-isrs"></a>

Below are the primary **FreeRTOS tasks** and **ISRs**:

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

#### Task Execution and Scheduling
- **Threads**: Tasks are scheduled using FreeRTOS with predefined initiation intervals. Higher-frequency tasks, such as `scanKeysTask`, run more often than lower-priority tasks like `displayUpdateTask`.
- **Interrupts**: The **sampleISR** function runs at a fixed **22kHz rate** to ensure continuous sound generation without blocking the main task execution.  
- **Synchronization & Resource Management**: Shared resources such as `sysState` are protected using **mutexes** to prevent race conditions.

This architecture ensures that real-time constraints are met while efficiently handling input, output, and communication.
### 3.2 Thread Scheduling and Timing <a id="thread-scheduling"></a>

- **FreeRTOS** uses priority-based preemptive scheduling. The tasks have assigned priorities (e.g. `scanKeysTask` has priority 2, while `displayUpdateTask`, `decodeTask`, and `CAN_TX_Task` have priority 1).  
- The **timer ISR** for audio sampling (`sampleISR()`) always preempts tasks because interrupts have higher priority than FreeRTOS threads.  

### 3.3 Shared Data Structures & Synchronization <a id="shared-data-and-sync"></a>


### Shared Data Structures and Access Methods


| Shared Data Structure      | Description | Protection Method |
|---------------------------|-------------|-------------------|
| **sysState.inputs**       | Stores the current key matrix state. | **Mutex (`sysState.mutex`)** to prevent concurrent access issues. |
| **sysState.rotationVariable** | Stores the current volume level from the rotary encoder. | **Atomic operations (`__atomic_load_n`, `__atomic_store_n`)** ensure thread-safe access. |
| **sysState.RX_Message**   | Holds the most recent received CAN message. | **Mutex (`sysState.mutex`)** ensures updates do not interfere with reads. |
| **activeKeys**            | Tracks which notes are currently active. | **Mutex (`sysState.mutex`)** to ensure updates are thread-safe. |
| **currentStepSize**       | Determines the frequency of the generated waveform. | **Atomic operations** for safe concurrent access. |
| **msgInQ** (Queue)        | Stores incoming CAN messages for processing. | **FreeRTOS queue (`xQueueSend`, `xQueueReceive`)** for thread-safe message passing. |
| **msgOutQ** (Queue)       | Stores outgoing CAN messages to be sent. | **FreeRTOS queue (`xQueueSend`, `xQueueReceive`)** to prevent data loss. |
| **CAN_TX_Semaphore**      | Controls CAN transmission availability. | **Semaphore (`xSemaphoreTake`, `xSemaphoreGive`)** ensures only one task sends at a time. |

### Synchronization Techniques Used

To ensure reliable operation in a multi-threaded environment, the synthesizer shares several key data structures between tasks and uses **mutexes, atomic operations, and queues** for synchronization.


1. **Mutexes (`sysState.mutex`)**  
   - Used to protect **sysState** variables from race conditions.
   - Ensures that only one task accesses or modifies shared data at a time.

2. **Atomic Operations (`__atomic_load_n`, `__atomic_store_n`)**  
   - Used for **fast** and **safe access** to simple variables like `currentStepSize` and `rotationVariable`, avoiding the overhead of mutexes.

3. **Queues (`msgInQ`, `msgOutQ`)**  
   - Used for **safe inter-task communication** of CAN messages.
   - Ensures message integrity without blocking tasks.

4. **Semaphores (`CAN_TX_Semaphore`)**  
   - Controls CAN message transmission to **prevent conflicts**.
   - Ensures **only one message** is transmitted at a time.

### Ensuring Data Consistency and Preventing Deadlocks

- **Non-blocking access**: Queues and atomic operations minimize waiting times.
- **Short critical sections**: Mutex-protected code is kept as brief as possible to reduce blocking delays.
- **No circular dependencies**: Mutexes are only used where necessary, avoiding deadlocks.

These techniques ensure the synthesizer operates **smoothly and efficiently**, even when handling concurrent key presses, display updates, and CAN communication.


---

## 4. Analysis of Real-Time Behavior <a id="analysis-of-real-time"></a>

### 4.1 Task Characterization <a id="task-characterization"></a>

For each task, we identify its **period (or minimum initiation interval)**, **deadline**, and the **measured maximum execution time**. 

To ensure the synthesizer meets real-time constraints, we measured the execution times of key functions using microsecond-level precision. The results are as follows:

#### scanKeysTask
- **Execution time for 32 iterations:** 3,078 µs  
- **Average execution time per call:** 96.2 µs  
- **Expected range:** 50–100 µs (Falls within expected limits)  
- **Analysis:** Efficient and ensures real-time key press detection without noticeable delay.  

#### displayUpdateTask
- **Execution time for 32 iterations:** 512,908 µs  
- **Average execution time per call:** 16,028 µs (~16 ms)  
- **Expected range:** 10–20 ms  
- **Analysis:**
  - Display updates are computationally expensive due to OLED rendering.
  - Meets the requirement of refreshing every 100 ms but could be optimized by reducing redraw complexity.  

#### decodeTask
- **Execution time for 32 iterations:** 165 µs  
- **Average execution time per call:** 5.16 µs  
- **Analysis:**  
  - Efficient and lightweight.  
  - Handles incoming CAN messages with minimal processing overhead.  

#### CAN_TX_Task
- **Execution time for 32 iterations:** 27,620 µs  
- **Average execution time per call:** 863 µs  
- **Analysis:**  
  - Handles sending CAN messages efficiently.  
  - Can be optimized to batch messages if needed.  

Overall, **scanKeysTask and decodeTask execute well within real-time constraints**, while **displayUpdateTask and CAN_TX_Task** are more time-intensive.  
Potential optimizations include **reducing the display update rate** or **optimizing how messages are sent over CAN**.


### Task and ISR Characterization Table

| **Task/ISR**       | **Type**    | **Period / Min Init Interval** | **Deadline** | **Measured Max Exec. Time** |CPU Utilization| **Shared Data / Synchronization** | **Comments** |
|--------------------|------------|--------------------------------|--------|-------------|---------------------------|----------------------------------|-------------|
| `scanKeysTask`      | Thread     | 20 ms                         | 20 ms       | (to be measured)           || `sysState.inputs`, `pressedMutex` | Scans keypad rows + knobs (receiver). |
| `displayUpdateTask` | Thread     | 100 ms                        | 100 ms      | (to be measured)           || `sysState.mutex`                  | Updates OLED + toggles LED. |
| `decodeTask`        | Thread     | Event-based                   | N/A         | (to be measured)           || `msgInQ`, `sysState.mutex`        | Responds to incoming CAN messages. |
| `CAN_TX_Task`       | Thread     | Event-based                   | N/A         | (to be measured)           || `msgOutQ`, `CAN_TX_Semaphore`     | Sends messages from `msgOutQ`. |
| `sampleISR()`       | Interrupt  | 1/22000 ≈ 45.45 µs            | 45.45 µs    | (to be measured)           || `currentStepSize[]`, `pressedMutex` | Summation of active notes + wave generation. |
| `CAN_RX_ISR_()`     | Interrupt  | Hardware-driven               | FIFO limit  | (to be measured)           || `msgInQ`                          | Places data into `msgInQ`. |
| `CAN_TX_ISR_()`     | Interrupt  | Hardware-driven               | FIFO limit  | (to be measured)           || `CAN_TX_Semaphore`                | Signals TX semaphore release. |


### 4.2 Worst-Case Execution Times and Rates <a id="worst-case-execution-times"></a>

- We leave this section as a placeholder. Measure each task (and ISR) in isolation (e.g., define `TEST_*` macros) and record the max times.

### 4.3 Critical Instant Analysis <a id="critical-instant-analysis"></a>

- A typical **Rate Monotonic (RM) analysis** for the tasks with fixed periods (like `scanKeysTask` at 20 ms and `displayUpdateTask` at 100 ms) would consider the highest-priority tasks first.  
- If you measure each task’s worst-case time and sum them per harmonic period, you can show whether deadlines are met.

### 4.4 CPU Utilization <a id="cpu-utilization"></a>

- Summation of (execution_time / period) across tasks plus some overhead for ISRs.  
- Because `decodeTask` and `CAN_TX_Task` are event-driven, they are not strictly periodic. You can approximate or measure typical or worst-case message frequencies.

### 4.5 Inter-Task Blocking and Deadlocks <a id="inter-task-blocking"></a>

- **Shared resources** are primarily protected by mutexes, so the system should not exhibit deadlock if each mutex is always freed quickly and tasks do not attempt to acquire multiple mutexes in conflicting orders.  
- Currently, each piece of data has its own dedicated mutex (e.g., `pressedMutex` for the pressed array, `sysState.mutex` for the system state). Because tasks do not hold more than one at the same time, **no cyclical wait** is observed.  
- Hence, risk of deadlock is minimal under the current design.



---

## Dependency Graph

Add dependency graph for different tasks here and possibly a short description
