# Coursework 2: Music Synthesizer
## Advanced Features
add information about advanced features and descriptions of how they're implemented

## Critical Instant Analysis

dk how to do this part yet

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

## 3. Performance Analysis

### 3.1 Timing Analysis

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

---

### 3.2 CPU Utilization Analysis

Using the formula:  

U = Σ (T_i / τ_i)

![Formula](https://latex.codecogs.com/png.latex?U%20%3D%20%5Csum%20%5Cfrac%7BT_i%7D%7B%5Ctau_i%7D)


where \( T_i \) is execution time and \( \tau_i \) is the initiation interval:

| **Task**             | **Execution Time (T_i)** | **Initiation Interval (τ_i)** | **CPU Utilization Contribution** |
|----------------------|------------------------|-----------------------------|--------------------------------|
| **scanKeysTask**     | 96.2 µs                 | 20,000 µs                   | 0.00481 (0.481%)               |
| **displayUpdateTask** | 16,028 µs               | 100,000 µs                  | 0.16028 (16.03%)               |
| **decodeTask**       | 5.16 µs                 | 20,000 µs                   | 0.000258 (0.0258%)             |
| **CAN_TX_Task**      | 863 µs                  | 20,000 µs                   | 0.04315 (4.315%)               |
| **Total CPU Utilization** | -                   | -                           | **0.208498 (20.85%)**          |

#### **Analysis:**
- **Total CPU utilization:** 20.85%, which is **well below the 70% real-time safety threshold**.  
- The system is **not overloaded** and can accommodate additional features such as **polyphony or waveform enhancements**.

---

## Data Management & Synchronisation

To ensure reliable operation in a multi-threaded environment, the synthesizer shares several key data structures between tasks and uses **mutexes, atomic operations, and queues** for synchronization.

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

## Dependency Graph

Add dependency graph for different tasks here and possibly a short description