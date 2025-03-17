
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
