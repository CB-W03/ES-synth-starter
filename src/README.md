# Coursework 2: Music Synthesizer
## Core Functionality


## Advanced Features


## 4. Performance Analysis

### 4.1 Timing Analysis

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

### 4.2 CPU Utilization Analysis

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



## Testing & Validation