// Uncomment the following lines as needed for fixed roles or test mode:
// #define TEST_BUILD      // Enable test suite

#ifdef TEST_BUILD
  // Uncomment any of these to run a specific test:
  // #define TEST_SCANKEYS
  // #define TEST_DISPLAYUPDATE
  // #define TEST_DECODE
  // #define TEST_CANTX
  // #define TEST_sampleISR
  // #define TEST_TX_ISR
  // #define TEST_RX_ISR
  // Optionally disable threads or ISRs for test:
  // #define DISABLE_THREADS
  // #define DISABLE_ISRS
#endif


#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <string.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include <ES_CAN.cpp>
// -------------------- Pin Definitions --------------------
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN  = A5;

const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits (also used for handshake)
const int DEN_BIT  = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;  // handshake output to west (enable)
const int HKOE_BIT = 6;  // handshake output to east (enable)

// -------------------- Global Shared State --------------------
struct {
  std::bitset<32> inputs;  // Holds state of 32 input bits (key matrix)
  SemaphoreHandle_t mutex;

  // Existing fields
  int rotationVariable;     // (If you only want one knob, you can re-use this.)
  uint8_t RX_Message[8];

  // New: separate fields for the 3 knobs (only relevant to the receiver)
  int volumeKnobVal;  // 0..8
  int octaveKnobVal;  // e.g. −3..+3
  int waveformSel;
} sysState;
// static int lastWaveSel = -1;  // -1 ensures the first read always updates
const int UPPER_THRESHOLD = 700;
const int LOWER_THRESHOLD = 300;

// -------------------- Multi–Board and Polyphony --------------------
#define KEY_SIZE 60   // e.g., support up to 60 keys across boards
volatile uint32_t currentStepSize[KEY_SIZE] = {0};
volatile uint8_t  pressed[KEY_SIZE]         = {0};
// volatile uint32_t txStartTime = 0;

SemaphoreHandle_t pressedMutex;  // Protects the pressed array

const char* noteNames[12] = {"C", "C#", "D", "D#", "E", "F",
                             "F#", "G", "G#", "A", "A#", "B"};

const uint32_t samplerate = 22000;
constexpr uint32_t calculateStepSize(float frequency) {
  return (uint32_t)((pow(2, 32) * frequency) / samplerate);
}
const uint32_t stepSizes[12] = {
  calculateStepSize(261.63),  // C4
  calculateStepSize(277.18),  // C#4
  calculateStepSize(293.66),  // D4
  calculateStepSize(311.13),  // D#4
  calculateStepSize(329.63),  // E4
  calculateStepSize(349.23),  // F4
  calculateStepSize(369.99),  // F#4
  calculateStepSize(392.00),  // G4
  calculateStepSize(415.30),  // G#4
  calculateStepSize(440.00),  // A4
  calculateStepSize(466.16),  // A#4
  calculateStepSize(493.88)   // B4
};

// -------------------- Board Handshake and Role --------------------
volatile uint8_t position = 0;       // Board position (set during handshake)
volatile bool receiverMode = false;  // true if this board processes keys locally
volatile bool position_set = false;
volatile bool west = false;
volatile bool east = false;
volatile bool handshakeComplete = false;

// -------------------- FreeRTOS Objects --------------------
HardwareTimer sampleTimer(TIM1);
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;

// -------------------- Display Driver --------------------
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// -------------------- Knob Class --------------------
class Knob {
public:
  Knob(int upperLimit, int lowerLimit)
    : _rotation(0), _upperLimit(upperLimit), _lowerLimit(lowerLimit),
      _previousState(0), prevIncrement(0) {}

  int getRotation() {
    return __atomic_load_n(&_rotation, __ATOMIC_RELAXED);
  }

  void setLimits(int upperLimit, int lowerLimit){
    _upperLimit = upperLimit;
    _lowerLimit = lowerLimit;
  }

  // Expects two bits (inputA, inputB) from your matrix to do standard "quadrature decode."
  void updateRotation(int inputA, int inputB){
    int currentState = (inputB << 1) | inputA;
    int increment = 0;
    // If both bits changed at once, we reuse the previous increment.
    if (((_previousState ^ currentState) & 0x3) == 0x3) {
      increment = prevIncrement;
    }
    else {
      if (_previousState == 0b00 && currentState == 0b01) increment = 1;
      else if (_previousState == 0b11 && currentState == 0b10) increment = 1;
      else if (_previousState == 0b01 && currentState == 0b00) increment = -1;
      else if (_previousState == 0b10 && currentState == 0b11) increment = -1;
      else increment = 0;
    }
    prevIncrement = increment;

    int newRotation = __atomic_load_n(&_rotation, __ATOMIC_RELAXED) + increment;
    newRotation = constrain(newRotation, _lowerLimit, _upperLimit);
    __atomic_store_n(&_rotation, newRotation, __ATOMIC_RELAXED);
    _previousState = currentState;
  }

private:
  int _rotation;
  int _upperLimit;
  int _lowerLimit;
  int _previousState;
  int prevIncrement;
};

// Create three knobs for the receiver:
Knob volumeKnob(8, 0);    // 0..8
Knob octaveKnob(3, -3);   // e.g. -3..+3
// Knob waveKnob(2, 0);      // 0..2 => 3 waveforms

// -------------------- Matrix Functions --------------------
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

// -------------------- Handshake Functions --------------------
void checkBoards() {
  setRow(5);
  delayMicroseconds(3);
  uint8_t keyRow4 = readCols().to_ulong();
  setRow(6);
  delayMicroseconds(3);
  uint8_t keyRow5 = readCols().to_ulong();
  west = !(keyRow4 & 0x08);
  east = !(keyRow5 & 0x08);
  // if (!east) CAN_Init(true);
  // Serial.print("checkBoards: west=");
  // Serial.print(west);
  // Serial.print(" east=");
  // Serial.println(east);
}

void broadcastPosition() {
  uint8_t TX_Message[8] = {0};
  TX_Message[0] = 'H';
  TX_Message[2] = position;
  // Serial.print("Broadcasting H: position=");
  // Serial.println(position);
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void broadcastEndHandshake() {
  uint8_t TX_Message[8] = {0};
  TX_Message[0] = 'E';
  // Serial.println("Broadcasting E: Handshake complete");
  handshakeComplete = true;
  xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
}

void initialHandshake() {
  if (!west) {
    position = 0;
    position_set = true;
    receiverMode = true;  // Leftmost board processes keys locally
    // Serial.println("Initial Handshake: Leftmost board detected. Position set to 0.");
    if (!east) {
      Serial.println("No east neighbor detected. Finishing handshake by broadcasting E.");
      broadcastEndHandshake();
    } else {
      setOutMuxBit(HKOE_BIT, LOW);
      delayMicroseconds(1000000);
      broadcastPosition();
    }
  } else {
    // Serial.println("Initial Handshake: Not leftmost. Waiting for handshake message.");
  }
}

// -------------------- Waveform Generation --------------------
inline int8_t generateSample(uint32_t phase, int waveSel) {
  switch (waveSel) {
    // 0 => Saw
    case 0: {
      return (int8_t)((phase >> 24) - 128);
    }
    // 1 => Square
    case 1: {
      // Use the sign bit
      return (phase & 0x80000000) ? 127 : -128;
    }
    // 2 => Triangle
    case 2: {
      // Basic triangle from top 9 bits
      uint8_t tri = (phase >> 23) & 0xFF; // 8 bits
      if (phase & 0x80000000) tri = 255 - tri;
      return (int8_t)tri - 128;
    }
    default:
      return 0;
  }
}

// -------------------- Note On/Off Functions --------------------
void noteOn(int localKey, int baseOctave) {
  // Combine the user's base octave with board position and the octave knob
  int localOctaveKnob = __atomic_load_n(&sysState.octaveKnobVal, __ATOMIC_RELAXED);
  int effectiveOctave = baseOctave + position + localOctaveKnob;

  int relative_octave = effectiveOctave - 4;
  uint32_t step = (relative_octave >= 0)
                  ? (stepSizes[localKey] << relative_octave)
                  : (stepSizes[localKey] >> -relative_octave);

  __atomic_store_n(&currentStepSize[localKey], step, __ATOMIC_RELAXED);

  xSemaphoreTake(pressedMutex, portMAX_DELAY);
  pressed[localKey] = 1;
  xSemaphoreGive(pressedMutex);
}

void noteOff(int localKey) {
  __atomic_store_n(&currentStepSize[localKey], 0, __ATOMIC_RELAXED);

  xSemaphoreTake(pressedMutex, portMAX_DELAY);
  pressed[localKey] = 0;
  xSemaphoreGive(pressedMutex);
}

// -------------------- CAN Bus ISR and Task Functions --------------------
void CAN_RX_ISR_(void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  #ifdef TEST_RX_ISR
    // Use a test message in test mode
    RX_Message_ISR[0] = 'P';
    RX_Message_ISR[1] = 4;
    RX_Message_ISR[2] = 3;
    RX_Message_ISR[3] = 0;
    RX_Message_ISR[4] = 0;
    RX_Message_ISR[5] = 0;
    RX_Message_ISR[6] = 0;
    RX_Message_ISR[7] = 0;
    ID = 0x123;
  #else
    CAN_RX(ID, RX_Message_ISR);
  #endif
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// volatile bool txIsrFired = false;

void CAN_TX_ISR_(void) {
  #ifdef TEST_TX_ISR
    xSemaphoreGive(CAN_TX_Semaphore);
  #else
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
  #endif
  }

void CAN_TX_Task(void * pvParameters){
    uint8_t msgOut[8];

    #ifdef TEST_CANTX
      xSemaphoreGive(CAN_TX_Semaphore);
    #endif
    while(1){
      #ifndef TEST_CANTX
        xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
      #endif

      #ifdef TEST_CANTX
        msgOut[0] = 'P';
        msgOut[1] = 4;
        msgOut[2] = 3;
      #endif

      #ifndef TEST_CANTX
      xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
      #endif

      CAN_TX(0x123, msgOut);

      #ifdef TEST_CANTX
        break;
      #endif
    }
}

// -------------------- FreeRTOS Tasks --------------------

// scanKeysTask: Scans the first 3 rows for keys. If receiver, also read 3 rows for knobs.
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int baseOctave = 4;
  uint8_t TX_Message[8] = {0};
  static std::bitset<12> previousKeyState;  // For local keys 0..11

  while (1) {
    #ifndef TEST_SCANKEYS
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    #endif

    // --- 1) Scan the first 3 rows for key presses (keys 0..11) ---
    for (uint8_t row = 0; row < 3; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();

      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      for (uint8_t col = 0; col < 4; col++) {
        uint8_t localKey = row * 4 + col;
        bool keyPressed = !cols.test(col);  // active low
        // Check for changes
        if (keyPressed != previousKeyState.test(localKey)) {
          previousKeyState.set(localKey, keyPressed);

          if (receiverMode) {
            // Directly play or stop the note
            if (keyPressed) noteOn(localKey, baseOctave);
            else            noteOff(localKey);
          }
          else {
            // Sender boards: send CAN messages
            TX_Message[0] = keyPressed ? 'P' : 'R';
            TX_Message[1] = localKey;
            TX_Message[2] = position;
            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
          }
        }
        sysState.inputs[localKey] = cols.test(col);
      }
      xSemaphoreGive(sysState.mutex);
    }

    // --- 2) Only the receiver has knobs. Read them from row 3..5 if you wish. ---
    if (receiverMode) {
      // Example: row 3 => volumeKnob
      setRow(3);
      delayMicroseconds(3);
      std::bitset<4> row3cols = readCols();
      // Invert bits if your hardware uses active low
      int volA = row3cols.test(0) ? 1 : 0;
      int volB = row3cols.test(1) ? 1 : 0;
      volumeKnob.updateRotation(volA, volB);
      __atomic_store_n(&sysState.volumeKnobVal, volumeKnob.getRotation(), __ATOMIC_RELAXED);

      // row 4 => octaveKnob
      // kob 1
      setRow(3);
      delayMicroseconds(3);
      std::bitset<4> row4cols = readCols();
      int octA = row4cols.test(2) ? 1 : 0;
      int octB = row4cols.test(3) ? 1 : 0;
      octaveKnob.updateRotation(octA, octB);
      __atomic_store_n(&sysState.octaveKnobVal, octaveKnob.getRotation(), __ATOMIC_RELAXED);
      // Example: Use joystick's Y-axis to pick the waveform

      static bool joystickTriggered = false;       // Indicates if a deflection has been processed
      static int currentWaveSel = __atomic_load_n(&sysState.waveformSel, __ATOMIC_RELAXED); // Latch current waveform
      
      // Read joystick value (0..1023)
      int joyVal = analogRead(JOYY_PIN);
      
      // If not already triggered, check if we have a new deflection event
      if (!joystickTriggered) {
        if (joyVal > UPPER_THRESHOLD) {
          // Joystick pushed upward: increment waveform if not at max (assume max waveSel is 2)
          if (currentWaveSel < 2) {
            currentWaveSel++;
          }
          joystickTriggered = true;
          __atomic_store_n(&sysState.waveformSel, currentWaveSel, __ATOMIC_RELAXED);
        } else if (joyVal < LOWER_THRESHOLD) {
          // Joystick pushed downward: decrement waveform if not at minimum (assume min waveSel is 0)
          if (currentWaveSel > 0) {
            currentWaveSel--;
          }
          joystickTriggered = true;
          __atomic_store_n(&sysState.waveformSel, currentWaveSel, __ATOMIC_RELAXED);
        }
      } else {
        // Wait until the joystick returns to the neutral region before allowing a new event
        if (joyVal >= LOWER_THRESHOLD && joyVal <= UPPER_THRESHOLD) {
          joystickTriggered = false;
        }
      }
  }

    #ifdef TEST_SCANKEYS
      break;
    #endif
  }
}

// displayUpdateTask: Show volume, wave, and octave only on receiver.
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while (1) {
    #ifndef TEST_DISPLAYUPDATE
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    #endif

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    // Basic board info
    u8g2.setCursor(2, 10);
    u8g2.print("Pos: ");
    u8g2.print(position);
    // u8g2.setCursor(50, 10);
    // u8g2.print(receiverMode ? "Recv" : "Send");

    // Only receiver shows volume/octave/wave + active notes
    if (receiverMode) {
      int localVol  = __atomic_load_n(&sysState.volumeKnobVal,  __ATOMIC_RELAXED);
      int localOct  = __atomic_load_n(&sysState.octaveKnobVal,  __ATOMIC_RELAXED);
      int localWave = __atomic_load_n(&sysState.waveformSel,    __ATOMIC_RELAXED);

      // Print Volume
      u8g2.setCursor(2, 20);
      u8g2.print("Vol: ");
      u8g2.print(localVol);

      // Print Octave
      u8g2.setCursor(40, 20);
      u8g2.print("Oct: ");
      u8g2.print(localOct + 4);

      // Print Wave
      u8g2.setCursor(50, 10);
      u8g2.print("Wave: ");
      switch(localWave) {
        case 0: u8g2.print("Saw");    break;
        case 1: u8g2.print("Square"); break;
        case 2: u8g2.print("Tri");    break;
        default: u8g2.print("?");     break;
      }

      // ---------------------------
      //  Display ALL active notes
      // ---------------------------
      // Create a string to list all active notes
      String activeNotes = "";
      
      // Lock the pressed[] array while we scan
      xSemaphoreTake(pressedMutex, portMAX_DELAY);

      for (int i = 0; i < KEY_SIZE; i++) {
        if (pressed[i]) {
          if (activeNotes.length() > 0) {
            // Add a separator if we already have notes in the string
            activeNotes += ", ";
          }

          // Calculate note name & octave
          int noteIndex    = i % 12;           // which note in the octave
          int boardOffset  = i / 12;           // which board segment
          int effectiveOct = 4 + boardOffset + localOct;
          
          // Append something like "C#(4)"
          activeNotes += noteNames[noteIndex];
          activeNotes += "(";
          activeNotes += effectiveOct;
          activeNotes += ")";
        }
      }

      // Unlock the pressed[] array
      xSemaphoreGive(pressedMutex);

      // If no keys are pressed, show a placeholder
      if (activeNotes.length() == 0) {
        activeNotes = "---";
      }

      // Print the list of active notes on the display
      // Position them on a separate line or reuse existing lines
      u8g2.setCursor(2, 30);
      u8g2.print(activeNotes);
    }

    // Send the buffer to the display
    u8g2.sendBuffer();

    // Toggle LED
    digitalToggle(LED_BUILTIN);
    #ifdef TEST_DISPLAYUPDATE
      break;
    #endif
  }
}


// decodeTask: Receives CAN messages and processes handshake and key events.
void decodeTask(void * pvParameters) {
  uint8_t localRX_Message[8];
  while (1) {
    #ifndef TEST_DECODE
      xQueueReceive(msgInQ, localRX_Message, portMAX_DELAY);
    #endif

    // Save received message for debug
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy(sysState.RX_Message, localRX_Message, sizeof(localRX_Message));
    xSemaphoreGive(sysState.mutex);
    
    char msgType = (char)localRX_Message[0];
    if (msgType == 'H') {
      checkBoards();
      if (!west) {
        if (!position_set) {
          position = localRX_Message[2] + 1;
          position_set = true;
          // Serial.print("Received H message. New position set to ");
          // Serial.println(position);
          if (!east) {
            // CAN_Init(true);
            // Serial.println("Rightmost board detected. Sending E message.");
            broadcastEndHandshake();
          } else {

            // Serial.println("Propagating H message to east.");
            setOutMuxBit(HKOE_BIT, LOW);
            delayMicroseconds(1000000);
            broadcastPosition();
          }
        }
      }
      continue;
    }
    else if (msgType == 'E') {
      receiverMode = (position == 0);
      // Serial.print("Received E message. Finalizing role. Board position: ");
      // Serial.print(position);
      // Serial.print(" Receiver mode: ");
      // Serial.println(receiverMode ? "true" : "false");
      handshakeComplete = true;
      setOutMuxBit(HKOW_BIT, HIGH);
      continue;
    }
    
    // Process key events (only if in receiver mode)
    if (receiverMode) {
      const int baseOctave = 4;
      int noteIndex  = localRX_Message[1];
      int senderPos  = localRX_Message[2];
      int globalKey  = senderPos * 12 + noteIndex;
      if (msgType == 'P') {
        xSemaphoreTake(pressedMutex, portMAX_DELAY);
        if (!pressed[globalKey]) {
          pressed[globalKey] = 1;
          xSemaphoreGive(pressedMutex);

          // Combine the board's wave + octave knob
          int localOctKnob = __atomic_load_n(&sysState.octaveKnobVal, __ATOMIC_RELAXED);
          int effectiveOctave = baseOctave + senderPos + localOctKnob;
          int relOct = effectiveOctave - 4;
          uint32_t step = (relOct >= 0)
                          ? (stepSizes[noteIndex] << relOct)
                          : (stepSizes[noteIndex] >> -relOct);
          __atomic_store_n(&currentStepSize[globalKey], step, __ATOMIC_RELAXED);
        } else {
          xSemaphoreGive(pressedMutex);
        }
      }
      else if (msgType == 'R') {
        xSemaphoreTake(pressedMutex, portMAX_DELAY);
        if (pressed[globalKey]) {
          pressed[globalKey] = 0;
          xSemaphoreGive(pressedMutex);
          __atomic_store_n(&currentStepSize[globalKey], 0, __ATOMIC_RELAXED);
        } else {
          xSemaphoreGive(pressedMutex);
        }
      }
      // Serial.print("Received key event: ");
      // Serial.print(msgType);
      // Serial.print(" keyIndex: ");
      // Serial.print(noteIndex);
      // Serial.print(" baseOctave: ");
      // Serial.println(baseOctave);
    }
    
    #ifdef TEST_DECODE
      break;
    #endif
  }
}

// sampleISR: Synthesizes audio by summing contributions from all active keys,
// applying the volumeKnob (0..8) and waveformKnob (0..2).
void sampleISR() {
  // If not receiver, output silence (or pass-through).
  if (!receiverMode) {
    analogWrite(OUTR_PIN, 128);
    return;
  }

  static uint32_t phaseAcc[KEY_SIZE] = {0};
  int32_t Vout = 0;

  // Read the user's selected waveform and volume
  int localWave = __atomic_load_n(&sysState.waveformSel, __ATOMIC_RELAXED);
  int localVolume = __atomic_load_n(&sysState.volumeKnobVal, __ATOMIC_RELAXED);

  // Sum contributions from all active keys
  for (int i = 0; i < KEY_SIZE; i++) {
    if (pressed[i]) {
      phaseAcc[i] += currentStepSize[i];
      int8_t sampleVal = generateSample(phaseAcc[i], localWave);
      Vout += sampleVal;
    }
  }

  // Apply volume scaling
  // The range is 0..8 => shift right (8 - volume)
  Vout = Vout >> (8 - localVolume);

  // Convert from −128..+127 to 0..255
  int output = Vout + 128;
  if (output < 0)   output = 0;
  if (output > 255) output = 255;
  analogWrite(OUTR_PIN, output);
}

// -------------------- Setup and Loop --------------------
void setup() {
  // Set pin modes for outputs
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set pin modes for inputs
  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialize display
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);

  Serial.begin(9600);
  Serial.println("Hello World");

  // Set up sample timer for 22 kHz audio sample rate
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // Create shared resources
  sysState.mutex = xSemaphoreCreateMutex();
  pressedMutex = xSemaphoreCreateMutex();
  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(384, 8);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);

  // Initialize CAN
  // You can pass `true` or `false` if you want to fix roles at compile time
  #ifndef TEST_BUILD
    CAN_Init(false);
  #else
    CAN_Init(true);
  #endif
  setCANFilter(0x123, 0x7ff);

  // Create FreeRTOS tasks (unless disabled)
  #ifndef DISABLE_THREADS
    xTaskCreate(scanKeysTask, "scanKeys", 128, NULL, 2, NULL);
    xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, NULL);
    xTaskCreate(decodeTask, "decode", 128, NULL, 1, NULL);
    xTaskCreate(CAN_TX_Task, "CAN_TX", 128, NULL, 1, NULL);
  #endif

  // Register CAN ISRs (unless disabled)
  #ifndef DISABLE_ISRS
    CAN_RegisterRX_ISR(CAN_RX_ISR_);
    CAN_RegisterTX_ISR(CAN_TX_ISR_);
  #endif

  // Initialize handshake outputs and start handshake
  setOutMuxBit(HKOW_BIT, HIGH);
  setOutMuxBit(HKOE_BIT, HIGH);
  delayMicroseconds(2000000);


  checkBoards();
  CAN_Start();
  delayMicroseconds(2000000);
  
  initialHandshake();


  // If in test mode, run the test loops
  #ifdef TEST_SCANKEYS
    {
      uint32_t startTime = micros();
      for (int iter = 0; iter < 32; iter++){
        scanKeysTask(NULL);
      }
      // Serial.print("Total time for scanKeys function: ");
      Serial.println(micros() - startTime);
      while(1);
    }
  #endif

  #ifdef TEST_DISPLAYUPDATE
    {
      uint32_t startTime = micros();
      for (int iter = 0; iter < 32; iter++){
        displayUpdateTask(NULL);
      }
      // Serial.print("Total time for displayUpdate function: ");
      Serial.println(micros() - startTime);
      while(1);
    }
  #endif

  #ifdef TEST_DECODE
    {
      uint32_t startTime = micros();
      for (int iter = 0; iter < 32; iter++){
        decodeTask(NULL);
      }
      // Serial.print("Total time for decode function: ");
      Serial.println(micros() - startTime);
      while(1);
    }
  #endif

  #ifdef TEST_CANTX
  uint8_t testMessage[8] = {'P', 4, 3, 0, 0, 0, 0, 0};
  for(int i = 0; i < 32; i++){
    xQueueSend(msgOutQ, testMessage, portMAX_DELAY);
  }
  uint32_t startTime = micros();
  for(int iter = 0; iter < 32; iter++){
    // Serial.println("1.....");
    CAN_TX_Task(NULL);
  }
  // Serial.print("Total time for CAN_TX function: ");
  Serial.println(micros() - startTime);
  while(1);
  #endif
    
  #ifdef TEST_sampleISR
    uint32_t startTime = micros();
    for (int iter = 0; iter < 32; iter++)
    {
      sampleISR();
    }
    Serial.println(micros() - startTime);
    while (1);
  #endif
  #ifdef TEST_TX_ISR
    uint32_t startTime = micros();
    
    for (int iter = 0; iter < 32; iter++) {
      CAN_TX_ISR_();
    }
  
    // Serial.print("Total time for 32 TX ISRs: ");
    Serial.println(micros() - startTime);
    
    while (1);
  #endif

  #ifdef TEST_RX_ISR
  uint32_t startTime = micros();
    uint8_t testMessage[8] = {'P', 4, 3, 0, 0, 0, 0, 0};
    uint32_t testID = 0x123;
    for (int iter = 0; iter < 32; iter++) {
      CAN_RX_ISR_();
      }
    Serial.print("Total time for 32 RX ISRs: ");
    Serial.println(micros() - startTime);
    while (1);
  #endif

  vTaskStartScheduler();
}

void loop() {
  // Nothing here since FreeRTOS tasks take over.
}
