#define SENDER // Uncomment this line for sender code
#define RECEIVER // Uncomment this line for receiver code
// #define DISABLE_THREADS // Uncomment this line to disable threading
// #define DISABLE_ISRS // Uncomment this line to disable ISRs
// # define TEST_SUITE // Uncomment this line to enable and run test suite
#ifdef TEST_SUITE
#define TEST_SCANKEYS
#define TEST_DISPLAYUPDATE
#define TEST_DECODE
#endif

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <array>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
#include <ES_CAN.cpp>

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

  struct {
    std::bitset<32> inputs;
    SemaphoreHandle_t mutex;
    int rotationVariable;
    uint8_t RX_Message[8] = {0};
  } sysState;
  const char* noteNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
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
  volatile uint32_t currentStepSize;

void sampleISR(){
  static uint32_t phaseAcc = 0;
  uint32_t localStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;
  int rotationVariable = __atomic_load_n(&sysState.rotationVariable, __ATOMIC_RELAXED);
  Vout = Vout >> (8 - rotationVariable);
  analogWrite(OUTR_PIN, Vout + 128);
}
HardwareTimer sampleTimer(TIM1);
int lastPressed = -1;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

class Knob {
public:
  Knob(int upperLimit, int lowerLimit) : _rotation(0), _upperLimit(upperLimit), _lowerLimit(lowerLimit), _previousState(0b00) {}

  int getRotation() {
    return __atomic_load_n(&_rotation, __ATOMIC_RELAXED);
  }

  void setLimits(int upperLimit, int lowerLimit){
    _upperLimit = upperLimit;
    _lowerLimit = lowerLimit;
  }

  void updateRotation(int inputA, int inputB){
    int increment = 0;
    int currentState = (inputB << 1) | inputA;
    if(_previousState == 0b00 && currentState == 0b01) increment = 1;
    if(_previousState == 0b11 && currentState == 0b10) increment = 1;
    if(_previousState == 0b01 && currentState == 0b00) increment = -1;
    if(_previousState == 0b10 && currentState == 0b11) increment = -1;

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
};

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

//reads the inputs from the four columns of the switch matrix (C0, C1, C2, C3)
//and return the four bits as a c++ bitset
std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

//select a given row of switch matrix by setting value of each row
//select address pin
void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

Knob knob(8, 0);
void scanKeysTask(void * pvParameters){
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static std::bitset<12> previousKeyState;
  uint32_t localCurrentStepSize = 0;
  uint8_t TX_Message[8] = {0};
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    lastPressed = -1;

    for(uint8_t row = 0; row < 4; row++){

      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();

      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      for(uint8_t col = 0; col < 4; col++){
        uint8_t keyIndex = col + row*4;
        bool keyPressed = !cols[col];
        if (keyPressed != previousKeyState[keyIndex]){
          previousKeyState[keyIndex] = keyPressed;

          #ifdef SENDER //only senders can send key press/release messages

            TX_Message[0] = keyPressed ? 'P' : 'R';
            TX_Message[1] = 4; //Octave number (can be changed)
            TX_Message[2] = keyIndex;
            TX_Message[3] = TX_Message[4] = TX_Message[5] = TX_Message[6] = TX_Message[7] = 0;

            xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);

          #endif

        }
        sysState.inputs[col + row*4] = cols[col];
        if (row < 3){
          if(keyPressed){
            lastPressed = col + row*4;
          }
        }
      }
      xSemaphoreGive(sysState.mutex);
    }

    #ifndef SENDER //only receivers can generate sound
      if(lastPressed >= 0 && lastPressed < 12){
        localCurrentStepSize = stepSizes[lastPressed];
      } 
      else{
        localCurrentStepSize = 0;
      }

      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

    #endif

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    int inputA = sysState.inputs[0 + 3*4];
    int inputB = sysState.inputs[1 + 3*4];
    xSemaphoreGive(sysState.mutex);
    knob.updateRotation(inputA, inputB);
    int knobValue = knob.getRotation();

    __atomic_store_n(&sysState.rotationVariable, knobValue, __ATOMIC_RELAXED);
    
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(35,10,"Jugg SZN!");  // write something to the internal memory
    u8g2.setCursor(2,20);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print(sysState.inputs.to_ulong(), HEX);
    xSemaphoreGive(sysState.mutex);

    u8g2.setCursor(2, 30);
    u8g2.print("Volume: ");
    int displayRotation = __atomic_load_n(&sysState.rotationVariable, __ATOMIC_RELAXED);
    u8g2.print(displayRotation);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.setCursor(66, 30);
    u8g2.print((char) sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    u8g2.print(sysState.RX_Message[2]);
    xSemaphoreGive(sysState.mutex);
    u8g2.sendBuffer(); 

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void CAN_RX_ISR_(void){
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void decodeTask(void * pvParameters){
  #ifdef RECEIVER // only receivers can decode messages
    int newStepSize;
    uint8_t localRX_Message[8];
    while(1){
      xQueueReceive(msgInQ, localRX_Message, portMAX_DELAY);

      xSemaphoreTake(sysState.mutex, portMAX_DELAY);

      memcpy(sysState.RX_Message, localRX_Message, sizeof(localRX_Message));
      xSemaphoreGive(sysState.mutex);

      char pressOrRelease = (char) sysState.RX_Message[0];
      if (pressOrRelease == 'R'){
        newStepSize = 0;
      }
      else if(pressOrRelease == 'P'){
        int keyIndex = sysState.RX_Message[2]; //note number
        int octaveNumber = sysState.RX_Message[1];
        newStepSize = stepSizes[keyIndex] >> (octaveNumber - 4);
      }

      __atomic_store_n(&currentStepSize, newStepSize, __ATOMIC_RELAXED);
    }

  #endif
}

void CAN_TX_Task(void * pvParameters){
  #ifdef SENDER
    uint8_t msgOut[8];
    while(1){
      xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
      xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
      CAN_TX(0x123, msgOut);
    }
  #endif
}

void CAN_TX_ISR_(void){
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void setup() {
  // put your setup code here, to run once:
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  #ifndef DISABLE_THREADS
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
      scanKeysTask, //function to implement the task
      "scanKeys", //text name for the text
      128, //stack size in words, not bytes
      NULL, //parameter passed into the task
      2, //task priority
      &scanKeysHandle //pointer to store the task handle
    );

    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(
      displayUpdateTask,
      "displayUpdate",
      256, //larger stack size 
      NULL,
      1, // lower priority as 100ms instead of 50 ms
      &displayUpdateHandle
    );

    TaskHandle_t decodeHandle = NULL;
    xTaskCreate(
      decodeTask,
      "decode",
      128, 
      NULL,
      1,
      &decodeHandle
    );

    TaskHandle_t CAN_TX_Handle = NULL;
    xTaskCreate(
      CAN_TX_Task,
      "CAN_TX",
      128,
      NULL,
      1,
      &CAN_TX_Handle
    );
  #endif
  sysState.mutex = xSemaphoreCreateMutex();
  msgInQ = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);

  #ifndef DISABLE_ISRS
    CAN_RegisterRX_ISR(CAN_RX_ISR_);
    CAN_RegisterTX_ISR(CAN_TX_ISR_);
  #endif
  CAN_Start();

  vTaskStartScheduler();

  #ifdef TEST_SCANKEYS
    uint32_t startTime = micros();
    for(int iter = 0; iter < 32; iter++){
      scanKeysTask();
    }
    Serial.println(micros() - startTime);
    while(1);
  #endif

  #ifdef TEST_DISPLAYUPDATE //implement method to test display update task
  #endif

  #ifdef TEST_DECODE 
  #endif
}
void loop() {

}