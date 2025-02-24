#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <array>
#include <STM32FreeRTOS.h>

//Constants
  const uint32_t interval = 100; //Display update interval

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
  phaseAcc += currentStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}
HardwareTimer sampleTimer(TIM1);

struct {
  std::bitset<32> inputs;
} sysState;
int lastPressed = -1;
//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

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

void scanKeysTask(void * pvParameters){
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    lastPressed = -1;
    for(uint8_t row = 0; row < 3; row++){
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();

      for(uint8_t col = 0; col < 4; col++){
        sysState.inputs[col + row*4] = cols[col];
        if(!cols[col]){
          lastPressed = col + row*4;
        }
      }
    }
    if(lastPressed >= 0 && lastPressed < 12){
      currentStepSize = stepSizes[lastPressed];
    } 
    else{
      currentStepSize = 0;
    }
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory
    u8g2.setCursor(2,20);
    u8g2.print(sysState.inputs.to_ulong(), HEX);
    if (lastPressed >= 0 && lastPressed < 12) {
      u8g2.setCursor(2, 30);
      u8g2.print(noteNames[lastPressed]);  // Display note name
    } else {
      u8g2.setCursor(2, 30);
      u8g2.print("None");
    }
    u8g2.sendBuffer(); 

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
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
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask, //function to implement the task
    "scanKeys", //text name for the text
    64, //stack size in words, not bytes
    NULL, //parameter passed into the task
    2, //task priority
    &scanKeysHandle ); //pointer to store the task handle

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,
    "displayUpdate",
    256, //larger stack size 
    NULL,
    1, // lower priority as 100ms instead of 50 ms
    &displayUpdateHandle
  );
  vTaskStartScheduler();
}
void loop() {
  

}