#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

//#define DISABLE_THREADS
//#define DISABLE_ISRs
//#define TEST_SCANKEYS

//#undef DISABLE_THREADS
//#undef DISABLE_ISRs
//#undef TEST_SCANKEYS

  volatile uint8_t keyArray[7];
  SemaphoreHandle_t keyArrayMutex;

  volatile int32_t currentStepSize;

  volatile int note_idx=0;

  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;

  SemaphoreHandle_t CAN_TX_Semaphore;

  SemaphoreHandle_t RX_MessageMutex;
  uint32_t ID; uint8_t RX_Message[8]={0};

  class Knob{
    public:
      Knob(){                     //constructor
        tracker=0;
        prevKnob_BA=0;
        last_increment=true;
      }

      void increment(){
        if (tracker<8){
          tracker++;
        }
      }

      void decrement(){
        if (tracker>0){
          tracker--;
        }
      }

      void update(uint8_t readings){
        if ((prevKnob_BA == 0b00 && readings == 0b01) || (prevKnob_BA == 0b11 && readings == 0b10)){
            increment();
            last_increment = true;
        }
        else if ((prevKnob_BA == 0b01 && readings == 0b00) || (prevKnob_BA == 0b10 && readings == 0b11)){
            decrement();
            last_increment=false;
        }
        else if((prevKnob_BA ^ readings) == 0b11){ //bitwise XOR on last 2 bits indicates impossible transitions
          //Serial.println("impossible transition");
          if (last_increment){increment();}
          else{decrement();}
        }
        prevKnob_BA = readings;
      }

      int8_t read(){
        return tracker;
      }

    private:
    int8_t tracker;
    uint8_t prevKnob_BA;
    bool last_increment;
  };

  Knob knob3, knob2, knob1, knob0;
  Knob knobs[4] = {knob3, knob2, knob1, knob0};

//Constants
  const uint32_t interval = 100; //Display update interval

  const char* noteNames [] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

  const uint32_t stepSizes [] = {
    51076057,
    54113197,
    57330935,
    60740010,
    64351799,
    68178356,
    72232452,
    76527617,
    81078186,
    85899346,
    91007187,
    96418756
  };

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

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

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

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW); // Disable row select enable
  if (rowIdx<0 || rowIdx >=7){
      Serial.println("Key Matrix row invalid, reading row 0:");
      digitalWrite(RA0_PIN, LOW);
      digitalWrite(RA1_PIN, LOW);
      digitalWrite(RA2_PIN, LOW);
  }
  else{
      digitalWrite(RA0_PIN, rowIdx & 0x01);
      digitalWrite(RA1_PIN, rowIdx & 0x02);
      digitalWrite(RA2_PIN, rowIdx & 0x04);
  }
  digitalWrite(REN_PIN, HIGH); // Enable row select enable
}

uint8_t readCols(){
  uint8_t result = 0;
  result |= digitalRead(C0_PIN) << 0;
  result |= digitalRead(C1_PIN) << 1;
  result |= digitalRead(C2_PIN) << 2;
  result |= digitalRead(C3_PIN) << 3;
  return result;
}

void sampleISR() {
    static uint32_t phaseAcc = 0;
    phaseAcc += currentStepSize;

    int32_t Vout = (phaseAcc >> 24) - 128;
    Vout = Vout >> (8 - knobs[0].read());
    analogWrite(OUTL_PIN, Vout + 128);
    analogWrite(OUTR_PIN, Vout + 128);
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

#ifndef TEST_SCANKEYS
  void scanKeysTask(void * pvParameters) {
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1){
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
      static bool key_states[12] = {false};
      static uint32_t localCurrentStepSize = 0;
      uint8_t keyArrayPreCopy[7];

      uint8_t TX_Message[8] = {0};
      
      for (int i = 0; i < 5; i++){      //for the first five rows of the matrix
        setRow(i);
        delayMicroseconds(3);
        uint8_t keys = readCols();
        keyArrayPreCopy[i]=keys;
        if (i<3){                       //if this is a key row (rows 0-2)
          for (int j = 0; j < 4; j++) {       // for every key (column of the matrix)
            note_idx = i*4+j;
            if ((keys & (1 << j)) == 0) {      // if jth column is LOW (key is pressed)
              //localCurrentStepSize = stepSizes[note_idx];
              if (!key_states[note_idx]){                            //if the key state has changed (pressed ro released)
                localCurrentStepSize = stepSizes[note_idx];
                  TX_Message[0] = 'P';
                  TX_Message[1] = 4;
                  TX_Message[2] = note_idx;
                  key_states[note_idx] = true;
                  xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
              }
            }
            else {                          //if jth column is high (key is not pressed)
              if (key_states[note_idx]){    // if the key used to be pressed, send release message
                TX_Message[0] = 'R';
                TX_Message[1] = 4;
                TX_Message[2] = note_idx;
                key_states[note_idx] = false;
                xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
                for (int k=11; k>=0; k--){
                  if (key_states[k]){
                    localCurrentStepSize = stepSizes[k];
                    break;
                  }
                }
              }
            }
          }
          if (std::all_of(key_states, key_states+12, [](bool key){return !key;})){
            localCurrentStepSize = 0;
          }
        }
        else{
          for (int j=0; j<2; j++){
            uint8_t Knob_BA = ((keys >> (2*j)) & 0b11);
            knobs[(i-3)*2+j].update(Knob_BA);
          }
        }
      }
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      std::copy(keyArrayPreCopy, keyArrayPreCopy+7, keyArray);
      xSemaphoreGive(keyArrayMutex);
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    }
  }
#endif

#ifdef TEST_SCANKEYS
  void scanKeysTask() {
    static bool key_states[12] = {false};
    static uint32_t localCurrentStepSize = 0;
    uint8_t keyArrayPreCopy[7];

    uint8_t TX_Message[8] = {0};
    
    for (int i = 0; i < 5; i++){      //for the first five rows of the matrix
      setRow(i);
      delayMicroseconds(3);
      uint8_t keys = readCols();
      keyArrayPreCopy[i]=keys;
      if (i<3){                       //if this is a key row (rows 0-2)
        for (int j = 0; j < 4; j++) {       // for every key (column of the matrix)
          note_idx = i*4+j;
          if ((keys & (1 << j)) == 0) {      // if jth column is LOW (key is pressed)
            //localCurrentStepSize = stepSizes[note_idx];
            if (!key_states[note_idx]){                            //if the key state has changed (pressed ro released)
              localCurrentStepSize = stepSizes[note_idx];
                TX_Message[0] = 'P';
                TX_Message[1] = 4;
                TX_Message[2] = note_idx;
                key_states[note_idx] = true;
                xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
            }
//else added just for timing purposes
            else{
                TX_Message[0] = 'P';
                TX_Message[1] = 4;
                TX_Message[2] = note_idx;
                key_states[note_idx] = true;
                xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
            }
          }
          else {                          //if jth column is high (key is not pressed)
            if (key_states[note_idx]){    // if the key used to be pressed, send release message
              TX_Message[0] = 'R';
              TX_Message[1] = 4;
              TX_Message[2] = note_idx;
              key_states[note_idx] = false;
              xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
              for (int k=11; k>=0; k--){
                if (key_states[k]){
                  localCurrentStepSize = stepSizes[k];
                  break;
                }
              }
            }
//else added just for timing purposes
            else{
                TX_Message[0] = 'R';
                TX_Message[1] = 4;
                TX_Message[2] = note_idx;
                key_states[note_idx] = false;
                xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
            }
          }
        }
        if (std::all_of(key_states, key_states+12, [](bool key){return !key;})){
          localCurrentStepSize = 0;
        }
      }
      else{
        for (int j=0; j<2; j++){
          uint8_t Knob_BA = ((keys >> (2*j)) & 0b11);
          knobs[(i-3)*2+j].update(Knob_BA);
        }
      }
    }
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    std::copy(keyArrayPreCopy, keyArrayPreCopy+7, keyArray);
    xSemaphoreGive(keyArrayMutex);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
#endif

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    //Update display
    uint8_t keyArrayCopy[7];

    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    std::copy(keyArray, keyArray+7, keyArrayCopy);
    Knob knob3_copy = knobs[0];
    Knob knob2_copy = knobs[1];
    Knob knob1_copy = knobs[2];
    Knob knob0_copy = knobs[3];
    xSemaphoreGive(keyArrayMutex);

    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"SIUU");  // write something to the internal memory
    u8g2.setCursor(2,20);
    u8g2.print(keyArrayCopy[2],HEX);
    u8g2.setCursor(9,20);
    u8g2.print(keyArrayCopy[1],HEX);
    u8g2.setCursor(16,20);
    u8g2.print(keyArrayCopy[0],HEX);
    if (!(keyArrayCopy[2] == 0xf && keyArrayCopy[1] == 0xf && keyArrayCopy[0] == 0xf)){
        u8g2.drawStr(2,30,noteNames[note_idx]);
    }
    u8g2.drawStr(45,10,"Vol: ");
    u8g2.setCursor(70, 10);
    u8g2.print(knob3_copy.read(), DEC);
    u8g2.drawStr(85,10,"K2: ");
    u8g2.setCursor(105, 10);
    u8g2.print(knob2_copy.read(), DEC);
    u8g2.drawStr(45,20,"K1: ");
    u8g2.setCursor(70, 20);
    u8g2.print(knob1_copy.read(), DEC);
    u8g2.drawStr(85,20,"K0: ");
    u8g2.setCursor(105, 20);
    u8g2.print(knob0_copy.read(), DEC);

    u8g2.setCursor(66,30);
    xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
    xSemaphoreGive(RX_MessageMutex);
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters){
    while(1){
        uint8_t RX_Message_precopy[8];
        xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);

        xSemaphoreTake(RX_MessageMutex, portMAX_DELAY);
        std::copy(RX_Message, RX_Message+8, RX_Message_precopy);
        xSemaphoreGive(RX_MessageMutex);
    }
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void setup() {
  // put your setup code here, to run once:

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

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  #ifndef DISABLE_ISRs
    CAN_RegisterRX_ISR(CAN_RX_ISR);
    CAN_RegisterTX_ISR(CAN_TX_ISR);
  #endif
  CAN_Start();

  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(384,8);

  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  #ifndef DISABLE_THREADS
  TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(
      scanKeysTask,		/* Function that implements the task */
      "scanKeys",		/* Text name for the task */
      64,      		/* Stack size in words, not bytes */
      NULL,			/* Parameter passed into the task */
      2,			/* Task priority */
      &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(
      displayUpdateTask,		/* Function that implements the task */
      "displayUpdate",		/* Text name for the task */
      256,      		/* Stack size in words, not bytes */
      NULL,			/* Parameter passed into the task */
      1,			/* Task priority */
      &displayUpdateHandle );	/* Pointer to store the task handle */

  TaskHandle_t decodeHandle = NULL;
    xTaskCreate(
      decodeTask,		/* Function that implements the task */
      "decode",		/* Text name for the task */
      256,      		/* Stack size in words, not bytes */
      NULL,			/* Parameter passed into the task */
      1,			/* Task priority */
      &decodeHandle );	/* Pointer to store the task handle */

  TaskHandle_t CAN_TX_Handle = NULL;
    xTaskCreate(
      CAN_TX_Task,		/* Function that implements the task */
      "CAN_TX_",		/* Text name for the task */
      256,      		/* Stack size in words, not bytes */
      NULL,			/* Parameter passed into the task */
      1,			/* Task priority */
      &CAN_TX_Handle );	/* Pointer to store the task handle */
  #endif
  
  keyArrayMutex = xSemaphoreCreateMutex();
  RX_MessageMutex = xSemaphoreCreateMutex();

  #ifndef DISABLE_THREADS
  vTaskStartScheduler();
  #endif

  

  #ifdef TEST_SCANKEYS
    uint32_t startTime = micros();
    for (int iter = 0; iter < 32; iter++) {
      scanKeysTask();
    }
    Serial.println(micros()-startTime);
    while(1);
  #endif
}

void loop() {}