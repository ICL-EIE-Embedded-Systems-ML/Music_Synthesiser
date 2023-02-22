#include <Arduino.h>
#include <U8g2lib.h>

//Constants
  const uint32_t interval = 100; //Display update interval
  const double A = 440; 
  const double twelfth_root_2 = 1.05946309436;
  volatile int32_t currentStepSize;
  const int32_t stepSizes[] = {
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 9)) * 22000.0), // C
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 8)) * 22000.0), // C#
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 7)) * 22000.0), // D
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 6)) * 22000.0), // D#
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 5)) * 22000.0), // E
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 4)) * 22000.0), // F
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 3)) * 22000.0),  // F#
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 2)) * 22000.0),  // G
    (int32_t) (4294967296.0 * A / (pow(twelfth_root_2, 1)) * 22000.0),  // G#
    (int32_t) (4294967296.0 * A / 22000.0),  // A
    (int32_t) (4294967296.0 * A * pow(twelfth_root_2, 1) / 22000.0),  // A#
    (int32_t) (4294967296.0 * A * pow(twelfth_root_2, 2) / 22000.0),  // B
};

  uint8_t keyArray[7];

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
  const int OUTL_PIN = A;
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
}

uint8_t readCols(){
  uint8_t result = 0;   
  result |= digitalRead(C0_PIN) << 0;
  result |= digitalRead(C1_PIN) << 1;
  result |= digitalRead(C2_PIN) << 2;
  result |= digitalRead(C3_PIN) << 3;

  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW); // Disable row select enable
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH); // Enable row select enable
}

void scanSwitchMatrix(){
  for(uint8_t row; row<3; row++){
    setRow(row);

    delayMicroseconds(100); 

    uint8_t cols = readCols();
    keyArray[row] = cols; 
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;
  
  if (millis() > next) {
    next += interval;

    scanSwitchMatrix();
    uint16_t hexData = (keyArray[2] << 8) | (keyArray[1] << 4) | keyArray[0];
    Serial.print("Keys: ");
    for (int i = 0; i < 3; i++) {
      Serial.print(keyArray[i], HEX);
    }
    Serial.println();
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    u8g2.setCursor(2,20);
    uint8_t keys = readCols();
    u8g2.setCursor(2,20);
    u8g2.print(keys,HEX); 
    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }

}