#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CE_PIN 5
#define CSN_PIN 6
#define jB1 1  // Joystick button 1
#define jB2 0  // Joystick button 2
#define t1 7   // Toggle switch 1
#define t2 4   // Toggle switch 1
#define b1 8   // Button 1
#define b2 9   // Button 2
#define b3 2   // Button 3
#define b4 3   // Button 4

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const byte slaveAddress[5] = { 'R', 'x', 'A', 'A', 'A' };

RF24 radio(CE_PIN, CSN_PIN);  // Create a Radio

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte key;
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};

struct Ack_Package {
  float current;
  float voltage;
};
Ack_Package ackDataPackage;  //Create a variable with the above structure

// char ackData[8];]
char currentChar[8];
char voltageChar[8];
float maxCurrent = 0.0f;
char maxCurrentChar[8];

Data_Package data;  //Create a variable with the above structure

unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 50;  // send once per 50 ms

void setup() {

  Serial.begin(9600);
  data.key = 245;
  // Set initial default values
  data.j1PotX = 127;  // Values from 0 to 255. When Joystick is in resting position, the value is in the middle, or 127. We actually map the pot value from 0 to 1023 to 0 to 255 because that's one BYTE value
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;

  // Activate the Arduino internal pull-up resistors
  pinMode(jB1, INPUT_PULLUP);
  pinMode(jB2, INPUT_PULLUP);
  pinMode(t1, INPUT_PULLUP);
  pinMode(t2, INPUT_PULLUP);
  pinMode(b1, INPUT_PULLUP);
  pinMode(b2, INPUT_PULLUP);
  pinMode(b3, INPUT_PULLUP);
  pinMode(b4, INPUT_PULLUP);

  Serial.println("SimpleTx Starting");

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 5);
  radio.openWritingPipe(slaveAddress);
  radio.enableAckPayload();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();

  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font

  display.setRotation(0);
  display.write("SMARS");
  display.display();
}

//====================

void loop() {
  data.button1 = digitalRead(b1);
  data.button2 = digitalRead(b2);
  data.button3 = digitalRead(b3);
  data.button4 = digitalRead(b4);

  // Read all analog inputs and map them to one Byte value
  data.j1PotX = map(analogRead(A1), 0, 1023, 0, 255);  // Convert the analog read value from 0 to 1023 into a BYTE value from 0 to 255
  data.j1PotY = map(analogRead(A0), 0, 1023, 0, 255);
  data.j2PotX = map(analogRead(A2), 0, 1023, 0, 255);
  data.j2PotY = map(analogRead(A3), 0, 1023, 0, 255);
  data.pot1 = map(analogRead(A7), 0, 1023, 0, 255);
  data.pot2 = map(analogRead(A6), 0, 1023, 0, 255);

  // Read all digital inputs
  data.j1Button = digitalRead(jB1);
  data.j2Button = digitalRead(jB2);
  data.tSwitch1 = digitalRead(t1);
  data.tSwitch2 = digitalRead(t2);
  data.button1 = digitalRead(b1);
  data.button2 = digitalRead(b2);
  data.button3 = digitalRead(b3);
  data.button4 = digitalRead(b4);

  currentMillis = millis();
  if (currentMillis - prevMillis >= txIntervalMillis) {
    send();
    prevMillis = millis();
  }
}

void send() {

  bool rslt;
  rslt = radio.write(&data, sizeof(data));

  display.clearDisplay();

  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font

  display.setRotation(0);

  if (rslt) {
    if (radio.isAckPayloadAvailable()) {
      radio.read(&ackDataPackage, sizeof(ackDataPackage));
      dtostrf(ackDataPackage.current, 6, 2, currentChar);
      dtostrf(ackDataPackage.voltage, 6, 2, voltageChar);
      // float ackFloat = String(ackData).toFloat();
      if (ackDataPackage.current > maxCurrent) {
        maxCurrent = ackDataPackage.current;
        dtostrf(maxCurrent, 6, 2, maxCurrentChar);
      }
      if (data.tSwitch1 == 0) {
        display.print("Max Ack: ");
        display.print(maxCurrentChar);
        display.println(" mA");
      } else {
        display.print(currentChar);
        display.print(" mA ");
        display.print(voltageChar);
        display.println(" mv");
      }
    } else {
      Serial.println("TX Ack");
      display.println("TX Ack");
    }

  } else {
    Serial.println("  Tx failed");
    display.println("TX Failed");
  }

  if (data.button1 == 0) {
        maxCurrent = 0.0f;
        dtostrf(maxCurrent, 6, 2, maxCurrentChar);
  }

  char J1[18];
  sprintf(J1, "J1(%03d,%03d) P1(%03d)", data.j1PotX, data.j1PotY, data.pot1);
  display.println(J1);
  char J2[18];
  sprintf(J2, "J2(%03d,%03d) P2(%03d)", data.j2PotX, data.j2PotY, data.pot2);
  display.println(J2);
  char Buttons[19];
  sprintf(Buttons, "B (%01d,%01d,%01d,%01d) T (%01d, %01d)", data.button1, data.button2, data.button3, data.button4, data.tSwitch1, data.tSwitch2);
  display.println(Buttons);
  display.display();
}