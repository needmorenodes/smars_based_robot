// #include <SMARS_Motor_Control.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define CE_PIN 7
#define CSN_PIN 10

#define HG7881_B_IA 9  // D9 --> Motor B Input A --> MOTOR B +
#define HG7881_B_IB 6  // D6 --> Motor B Input B --> MOTOR B -

#define HG7881_A_IA 5  // D5 --> Motor A Input A --> MOTOR A +
#define HG7881_A_IB 3  // D3 --> Motor A Input B --> MOTOR A -

// functional connections
#define MOTOR_A_IA HG7881_A_IA
#define MOTOR_A_IB HG7881_A_IB

// functional connections
#define MOTOR_B_IA HG7881_B_IA
#define MOTOR_B_IB HG7881_B_IB

#define DIR_DELAY 1000  // brief delay for abrupt motor changes

const byte thisSlaveAddress[5] = { 'R', 'x', 'A', 'A', 'A' };

RF24 radio(CE_PIN, CSN_PIN);

// SMARS_Motor_Control motorControl(MOTOR_A_IA, MOTOR_A_IB, MOTOR_B_IA, MOTOR_B_IB);

char dataReceived[10];  // this must match dataToSend in the TX
bool newData = false;

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

Data_Package data;  //Create a variable with the above structure

struct Motor_Power {
  int pwm;
  int direction;
};

byte max_power = 255;

Motor_Power apower;
Motor_Power bpower;
int xMap, yMap;
bool forward = true;
bool left = true;

//===========

void setup() {

  Serial.begin(9600);
  data.key = 0;
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

  apower.pwm = 0;
  apower.direction = 0;
  bpower.pwm = 0;
  bpower.direction = 0;

  // motorControl.begin();
  pinMode(MOTOR_A_IA, OUTPUT);
  pinMode(MOTOR_A_IB, OUTPUT);
  pinMode(MOTOR_B_IA, OUTPUT);
  pinMode(MOTOR_B_IB, OUTPUT);

  Serial.println("SimpleRx Starting");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, thisSlaveAddress);
  radio.startListening();

  Serial.println("STOP");
  digitalWrite(MOTOR_A_IA, LOW);  // PWM speed = fast
  analogWrite(MOTOR_A_IB, 0);
  digitalWrite(MOTOR_B_IA, LOW);  // PWM speed = fast
  analogWrite(MOTOR_B_IB, 0);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();

  display.setTextSize(1);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setCursor(0, 0);              // Start at top-left corner
  display.cp437(true);                  // Use full 256 char 'Code Page 437' font

  display.setRotation(2);
  display.write("SMARS");
  display.display();
  // delay(2000);
}

//=============

void loop() {
  getData();
  // showData();
  if (data.tSwitch2 == 0) {
    displayDebug();
  } else {
    displayText("SMARS");
  }
  if (data.key = 245) {
    processJoystick();
    if (apower.direction == 0) {
      digitalWrite(MOTOR_A_IA, LOW);  // PWM speed = fast
      analogWrite(MOTOR_A_IB, apower.pwm);
    } else if (apower.direction == 1) {
      analogWrite(MOTOR_A_IA, apower.pwm);
      digitalWrite(MOTOR_A_IB, LOW);  // PWM speed = fast
    }

    if (bpower.direction == 0) {
      digitalWrite(MOTOR_B_IA, LOW);  // PWM speed = fast
      analogWrite(MOTOR_B_IB, bpower.pwm);
    } else if (bpower.direction == 1) {
      analogWrite(MOTOR_B_IA, bpower.pwm);
      digitalWrite(MOTOR_B_IB, LOW);  // PWM speed = fast
    }

  } else {
    Serial.println("STOP");
    digitalWrite(MOTOR_A_IA, LOW);  // PWM speed = fast
    analogWrite(MOTOR_A_IB, 0);
    digitalWrite(MOTOR_B_IA, LOW);  // PWM speed = fast
    analogWrite(MOTOR_B_IB, 0);
  }
}

//==============

void getData() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    newData = true;
  } else {
    //IF LAST RECIEVED OVER 1 SECOND AGO RESET KEY
    // displayText("No Data\nRecieved!");
  }
}

void showData() {
  if (newData == true) {
    Serial.print("Data received ");
    Serial.print("|");
    Serial.print(data.key);
    Serial.print("|");
    Serial.print(data.button1);
    Serial.print("|");
    Serial.print(data.button2);
    Serial.print("|");
    Serial.print(data.button3);
    Serial.print("|");
    Serial.print(data.button4);
    Serial.print("|");
    Serial.print(data.pot1);
    Serial.print("|");
    Serial.print(data.pot2);
    Serial.print("|");
    Serial.print(data.j1PotX);
    Serial.print("|");
    Serial.print(data.j1PotY);
    Serial.print("|");
    Serial.print(data.j2PotX);
    Serial.print("|");
    Serial.print(data.j2PotY);
    Serial.print("|");
    Serial.print(data.tSwitch1);
    Serial.print("|");
    Serial.print(data.tSwitch2);
    // Serial.print("|");
    // Serial.print(power.aPWM);
    // Serial.print("|");
    // Serial.print(power.bPWM);
    Serial.println("|");
    newData = false;
  }
}

void displayText(const char* text) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.write(text);
  display.display();
}

void displayDebug() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.write("DEBUG MODE\n");
  char pwma[17];
  sprintf(pwma, "aPWM: %03d adir: %01d", apower.pwm, apower.direction);
  display.println(pwma);

  char pwmb[17];
  sprintf(pwma, "bPWM: %03d bdir: %01d", bpower.pwm, bpower.direction);
  display.println(pwmb);
  display.display();
}

void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
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
}

void processJoystick() {
  yMap = map(data.j1PotY, 0, 255, -255, 255);
  xMap = map(data.j2PotX, 0, 255, -255, 255);

  if (yMap > 0) {
    yMap = map(yMap, 0, 255, 0, 255);
    forward = true;
  } else {
    forward = false;
    yMap = map(yMap, -255, 0, 255, 0);
  }
  if (yMap < 40) {
    yMap = 0;
  }

  if (xMap > -10 && xMap < 10) {
    xMap = 0;
  }
  if (xMap > 0) {
    xMap = map(xMap, 0, 255, 0, 255);
    left = true;
  } else if (xMap < 0) {
    xMap = map(xMap, -255, 0, 255, 0);
    left = false;
  } else {
    xMap = 0;
  }

  if(yMap == 0) {
    if (xMap != 0 ) {
      if (left) {
        apower.direction = 0;
        bpower.direction = 0;
      } else {
        apower.direction = 1;
        bpower.direction = 1;
      }
      apower.pwm = xMap;
      bpower.pwm = xMap;
    } else {
      apower.pwm = 0;
      bpower.pwm = 0;
    }
  } else {
    if (forward){
        apower.direction = 1;
        bpower.direction = 0;
    } else {
        apower.direction = 0;
        bpower.direction = 1;
    }

    if (xMap == 0){
      apower.pwm = yMap;
      bpower.pwm = yMap;
    } else {
      if (left) {
        apower.pwm = yMap - map(xMap, 0, 255, 0, yMap);
        bpower.pwm = yMap;
      } else {
        apower.pwm = yMap;
        bpower.pwm = yMap - map(xMap, 0, 255, 0, yMap);
      }
    }
  }
}