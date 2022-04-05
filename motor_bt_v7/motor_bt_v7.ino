#include <ServoTimer2.h>
//has motors, relays, t200 , doesnt all work no claw on this one, also changed relay some
// no liquid display

#include <PS3BT.h>
#include <usbhub.h>
//#include <Servo.h> i think this library is conflicting wiht the other
#include <SPI.h>

//relays
#define pinRelay1 28 //originally 6
#define pinRelay2 29 //originally 7
#define delayTime 2000

USB Usb;              

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
PS3BT PS3(&Btd); // This will just create the instance

bool printTemperature, printAngle;

#include <SoftwareSerial.h>    // used for Sabretooth & BlueTooth

// Sabertooth device driver
#include <SabertoothSimplified.h>

int SerialBaudRate = 9600;

ServoTimer2 servoleft, servoright, claw;

// Initialize the LCD screen
//int LCD_I2C_Addr = 0x27;   //I2C address for the LCD Screen (Default=0x27)
//LiquidCrystal_I2C lcd(LCD_I2C_Addr, 20, 4); // Set the LCD I2C address. Use 20 Space 4-line LCD. 

// Initialize Arduino serial communications
SoftwareSerial SWSerial(NOT_A_PIN, 23); // RX on no pin (unused), TX on pin 10 (to S1), now its pin 23 

// Initialize Sabertooth driver passing it the Arduino serial communications object
SabertoothSimplified ST(SWSerial);      // Use SWSerial as the serial port.
int ST1_S2 = 25;                         // Arduino pin attached to Sabertooth controller
int ST2_S2 = 24;                         // Arduino pin attached to Sabertooth controller


//LED test
int led = 22;

void setleftMotor(int motorNum, int power){
  int controllerNum = -1;
  int motorNumber = -1;

  switch(motorNum){
    case 1: 
      controllerNum = ST2_S2;
      motorNumber = 1;
      break;
    case 2:
      controllerNum = ST2_S2;
      motorNumber = 2;
      break;
  }
  digitalWrite(controllerNum, HIGH);
  ST.motor(motorNumber, power);
  delayMicroseconds(50);
  digitalWrite(controllerNum, LOW);
}
void setrightMotor(int motorNum, int power){
  int controllerNum = -1;
  int motorNumber = -1;

  switch(motorNum){
    case 1: 
      controllerNum = ST1_S2;
      motorNumber = 1;
      break;
    case 2:
      controllerNum = ST1_S2;
      motorNumber = 2;
      break;
  }
  digitalWrite(controllerNum, HIGH);
  ST.motor(motorNumber, power);
  delayMicroseconds(50);
  digitalWrite(controllerNum, LOW);
}

void setup() {
  SWSerial.begin(SerialBaudRate);
  Serial.begin(SerialBaudRate);

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  
  // Set Arduino pin for each sabertooth as OUTPUT
  pinMode(ST1_S2, OUTPUT);            // Arduino pin control to sabertooth
  pinMode(ST2_S2, OUTPUT);            // Arduino pin control to sabertooth
  
  pinMode(led, OUTPUT);             //for led test


  pinMode(pinRelay1, OUTPUT); //relays
  pinMode(pinRelay2, OUTPUT); //relays
  digitalWrite(pinRelay1, HIGH); //relays
  digitalWrite(pinRelay2, HIGH); //relays
   
  servoleft.write(1500);
  servoright.write(1500);
  delay(1000);
  
  claw.attach(30);
  
  //delay(5000);
  // Start Serial Communications
  SWSerial.begin(SerialBaudRate);    // Start the Sabretooth channel
  //BTSerial.begin(SerialBaudRate);    // Start the Bluetooth channel
  
  // Set up LCD 
 

}

void loop() {
  
  Usb.Task();

  int Joy1_H, Joy1_V, Joy2_H, Joy2_V;
  int pwr1, pwr2, pwr3, pwr4; 
  int MtrFL, MtrBL, MtrFR, MtrBR;

  int up;
  int down;

  int potpin = analogRead(A8); //potentiometer for claw
  int val; //read from analog pin

  val = map(potpin, 0, 1023, 750, 1500);
  claw.write(val);


  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    
    Joy1_V = PS3.getAnalogHat(LeftHatY);       // get the left vertical (Y) joystick input
    Joy1_H = PS3.getAnalogHat(LeftHatX);       // get the left horizontal (X) joystick input
    Joy2_V = PS3.getAnalogHat(RightHatY);       // get the right vertical (Y) joystick input
    Joy2_H = PS3.getAnalogHat(RightHatX);       // get the right horizontal (X) joystick input

//might have to scale these to the PS3 controller... min is 0 and max is 255 on all joysticks
//middle values are 117-137 for X and Y
    
    pwr1 = map(Joy1_V, 0, 255, 127, -127);
    pwr2 = map(Joy1_H, 0, 255, -127, 127);   //scale the left inputs for further math
    pwr3 = map(Joy2_V, 0, 255, 127, -127);   //scale the right inputs for direct use
    pwr4 = map(Joy2_H, 0, 255, -127, 127);   //scale the right inputs for direct use

    MtrFR = -pwr1;
    MtrBR = -pwr3;

    up = map(PS3.getAnalogButton(R2), 0, 225, 1500, 1900);//going up
    down= map(PS3.getAnalogButton(L2), 0, 225, 1500, 1100);//going down

    if (Joy1_V > 137 || Joy1_V < 117){
      setleftMotor(1, pwr1);
      delay(1);
      setleftMotor(2, -MtrFR);
      Serial.print(F("\npwr1: "));
      Serial.print(pwr1);
    }else{
      setleftMotor(1, 0);
      delay(1);
      setleftMotor(2, 0);
    }

    if (Joy2_V > 137 || Joy2_V < 117){
      setrightMotor(1, pwr3);
      delay(1);
      setrightMotor(2, -MtrBR);
      Serial.print(F("\npwr3: "));
      Serial.print(pwr3);
    }else{
      setrightMotor(1, 0);
      delay(1);
      setrightMotor(2, 0);
    }

  
    if (PS3.getAnalogButton(R2) > 20) {
      servoleft.write(up);
      servoright.write(up);
    } else {
      servoleft.write(1500);
      servoright.write(1500);
    }
    
    if (PS3.getAnalogButton(L2) > 20) {
      servoleft.write(down);
      servoright.write(down);
    } else {
      servoleft.write(1500);
      servoright.write(1500);
    }
    
    if (PS3.getButtonClick(PS)) {
     Serial.print(F("\r\nPS"));
     PS3.disconnect();
    }
    else{
      if (PS3.getButtonClick(TRIANGLE)) {
        Serial.print(F("\r\nTriangle"));
        PS3.setRumbleOn(RumbleLow);
        digitalWrite(led, HIGH);
      }
      if (PS3.getButtonClick(CIRCLE)) {
        Serial.print(F("\r\nCircle"));
        PS3.setRumbleOn(RumbleHigh);
      }
      if (PS3.getButtonClick(CROSS)){
        Serial.print(F("\r\nCross"));
        digitalWrite(led, LOW);
      }
        
      if (PS3.getButtonClick(SQUARE)){
        Serial.print(F("\r\nSquare"));
        digitalWrite(led, HIGH);
      }
    }
  }
 }
