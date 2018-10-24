#include <LiquidCrystal_I2C.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <LiquidCrystal_I2C.h>
#include <Console.h>

String input;
int rpm;
int sensorPin = 0;
float tempC;
float tempF;

float prevC = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *stepper = AFMS.getStepper(200, 2);
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  lcd.init();
  lcd.backlight();

  Serial.begin(9600);

  AFMS.begin();
  stepper->setSpeed(0);
}

void loop() {
  String tempCS = String(tempC);
  lcd.setCursor(2, 0);
  int reading = analogRead(sensorPin);
  float voltage = reading * 5.0;
  voltage /= 1024.0;
  tempC = (voltage - 0.5) * 100;
  if (tempC != prevC) {
    String tempCS = String(tempC);
    
    lcd.setCursor(4, 0);
    lcd.print("Temperature:");
    lcd.setCursor(2,1);
    lcd.print(tempCS + " degrees C");
    Serial.println(tempCS + " degrees C");
    tempF = (tempC * 9.0 / 5.0) + 32;
    String tempFS = String(tempF);
    lcd.setCursor(2,2);
    lcd.print(tempFS + " degrees F");
    Serial.println(tempFS + " degrees F");
  }
  if (Serial.available() > 0) {
    rpm = Serial.readString().toInt();
    stepper->setSpeed(rpm);
    Serial.println("Speed changed to" + String(rpm));
  } else {
    stepper->step(100, FORWARD, SINGLE);
  }
  prevC = tempC;
}
