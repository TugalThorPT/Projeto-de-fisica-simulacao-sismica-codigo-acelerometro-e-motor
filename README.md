#include <Wire.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
// Definir os pinos
#define LEDMT 12 // pin do LED relativo ao motor
#define LEDAC 13 // pin do LED relativo ao acelerometro
#define in1 3 // pin IN1 do L298N
#define in2 2  // pin IN2 do L298N
#define enablePin 4 // pin EN1 do L298N
#define buttonStart 5 // pin do botão que liga o motor
#define buttonAC 6


byte lastbuttonStateStart;
byte lastbuttonStateAC;
byte in1State = LOW;
byte in2State = LOW;
byte enablePinState = LOW;
byte LEDMTState = LOW;
byte LEDACState = LOW;

//Evitar bouce dos botões no circuito
unsigned long LasttimebuttonStatechangedStart = millis();
unsigned long LasttimebuttonStatechangedAC = millis();
unsigned long debouceDuration = 50; //millis

bool isAccelerometerActive = false;

void setup() {
  // Pins do motor e do LED
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(LEDMT, OUTPUT);
  pinMode(LEDAC, OUTPUT);
  // Coisas relativas aos botões
  pinMode(buttonStart, INPUT);
  pinMode(buttonAC, INPUT);

  lastbuttonStateStart = digitalRead(buttonStart);
  lastbuttonStateAC = digitalRead(buttonAC);

  //Acelerometro
  Serial.begin(9600);  
   if(!accel.begin())
   {
      Serial.println("No ADXL345 sensor detected.");
      while(1);
   }
}
void loop() {
  // Liga o motor
  if (millis() - LasttimebuttonStatechangedStart >= debouceDuration){
    byte buttonStateStart = digitalRead(buttonStart);
    if (buttonStateStart != lastbuttonStateStart) {
      lastbuttonStateStart = buttonStateStart;
      if (buttonStateStart == LOW){ //Largou o botão
        if (in1State == HIGH && in2State == LOW && enablePinState == HIGH && LEDMTState == HIGH){ 
          LEDMTState = LOW;
          in1State = LOW;  
          in2State = LOW;  
          enablePinState = LOW;
        }
        else if (in1State == LOW && in2State == LOW && enablePinState == LOW && LEDMTState == LOW){
          LEDMTState = HIGH;
          in1State = HIGH;  
          in2State = LOW;  
          enablePinState = HIGH; 
        }
        digitalWrite(LEDMT, LEDMTState);
        digitalWrite(in1, in1State);  
        digitalWrite(in2, in2State);  
        digitalWrite(enablePin, enablePinState); 
      }
    }   
  }


  // Acelerometro
  if (millis() - LasttimebuttonStatechangedAC >= debouceDuration){
    byte buttonStateAC = digitalRead(buttonAC);
    if (buttonStateAC != lastbuttonStateAC) {
      lastbuttonStateAC = buttonStateAC;
      if (buttonStateAC == LOW){ //Largou o botão
        if (LEDACState == HIGH){ 
          LEDACState = LOW;
          isAccelerometerActive = false;
        }
        else if (LEDACState == LOW){
          LEDACState = HIGH; 
          isAccelerometerActive = true;   
        }
        digitalWrite(LEDAC, LEDACState);
      }
    }   
  }
  // Acelerometro
  if (isAccelerometerActive) {
    sensors_event_t event;
    accel.getEvent(&event);
    Serial.print(event.acceleration.x);
    Serial.println();
    delay(50); // Impede overflow de valores do acelerometro
  }  
}
