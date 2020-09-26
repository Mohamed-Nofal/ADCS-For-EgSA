/*
 * L298N Motor Driver Library
 */

#include "Motor.h"

Motor::Motor(){
  ENA = 11;
  IN1 = 10;
  IN2 = 9;
}

Motor::Motor(int ENA_pin, int IN1_pin, int IN2_pin){
  ENA = ENA_pin;
  IN1 = IN1_pin;
  IN2 = IN2_pin;
}

void Motor::init(){
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

// This functaion takes speed of motor from -1.0 to 1.0
// where positive numbers move in one direction and negative numbers in the other direction
void Motor::set_speed(float s){
  
  digitalWrite(IN1, s>0);
  digitalWrite(IN2, s<0);

  //next two lines are used to give motor speed for short time
  //because in low speeds motor can not start rotation
  analogWrite(ENA, 0.6*255);
  delay(40);

  analogWrite(ENA, abs(s)*255);
}
 
