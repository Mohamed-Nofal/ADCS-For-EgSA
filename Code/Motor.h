/*
 * L298N Motor Driver Library
 */
#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include <Arduino.h>

class Motor{
  public:
    Motor();
    Motor(int ENA_pin, int IN1_pin, int IN2_pin);\
    void init();
    void set_speed(float s);
  private:
    int ENA, IN1, IN2;
};

#endif
