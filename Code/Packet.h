/*
 * Packet handle library
 */

#ifndef _PACKET_H_
#define _PACKET_H_

#define ID 0x21
#define ACK 0x00
#define ID_ERROR 0xF0
#define C1_ERROR 0xF1
#define C2_ERROR 0xF2
#define NM_ERROR 0xF3
#define BYTES_ERROR 0xF4
#define CRC_ERROR 0xF5

#define CRC7_POLY 0x91

#define LED 13

#include <Arduino.h>
#include "Motor.h"


typedef union floatNum{
  float value;
  byte bytes[4];
}Float;

class Packet{
  public:
    Float data;
    Float gyro[3];
    Float accl[3];
    Float angl[3];
    Float temperature;
    uint8_t pack[4];
    uint8_t crc_received;
    bool new_packet = false;
    
    Packet(uint8_t *s, float *v, void (*r)());
    void init();
    bool unpacking();
  private:
       
    
    uint8_t *runStatus;
    float *value;
    void (*reset_e)();

    uint8_t crc_check();
};

#endif
 
