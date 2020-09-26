/*
 * Packet handle library
 */
#include "Packet.h"

Packet::Packet(uint8_t *s, float *v, void (*r)()){
  runStatus = s;
  value = v;
  reset_e = r;
}

void Packet::init(){
  Serial1.begin(9600);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  return;
}


//unpack packets
bool Packet::unpacking(){
  new_packet = false;

  // reverse float bytes
  byte temp;
  temp = data.bytes[0];
  data.bytes[0] = data.bytes[3];
  data.bytes[3] = temp;
  temp = data.bytes[1];
  data.bytes[1] = data.bytes[2];
  data.bytes[2] = temp;

  switch(pack[0]){
    case 0x01:
      switch(pack[1]){
        case 0x0A:
          if(pack[2] == 0x00 && pack[3] == 0x04){
            Serial.println(data.value);
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }else{
              Serial1.write(ACK);
              *runStatus = 0x01;
              *value = data.value/100.0;
              (*reset_e)();
              return true;
            }
          }else{
            Serial1.write(NM_ERROR); 
            return false;
          }
        case 0x0B:
          if(pack[2] == 0x00 && pack[3] == 0x04){
            Serial.println(data.value);
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }
            
            Serial1.write(ACK);
            *runStatus = 0x01;
            *value = -data.value/100.0;
            (*reset_e)();
            return true;
          }else{
            Serial1.write(NM_ERROR);
            return false;
          }
        case 0x0C:
          if(pack[2] == 0x00 && pack[3] == 0x04){
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }
            
            Serial1.write(ACK);
            *runStatus = 0x02;
            *value = data.value*M_PI/180.0 - 180.0;
            (*reset_e)();
            return true;
          }else{
            Serial1.write(NM_ERROR); 
            return false;
          }
        default:
          Serial1.write(C2_ERROR);
          return false;
      }
      
      
    case 0x02:
      switch(pack[1]){
        case 0x0A:
          if(pack[2] == 0x00 && pack[3] == 0x00){
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }else{
            
              Serial1.write(ACK);
              digitalWrite(LED, HIGH);
              return true;
            }
          }else{
            Serial1.write(NM_ERROR);
            return false;
          }
        case 0x0B:
          if(pack[2] == 0x00 && pack[3] == 0x00){
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }else{
            
              Serial1.write(ACK);
              digitalWrite(LED, LOW);
              return true;
            }
          }else{
            Serial1.write(NM_ERROR);
            return false;
          }
        default:
          Serial1.write(C2_ERROR);     
          return false;
      }
      
      
    case 0x03:
      switch(pack[1]){
        case 0x0A:
          if(pack[2] == 0x0C && pack[3] == 0x00){
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }else{
              Serial1.write(ACK);
              
              Serial1.write(gyro[0].bytes, 4);
              Serial1.write(gyro[1].bytes, 4);
              Serial1.write(gyro[2].bytes, 4);
              return true;
            }
          }else{
            Serial1.write(NM_ERROR);
            return false;
          }
        case 0x0B:
          if(pack[2] == 0x0C && pack[3] == 0x00){
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }
            
            Serial1.write(ACK);
            
            Serial1.write(accl[0].bytes, 4);
            Serial1.write(accl[1].bytes, 4);
            Serial1.write(accl[2].bytes, 4);
            return true;
          }else{
            Serial1.write(NM_ERROR);
            return false;
          }
        case 0x0C:
          if(pack[2] == 0x04 && pack[3] == 0x00){
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }
            
            Serial1.write(ACK);
            
            Serial1.write(temperature.bytes, 4);
            return true;
          }else{
            Serial1.write(NM_ERROR);
            return false;
          }
        case 0x0D:
          if(pack[2] == 0x0C && pack[3] == 0x00){
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }
            
            Serial1.write(ACK);
            
            Serial1.write(angl[0].bytes, 4);
            Serial1.write(angl[1].bytes, 4);
            Serial1.write(angl[2].bytes, 4);
            return true;
          }else{
            Serial1.write(NM_ERROR);
            return false;
          }
        case 0x0E:
          if(pack[2] == 0x28 && pack[3] == 0x00){
            if(crc_check() != crc_received){
              Serial1.write(CRC_ERROR);
              return false;
            }
            
            Serial1.write(ACK);

            Serial1.write(gyro[0].bytes, 4);
            Serial1.write(gyro[1].bytes, 4);
            Serial1.write(gyro[2].bytes, 4);
            Serial1.write(accl[0].bytes, 4);
            Serial1.write(accl[1].bytes, 4);
            Serial1.write(accl[2].bytes, 4);
            Serial1.write(temperature.bytes, 4);
            Serial1.write(angl[0].bytes, 4);
            Serial1.write(angl[1].bytes, 4);
            Serial1.write(angl[2].bytes, 4);
            
            return true;
          }else{
            Serial1.write(NM_ERROR);
            return false;
          }
        default:
          Serial1.write(C2_ERROR);     
          return false;
      }
    default:
      Serial1.write(C1_ERROR);
      return false;
  }
}


// CRC Function check
uint8_t Packet::crc_check(){
  int i;
  uint8_t  j, crc = 0;
  //crc for device ID
  //return crc_received;
  crc ^= ID;
  for (j = 0; j < 8; j++)
  {
    if ((crc & 0x80) != 0){
      crc = (crc << 1) ^ CRC7_POLY;
    }
    else{
      crc <<= 1;
    }
  }

  
  //crc for packet
  for (i = 0; i < 4 ; i++)
  {
    crc ^= pack[i];
    for (j = 0; j < 8; j++)
    {
      if ((crc & 0x80) != 0){
        crc = (crc << 1) ^ CRC7_POLY;
      }
      else{
        crc <<= 1;
      }
    }
  }

  //crc for data if avaialable
  for (i = pack[3]-1; i >= 0; i--)
  {
    crc ^= data.bytes[i];
    for (j = 0; j < 8; j++)
    {
      if ((crc & 0x80) != 0){
        crc = (crc << 1) ^ CRC7_POLY;
      }
      else{
        crc <<= 1;
      }
    }
  }

  
  return crc;
}
