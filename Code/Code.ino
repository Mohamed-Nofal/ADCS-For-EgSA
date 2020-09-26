/*
 * Satellite Main code
 */
#include "Motor.h"
#include "Packet.h"
#include "I2Cdev.h"
#include "MPU6050.h"


bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 g;
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t temperatue;
uint8_t runStatus = 0x00;
float req_value = 0.0,Kp=0.5,Ki=0,Kd=1,e=0, e_prev=0,E=0,E_dot=0;

void reset_e();

MPU6050 mpu;
Motor motor(11, 10, 9);
Packet packet(&runStatus, &req_value, &reset_e);

void mpu_init();
void PID_angle();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mpu_init();
  motor.init();
  packet.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(packet.new_packet){//check for new packet
    if(packet.unpacking()){//unpack the command
    }else{
      Serial.println("Invalid Data");
    }
  }

  //get angles in each loop
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetGyro(&g, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    packet.temperature.value = mpu.getTemperature() / 340.0 + 36.53;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    packet.gyro[0].value = g.x / 16.4;
    packet.gyro[1].value = g.y / 16.4;
    packet.gyro[2].value = g.z / 16.4;
    packet.accl[0].value = aa.x / 8192.0 * 9.8;
    packet.accl[1].value = aa.y / 8192.0 * 9.8;
    packet.accl[2].value = aa.z / 8192.0 * 9.8;
    packet.angl[0].value = ypr[2] * 180/M_PI + 180.0;
    packet.angl[1].value = ypr[1] * 180/M_PI + 180.0;
    packet.angl[2].value = ypr[0] * 180/M_PI + 180.0;
  }

  switch(runStatus){
    case 0x01:
      motor.set_speed(req_value);
      runStatus = 0x00;
      break;
    case 0x02:
      PID_angle();
      break;
    default:
      break;
  }
}

// Interrupt on receiving Data from bluetooth
void serialEvent1(){
  char c = Serial1.read();
  if(c == 0x21){
    Serial1.readBytes(packet.pack, 4);
    Serial1.readBytes(packet.data.bytes, packet.pack[3]);
    packet.crc_received = Serial1.readBytes(&packet.crc_received, 1);
    packet.new_packet = true;
    return;
  }else{
  }
  return;
}


// initialization function for MPU6050
void mpu_init(){
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    mpu.initialize();
    
    mpu.testConnection();

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        
    }
}


//control function
void PID_angle(){
  e = atan2(sin(req_value - ypr[0]), cos(req_value - ypr[0]));
  E = E + e;
  E_dot = e - e_prev;

  motor.set_speed(Kp * e + Ki * E + Kd * E_dot);

  e_prev = e;
}


// reset error values
void reset_e(){
  e = 0;
  E = 0;
  E_dot = 0;
  e_prev = 0;
}
