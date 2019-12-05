
/*
  -------------------------------------------------------------
  Mechaduino 0.1 & 0.2 Firmware  v0.1.5
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  All Mechaduino related materials are released under the
  Creative Commons Attribution Share-Alike 4.0 License
  https://creativecommons.org/licenses/by-sa/4.0/

  Many thanks to all contributors!
  --------------------------------------------------------------

  Controlled via a SerialUSB terminal at 115200 baud.

  Implemented serial commands are:

 s  -  step
 d  -  dir
 p  -  print [step number] , [encoder reading]

 c  -  calibration routine
 e  -  check encoder diagnositics
 q  -  parameter query

 x  -  position mode
 v  -  velocity mode
 t  -  torque mode

 y  -  enable control loop
 n  -  disable control loop
 r  -  enter new setpoint

 j  -  step response
 k  -  edit controller gains -- note, these edits are stored in volatile memory and will be reset if power is cycled
 g  -  generate sine commutation table
 m  -  print main menu


  ...see serialCheck() in Utils for more details

*/
// VERSION
// enum Motor_type{
//  17HS16-2004S = 0;
//  17HS4401S = 1;

// }

// const Motor_type MOTOR = 17HS4401S;

#include "Utils.h"
#include "Parameters.h"
#include "State.h"
#include "analogFastWrite.h"
// #include "spiSlave.h"
#include <Wire.h>
#include <math.h>


//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////

const uint8_t I2C_BUF_SIZE = 10;
const uint8_t CHECKSUMSIZE = 2;

uint8_t rx_data[I2C_BUF_SIZE];
uint8_t tx_data[I2C_BUF_SIZE];

uint16_t checksum_tx = 0;
uint16_t checksum_rx = 0;
uint16_t sum = 0;

float tmp;
int16_t angle_rounded = 0;
char mode_rec = 't';
float velocity = 0;
int16_t torque = 0;
int32_t err = 0;


uint16_t calcsum(uint8_t buf[], int length) {
  uint32_t val = 0;
  for (int k = 0; k < length; k++) {
    val += buf[k];
  }
  return val;
}


void receiveI2C(int how_many) {
  uint8_t k = 0;
  while (Wire.available()) {
    rx_data[k] = Wire.read();
    k++;
    if (k > I2C_BUF_SIZE + CHECKSUMSIZE) break;
  }
  memcpy(&checksum_rx, rx_data + I2C_BUF_SIZE, 2);

  if (checksum_rx != calcsum(rx_data, I2C_BUF_SIZE)) { // error in recieved data
    // do something ?
    err++;
  } else {
    memcpy(&torque, rx_data, 2); // int16
    memcpy(&velocity, rx_data + 2, 4); // float
    memcpy(&mode_rec, rx_data + 6, 1); // char
    if (mode_rec == 't' || mode_rec == 'x' || mode_rec == 'v') {
      mode = mode_rec;
    };
  }

}



void sendI2C() {
  // Angle
  tmp = y_1;
  angle_rounded = static_cast<int16_t> (tmp * 10 + 0.5);
  // angle_rounded = static_cast<int16_t> (read_angle() * 10 + 0.5);
  memcpy(tx_data, &angle_rounded, 2);

  // wrapped angle_rounded
  tmp = yw - PA;
  angle_rounded = static_cast<int16_t>(tmp * 10 + 0.5);
  memcpy(tx_data + 2, &angle_rounded, 2); // uint

  // Velocity
  tmp = v;
  memcpy(tx_data + 4, &tmp, 4); // float

  // Checksum
  checksum_tx = calcsum(tx_data, I2C_BUF_SIZE);
  memcpy(tx_data + I2C_BUF_SIZE, &checksum_tx, 2); // uint16

  // Send tx_data to I2C master
  Wire.write(tx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
}



void setup()        // This code runs once at startup
{

  digitalWrite(ledPin, HIGH);       // turn LED on
  setupPins();                      // configure pins
  setupTCInterrupts();              // configure controller interrupt

  SerialUSB.begin(115200);
  delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.
  serialMenu();                     // Prints menu to serial monitor
  setupSPI();                       // Sets up SPI for communicating with encoder
  digitalWrite(ledPin, LOW);        // turn LED off

  // spot check some of the lookup table to decide if it has been filled in
  if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0)
    SerialUSB.println("WARNING: Lookup table is empty! Run calibration");

  // Uncomment the below lines as needed for your application.
  // Leave commented for initial calibration and tuning.

//      configureStepDir();           // Configures setpoint to be controlled by step/dir interface
//      configureEnablePin();         // Active low, for use wath RAMPS 1.4 or similar
  enableTCInterrupts();         // uncomment this line to start in closed loop
//      mode = 'x';                   // start in position mode
  mode = 't';                   // start in torque mode


// I2C
  Wire.begin(8);
  Wire.onReceive(receiveI2C);
  Wire.onRequest(sendI2C);
}

int sign(float x) {
  return (x < 0) ? -1 : (x > 0);
}

float fold360(float x) {
  return x < 0 ? 360 - x : x;
}


//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////


void loop()                 // main loop
{

  serialCheck();              //must have this execute in loop for serial commands to function
  switch (mode) {
  case 't':
    r = torque;
    break;
  case 'v':
    if (abs(r - velocity) > 1) r = velocity;
    break;
  }

  // float angleDiff = 180 - y;
  // int sgn = sign(angleDiff);
  // angleDiff *= sgn;

////  Torque Detent
//  if(angleDiff < angleThreshold){
//    r = sgn*(-1)*(angleThreshold-angleDiff)*30;
//  } else {
//    r = 0;
//  }

//  Torque Snap

//  if(angleDiff < angleThreshold){
//    mode = 'x';
//    r = 180;
//    delay(500);
//    mode = 't';
//  } else {
//    mode = 't';
//    r = 0;
//  }



// Snap detents - postion controlled
//if (u > torqueThreshold & abs(e) > detentAngle/2) { // Snap to grid/detent at half the angle
//r -= detentAngle;
//delay (50);
//}
//else if (u < - torqueThreshold & abs(e) > detentAngle/2){
//r += detentAngle;
//delay (50);
//}



  // if (round(y) != round(y_last)) {
  //   if (mode == 'x') SerialUSB.println(y);
  //   y_last = y;
  // }

  // // Max prints

  // if (round(v) != round(v_last)) {
  //   v_last = v;
  //   if (mode == 'v') serialUSB.println(v);
  // }


// Torque curve
  // time = micros();
  // if ((time - time_last) > (1.0 / Fs * 1000000)) {
  //   idx = round((fold360(y) * 10.0 * detents)) % 3600; // Angle to table idx
  //   r = filter(sin_1[idx] / 1024.0 * 80);
  //   time_last = time;
  // }


// Switching noise test
  // time = micros();
  // if ((time - time_last) > (1.0 / freq * 1000000)) {
  //   r = state ? 115 : 0;
  //   state = !state;
  //   time_last = time;
  // }

  // if ((time - time_last_print) > 1000000) {
  //   freq += 5;
  //   SerialUSB.println(freq);
  //   time_last_print = time;
  // }


  //r=0.1125*step_count;      //Don't use this anymore. Step interrupts enabled above by "configureStepDir()", adjust step I2C_BUF_size ("stepangle")in parameters.cpp

}
