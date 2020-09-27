
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

#include <Wire.h>
#include <math.h>

#include "Parameters.h"
#include "State.h"
#include "Utils.h"
#include "analogFastWrite.h"

//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////

// I2C communication
const uint8_t I2C_BUF_SIZE   = 10;
const uint8_t CHECKSUMSIZE   = 2;
const uint8_t RX_DATALENGTH  = 1;
const uint8_t TX_DATALENGTH  = 2;
const uint8_t TX_POS_SAMPLES = 1;

uint8_t rx_data[I2C_BUF_SIZE];
uint8_t tx_data[I2C_BUF_SIZE];

uint16_t checksum_tx = 0;
uint16_t checksum_rx = 0;
uint16_t sum         = 0;
int32_t  err         = 0;

long last = 0;

// State variables
float   tmp;
int16_t angle_rounded    = 0;
char    mode_rec         = 't';
int16_t velocity_rounded = 0;
int16_t torque           = 0;
long    now              = 0;

float motor_inertia = 10.0;

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
        if (k > RX_DATALENGTH * 2) break;
    }
    memcpy(&torque, rx_data, RX_DATALENGTH * 2);  // int16
    //  SerialUSB.println(torque);
}

void sendI2C() {
    int16_t N      = itr_count;
    itr_count      = 0;
    int16_t offset = 0;

    // Position samples
    if (N > TX_POS_SAMPLES) {
        //    SerialUSB.print("Overflow in angle buffer");
        //    SerialUSB.println(N);
        N = TX_POS_SAMPLES;
    }
    //   memcpy(tx_data, &N, 2);
    //  offset += 2;

    for (int i = 0; i < N; i++) {
        tmp           = ybuf[N - i - 1];
        angle_rounded = static_cast<int16_t>(tmp * 10 + 0.5);
        memcpy(tx_data + offset, &angle_rounded, 2);
        offset += 2;
    }

    // Velocity sample
    tmp              = v;
    velocity_rounded = static_cast<int16_t>(tmp * 10 + 0.5);
    memcpy(tx_data + offset, &velocity_rounded, 2);

    // Send tx_data to I2C master
    //  Wire.write(tx_data, TX_DATALENGTH*2 + 2);
    Wire.write(tx_data, TX_DATALENGTH * 2);
}

int torqueToCurrent(int torque) {
    //  Offset with motoor inertia
    if (torque < 0) {
        return torque - motor_inertia;
    } else if (torque > 0) {
        return torque + motor_inertia;
    } else {
        return 0;
    }
}

void setup() {
    digitalWrite(ledPin, HIGH);  // turn LED on
    setupPins();                 // configure pins
    setupTCInterrupts();         // configure controller interrupt

    SerialUSB.begin(115200);
    delay(3000);   // This delay seems to make it easier to establish a connection when the Mechaduino is configured to
                   // start in closed loop mode.
    serialMenu();  // Prints menu to serial monitor
    setupSPI();    // Sets up SPI for communicating with encoder
    digitalWrite(ledPin, LOW);  // turn LED off

    // spot check some of the lookup table to decide if it has been filled in
    if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0)
        SerialUSB.println("WARNING: Lookup table is empty! Run calibration");

    enableTCInterrupts();  // uncomment this line to start in closed loop

    mode   = 't';  // start in torque mode
    torque = 1;

    // I2C
    Wire.begin(0);
    Wire.onReceive(receiveI2C);
    Wire.onRequest(sendI2C);
}

//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////

void loop() {
    long now = millis();
    serialCheck();
    //  r = torqueToCurrent(torque);
    r = torque;
    //  r = sin_1[(int)(y*10)]/4;
    //   if (now - last > 100) {
    //     SerialUSB.print("Torque: ");
    //     SerialUSB.println(r);
    //     last = now;
    //   }

    // Motor inertia test
    // if (now - last > 500) {
    //   motor_inertia += 0.1;
    //   SerialUSB.print("Motor Inertia: ");
    //   SerialUSB.println(motor_inertia);
    //   last = now;
    // }
}
