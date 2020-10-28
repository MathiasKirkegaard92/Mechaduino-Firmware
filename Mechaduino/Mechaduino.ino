
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

#include <Wire.h>
#include <math.h>

#include "Controller.h"
#include "Parameters.h"
#include "State.h"
#include "Utils.h"
#include "analogFastWrite.h"
#include "tests.h"

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
uint16_t debug_val   = 1000;

long last = 0;

// State variables
float   tmp_v            = 0;
float   tmp_y            = 0;
int16_t angle_rounded    = 0;
char    mode_rec         = 't';
int16_t velocity_rounded = 0;
int16_t torque           = 0;
long    now              = 0;

uint16_t calcsum(uint8_t buf[], int length) {
    uint32_t val = 0;
    for (int k = 0; k < length; k++) {
        val += buf[k];
    }
    return val;
}

void receiveI2C(int how_many) {
    if (!Wire.available()) {
        return;
    }

    uint8_t k = 0;
    while (Wire.available()) {
        rx_data[k] = Wire.read();
        k++;
        if (k > RX_DATALENGTH * 2) break;
    }
    memcpy(&torque, rx_data, RX_DATALENGTH * 2);  // int16
#if SAMPLE_ON_REQUEST
    r = torque;
    updateMotorCurrent();
    TEST1_LOW();
#else
    r = torque;
#endif
#if LATENCY_DEBUG
    if (r == debug_val) {
        TEST1_LOW();
        debug_val++;
        if (debug_val > 3600) {
            debug_val = 0;
        }
    }
#endif
}

void sendI2C() {
    TEST1_HIGH();
#if SAMPLE_ON_REQUEST
    updateAngle();
    itr_count++;
    updateVelocity();
#endif
#if USE_FIXED_POINT
    v = fixed_to_float(v_fixed);
#endif
    tmp_v = v;
    tmp_y = ybuf[0];

    int16_t offset = 0;

// Position samples
#if BUFFER_ANGLE_READINGS
    int16_t N = itr_count;
    itr_count = 0;
    if (N > TX_POS_SAMPLES) {
        N = TX_POS_SAMPLES;
    }
    memcpy(tx_data, &N, 2);
    offset += 2;

    for (int i = 0; i < N; i++) {
        tmp_y         = ybuf[N - i - 1];
        angle_rounded = int16_t(tmp * 10 + 0.5);
        memcpy(tx_data + offset, &angle_rounded, 2);
        offset += 2;
    }
#elif LATENCY_DEBUG
    memcpy(tx_data + offset, &debug_val, 2);
    offset += 2;
#else
    angle_rounded = uint16_t(tmp_y * 10 + 0.5);
    memcpy(tx_data + offset, &angle_rounded, 2);
    offset += 2;
#endif

    // Velocity sample
    velocity_rounded = (int16_t)(tmp_v * 10 + 0.5);
    memcpy(tx_data + offset, &velocity_rounded, 2);

    // Send tx_data to I2C master
    Wire.write(tx_data, TX_DATALENGTH * 2);
}

void setup() {
    SerialUSB.begin(115200);
    digitalWrite(ledPin, HIGH);  // turn LED on
    setupPins();                 // configure pins
    setupTCInterrupts();         // configure controller interrupt

    delay(3000);   // This delay seems to make it easier to establish a connection when the Mechaduino is configured to
                   // start in closed loop mode.
    serialMenu();  // Prints men    to serial monitor
    setupSPI();    // Sets up SPI for communicating with encoder
    digitalWrite(ledPin, LOW);  // turn LED off

    // spot check some of the lookup table to decide if it has been filled in
    if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0)
        SerialUSB.println("WARNING: Lookup table is empty! Run calibration");
#if !SAMPLE_ON_REQUEST
    enableTCInterrupts();
#endif

    mode   = 't';  // start in torque mode
    torque = 1;

    // I2C
#if ANTI_COGG_CAL
    cal_cogg();
#else
    Wire.begin(0);
    Wire.onReceive(receiveI2C);
    Wire.onRequest(sendI2C);
#endif

    // Tests

    // move_to_zero();
    // calibrate();

    // pos_resolution_tests(1);  // Fullstep
    // pos_resolution_tests(2);  // Halfstep
    // pos_resolution_tests(4);  // 1/4 step
    // pos_resolution_tests(8);  // 1/8 step
    // pos_resolution_tests(16); // 1/16 step
    // pos_resolution_tests(32); // 1/32 step

    // dead_zone_test();
    // saturation_test();
    // torque_test(-0, -uMAX, -5);
    // torque_test(-uMAX,0,5);
}

//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////

void loop() {
    long now = millis();
    serialCheck();

#if !SAMPLE_ON_REQUEST && CALC_VELOCITY_ASYNC
    updateVelocity();
#endif

    if (now - last > 100) {
        // velocity_test();
        // SerialUSB.println((int)(ybuf[0] * 10));
        last = now;
    }
}
