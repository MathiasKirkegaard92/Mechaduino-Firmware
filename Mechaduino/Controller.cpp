// Contains TC5 Controller definition
// The main control loop is executed by the TC5 timer interrupt:

#include <SPI.h>

#include "Parameters.h"
#include "State.h"
#include "Utils.h"
#include "stdint.h"

bool USE_EXTERNAL_INTERRUPTS = false;

int clip(int in, int lo, int hi) {
    if (in > hi) {
        return hi;
    } else if (in < lo) {
        return lo;
    } else {
        return in;
    }
}

void updatePID() {
    y = ybuf[0];
#if VELOCITY_TEST
    static float test_torque = 50;
    // Sweep
    test_torque += 0.198;  // 1000 hz
    // test_torque += 0.033; // 6000 hz
    if (test_torque > 150) {
        test_torque = 30;
    }
    u = test_torque;
#else
    switch (mode) {
        case 'h':
            e = (r - v);  // error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

            ITerm += (vKi * e);  // Integral wind up limit
            if (ITerm > 200)
                ITerm = 200;
            else if (ITerm < -200)
                ITerm = -200;

            u = ((vKp * e) + ITerm - (vKd * (e - e_1)));
            u *= uMAX;
            // u = clip(u, -max_torque, max_torque);
            // SerialUSB.println(e);
            break;

        case 'x':  // position control
            yw = y + wrap_count * 360.0;
            e  = (r - yw);

            ITerm += (pKi * e);  // Integral wind up limit
            if (ITerm > 150.0)
                ITerm = 150.0;
            else if (ITerm < -150.0)
                ITerm = -150.0;

            DTerm = pLPFa * DTerm - pLPFb * pKd * (yw - yw_1);

            u = (pKp * e) + ITerm + DTerm;

            break;

        case 'v':         // velocity controller
            e = (r - v);  // error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

            ITerm += (vKi * e);  // Integral wind up limit
            if (ITerm > 200)
                ITerm = 200;
            else if (ITerm < -200)
                ITerm = -200;

            u = ((vKp * e) + ITerm - (vKd * (e - e_1)));

            break;

        case 't':  // torque control
            u = 1.0 * r;
            if (use_anti_cogg) {
                // Anti cogg
                u += v > 0 ? anti_cogg_fw[(int)(y * 10)] : anti_cogg_bw[(int)(y * 10)];
            }
            break;
        default:
            u = 0;
            break;
    }
    e_1 = e;
#endif
}

void updateMotorCurrent() {
    if (u > 0)    // Depending on direction we want to apply torque, add or subtract a phase angle of PA for max
                  // effective torque.  PA should be equal to one full step angle: if the excitation angle is the same
                  // as the current position, we would not move!
    {             // You can experiment with "Phase Advance" by increasing PA when operating at high speeds
        y += PA;  // update phase excitation angle
        if (u > uMAX)  // limit control effort
            u = uMAX;  // saturation limits max current command
    } else {
        y -= PA;        // update phase excitation angle
        if (u < -uMAX)  // limit control effort
            u = -uMAX;  // saturation limits max current command
    }

    U = abs(u);  //

    TEST2_LOW();
    output(-y, round(U));  // update phase currents
}

void calcVelocity_float(int delay) {
    float delta = ybuf[delay] - ybuf[delay + 1];
    if (delta < -180.0) {
        wrap_count += 1;
    } else if (delta > 180.0) {
        wrap_count -= 1;
    }

    yw = (ybuf[delay] + (360.0 * wrap_count));

    v    = vLPFa * v + vLPFb * (yw - yw_1);
    yw_1 = yw;
}

void calcVelocity_fixed(int delay) {
    fixed_point_t delta = ybuf_fixed[delay] - ybuf_fixed[delay + 1];
    if (delta < fixed_neg_180) {
        wrap_count += 1;
    } else if (delta > fixed_180) {
        wrap_count -= 1;
    }

    yw_fixed   = (ybuf_fixed[delay] + (fixed_360 * wrap_count));
    v_fixed    = fixed_mult(vLPFa_fixed, v_fixed) + fixed_mult(vLPFb_fixed, (yw_fixed - yw_1_fixed));
    yw_1_fixed = yw_fixed;
};

void updateVelocity() {
    while (itr_count > 0) {
        // TEST2_HIGH();
        int idx = itr_count - 1;
#if USE_FIXED_POINT
        calcVelocity_fixed(idx);
#else
        calcVelocity_float(idx);
#endif
        itr_count--;
        // TEST2_LOW();
    }
#if USE_FIXED_POINT
    v = fixed_to_float(v_fixed);
#endif
};

void updateAngle() {
    TEST2_HIGH();
    static int reading;
    reading = readEncoder();
#if USE_FIXED_POINT
    for (int i = 0; i < BUFLENGTH - 1; ++i) {  // Shift encoder buffer array
        ybuf_fixed[i + 1] = ybuf_fixed[i];
    }
    ybuf_fixed[0] = lookup_fixed[reading];
#else
    for (int i = 0; i < BUFLENGTH - 1; ++i) {  // Shift encoder buffer array
        ybuf[i + 1] = ybuf[i];
    }
#endif
    ybuf[0] = lookup[reading];
}

void TC5_Handler() {
    if (TC5->COUNT16.INTFLAG.bit.OVF == 1) {  // A counter overflow caused the interrupt
        itr_count++;

        updateAngle();
#if !CALC_VELOCITY_ASYNC
        updateVelocity();
#endif   
        updatePID();
        updateMotorCurrent();

        TC5->COUNT16.INTFLAG.bit.OVF = 1;  // writing a one clears the flag ovf flag
    }
}

void TC5_Handler_Legacy() {
    itr_count++;
    static int print_counter = 0;  // this is used by step response

    if (TC5->COUNT16.INTFLAG.bit.OVF == 1) {  // A counter overflow caused the interrupt

        TEST2_HIGH();  // digitalWrite(3, HIGH);       //Fast Write to Digital 3 for debugging

        y = lookup[readEncoder()];  // read encoder and lookup corrected angle in calibration lookup table

        if ((y - ybuf[0]) < -180.0)
            wrap_count += 1;  // Check if we've rotated more than a full revolution (have we "wrapped" around from 359
            // degrees to 0 or from 0 to 359?)
            else if ((y - ybuf[0]) > 180.0) wrap_count -= 1;

        yw = (y + (360.0 * wrap_count));  // yw is the wrapped angle (can exceed one revolution)

         // filtered velocity called "DTerm" because it is similar to derivative action in position loop;
        v = vLPFa * v + vLPFb * (yw - yw_1); 
        switch (mode) {
            case 'h':
                e = (r -
                     v);  // error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

                ITerm += (vKi * e);  // Integral wind up limit
                if (ITerm > 200)
                    ITerm = 200;
                else if (ITerm < -200)
                    ITerm = -200;

                u = ((vKp * e) + ITerm - (vKd * (e - e_1)));
                u *= uMAX;
                // u = clip(u, -max_torque, max_torque);
                // SerialUSB.println(e);
                break;

            case 'x':  // position control
                e = (r - yw);

                ITerm += (pKi * e);  // Integral wind up limit
                if (ITerm > 150.0)
                    ITerm = 150.0;
                else if (ITerm < -150.0)
                    ITerm = -150.0;

                DTerm = pLPFa * DTerm - pLPFb * pKd * (yw - yw_1);

                u = (pKp * e) + ITerm + DTerm;

                break;

            case 'v':  // velocity controller
                e = (r -
                     v);  // error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

                ITerm += (vKi * e);  // Integral wind up limit
                if (ITerm > 200)
                    ITerm = 200;
                else if (ITerm < -200)
                    ITerm = -200;

                u = ((vKp * e) + ITerm - (vKd * (e - e_1)));

                break;

            case 't':  // torque control
                u = 1.0 * r;
                if (use_anti_cogg) {
                    // Anti cogg
                    u += v > 0 ? anti_cogg_fw[(int)(y * 10 + 0.5)] : anti_cogg_bw[(int)(y * 10 + 0.5)];
                    // Constant stiction
                }
                break;
            default:
                u = 0;
                break;
        }

        // ybuf[0] = y;  // copy current value of y to previous value (y_1) for next control cycle before PA angle added
        y = ybuf[0];
        if (u > 0)  // Depending on direction we want to apply torque, add or subtract a phase angle of PA for max
                    // effective torque.  PA should be equal to one full step angle: if the excitation angle is the same
                    // as the current position, we would not move!
        {           // You can experiment with "Phase Advance" by increasing PA when operating at high speeds
            y += PA;       // update phase excitation angle
            if (u > uMAX)  // limit control effort
                u = uMAX;  // saturation limits max current command
        } else {
            y -= PA;        // update phase excitation angle
            if (u < -uMAX)  // limit control effort
                u = -uMAX;  // saturation limits max current command
        }

        U = abs(u);  //

        if (abs(e) < 0.1)
            ledPin_HIGH();  // turn on LED if error is less than 0.1
        else
            ledPin_LOW();  // digitalWrite(ledPin, LOW);

        output(-y, round(U));  // update phase currents

        // e_3 = e_2;    //copy current values to previous values for next control cycle
        // e_2 = e_1;  // these past values can be useful for more complex controllers/filters.  Uncomment as necessary
        e_1 = e;
        // u_3 = u_2;
        u_2  = u_1;
        u_1  = u;
        yw_1 = yw;
        // y_1 = y;

        if (print_yw == true) {  // for position step resonse... still under development
            print_counter += 1;
            if (print_counter >= 20) {  // print position every 5th loop (every time is too much data for plotter and
                                        // may slow down control loop
                SerialUSB.println(int(yw * 1024));  //*1024 allows us to print ints instead of floats... may be faster
                print_counter = 0;
            }
        } else if (print_v == true) {  // fo velocity step resonse...
            print_counter += 1;
            if (print_counter >= 20) {
                SerialUSB.println(v);
                print_counter = 0;
            }
        }
        TEST2_LOW();                       // for testing the control loop timing
        TC5->COUNT16.INTFLAG.bit.OVF = 1;  // writing a one clears the flag ovf flag
    }
}
