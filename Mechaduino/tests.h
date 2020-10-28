
#ifndef __TESTS_H__
#define __TESTS_H__

#include "Utils.h"

const int N_AVERAGE   = 20;

void pos_resolution_tests(int microsteps) {
    move_to_zero();
    delay(1000);

    // Position resolution test
    SerialUSB.print("\%Starting position test with ");
    SerialUSB.print(microsteps);
    SerialUSB.println(" microsteps");
    SerialUSB.print("microstep_");
    SerialUSB.print(microsteps);
    SerialUSB.println(" = [");

    disableTCInterrupts();
    for (int i = 0; ceil(i / microsteps) <= spr - 1; ++i) {
        // oneStep();
        SerialUSB.print(float(i) / microsteps);
        SerialUSB.print(",");
        SerialUSB.print(readEncoder());
        SerialUSB.print(",");
        SerialUSB.println(lookup[readEncoder()]);
        output(aps * i / microsteps, (int)(0.5 * uMAX));
        delay(50);
    }
    SerialUSB.println("]");
    SerialUSB.println("\%Position test finished");
    enableTCInterrupts();
    delay(2000);
}
void dead_zone_test() {
    int limit = uMAX / 2;
    // dead zone test test
    SerialUSB.println("\%Starting dead zone test");
    SerialUSB.println("forwards = [");
    for (int i = 0; i < limit; ++i) {
        r = i;
        for (int j = 0; j < 5; j++) {
            SerialUSB.print(i);
            SerialUSB.print(", ");
            SerialUSB.println(v);
            delay(50);
        }
    }
    SerialUSB.println("];");
    SerialUSB.println("backwards = [");
    for (int i = limit; i > -limit; --i) {
        r = i;
        for (int j = 0; j < 5; j++) {
            SerialUSB.print(i);
            SerialUSB.print(", ");
            SerialUSB.println(v);
            delay(50);
        }
    }
    SerialUSB.println("];");
    SerialUSB.println("forwards_return = [");
    for (int i = -limit; i <= 0; ++i) {
        r = i;
        for (int j = 0; j < 5; j++) {
            SerialUSB.print(i);
            SerialUSB.print(", ");
            SerialUSB.println(v);
            delay(50);
        }
    }

    SerialUSB.println("];");
    SerialUSB.println("\%saturation test finished");
}

void saturation_test() {
    int limit = uMAX / 2;
    // dead zone test test
    SerialUSB.println("\%Starting dead zone test");
    SerialUSB.println("forwards = [");
    for (int i = 0; i < limit; i += 2) {
        r = i;
        // for (int j = 0; j < 5; j++) {
        //     SerialUSB.print(i);
        //     SerialUSB.print(", ");
        //     SerialUSB.println(v);
        //     delay(50);
        // }
        delay(200);
    }
    r = 0;
    delay(1000);
    SerialUSB.println("];");
    SerialUSB.println("\%saturation test finished");
}

void torque_test(int low, int high, int inc) {
    for (int i = low; abs(i) < abs(high); i += inc) {
        r = i;
        SerialUSB.println(r);
        delay(5000);
        r = 0;
        delay(500);
    }
}

void velocity_test() {
    static long now   = 0;
    static int last = 0;
    static int  count = 0;
    static int  wait  = 0;
    static double sum = 0;
    static float vbuf[N_AVERAGE]  = {0}; 
    now               = millis();
    if (now - last > 500 + wait) {
        wait = 0;
        if (count == N_AVERAGE) {
            int32_t deviation = 0;
            int32_t variance  = 0;
            int32_t avg       = sum / count;
            for (int k = 0; k < count; ++k) {
                variance += abs(vbuf[k] - avg);
            }
            variance = variance / count;
            SerialUSB.print(avg);
            SerialUSB.print(",");
            SerialUSB.println(variance);
            count = 0;
            sum = 0;
            wait  = 200;
        } else {
#if USE_FIXED_POINT
            vbuf[count] = fixed_to_float(v_fixed);
#else
            vbuf[count] = v;
#endif
            sum += vbuf[count];
            count++;
        }
        last = now;
    }
}

void cal_cogg() {
    mode = 'x';
    r    = 0;
    SerialUSB.println("Starting anti cogging calibration for cw direction");
    delay(2000);
    int   N                  = 5;
    int   variance_threshold = 30;
    float sum                = 0;
    int   effort[N];
    float variance = 0;
    for (int i = 0; i < 3600; ++i) {
        r     = i / 10.0;
        int j = 0;
        delay(100);
        while (j < N) {
            if (abs(e) < 0.1) {
                sum += u;
                effort[j] = u;
                delay(1);
                j++;
            }
        }
        int avg = (int)(sum / (float)(N));
        for (int k = 0; k < N; ++k) {
            float deviation = (effort[k] - avg);
            variance += deviation * deviation;
        }
        variance /= N;
        if (variance < variance_threshold) {  // Log data-point
            SerialUSB.print(r);
            SerialUSB.print(",");
            SerialUSB.print(ybuf[0]);
            SerialUSB.print(",");
            SerialUSB.print(avg);
            SerialUSB.print(",");
            SerialUSB.println(variance);
        } else {  // Repeat measurement
            i--;
        }

        variance = 0;
        sum      = 0;
    }
    r = 360;
    SerialUSB.println("Starting anti cogging calibration for ccw direction");
    delay(2000);
    for (int i = 3600; i > 0; --i) {
        r     = i / 10.0;
        int j = 0;
        delay(100);
        while (j < N) {
            if (abs(e) < 0.1) {
                sum += u;
                effort[j] = u;
                delay(1);
                j++;
            }
        }
        int avg = (int)(sum / (float)(N));
        for (int k = 0; k < N; ++k) {
            float deviation = (effort[k] - avg);
            variance += deviation * deviation;
        }
        variance /= N;
        if (variance < variance_threshold) {  // Log data-point
            SerialUSB.print(r);
            SerialUSB.print(",");
            SerialUSB.print(ybuf[0]);
            SerialUSB.print(",");
            SerialUSB.print(avg);
            SerialUSB.print(",");
            SerialUSB.println(variance);
        } else {  // Repeat measurement
            i++;
        }

        variance = 0;
        sum      = 0;
    }
}

#endif
