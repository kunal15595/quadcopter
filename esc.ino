#include <Servo.h>

Servo m1, m2, m3, m4;

// 1->4, 2->5, 4->7

// pin 6, 10, 11 are damaged

int ESC_1 = 4;
int ESC_2 = 5;
int ESC_3 = 9;
int ESC_4 = 7;
int ESC_MIN = 1000;
int ESC_MAX = 2000;

void esc_init() {
    m1.attach(ESC_1);
    m2.attach(ESC_2);
    m3.attach(ESC_3);
    m4.attach(ESC_4);
    delay(100);
    stop_motors();
    // enable_pitch = false;
    enable_roll = true;
    enable_pitch = true;
    // enable_roll = false;
}

void stop_motors() {
    m1.writeMicroseconds(ESC_MIN);
    m2.writeMicroseconds(ESC_MIN);
    m3.writeMicroseconds(ESC_MIN);
    m4.writeMicroseconds(ESC_MIN);
}


inline void esc_update()
{
    //The output of second pid is according to desired_rate-cur_rate i.e its proportional
    //to what extra rate must be given to achieve the desired rate.
    //PID signs are according to this extra rate
    //Reference is +ve extra rate
    //if extra rate is +ve which all speeds should gain from it and which all will loose from it

    // m1_speed = base_speed + rate_pid_result[1] - rate_pid_result[2];
    // m2_speed = base_speed + rate_pid_result[1] + rate_pid_result[2];
    // m3_speed = base_speed - rate_pid_result[1] + rate_pid_result[2];
    // m4_speed = base_speed - rate_pid_result[1] - rate_pid_result[2];

    // m1_speed = base_speed + height_pid_result - rate_pid_result[0] + rate_pid_result[1] - rate_pid_result[2];
    // m2_speed = base_speed + height_pid_result + rate_pid_result[0] + rate_pid_result[1] + rate_pid_result[2];
    // m3_speed = base_speed + height_pid_result - rate_pid_result[0] - rate_pid_result[1] + rate_pid_result[2];
    // m4_speed = base_speed + height_pid_result + rate_pid_result[0] - rate_pid_result[1] - rate_pid_result[2];

    m1_speed = base_speed - rate_pid_result[0] + rate_pid_result[1] - rate_pid_result[2];
    m2_speed = base_speed + rate_pid_result[0] + rate_pid_result[1] + rate_pid_result[2];
    m3_speed = base_speed - rate_pid_result[0] - rate_pid_result[1] + rate_pid_result[2];
    m4_speed = base_speed + rate_pid_result[0] - rate_pid_result[1] - rate_pid_result[2];

    //constrain to to the pulse width limit we can give to the motor
    m1_speed = constrain(m1_speed, min_speed, max_speed);
    m2_speed = constrain(m2_speed, min_speed, max_speed);
    m3_speed = constrain(m3_speed, min_speed, max_speed);
    m4_speed = constrain(m4_speed, min_speed, max_speed);



    if (enable_motors && enable_pitch) {
        m1.writeMicroseconds(m1_speed);
        m3.writeMicroseconds(m3_speed);
    } else {
        m1.writeMicroseconds(1000);
        m3.writeMicroseconds(1000);
    }

    if (enable_motors && enable_roll) {
        m4.writeMicroseconds(m4_speed);
        m2.writeMicroseconds(m2_speed);
    } else {
        m4.writeMicroseconds(1000);
        m2.writeMicroseconds(1000);
    }

    // m1.writeMicroseconds(1000);
    // m2.writeMicroseconds(1000);
    // m3.writeMicroseconds(1000);
    // m4.writeMicroseconds(1400);
    

}
