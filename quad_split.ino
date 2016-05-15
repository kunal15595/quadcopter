#include <ApplicationMonitor.h>
#include <EEPROM.h>
#include <EnableInterrupt.h>
#include <I2Cdev.h>
#include <avr/pgmspace.h>
#include <MemoryFree.h>
#include <stdarg.h>//for sprintf
#include <string.h>
#include <ctype.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

#define OUTPUT_READABLE_YAWPITCHROLL

Watchdog::CApplicationMonitor ApplicationMonitor;

double gps_lat;
double gps_lng;
double gps_alt;

double gps_next_lat;
double gps_next_lng;
double gps_next_alt;


int m1_speed, m2_speed, m3_speed, m4_speed;
int base_speed = 1100, max_speed = 1900, min_speed = 1000;

int angle_pid_result[3];
int rate_pid_result[3];
int desired_angle[3] = {0, 0, 0};
int desired_yaw;
boolean desired_yaw_got = false;

String in_str, in_key, in_value, in_index, serial_send = "";

boolean enable_motors = true,  enable_pitch = false, enable_roll = false;

int count_check_serial = 0, count_serial = 0;

int ch1 = 0, ch2 = 0, ch3 = 0, ch4 = 0, ch5 = 0, ch6 = 0;
int ch1_old = 0, ch2_old = 0, ch3_old = 0, ch4_old = 0;
float ch_angle_retain = 0.1;
float ch_throttle_retain = 0.9;
volatile int count_ch5 = 0;
volatile int ch1_val = 0, ch2_val = 0, ch3_val = 0, ch4_val = 0, ch5_val = 0, ch6_val = 0;
volatile unsigned long prev_ch_update;
unsigned long prev_ch_update_copy;
int receiver_lost_threashold = 100; //in ms
volatile unsigned long ch1_prev = 0, ch2_prev = 0, ch3_prev = 0, ch4_prev = 0, ch5_prev = 0, ch6_prev = 0;
boolean ch_changed = false;

int take_down_cutoff = 1500;
int take_down_gradient = 5;

byte sregRestore;

int rate_ypr[3];
float ypr[3];
int ypr_int_offset[3] = {0, -2, 2};
int int_angle[3];
int int_rate[3];
int gyro_int_offset[3] = { -21, -10, 8};
int gyro_int_raw[3];
float gyro_retain = 0.3;
int16_t gx, gy, gz;

boolean height_changed = false;
int ping_interval = 50;
unsigned long prev_ping_time = 0;
long ping_height = 0;
float actual_height = 0;
long desired_height = 0;
boolean alt_hold = false;
int height_pid_result = 0;
int height_d_term = 0;
int height_i_term = 0;
boolean bypass_height_filter = false; //for calibration
boolean started_landing = false;

int ch_diff, height_diff;


float yaw_threashold = 0.8f;
int height_threashold = 5000;
int ch_threashold = 30;


int temp_height_old = 0;

int angle_pid_constraint[3] = {4000, 3000, 3000};
int rate_pid_constraint[3] = {100, 100, 100};
int rate_i_term_calc_interval = 800;
int angle_i_term_calc_interval = 50;
float angle_i_term[3] = {0, 0, 0};
float rate_i_term[3] = {0, 0, 0};
unsigned long angle_i_prev_calc_time = 0;
unsigned long rate_i_prev_calc_time = 0;
float angle_kp[3] = {58.0f, 44.0f, 44.0f}, angle_kd[3] = {0.0f, 0.0f, 0.0f}, angle_ki[3] = {0.1f, 0.0f, 0.0f};
float rate_kp[3] = {0.035f, 0.02f, 0.02f}, rate_kd[3] = {0.0f, 0.0f, 0.0f}, rate_ki[3] = {0.0f, 0.0f, 0.0f};
int prev_angle[3] = {0, 0, 0};
int prev_rate[3] = {0, 0, 0};
int angle_d_term[3] = {0, 0, 0};
int rate_d_term[3] = {0, 0, 0};

bool follow_traj = true;


void setup() {
    ApplicationMonitor.DisableWatchdog();

    Serial.begin(9600);
    Serial1.begin(9600);
    Serial3.begin(9600);

    while (!Serial);
    while (!Serial1);
    while (!Serial3);

    mpu_init();
    esc_init();
    rc_init();
    ping_init();
    pid_init();
    bmp_init();
    gps_init();

    ApplicationMonitor.Dump(Serial);
    ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_500ms);
}

void loop() {
    unsigned long loop_start = micros();

    send_log_bluetooth();

    bmp_update();
    gps_update();
    ypr_update();
    rc_update();
    ping_update();
    pid_update();
    esc_update();


    ApplicationMonitor.IAmAlive();
    // Serial.println(micros()-loop_start);
}


/*
                               up [*] -ve pitch

                                       M 1

                                       ||
                                       ||
                                    sparkfun
                                       ||
  down [X] +ve roll    M 2 ============||============  M 4     up [*] -ve roll
                                       ||
                                       ||
                                       ||
                                       ||

                                       M 3

                               down [X] +ve pitch



                _______\
                       /
                (  yaw  )     clockwise +ve
                /______
                \


roll values


1500  27.00  25.00  5.00

*/
