
int ch1_temp_old, ch2_temp_old, ch3_temp_old, ch4_temp_old;

const int CH1_PIN = 13;
const int CH2_PIN = 12;
const int CH3_PIN = 11;
const int CH4_PIN = 10;
const int CH5_PIN = 50;
const int CH6_PIN = 52;

const int CH1_MAX = 1808;
const int CH1_MIN = 1208;
const int CH2_MAX = 1820;
const int CH2_MIN = 1216;
const int CH3_MAX = 2020;
const int CH3_MIN = 1016;
const int CH4_MAX = 1808;
const int CH4_MIN = 1208;
const int CH5_MAX = 2000;
const int CH5_MIN = 1000;
const int CH6_MAX = 2000;
const int CH6_MIN = 1000;

float CH1_EFFECT = 1.5f;

// actual range is 0.40f
float CH2_EFFECT = 0.10f;
float CH4_EFFECT = 0.10f;

// actual range is 1400 to 1700;
int CH3_MIN_EFFECT = 1400;
int CH3_MAX_EFFECT = 1700;

// actual dev is 150
int CH3_MAX_DEV = 150;

int BASE_SPEED = 1550;

const int CH5_EFFECT = 100;
const int CH6_EFFECT = 100;
const int CH3_MIN_CUTOFF = 50;

void rc_init() {
    pinMode(CH1_PIN, INPUT);
    pinMode(CH2_PIN, INPUT);
    pinMode(CH3_PIN, INPUT);
    pinMode(CH4_PIN, INPUT);
    pinMode(CH5_PIN, INPUT);
    pinMode(CH6_PIN, INPUT);

    prev_ch_update = millis();

    enableInterrupt(CH1_PIN, ch1_change, CHANGE);
    enableInterrupt(CH2_PIN, ch2_change, CHANGE);
    enableInterrupt(CH3_PIN, ch3_change, CHANGE);
    enableInterrupt(CH4_PIN, ch4_change, CHANGE);
    enableInterrupt(CH5_PIN, ch5_change, CHANGE);
    enableInterrupt(CH6_PIN, ch6_change, CHANGE);
}

void rc_update() {
    boolean receiver_lost = millis() - prev_ch_update_copy > receiver_lost_threashold;
    int ch1_temp = 0, ch2_temp = 0, ch3_temp = 0, ch4_temp = 0;
    if (ch_changed || receiver_lost) {
        ch_changed = false;
        sregRestore = SREG;
        cli(); // clear the global interrupt enable flag
        ch1_temp = ch1_val;
        ch2_temp = ch2_val;
        ch3_temp = ch3_val;
        ch4_temp = ch4_val;
        ch5 = ch5_val;
        ch6 = ch6_val;
        prev_ch_update_copy = prev_ch_update;
        SREG = sregRestore; // restore the status register to its previous value

        boolean landing_condition = ch5 > CH5_EFFECT / 2;
        //in the case the when transmitter signal is lost... failsafe values
        //needs to be loaded which might be far away from current values
        boolean motors_off = ch6 < 0;
        boolean exception = motors_off || landing_condition;

        int ch1_diff, ch2_diff, ch3_diff, ch4_diff;

        ch1_diff = abs(ch1_temp - ch1_temp_old);
        ch2_diff = abs(ch2_temp - ch2_temp_old);
        ch3_diff = abs(ch3_temp - ch3_temp_old);
        ch4_diff = abs(ch4_temp - ch4_temp_old);

        ch1_temp_old = ch1_temp;
        ch2_temp_old = ch2_temp;
        ch3_temp_old = ch3_temp;
        ch4_temp_old = ch4_temp;

        ch_diff = max(max(ch1_diff, ch2_diff), max(ch3_diff, ch4_diff));

        if (close_by(ch1_temp, ch1, ch_threashold) || exception) {
            ch1 = ch1_temp;
        }

        if (close_by(ch2_temp, ch2, ch_threashold) || exception) {
            ch2 = ch2_temp;
        }

        if (close_by(ch3_temp, ch3, ch_threashold) || exception) {
            ch3 = ch3_temp;
        }

        if (close_by(ch4_temp, ch4, ch_threashold) || exception) {
            ch4 = ch4_temp;
        }

        ch1 = (ch1 == 0) ? (CH1_MAX + CH1_MIN) / 2 : ch1;
        ch2 = (ch2 == 0) ? (CH2_MAX + CH2_MIN) / 2 : ch2;
        ch3 = (ch3 == 0) ? CH3_MIN_EFFECT : ch3;
        ch4 = (ch4 == 0) ? (CH4_MAX + CH4_MIN) / 2 : ch4;
        ch5 = (ch5 == 0) ? (CH5_MAX + CH5_MIN) / 2 : ch5;
        ch6 = (ch6 == 0) ? (CH6_MAX + CH6_MIN) / 2 : ch6;


        ch1 = constrain(ch1, CH1_MIN, CH1_MAX);
        ch2 = constrain(ch2, CH2_MIN, CH2_MAX);
        ch3 = constrain(ch3, CH3_MIN, CH3_MAX);
        ch4 = constrain(ch4, CH4_MIN, CH4_MAX);
        ch5 = constrain(ch5, CH5_MIN, CH5_MAX);
        ch6 = constrain(ch6, CH6_MIN, CH6_MAX);

        ch1 = map(ch1, CH1_MIN, CH1_MAX, -CH1_EFFECT * YPR_RATIO, CH1_EFFECT * YPR_RATIO);
        ch2 = map(ch2, CH2_MIN, CH2_MAX, -CH2_EFFECT * YPR_RATIO, CH2_EFFECT * YPR_RATIO);
        ch3 = map(ch3, CH3_MIN, CH3_MAX, -CH3_MAX_DEV, CH3_MAX_DEV);
        ch4 = map(ch4, CH4_MIN, CH4_MAX, -CH4_EFFECT * YPR_RATIO, CH4_EFFECT * YPR_RATIO);
        ch5 = map(ch5, CH5_MIN, CH5_MAX, -CH5_EFFECT, CH5_EFFECT);
        ch6 = map(ch6, CH6_MIN, CH6_MAX, -CH6_EFFECT, CH6_EFFECT);

        // low pass filter
        ch1 = ch1_old * (1 - ch_angle_retain) + ch_angle_retain * ch1;
        ch2 = ch2_old * (1 - ch_angle_retain) + ch_angle_retain * ch2;
        ch3 = ch3_old * (1 - ch_throttle_retain) + ch_throttle_retain * ch3;
        ch4 = ch4_old * (1 - ch_angle_retain) + ch_angle_retain * ch4;

        ch1_old = ch1;
        ch2_old = ch2;
        ch3_old = ch3;
        ch4_old = ch4;

        if (receiver_lost) {
            //landing
            ch1 = 0;
            ch2 = 0;
            ch4 = 0;
            ch5 = CH5_EFFECT;
        }


        //- is there so that the channel values correspond to angle sign
        desired_angle[0] = desired_yaw - ch1;
        desired_angle[1] = 0 - ch2;
        desired_angle[2] = 0 - ch4;

        if (ch6 > 0) {
            bypass_height_filter = false;
            desired_yaw_got = false;
            if (ch5 > CH5_EFFECT / 2) {
                // left up, right down ==> land or height balance
                if (!started_landing) {
                    desired_height = ping_height;
                    started_landing = true;
                }
                alt_hold = true;
                if (ping_height > take_down_cutoff) {
                    desired_height -= take_down_gradient;
                } else {
                    alt_hold = false;
                    base_speed = CH3_MIN_EFFECT;
                    enable_motors = false;
                    clear_i_terms(0);
                    bypass_height_filter = true; //for calibration
                }
                follow_traj = false;
            } else if (ch5 < -CH5_EFFECT / 2) {
                // left up, right up
                started_landing = false;
                if (!alt_hold)
                    desired_height = ping_height;
                alt_hold = true;
                enable_motors = true;
                base_speed = BASE_SPEED + ch3;
                follow_traj = false;
            } else {
                // left up, right mid ==> follow trajectory
                follow_traj = true;
                started_landing = false;
                alt_hold = false;
                enable_motors = true;
                base_speed = BASE_SPEED + ch3;
            }
        } else {
            bypass_height_filter = true; //for calibration
            alt_hold = false;
            enable_motors = false;
            started_landing = false;

            // So the error do not accumulate while sitting
            clear_i_terms(0);
            
            if (ch5 > CH5_EFFECT / 2) { 
                // left down, right down
                desired_yaw_got = false;
            } else if (ch5 < -CH5_EFFECT / 2) {
                // left down, right up ==> desired yaw
                desired_yaw_update();
            } else { 
                // left down, right mid
                desired_yaw_got = false;
            }
        }
    }

}

void ch1_change() {
    if (digitalRead(CH1_PIN) == HIGH) {
        ch1_prev = micros();
    } else {
        ch1_val = micros() - ch1_prev;
        ch_changed = true;
        prev_ch_update = millis();
    }
}
void ch2_change() {
    if (digitalRead(CH2_PIN) == HIGH) {
        ch2_prev = micros();
    } else {
        ch2_val = micros() - ch2_prev;
        ch_changed = true;
        prev_ch_update = millis();
    }
}
void ch3_change() {
    if (digitalRead(CH3_PIN) == HIGH) {
        ch3_prev = micros();
    } else {
        ch3_val = micros() - ch3_prev;
        ch_changed = true;
        prev_ch_update = millis();
    }
}
void ch4_change() {
    if (digitalRead(CH4_PIN) == HIGH) {
        ch4_prev = micros();
    } else {
        ch4_val = micros() - ch4_prev;
        ch_changed = true;
        prev_ch_update = millis();
    }
}
void ch5_change() {
    if (digitalRead(CH5_PIN) == HIGH) {
        ch5_prev = micros();
    } else {
        ch5_val = micros() - ch5_prev;
        ch_changed = true;
        prev_ch_update = millis();
    }
}
void ch6_change() {
    if (digitalRead(CH6_PIN) == HIGH) {
        ch6_prev = micros();
    } else {
        ch6_val = micros() - ch6_prev;
        ch_changed = true;
        prev_ch_update = millis();
    }
}
