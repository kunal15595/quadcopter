unsigned long height_i_term_calc_time = 0;
int height_i_term_calc_interval = 500;
long prev_height = 0;
int height_pid_constraint = 100;
float height_kp = 0.016f, height_ki = 0, height_kd = 0.004f;

const unsigned long TIME_TILL_PROPER_YAW = 2000UL;

void pid_init() {
    unsigned long yaw_tune_start = millis();
    while (millis() - yaw_tune_start < TIME_TILL_PROPER_YAW)
        ypr_update();//we could have used delay but then we could have gotten fifo overflow or stale values
    desired_yaw_update();
    prev_angle[0] = desired_yaw;
}

inline void pid_update() {
    // pid equation is ( + / - ){kp*(desired_val - cur_val) + i_term - d_term}
    // where i_term = i_term + ki*(desired_val - cur_val)
    // and d_term = kd*(cur_val - prev_val)
    // notice the opposite sign of d_term in the PID equation...dont forget it
    // the forward ( + / - ) depends on how the result of PID is used
    // P depends on the present error, I on the accumulation of past errors, and
    // D is a prediction of future errors, based on current rate of change.

    if (millis() - angle_i_prev_calc_time > angle_i_term_calc_interval) {
        angle_i_term[0] += - angle_ki[0] * (desired_angle[0] - int_angle[0]);
        angle_i_term[1] += angle_ki[1] * (desired_angle[1] - int_angle[1]);
        angle_i_term[2] += - angle_ki[2] * (desired_angle[2] - int_angle[2]);

        angle_d_term[0] = - angle_kd[0] * (int_angle[0] - prev_angle[0]);
        angle_d_term[1] = angle_kd[1] * (int_angle[1] - prev_angle[1]);
        angle_d_term[2] = - angle_kd[2] * (int_angle[2] - prev_angle[2]);

        prev_angle[0] = int_angle[0];
        prev_angle[1] = int_angle[1];
        prev_angle[2] = int_angle[2];

        angle_i_prev_calc_time = millis();
    }

    angle_pid_result[0] = -angle_kp[0] * (desired_angle[0] - int_angle[0]) + angle_i_term[0] - angle_d_term[0];
    angle_pid_result[1] = angle_kp[1] * (desired_angle[1] - int_angle[1]) + angle_i_term[1] - angle_d_term[1];
    angle_pid_result[2] = -angle_kp[2] * (desired_angle[2] - int_angle[2]) + angle_i_term[2] - angle_d_term[2];

    angle_pid_result[0] = constrain(angle_pid_result[0], - angle_pid_constraint[0], angle_pid_constraint[0]);
    angle_pid_result[1] = constrain(angle_pid_result[1], - angle_pid_constraint[1], angle_pid_constraint[1]);
    angle_pid_result[2] = constrain(angle_pid_result[2], - angle_pid_constraint[2], angle_pid_constraint[2]);

    if (millis() - rate_i_prev_calc_time > rate_i_term_calc_interval) {
        rate_i_term[0] += rate_ki[0] * (angle_pid_result[0] - int_rate[0]);
        rate_i_term[1] += rate_ki[1] * (angle_pid_result[1] - int_rate[1]);
        rate_i_term[2] += rate_ki[2] * (angle_pid_result[2] - int_rate[2]);

        rate_d_term[0] = rate_kd[0] * (int_rate[0] - prev_rate[0]);
        rate_d_term[1] = rate_kd[1] * (int_rate[1] - prev_rate[1]);
        rate_d_term[2] = rate_kd[2] * (int_rate[2] - prev_rate[2]);

        prev_rate[0] = int_rate[0];
        prev_rate[1] = int_rate[1];
        prev_rate[2] = int_rate[2];

        rate_i_prev_calc_time = millis();
    }

    rate_pid_result[0] = rate_kp[0] * (angle_pid_result[0] - int_rate[0]) + rate_i_term[0] - rate_d_term[0];
    rate_pid_result[1] = rate_kp[1] * (angle_pid_result[1] - int_rate[1]) + rate_i_term[1] - rate_d_term[1];
    rate_pid_result[2] = rate_kp[2] * (angle_pid_result[2] - int_rate[2]) + rate_i_term[2] - rate_d_term[2];

    rate_pid_result[0] = constrain(rate_pid_result[0], - rate_pid_constraint[0], rate_pid_constraint[0]);
    rate_pid_result[1] = constrain(rate_pid_result[1], - rate_pid_constraint[1], rate_pid_constraint[1]);
    rate_pid_result[2] = constrain(rate_pid_result[2], - rate_pid_constraint[2], rate_pid_constraint[2]);

    if (alt_hold) {
        if (millis() - height_i_term_calc_time > height_i_term_calc_interval) {
            height_i_term += height_ki * (desired_height - ping_height);
            height_d_term = height_kd * (ping_height - prev_height);
            prev_height = ping_height;
            height_i_term_calc_time = millis();
        }
        height_pid_result = height_kp * (desired_height - ping_height) + height_i_term - height_d_term;
        height_pid_result = constrain(height_pid_result, - height_pid_constraint, height_pid_constraint);
    } else {
        height_pid_result = 0;
        height_i_term = 0;
        height_d_term = 0;
    }

}

void clear_i_terms(int which) {
    if (which == 0) { //Clear all
        angle_i_term[0] = 0;
        angle_i_term[1] = 0;
        angle_i_term[2] = 0;
        rate_i_term[0] = 0;
        rate_i_term[1] = 0;
        rate_i_term[2] = 0;
        height_i_term = 0;
    } else if (which == 1) { //clear pitch/roll
        angle_i_term[1] = 0;
        angle_i_term[2] = 0;
        rate_i_term[1] = 0;
        rate_i_term[2] = 0;
    } else if (which == 2) { //clear yaw
        angle_i_term[0] = 0;
        rate_i_term[0] = 0;
    } else if (which == 3) {
        height_i_term = 0;
    }
}
