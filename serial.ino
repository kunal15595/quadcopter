
inline void check_serial() {

    Serial.print(m1_speed); Serial.print("\t");
    Serial.print(m2_speed); Serial.print("\t");
    Serial.print(m3_speed); Serial.print("\t");
    Serial.print(m4_speed); Serial.print("\t");
    Serial.println(" -- ");

    serial_send = "";

    if (Serial1.available() > 0) {
        // read the value
        char ch = Serial1.read();

        if (ch != ';' && ch != '=') {
            // Add the character to the in_str
            in_str += ch;
        } else if (ch == '=') {
            in_key = in_str;
            in_str = "";
        } else if (ch == ';') {
            boolean wrong_command = false;
            in_value = in_str;

            // Convert the string to an integer
            float val = in_value.toFloat();

            if (in_key == "a_c") {
                angle_pid_constraint[1] = angle_pid_constraint[2] = val;
            } else if (in_key == "y_a_c") {
                angle_pid_constraint[0] = val;
            } else if (in_key == "r_c") {
                rate_pid_constraint[1] = rate_pid_constraint[2] = val;
            } else if (in_key == "y_r_c") {
                rate_pid_constraint[0] = val;
            } else if (in_key == "a_m") {
                angle_i_term_calc_interval = val;
            } else if (in_key == "r_m") {
                rate_i_term_calc_interval = val;
            } else if (in_key == "pe") {
                CH2_EFFECT = val;
                /* Serial.println(CH2_EFFECT); */
            } else if (in_key == "re") {
                CH4_EFFECT = val;
                /* Serial.println(CH4_EFFECT); */
            } else if (in_key == "ye") {
                CH1_EFFECT = val;
                /* Serial.println(CH1_EFFECT); */
            } else if (in_key == "tl") {
                CH3_MIN_EFFECT = val;
                /* Serial.println(CH3_MIN_EFFECT); */
            } else if (in_key == "th") {
                CH3_MAX_EFFECT = val;
                /* Serial.println(CH3_MAX_EFFECT); */
            } else if (in_key == "r_kd") {
                rate_kd[1] = rate_kd[2] = val;
                /* Serial.println(rate_kd[1]); */
            } else if (in_key == "r_kp") {
                rate_kp[1] = rate_kp[2] = val;
                /* Serial.println(rate_kp[1]); */
            } else if (in_key == "r_ki") {
                rate_ki[1] = rate_ki[2] = val;
                //Should reset the previously accumulated error
                //cause they are calculated with a differnet ki
                clear_i_terms(1);
                /* Serial.println(rate_ki[1]); */
            } else if (in_key == "y_r_kd") {
                rate_kd[0] = val;
                /* Serial.println(rate_kd[0]); */
            } else if (in_key == "y_r_kp") {
                rate_kp[0] = val;
                /* Serial.println(rate_kp[0]); */
            } else if (in_key == "y_r_ki") {
                rate_ki[0] = val;
                clear_i_terms(2);
                /* Serial.println(rate_ki[0]); */
            } else if (in_key == "a_kd") {
                angle_kd[1] = angle_kd[2] = val;
                /* Serial.println(angle_kd[1]); */
            } else if (in_key == "a_kp") {
                angle_kp[1] = angle_kp[2] = val;
                /* Serial.println(angle_kp[1]); */
            } else if (in_key == "a_ki") {
                angle_ki[1] = angle_ki[2] = val;
                clear_i_terms(1);
                /* Serial.println(angle_ki[1]); */
            } else if (in_key == "y_a_kd") {
                angle_kd[0] = val;
                /* Serial.println(angle_kd[0]); */
            } else if (in_key == "y_a_kp") {
                angle_kp[0] = val;
                /* Serial.println(angle_kp[0]); */
            } else if (in_key == "y_a_ki") {
                angle_ki[0] = val;
                clear_i_terms(2);
                /* Serial.println(angle_ki[0]); */
            } else if (in_key == "car") {
                ch_angle_retain = val;
            } else if (in_key == "ctr") {
                ch_throttle_retain = val;
            } else if (in_key == "yar") {
                yaw_average_retain = val;
            } else if (in_key == "g_r") {
                gyro_retain = val;
            } else if (in_key == "h_kp") {
                height_kp = val;
            } else if (in_key == "h_kd") {
                height_kd = val;
            } else if (in_key == "h_ki") {
                height_ki = val;
                clear_i_terms(3);
            } else if (in_key == "h_m") {
                height_i_term_calc_interval = val;
            } else if (in_key == "h_c") {
                height_pid_constraint = val;
            } else if (in_key == "h_d") {
                take_down_gradient = val;
            } else if (in_key == "h_th") {
                height_threashold = val;
            } else if (in_key == "y_th") {
                yaw_threashold = val;
            } else if (in_key == "ch_th") {
                ch_threashold = val;
            } else if (in_key == "t_d_c") {
                take_down_cutoff = val;
            } else {
                wrong_command = true;
            }

            if (!wrong_command) {
                Serial1.println(in_key + "=%; " + in_value);
                Serial.println(in_key + "=%; " + in_value);
            } else {
                serial_send += "Error with the input ";
                serial_send += in_key;
                //It will print on terminal
                Serial.println(serial_send);
            }
            in_key = in_value = in_str = in_index = "";
        }
    }
}
