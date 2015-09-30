int bluetooth_send_interval = 30; //increase this if the live graph in the app is lagging
unsigned long bluetooth_send_start = 0;

char buf[200];

inline void send_log_bluetooth(){

/*
    if (count_serial == 10 * bluetooth_send_interval) {
        count_serial = 0;
        sprintf(buf, "bs %d\r\nm1 %d\r\nm2 %d\r\n", base_speed, m1_speed, m2_speed);
        Serial1.print(buf);
         // Serial.println(millis()-bluetooth_send_start); 
         // bluetooth_send_start=millis(); 
    } 

    else if (count_serial == 9 * bluetooth_send_interval) {
        // count_serial = 0;
        sprintf(buf, "lat %lf\r\nlng %d\r\nalt %d\r\n", gps_lat, gps_lng, gps_alt);
        Serial1.print(buf);
    } else if (count_serial == 8 * bluetooth_send_interval) {
        // count_serial = 0;
        sprintf(buf, "lat %lf\r\nlng %d\r\nalt %d\r\n", gps_lat, gps_lng, gps_alt);
        Serial1.print(buf);
    } 

    else if (count_serial == 7 * bluetooth_send_interval) {
        sprintf(buf, "m3 %d\r\nm4 %d\r\nchd %d\r\nhd %d\r\n", m3_speed, m4_speed, ch_diff, height_diff);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 6 * bluetooth_send_interval) {
        sprintf(buf, "h %ld\r\nh_pid %d\r\nh_d_t %d\r\nh_i_t %d\r\n", cur_height, height_pid_result, height_d_term, height_i_term);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 5 * bluetooth_send_interval) {
        sprintf(buf, "y %d\r\np %d\r\nr %d\r\n", int_angle[0], int_angle[1], int_angle[2]);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 4 * bluetooth_send_interval) {
        sprintf(buf, "gy %d\r\ngp %d\r\ngr %d\r\n", int_rate[0], int_rate[1], int_rate[2]);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 3 * bluetooth_send_interval) {
        sprintf(buf, "chp %d\r\nchr %d\r\nchy %d\r\n", ch2, ch4, ch1);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 2 * bluetooth_send_interval) {
        sprintf(buf, "ay %d\r\nap %d\r\nar %d\r\n", angle_pid_result[0], angle_pid_result[1], angle_pid_result[2]);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == bluetooth_send_interval) {
        unsigned long cur_milli = millis();
        sprintf(buf, "cm %lu\r\nry %d\r\nrp %d\r\nrr %d\r\n", cur_milli, rate_pid_result[0], rate_pid_result[1], rate_pid_result[2]);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else {
        count_serial = count_serial + 1;
    }

    if (count_check_serial & 0x40) {
        check_serial();
    } else {
        count_check_serial = count_check_serial + 1;
    }

*/

    if (count_serial == 8 * bluetooth_send_interval) {
        count_serial = 0;
        sprintf(buf, "bs %d\r\nm1 %d\r\nm2 %d\r\n", base_speed, m1_speed, m2_speed);
        Serial1.print(buf);
        /* Serial.println(millis()-bluetooth_send_start); */
        /* bluetooth_send_start=millis(); */
    } else if (count_serial == 7 * bluetooth_send_interval) {
        sprintf(buf, "m3 %d\r\nm4 %d\r\nchd %d\r\nhd %d\r\n", m3_speed, m4_speed, ch_diff, height_diff);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 6 * bluetooth_send_interval) {
        sprintf(buf, "h %ld\r\nh_pid %d\r\nh_d_t %d\r\nh_i_t %d\r\n", cur_height, height_pid_result, height_d_term, height_i_term);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 5 * bluetooth_send_interval) {
        sprintf(buf, "y %d\r\np %d\r\nr %d\r\n", int_angle[0], int_angle[1], int_angle[2]);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 4 * bluetooth_send_interval) {
        sprintf(buf, "gy %d\r\ngp %d\r\ngr %d\r\n", int_rate[0], int_rate[1], int_rate[2]);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 3 * bluetooth_send_interval) {
        sprintf(buf, "chp %d\r\nchr %d\r\nchy %d\r\n", ch2, ch4, ch1);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == 2 * bluetooth_send_interval) {
        sprintf(buf, "ay %d\r\nap %d\r\nar %d\r\n", angle_pid_result[0], angle_pid_result[1], angle_pid_result[2]);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else if (count_serial == bluetooth_send_interval) {
        unsigned long cur_milli = millis();
        sprintf(buf, "cm %lu\r\nry %d\r\nrp %d\r\nrr %d\r\n", cur_milli, rate_pid_result[0], rate_pid_result[1], rate_pid_result[2]);
        Serial1.print(buf);
        count_serial = count_serial + 1;
    } else {
        count_serial = count_serial + 1;
    }

    if (count_check_serial & 0x40) {
        check_serial();
    } else {
        count_check_serial = count_check_serial + 1;
    }

}
