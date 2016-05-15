
const int PING_ECHO_PIN = 53;
const int PING_TRIG_PIN = 28;
volatile unsigned long ping_prev = 0, ping_val = 0;

void ping_init() {
    enableInterrupt(PING_ECHO_PIN, ping_change, CHANGE);
    pinMode(PING_TRIG_PIN, OUTPUT);
    pinMode(PING_ECHO_PIN, INPUT);
}

void ping_change() {
    if (digitalRead(PING_ECHO_PIN) == HIGH) {
        ping_prev = micros();
    } else {
        ping_val = micros() - ping_prev;
        height_changed = true;
    }
}

void ping_update() {
    long temp_height;
    if (height_changed) {
        sregRestore = SREG;
        temp_height = ping_val;
        SREG = sregRestore ;
        height_changed = false;
        height_diff = abs(temp_height_old - temp_height);
        temp_height_old = temp_height;
        if (close_by(temp_height, ping_height, height_threashold) || bypass_height_filter) //Ignoring spikes in ping values
            if(temp_height > 0)
                ping_height = temp_height;
    }

    //we cannot read every loop we need some delay between each read
    //see manual https://www.parallax.com/sites/default/files/downloads/28015-PING-Sensor-Product-Guide-v2.0.pdf
    if (millis() - prev_ping_time >= ping_interval) {
        // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
        // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
        digitalWrite(PING_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(PING_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(PING_TRIG_PIN, LOW);

        //Now get ready to read the pulse
        prev_ping_time = millis();
    }
}

