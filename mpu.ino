#include <MPU9150_9Axis_MotionApps41.h>
#include "AK8975.h"
#include "HMC5883L.h"

HMC5883L mag_hmc;

AK8975 mag(0x0C);
MPU9150 mpu;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

const int DMP_INT_PIN = 2;

const int NUM_SAMPLES_FOR_YAW_AVERAGE = 10;
float yaw_average_retain = 0.9;

const int YPR_RATIO = 128;

volatile boolean mpu_interrupt = false;     // indicates whether MPU interrupt pin has gone high
 
#define MX_OFFSET 35 
#define MY_OFFSET 10 
#define HEADING_OFFSET 250 

boolean dmp_ready = false;  // set true if DMP init was successful
uint8_t mpu_int_status;   // holds actual interrupt status byte from MPU
uint8_t dev_status;      // return status after each device operation (0 = success, !0 = error)
uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
uint16_t fifo_count;     // count of all bytes currently in FIFO
uint8_t fifo_buffer[64]; // FIFO storage buffer

int16_t mx, my, mz;
float heading;
int num_rounds = 0;
float pi = 3.14f; //value of PI actually yaw value is from [0,PI],[-PI,0] so the correction
float infinity = 1000000;
float yaw_prev = -infinity;

bool disorder = false;
bool mag_initiated = false;
unsigned long mag_initiated_at;

void dmp_data_ready() {
    mpu_interrupt = true;
}

void mpu_init() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(true);
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize device
    mpu.initialize();
    mpu.setRate(0);
    mpu.setFullScaleGyroRange(250);

    // verify connection
    if (!mpu.testConnection()){
        Serial1.println(F("MPU connection failed"));
        Serial.println(F("MPU connection failed"));
    }

    // load and configure the DMP
    dev_status = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (dev_status == 0) {
        // turn on the DMP, now that it's ready
        /* Serial1.println(F("Enabling DMP...")); */
        mpu.setDMPEnabled(true);


        // enable Arduino interrupt detection
        enableInterrupt(DMP_INT_PIN, dmp_data_ready, RISING);
        mpu_int_status = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmp_ready = true;

        // get expected DMP packet size for later comparison
        packet_size = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial1.print(F("DMP Initialization failed (code "));
        Serial1.print(dev_status);
        Serial1.println(F(")"));
        asm volatile ("  jmp 0");
    }

    mpu.setI2CMasterModeEnabled(false);
    mpu.setI2CBypassEnabled(true);

    mag.initialize();    
    Serial.println(mag.testConnection() ? "AK8975 connection successful" : "AK8975 connection failed");

    // mag_hmc.initialize();
    // Serial.println(mag_hmc.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    mag_initiated_at = micros();
    mag_initiated = false;
}


void desired_yaw_update() {
    if (!desired_yaw_got) {
        int num_samples_for_yaw_average_copy = NUM_SAMPLES_FOR_YAW_AVERAGE;
        desired_yaw = int_angle[0];

        while (num_samples_for_yaw_average_copy > 0) {
            delay(2);//ms delay to get fresh values
            ypr_update();
            desired_yaw = desired_yaw * (1 - yaw_average_retain) + yaw_average_retain * int_angle[0];
            num_samples_for_yaw_average_copy--;
        }

        desired_angle[0] = desired_yaw;
        desired_yaw_got = true;
    }
}


void ypr_update() {

    // wait for MPU interrupt or extra packet(s) available
    // reset interrupt flag and get INT_STATUS byte
    if (mpu_interrupt || mpu.getFIFOCount() > packet_size) {
        mpu_interrupt = false;
        mpu_int_status = mpu.getIntStatus();

        // get current FIFO count
        fifo_count = mpu.getFIFOCount();
        // Serial.print(fifo_count); Serial.print("\t");

        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpu_int_status & 0x10) || fifo_count == 1024) {
            // reset so we can continue cleanly
            Serial1.println(fifo_count);
            mpu.resetFIFO();
            Serial1.println(F("FIFO overflow!"));
            Serial.println(F("FIFO overflow!"));


            // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpu_int_status & 0x02) {
            if (fifo_count >= packet_size) {

                // unsigned long loop_start = micros();

                // read a packet from FIFO
                mpu.getFIFOBytes(fifo_buffer, packet_size);


                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifo_count -= packet_size;
                while (fifo_count >= packet_size) {
                    mpu.getFIFOBytes(fifo_buffer, packet_size);
                    fifo_count -= packet_size;
                }
                    
                // Serial.println(micros()-loop_start); 

                if (fifo_count != 0) {
                    if(disorder){
                        Serial.println(F("FIFO RESET !!"));
                        mpu.resetFIFO();
                    }else{
                        disorder = true;
                    }
                }else{
                    disorder = false;
                }



                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifo_buffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                
            }
        }
    }

    if(mag_initiated && (micros() - mag_initiated_at) > 10000){
        mag.getHeadingFast(&mx, &my, &mz);
        mag_initiated = false;

        mx -= MX_OFFSET;
        my -= MY_OFFSET;

        // Serial.print("mag:\t");
        // Serial.print(mx); Serial.print("\t");
        // Serial.print(my); Serial.print("\t");
        // Serial.print(mz);  Serial.print("\t");

        

    }else if(!mag_initiated){
        mag.initiateReading();
        mag_initiated = true;
        mag_initiated_at = micros();
    }

    ypr[0] = atan2((double)my, (double)mx);
    
    // Serial.print("ypr:\t");
    // Serial.print(ypr[0]); Serial.print("\t");
    // Serial.print(ypr[1]); Serial.print("\t");
    // Serial.print(ypr[2]); Serial.print("\t");

    heading = ypr[0] * 180.0/3.14159265 + 180 - HEADING_OFFSET;
    while (heading < 0) heading += 360;
    while (heading > 360) heading -= 360;

    // Serial.print(heading);
    // Serial.print(" deg"); Serial.print("\t");


    mpu.getRotation(&gx, &gy, &gz);
    gyro_int_raw[0] = gz + gyro_int_offset[0];
    // x and y are reversed to match pitch and roll.
    // pitch is about y axis and roll about x axis
    gyro_int_raw[1] = gy + gyro_int_offset[1];
    gyro_int_raw[2] = gx + gyro_int_offset[2];

    int_rate[0] = int_rate[0] * (1 - gyro_retain) + gyro_retain * gyro_int_raw[0];
    int_rate[1] = int_rate[1] * (1 - gyro_retain) + gyro_retain * gyro_int_raw[1];
    int_rate[2] = int_rate[2] * (1 - gyro_retain) + gyro_retain * gyro_int_raw[2];

    Serial.print(ypr[0]); 
    Serial.print("\t"); 

    if (ypr[0] < 0)
        ypr[0] += 2 * pi; //converted from [0,PI][-PI,0] to [0,2*PI]
    else if(ypr[0] > 2 * pi)
        ypr[0] -= 2 * pi;

    //this yaw is still wrong cause we are asked to maintain yaw at around 2*PI then
    //the yaw jumps from 2*Pi to 0 here
    if (yaw_prev != -infinity) {
        if (close_by(yaw_prev, 2 * pi, yaw_threashold) && close_by(ypr[0], 0, yaw_threashold))
            num_rounds++;
        else if (close_by(yaw_prev, 0, yaw_threashold) && close_by(ypr[0], 2 * pi, yaw_threashold))
            num_rounds--;
    }

    // Serial.print(yaw_prev);
    // Serial.print("\t");
    // Serial.print(ypr[0]);
    // Serial.print("\t");
    // Serial.print(num_rounds);
    // Serial.print("\t");

    yaw_prev = ypr[0];
    ypr[0] += 2 * pi * num_rounds;

    /* Serial.println(ypr[0]); */

    int_angle[0] = ypr[0] * YPR_RATIO + ypr_int_offset[0];
    int_angle[1] = ypr[1] * YPR_RATIO + ypr_int_offset[1];
    int_angle[2] = ypr[2] * YPR_RATIO + ypr_int_offset[2];

    Serial.print(int_angle[0]); Serial.print("\t");
    Serial.print(int_angle[1]); Serial.print("\t");
    Serial.print(int_angle[2]); Serial.print("\t");

    // if(!close_by(ypr[0], yaw_prev, 0.3)){
    //     mpu_init();
    // }

    // Serial.println("\t\t");
}
