#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
float bmp_alt, bmp_base_alt;  

bool pres_initiated = false;
unsigned long pres_initiated_at;

bool temp_initiated = false;
unsigned long temp_initiated_at;

void bmp_init(){
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    }

    pres_initiated = false;
    temp_initiated = false;
    pres_initiated_at = micros();
    temp_initiated_at = micros();

    bmp_base_alt = bmp.readAltitude(101500);
}

inline void bmp_update(){

	if(pres_initiated && (micros() - pres_initiated_at) > 26000){
        bmp.readRawPressureFast();
        pres_initiated = false;
    }else if(!pres_initiated){
        bmp.initiateRawPressure();
        pres_initiated = true;
        pres_initiated_at = micros();
    }

	if(temp_initiated && (micros() - temp_initiated_at) > 5000){
        bmp.readRawTemperatureFast();
        temp_initiated = false;
    }else if(!temp_initiated){
        bmp.initiateRawTemperature();
        temp_initiated = true;
        temp_initiated_at = micros();
    }

	bmp_alt = bmp.readAltitudeFast(101500) - bmp_base_alt;
    
    if(ping_height/29/2 > 150){
        actual_height = bmp_alt*100;
    }else{
        actual_height = ping_height/29/2;
    }

    Serial.print("height = ");
    Serial.print(actual_height);
    Serial.print("\t");
    
    // Serial.println(" meters");
    
}
