#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
float bmp_alt;  

void bmp_init(){
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    }
}

inline void bmp_update(){
    bmp_alt = bmp.readAltitude(101500);

    // Serial.print("Real altitude = ");
    // Serial.print(bmp_alt);
    // Serial.println(" meters");
    
}
