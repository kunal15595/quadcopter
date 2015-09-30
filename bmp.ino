#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
float bmp_alt;  
int count_bmp_read;

void bmp_init(){
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    }

    count_bmp_read = 0;
}

inline void bmp_update(){
	count_bmp_read++;

	if(count_bmp_read > 10){
		count_bmp_read = 0;
		
		// bmp_alt = bmp.readAltitude(101500);
	}
    

    // Serial.print("Real altitude = ");
    // Serial.print(bmp_alt);
    // Serial.println(" meters");
    
}
