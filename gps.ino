#include "TinyGPS++.h"

TinyGPSPlus gps;

TinyGPSLocation traj[10];
int traj_pts, traj_state, state_next;
double dist_to_next, old_dist_to_next;
double old_course;

uint8_t inByte;

int count_gps_read;


inline void gps_init(){
    traj_pts = 2;
    traj_state = 0;

    dist_to_next = old_dist_to_next = 100;
    old_course = 0.0;
    
    traj[0].rawLatData.deg = 26;
    traj[0].rawLatData.billionths = 186695000;
    
    traj[0].rawLngData.deg = 91;
    traj[0].rawLngData.billionths = 692038300;
    
    traj[1].rawLatData.deg = 26;
    traj[1].rawLatData.billionths = 186857830;
    
    traj[1].rawLngData.deg = 91;
    traj[1].rawLngData.billionths = 691838330;
    
    count_gps_read = 0;
}

inline void gps_update(){
    count_bmp_read++;

    if(count_bmp_read < 10){
        
        return;
        // bmp_alt = bmp.readAltitude(101500);
    }else{
        count_bmp_read = 0;
    }

    if (Serial3.available()) {
        int read_max = 128;
        while(Serial3.available() && --read_max){
            inByte = Serial3.read();
            // Serial.print(inByte);
            gps.encode(inByte);
        }        
    }

    if (gps.altitude.isUpdated()) {
        gps_alt = gps.altitude.meters();
        // Serial.print("Altitude="); Serial.println(gps_alt);
    }

    if (gps.location.isUpdated()) {
        gps_lng = gps.location.lng();
        gps_lat = gps.location.lat();
        
        // Serial.print("LAT="); Serial.print(gps_lat, 9);Serial.print("\t");
        // Serial.print("LNG="); Serial.print(gps_lng, 9);Serial.print("\t");
    }

    state_next = (traj_state+1)%traj_pts;
    gps_next_lat = traj[state_next].rawLatData.deg + traj[state_next].rawLatData.billionths / 1000000000.0;
    gps_next_lng = traj[state_next].rawLngData.deg + traj[state_next].rawLngData.billionths / 1000000000.0;
    
    if(follow_traj){
        desired_yaw = 0.9 * old_course + 0.1 * gps.courseTo(gps_lat, gps_lng, gps_next_lat, gps_next_lng);    
    }else{
        // desired_yaw = int_angle[0];
    }
    
    old_course = desired_yaw;

    dist_to_next = 0.9 * old_dist_to_next + 0.1 * gps.distanceBetween(gps_lat, gps_lng, gps_next_lat, gps_next_lng);
    old_dist_to_next = dist_to_next;

    // Serial.print(desired_yaw); Serial.print("\t");
    // Serial.print(dist_to_next); Serial.print("\t");
    
    if(dist_to_next < 2){
        traj_state++;
    }
}

