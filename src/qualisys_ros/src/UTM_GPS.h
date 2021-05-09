#pragma once
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>



typedef struct utm_gps_data{
    double gps_latitude;//维度
    double gps_longitude;//经度
    double utmx;
    double utmy;
    std::string zone;
    utm_gps_data(double _gps_latitude=31.231,double _gps_longitude=121.47,double _utmx=0,double _utmy=0):
        gps_latitude(_gps_latitude),
        gps_longitude(_gps_longitude),
        utmx(_utmx),
        utmy(_utmy),
        zone("none")
        {
    };
}utm_gps_data;



class UTM_GPS{
private:
    utm_gps_data my_utm_gps_data;
    utm_gps_data origin_utm_gps_data;

public:
    UTM_GPS(const utm_gps_data&_my_utm_gps_data ):my_utm_gps_data(_my_utm_gps_data)
    {
        origin_utm_gps_data=utm_gps_data();
        double temp_utmx, temp_utmy;
        std::string zone;
        gps_common::LLtoUTM(origin_utm_gps_data.gps_latitude, origin_utm_gps_data.gps_longitude, temp_utmx, temp_utmy, zone);
        origin_utm_gps_data.utmx=temp_utmx;
        origin_utm_gps_data.utmy=temp_utmy;
        origin_utm_gps_data.zone=zone;
        my_utm_gps_data=origin_utm_gps_data;
    };
    UTM_GPS(){
        my_utm_gps_data=utm_gps_data();
        origin_utm_gps_data=utm_gps_data();
        double temp_utmx, temp_utmy;
        std::string zone;
        gps_common::LLtoUTM(origin_utm_gps_data.gps_latitude, origin_utm_gps_data.gps_longitude, temp_utmx, temp_utmy, zone);
        origin_utm_gps_data.utmx=temp_utmx;
        origin_utm_gps_data.utmy=temp_utmy;
        origin_utm_gps_data.zone=zone;
        my_utm_gps_data=origin_utm_gps_data;
    };
    ~UTM_GPS(){};
private:
   
public:   
    utm_gps_data get_my_utm_gps_data(double _qualisys_posx,double _qualisys_posy){
        my_utm_gps_data.utmx=_qualisys_posx+origin_utm_gps_data.utmx;
        my_utm_gps_data.utmy=_qualisys_posy+origin_utm_gps_data.utmy;
        gps_common::UTMtoLL(my_utm_gps_data.utmx, my_utm_gps_data.utmy, my_utm_gps_data.zone, my_utm_gps_data.gps_latitude, my_utm_gps_data.gps_longitude);
        return my_utm_gps_data;
    };
    



};