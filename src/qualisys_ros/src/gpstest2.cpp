#include"UTM_GPS.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpstest2");
    ros::NodeHandle nh("~");
    UTM_GPS mygps=UTM_GPS();
    utm_gps_data tempgpsdata=mygps.get_my_utm_gps_data(10,10);
    utm_gps_data tempgpsdata2=mygps.get_my_utm_gps_data(100,100);

    std::cout<<"纬度"<<tempgpsdata.gps_latitude<<"经度"<<tempgpsdata.gps_longitude<<std::endl;
    return 0;
}