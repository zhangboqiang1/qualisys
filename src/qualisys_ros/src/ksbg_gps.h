#pragma once
#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include<vector>
#include<string>
const int az_number=9;
int creat_udp_socket(int myport=12351)
{
    int sockfd;

    // 创建socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd)
    {
        return -1;
        puts("Failed to create socket");
    };

    // 设置地址与端口
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;      // Use IPV4
    addr.sin_port = htons(myport); //
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    // Time out
    // struct timeval tv;
    // tv.tv_sec = 0;
    // tv.tv_usec = 200000; // 200 ms
    // setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(struct timeval));
    // 绑定获取数据的端口，作为发送方，不绑定也行
    if (bind(sockfd, (struct sockaddr *)&addr, addr_len) == -1)
    {
        printf("Failed to bind socket on port %d\n", myport);
        close(sockfd);
        return -1;
    };
    return sockfd;
}





//用来解析校验位
std::string to_string_ox(int i){
    if(i<10)return std::to_string(i);
    std::string ans="a";
    ans[0]='a'+i-10;
    return ans;
}

std::vector<std::string>pack_udp_TRC(std::vector<double>propellers_rpm,std::vector<double>propeller_angle,const int ship_id){
    //propellers.size()=9=az_number;propellers[i][0]为转速度，propeller[i][1]为角度;
    std::vector<std::string>ans(propellers_rpm.size());
    for(int i=0;i<propellers_rpm.size();i++){
        std::string temp="$__TRC,";
        temp+=std::to_string(i+(ship_id-1)*9+1);//id//9=az_number
        temp+=",";


//保留后两位小数。
        std::string rpmstring=std::to_string(propellers_rpm[i]);
        for(int j=0;j<rpmstring.size();j++){
            if(rpmstring[j]=='.'){
                temp+=rpmstring.substr(0,j+3);
                break;
            };
        }


        //temp+=std::to_string(propellers_rpm[i]);//转速
        temp+=",T,,,";

//保留后1位小数
        std::string anglestring=std::to_string(propeller_angle[i]);
        for(int j=0;j<anglestring.size();j++){
            if(anglestring[j]=='.'){
                temp+=anglestring.substr(0,j+2);
                break;
            };
        }

        //temp+=std::to_string(propeller_angle[i]);//角度
        temp+="*";
        int checksum=0;
        for(int i=1;i<temp.size()-1;++i){
            checksum^=(int)temp[i];
        };
        temp+=to_string_ox(checksum/16);
        temp+=to_string_ox(checksum%16);      
        temp+="\r";
        temp+="\n"; 
        ans[i]=temp;   
        //std::cout<<temp<<std::endl;
    }
    return ans;
};

std::string pack_udp_gps(std::string _time,double _gps_lati,double _gps_long){
    std::string ans="$__GGA,";
    ans+=_time;
    ans+=",";
//保留后两位小数。

    std::string _gps_lati_string=std::to_string(_gps_lati);
    std::string pre_temp_lati;
        for(int j=0;j<_gps_lati_string.size();j++){
            if(_gps_lati_string[j]=='.'){
                pre_temp_lati=_gps_lati_string.substr(0,j+5);
                //ans+=_gps_lati_string.substr(0,j+5);
                break;
            };
        }
        ans+="00";
        ans+=pre_temp_lati;

    //ans+=std::to_string(_gps_lati);

    ans+=",N,";
    std::string _gps_long_string=std::to_string(_gps_long);
    std::string pre_temp_long;
        for(int j=0;j<_gps_long_string.size();j++){
            if(_gps_long_string[j]=='.'){
                pre_temp_long=_gps_long_string.substr(0,j+5);
                //ans+=_gps_long_string.substr(0,j+5);
                break;
            };
        }
    ans+="00";
    ans+=pre_temp_long;
   // ans+=std::to_string(_gps_long);

    ans+=",E,2,5,1.0,0.2,M,0.0,M,0.0,1234,*";
     int checksum=0;
        for(int i=1;i<ans.size()-1;++i){
            checksum^=(int)ans[i];
        };
        ans+=to_string_ox(checksum/16);
        ans+=to_string_ox(checksum%16);      
        ans+="\r";
        ans+="\n"; 
    return ans;
}

std::string pack_udp_heading(double _heading){
    std::string ans="$HEHDT,";
    //保留后两位小数。
    std::string _heading_string=std::to_string(_heading);
        for(int j=0;j<_heading_string.size();j++){
            if(_heading_string[j]=='.'){
                ans+=_heading_string.substr(0,j+3);
                break;
            };
        }
        ans+=",T*";
        int checksum=0;
        for(int i=1;i<ans.size()-1;++i){
            checksum^=(int)ans[i];
        };
        ans+=to_string_ox(checksum/16);
        ans+=to_string_ox(checksum%16);      
        ans+="\r";
        ans+="\n"; 
    return ans;
}

std::string pack_udp_PSXN(double _pitch,double _roll){
    std::string temp="$__SXN,10,,";
    //保留后两位小数。
    std::string _pitch_string=std::to_string(_pitch);
        for(int j=0;j<_pitch_string.size();j++){
            if(_pitch_string[j]=='.'){
                temp+=_pitch_string.substr(0,j+5);
                break;
            };
        }
        temp+=",";
     std::string _roll_string=std::to_string(_roll);

        for(int j=0;j<_roll_string.size();j++){
            if(_roll_string[j]=='.'){
                temp+=_roll_string.substr(0,j+5);
                break;
            };
        }
        temp+=",,,,*"; 
    int checksum=0;
        for(int i=1;i<temp.size()-1;++i){
            checksum^=(int)temp[i];
        };
        temp+=to_string_ox(checksum/16);
        temp+=to_string_ox(checksum%16);      
        temp+="\r";
        temp+="\n";    
   
    return temp;

}



void send_udp(int sockfd,int port_out,std::string s){//8081
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    memset(&addr, 0, sizeof(addr));    
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_out);
    //addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_addr.s_addr = inet_addr("172.21.101.1");
    char sendbuf[1024];
    memset(&sendbuf, 0, sizeof(sendbuf));
   
    for(int i=0;i<s.size();++i){
        sendbuf[i]=s[i];
    }

    sendto(sockfd, sendbuf, s.size()+1, 0, (sockaddr *)&addr, addr_len);
    printf("Sended %s\n",sendbuf);
    //usleep(1);
}
