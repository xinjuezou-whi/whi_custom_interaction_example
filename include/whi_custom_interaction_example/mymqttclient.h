/******************************************************************
class to handle MqttClient

Features:
- MqttClient
- xxx

Dependencies:
- sudo apt install libmosquitto-dev

Written by Yue Zhou, sevendull@163.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-04-15: Initial version
2024-xx-xx: xxx
******************************************************************/
#include <iostream>
#include <mosquitto.h>

namespace whi_custom_interaction_example
{
	class MyMqttClient
	{
    public:
        MyMqttClient();
        ~MyMqttClient();

        void init(const std::string addr , const int port);
        bool getStart() { return is_start_; }
        bool mosquittoInit();
        bool mosquittoConnect();
        void mosquittoPublish(const std::string& topic, const std::string& message);
        void mosquittoSubscribe(const std::string& topic) ;
        static void onMessage(struct mosquitto* mosq,void* userdata, const struct mosquitto_message* message);
        void messageLoop();

    protected:
 

    protected:
        std::string mqtt_broker_addr_ = "localhost";  // Mosquitto broker 的地址
        int mqtt_broker_port_ = 1883;  // Mosquitto broker 的端口号    
        struct mosquitto* mosq_ = nullptr;
        bool is_start_ = false;
 
	};
}