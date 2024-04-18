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
#include "whi_custom_interaction_example/mymqttclient.h"


namespace whi_custom_interaction_example
{
    MyMqttClient::MyMqttClient()
    {

    }

    void MyMqttClient::init(const std::string addr , const int port)
    {
        mqtt_broker_addr_ = addr;
        mqtt_broker_port_ = port;
        if(mosquittoInit())
        {
            if(mosquittoConnect())
                is_start_ = true;
        }
    }

    MyMqttClient::~MyMqttClient()
    {
        // 断开连接并清理资源
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosquitto_lib_cleanup();
    }

    // Mosquitto库初始化
    bool MyMqttClient::mosquittoInit() 
    {
        mosquitto_lib_init();
        mosq_ = mosquitto_new("mosq_cpp_example", true, nullptr);
        if (!mosq_) 
        {
            std::cerr << "Error: Unable to initialize Mosquitto library." << std::endl;
            return false;
            //exit(EXIT_FAILURE);
        }
        return true;
    }

    // 连接到Mosquitto broker
    bool MyMqttClient::mosquittoConnect() 
    {
        int ret = mosquitto_connect(mosq_, mqtt_broker_addr_.c_str(), mqtt_broker_port_, 60);
        if (ret != MOSQ_ERR_SUCCESS) 
        {
            std::cerr << "Error: Unable to connect to Mosquitto broker. Return code: " << ret << std::endl;
            return false;
        }
        return true;
    }

    // 发布消息到指定主题
    void MyMqttClient::mosquittoPublish(const std::string& topic, const std::string& message) 
    {
        int ret = mosquitto_publish(mosq_, nullptr, topic.c_str(), message.size(), message.c_str(), 0, false);
        if (ret != MOSQ_ERR_SUCCESS) 
        {
            std::cerr << "Error: Unable to publish message. Return code: " << ret << std::endl;
        }
    }

    // 订阅指定主题
    void MyMqttClient::mosquittoSubscribe(const std::string& topic) 
    {
        int ret = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), 0);
        if (ret != MOSQ_ERR_SUCCESS) 
        {
            std::cerr << "Error: Unable to subscribe to topic. Return code: " << ret << std::endl;
        }
    }

    // 回调函数处理接收到的消息
    void MyMqttClient::onMessage(struct mosquitto* mosq,void* userdata, const struct mosquitto_message* message) 
    {
        std::string receivedMessage((char*)message->payload, message->payloadlen); // char* -> string
        std::cout << "Received message on topic: " << message->topic << ", Message: " << receivedMessage << std::endl;
    }


    void MyMqttClient::messageLoop()
    {
        // 设置消息接收回调函数
        mosquitto_message_callback_set(mosq_, MyMqttClient::onMessage);
        // 循环处理消息
        mosquitto_loop_forever(mosq_, -1, 1);
    }






}