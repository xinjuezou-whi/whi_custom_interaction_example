/******************************************************************
node to handle httpclient get request

Features:
- http get request
- post states to service 

Written by Yue Zhou, sevendull@163.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-03-19: Initial version
2024-04-02: post state module
******************************************************************/
#include "whi_custom_interaction_example/httplib.h"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <jsoncpp/json/json.h>
#include <iostream>
#include "whi_interfaces/WhiBattery.h"
#include "whi_interfaces/WhiMotionState.h"
#include "whi_interfaces/WhiBoundingBox.h"
#include "whi_interfaces/WhiBoundingBoxes.h"
#include <csignal>

std::string host;
int port;
int timeout = 30;
std::string posthost;
int postport;

static bool jsonRoot(const std::string& Src, Json::Value& Root)
{
    const auto rawJsonLength = static_cast<int>(Src.length());
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    JSONCPP_STRING err;
    if (reader->parse(Src.c_str(), Src.c_str() + rawJsonLength, &Root, &err))
    {
        return true;
    }
    else
    {
        return false;
    }
}

// 开始动作的请求, "开始放片" 和 "开始抓片" 的请求
bool StartActionRequst(httplib::Client& Cli, const std::string& Apistr)
{
    int ratei = 1;
    ros::Rate rate(ratei);
    int mytimeout = timeout * ratei;
    int gettime = 0;
    httplib::Params params;
    std::string getStr = Apistr;
    httplib::Headers headers = { { "Accept", "application/json" } };
    if (Apistr == "place")
    {
        getStr = "/placing";
    }
    else if (Apistr == "reclaim")
    {
        getStr = "/reclaiming";
    }

    while (true)
    {
        if (auto getres = Cli.Get(getStr, params, headers,
            [](uint64_t len, uint64_t total)
            {
                return true; 
            }))
        {
            std::cout << getres->status << std::endl;
            std::cout << getres->get_header_value("Content-Type") << std::endl;
            std::cout << getres->body << std::endl;
            if(gettime > mytimeout)
            {
                ROS_INFO("%s request timeout ! , %d " ,getStr.c_str() ,gettime );
                return false;
            }

            Json::Value root;
            if (!jsonRoot(getres->body, root))
            {
                ROS_INFO("json::parse error!");
                return false;
            }
            int status = root["status"].asInt();
            std::string msgStr = root["msg"].asString();
            if (status == 100)
            {
                ROS_INFO("%s request permit , msg:%s", getStr.c_str() , msgStr.c_str());
                return true;

            }
            else if (status == 101)
            {
                ROS_INFO("%s request prohibit , msg:%s", getStr.c_str(), msgStr.c_str());
            }
            else
            {
                ROS_INFO("request error , check your request or server response " );
                return false;
            }

        }
        else
        {
            std::cout << "error code: " << getres.error() << std::endl;
            return false;
        }
        gettime++;
        rate.sleep();
    }
}

// 动作请求
bool ActionRequest(httplib::Client& Cli, const std::string& Apistr, std_srvs::SetBool::Response& Res)
{
    int ratei = 1;
    ros::Rate rate(ratei) ;
    int mytimeout = timeout * ratei;
    int gettime = 0;
    httplib::Params params;
    std::string getStr = "/getRequest";
    getStr = "/" + Apistr;
    httplib::Headers headers = { { "Accept", "application/json" } };
    Res.success = false;
    while (true)
    {
        if (auto getres = Cli.Get(getStr, params, headers,
            [](uint64_t len, uint64_t total)
            {
                return true; 
            }))
        {
            std::cout << getres->status << std::endl;
            std::cout << getres->get_header_value("Content-Type") << std::endl;
            std::cout << getres->body << std::endl;
            if(gettime > mytimeout)
            {
                ROS_INFO("%s request timeout ! , %d " ,getStr.c_str() ,gettime );
                Res.success = false;
                Res.message = "time out " ;
                return true;
            }
            Json::Value root;
            if (!jsonRoot(getres->body, root))
            {
                ROS_INFO("json::parse error!");
                return true;
            }
            int status = root["status"].asInt();
            std::string msgStr = root["msg"].asString();
            if (status == 100)
            {
                ROS_INFO("%s request permit , msg:%s", getStr.c_str(), msgStr.c_str());
                // 如果是place或者reclaim ，需要在动作开始执行前 发送请求
                if (Apistr == "place" || Apistr == "reclaim")
                {
                    bool getStart = StartActionRequst(Cli, Apistr);
                    if (getStart)
                    {
                        Res.success = true;
                        Res.message = msgStr ;
                    }
                    else
                    {
                        msgStr = Apistr + "ing failed";
                        Res.success = false;
                        Res.message = msgStr ;
                        return true;
                    }
                }
                else
                {
                    Res.success = true;
                    Res.message = msgStr ;
                }
                return true;
            }
            else if (status == 101)
            {
                ROS_INFO("%s request prohibit , msg:%s",getStr.c_str(), msgStr.c_str());
            }
            else
            {
                ROS_INFO("request error , check your request or server response");
                Res.success = false;
                return true;
            }
        }
        else
        {
            std::cout << "error code: " << getres.error() << std::endl;
            Res.success = false;
            return true;
        }
        gettime++;
        rate.sleep();
    }
}

bool RequestPlace(std_srvs::SetBool::Request& Req,
    std_srvs::SetBool::Response& Res)
{
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);  
    if (Req.data)
    {
        ActionRequest(cli, "place", Res);
    }
    ROS_INFO("sending back response: [%d]", Res.success);
    return true;
}

bool RequestPlaced(std_srvs::SetBool::Request& Req,
    std_srvs::SetBool::Response& Res)
{
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);      
    if (Req.data)
    {
        ActionRequest(cli, "placed", Res);
    }
    ROS_INFO("sending back response: [%d]", Res.success);
    return true;
}

bool RequestReclaim(std_srvs::SetBool::Request& Req,
    std_srvs::SetBool::Response& Res)
{
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);      
    if (Req.data)
    {
        ActionRequest(cli, "reclaim", Res);
    }
    ROS_INFO("sending back response: [%d]", Res.success);
    return true;
}

bool RequestReclaimed(std_srvs::SetBool::Request& Req,
    std_srvs::SetBool::Response& Res)
{
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);      
    if (Req.data)
    {
        ActionRequest(cli, "reclaimd", Res);
    }
    ROS_INFO("sending back response: [%d]", Res.success);
    return true;
}


//-----------------------POST STATES--------------------------------

std::atomic<bool> Update = {false};
std::atomic<bool> UpdateDet = {false};
Json::Value stateJson;
Json::Value taskJson;
Json::Value detJson;
std::string postAddr;
std::atomic_bool exit_app(false);

void signal_handler(int signum) 
{
    if (signum == SIGINT) 
    {
        std::cout << "Caught SIGINT, exiting gracefully..." << std::endl;
        exit_app = true;
        ros::shutdown();
    }
}

void stateCallback(const whi_interfaces::WhiBattery::ConstPtr& MsgBat)
{
    int battery = MsgBat->state;
    stateJson["power"] = battery ;


}

void taskCallback(const whi_interfaces::WhiMotionState::ConstPtr& MsgTask)
{
    //std::string order = MsgTask->order; 
    //std::string suborder = MsgTask->suborder;
    taskJson["order"] = "order";
    taskJson["subOrder"] = "subOrder";

}

void detectionCallback(const whi_interfaces::WhiBoundingBoxes::ConstPtr& MsgDet)
{
    ROS_INFO("indetection callback ");
    UpdateDet = true;
    detJson.clear();
    std::vector<whi_interfaces::WhiBoundingBox> detResults(MsgDet->bounding_boxes);
    std::string clsname = detResults.front().cls;
    std::string resultStr = "result"+clsname;
    Json::Value detArray;
    int i = 0;
    for (auto& onedet : detResults)
    {
        i++;
        Json::Value detvalue;
        detvalue["name"] = onedet.cls + std::to_string(i);
        detvalue["value"] =  onedet.state;
        detvalue["unitName"] = "Unit";
        detArray.append(detvalue);
    }
    detJson[resultStr] = detArray;

}


bool GetAction(std_srvs::SetBool::Request& Req,
    std_srvs::SetBool::Response& Res)
{
    if (Req.data)
    {
        Update = true;
        Res.success = true;
    }

    return true;
}

void senddataFun()
{
    const char* hostaddr = posthost.c_str();
    httplib::Client cli(hostaddr, postport);
    cli.set_connection_timeout(0, 800000); // 800 milliseconds
    cli.set_read_timeout(20, 0); // 20 seconds
    Json::Value sendJson;
    ros::Rate loop_rate(1);
    while (!exit_app.load())
    {
        if (Update)
        {
            sendJson.clear();
            sendJson = taskJson;
            for (const auto& key : stateJson.getMemberNames()) 
            {
                sendJson[key] = stateJson[key];
            }
            if (UpdateDet)
            {
                for (const auto& key : detJson.getMemberNames()) 
                {
                    sendJson[key] = detJson[key];
                }
                UpdateDet = false;
            }
            Update = false;
        }

        if (sendJson.isNull() || sendJson.empty())
        {
            ROS_INFO("sendJson is empty " );
        }
        else
        {
            Json::FastWriter writer;
	        std::string sendStr = writer.write(sendJson);
            ROS_INFO("sendstr data is: %s",sendStr.c_str());
            httplib::Params params;
            params.emplace("state", sendStr);
            std::string poststr = "/" + postAddr;
            httplib::Headers headers = { { "Accept", "application/json" } };
            if (auto res = cli.Post(poststr, params))
            {
                if (res->status == httplib::StatusCode::OK_200)
                {
                    ROS_INFO("POST success ,post data is: %s",sendStr.c_str());
                }
            }
            else
            {
                ROS_INFO("POST fail ,error ,%d ",res.error());
            }

        }


        loop_rate.sleep();
    }

}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, ""); // for Chinese char: setlocale(LC_CTYPE, "zh_CN.utf8");

    const std::string nodeName("whi_custom_interaction_example");
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nd(nodeName);

//---------get request -----------------------
    bool gethost=nd.getParam("host", host);
	bool getport=nd.getParam("port", port);
    nd.getParam("timeout",timeout);
    ROS_INFO("getparam host:%s , port:%d ,timeout:%d",host.c_str(),port,timeout);

    ros::ServiceServer servicePlace = nd.advertiseService("place", RequestPlace);
    ros::ServiceServer servicePlaced = nd.advertiseService("placed", RequestPlaced);
    ros::ServiceServer serviceReclaim = nd.advertiseService("reclaim", RequestReclaim);
    ros::ServiceServer serviceReclaimed = nd.advertiseService("reclaimed", RequestReclaimed);
    
//---------post states -----------------------

    gethost=nd.getParam("posthost", posthost);
	getport=nd.getParam("postport", postport);
    ROS_INFO("post host:%s , post port:%d",posthost.c_str(),postport);   

    std::string stateTopic,taskTopic,detTopic;
    nd.getParam("state_topic", stateTopic);
    nd.getParam("task_topic", taskTopic);
    nd.getParam("det_topic", detTopic);
    nd.getParam("post_addr",postAddr);
    ROS_INFO("state_topic:%s , task_topic:%s , det_topic:%s",stateTopic.c_str(),taskTopic.c_str(),detTopic.c_str());   

    signal(SIGINT, signal_handler);

    ros::Subscriber subState = nd.subscribe<whi_interfaces::WhiBattery>(stateTopic, 10, stateCallback);
    ros::Subscriber subTask = nd.subscribe<whi_interfaces::WhiMotionState>(taskTopic, 10, taskCallback);
    ros::Subscriber subDetection = nd.subscribe<whi_interfaces::WhiBoundingBoxes>(detTopic, 10, detectionCallback);
    ros::ServiceServer serviceAction = nd.advertiseService("action", GetAction);

    std::thread senddataTh(senddataFun);
    ROS_INFO("Ready to httpclient.");

    ros::spin();
    senddataTh.join();
    
    return 0;
}
