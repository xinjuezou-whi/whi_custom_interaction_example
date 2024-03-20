/******************************************************************
node to handle httpclient get request

Features:
- http get request
-

Written by Yue Zhou, sevendull@163.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-03-19: Initial version
2024-xx-xx: xxx
******************************************************************/
#include "whi_custom_interaction_example/httplib.h"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <jsoncpp/json/json.h>
#include <iostream>

std::string host;
int port;

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
    ros::Rate rate(1);
    httplib::Params params;
    std::string getStr = Apistr;
    httplib::Headers headers = { { "Accept", "application/json" } };
    if (Apistr == "place")
    {
        getStr = "/api/placing";
    }
    else if (Apistr == "reclaim")
    {
        getStr = "/api/reclaiming";
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

        rate.sleep();
    }
}

// 动作请求
bool ActionRequest(httplib::Client& Cli, const std::string& Apistr, std_srvs::SetBool::Response& Res)
{
    ros::Rate rate(1) ;
    httplib::Params params;
    std::string getStr = "/getRequest";
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
        rate.sleep();
    }
}

bool RequestPlace(std_srvs::SetBool::Request& Req,
    std_srvs::SetBool::Response& Res)
{
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
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
    if (Req.data)
    {
        ActionRequest(cli, "reclaimd", Res);
    }
    ROS_INFO("sending back response: [%d]", Res.success);
    return true;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, ""); // for Chinese char: setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "http_client");
    const std::string nodeName("http_client"); 
    ros::NodeHandle nd(nodeName);

    bool gethost=nd.getParam("host", host);
	bool getport=nd.getParam("port", port);
    ROS_INFO("getparam host:%s , port:%d",host.c_str(),port);

    ros::ServiceServer service1 = nd.advertiseService("place", RequestPlace);

    ros::ServiceServer service2 = nd.advertiseService("placed", RequestPlaced);

    ros::ServiceServer service3 = nd.advertiseService("reclaim", RequestReclaim);

    ros::ServiceServer service4 = nd.advertiseService("reclaimed", RequestReclaimed);
    
    ROS_INFO("Ready to httpclient.");
    ros::spin();
    
    return 0;
}
