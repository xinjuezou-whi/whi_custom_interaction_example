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
#include "whi_interfaces/WhiTaskState.h"
#include <csignal>
#include "whi_custom_interaction_example/mymqttclient.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <angles/angles.h>

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
        getStr = "/api/shemt/placing";
    }
    else if (Apistr == "reclaim")
    {
        getStr = "/api/shemt/reclaiming";
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
            if (gettime > mytimeout)
            {
                ROS_INFO("%s request timeout !, %d ", getStr.c_str(), gettime);
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
                ROS_INFO_STREAM("request " << getStr << " succeed, with msg: " << msgStr);
                return true;
            }
            else if (status == 101)
            {
                ROS_INFO("%s request prohibit , msg:%s", getStr.c_str(), msgStr.c_str());
            }
            else
            {
                ROS_ERROR_STREAM("request " << getStr << " exception, with msg: " << msgStr);
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
void ActionRequest(httplib::Client& Cli, const std::string& Apistr, std_srvs::SetBool::Response& Res)
{
    int ratei = 1;
    ros::Rate rate(ratei) ;
    int mytimeout = timeout * ratei;
    int gettime = 0;
    httplib::Params params;
    std::string getStr = "/getRequest";
    getStr = "/api/shemt/" + Apistr;
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
            if (gettime > mytimeout)
            {
                ROS_INFO("%s request timeout !, %d", getStr.c_str(), gettime);
                Res.message = "time out";
                break;;
            }

            Json::Value root;
            if (!jsonRoot(getres->body, root))
            {
                ROS_INFO("json::parse error!");
                Res.message = "parse message from server error";
                break;
            }
            int status = root["status"].asInt();
            std::string msgStr = root["msg"].asString();
            if (status == 101)
            {
                // loop for next valid reponse
                ROS_INFO("%s request prohibit , msg:%s",getStr.c_str(), msgStr.c_str());
            }
            else if (status == 100)
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
                    }
                }
                else
                {
                    Res.success = true;
                    Res.message = msgStr ;
                }

                break;
            }
            else if (status == 102)
            {
                Res.success = false;
                Res.message = "response with exception";
                ROS_INFO_STREAM(Res.message);

                break;
            }
            else
            {
                Res.success = false;
                Res.message = "undefined response";
                ROS_INFO_STREAM(Res.message);

                break;
            }
        }
        else
        {
            std::cout << "error code: " << getres.error() << std::endl;
            Res.success = false;
            Res.message = "web server connection failed";
            ROS_INFO_STREAM(Res.message);

            break;
        }
        gettime++;
        rate.sleep();
    }
}

bool RequestPlace(std_srvs::SetBool::Request& Req,
    std_srvs::SetBool::Response& Res)
{
    ROS_INFO("sending request: place");
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
    ROS_INFO("sending request: placed");
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
    ROS_INFO("sending request: reclaim");
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
    ROS_INFO("sending request: reclaimed");
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);      
    if (Req.data)
    {
        ActionRequest(cli, "reclaimed", Res);
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

void taskCallback(const whi_interfaces::WhiTaskState::ConstPtr& MsgTask)
{
    taskJson["order"] = MsgTask->parent_name;
    taskJson["subOrder"] = MsgTask->name;
}

void detectionCallback(const whi_interfaces::WhiBoundingBoxes::ConstPtr& MsgDet)
{
    ROS_INFO("in detection callback ");
    UpdateDet = true;
    detJson.clear();
    std::vector<whi_interfaces::WhiBoundingBox> detResults(MsgDet->bounding_boxes);
    std::string clsname = detResults.front().cls;
    std::string resultStr;
    std::string substr = clsname.substr(0,4);
    ROS_INFO("substr is %s",substr.c_str());
    Json::Value detArray;
    if(substr == "belt")
    {
        resultStr = "resultBelt";
        std::string isNormal = clsname.substr(5);
        if(isNormal == "abnormal")
        {
            detArray["status"] = 101;
            detArray["msg"] = "fail";
        }
        else if(isNormal == "normal")
        {
            detArray["status"] = 100;
            detArray["msg"] = "success";
        }
    }else
    {
        resultStr = "resultDial";
        int i = 0;
        for (auto& onedet : detResults)
        {
            i++;
            Json::Value detvalue;
            detvalue["name"] = onedet.cls;
            detvalue["value"] =  onedet.state;
            detvalue["unitName"] = "Unit";
            detArray.append(detvalue);
        }
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
        // if (Update)
        // {
            //----  test start -----
            //taskJson["order"] = 5;
            //taskJson["subOrder"] = 100;
            //----- test end ------

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
            // Update = false;
        // }

        if (sendJson.isNull() || sendJson.empty())
        {
            // ROS_INFO("sendJson is empty");
        }
        else
        {
            Json::FastWriter writer;
	        std::string sendStr = writer.write(sendJson);
            ROS_INFO("sendstr data is: %s",sendStr.c_str());
            std::string poststr = "/" + postAddr;
            httplib::Headers headers = { { "content-type", "application/json" } };
            if ( auto res = cli.Post(poststr, headers, sendStr, "application/json"))
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

//---------------------- mqtt client  --------------------------------
whi_custom_interaction_example::MyMqttClient myMqtt;
std::string mqtttopic;
Json::Value jointJson;
Json::Value poseJson;
Json::Value motionJson;
std::string base_link;
tf2_ros::Buffer buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener{ nullptr };
std::vector<double> offsets_idiot;

void jointCallback(const sensor_msgs::JointStateConstPtr& MsgJoint)
{
    jointJson.clear();
    Json::Value joints;
    int i = 0;
    for(auto & onejoint : MsgJoint->position)
    {
        i++;
        std::string jointstr = "joint"+std::to_string(i);
        jointJson[jointstr] = onejoint * 180 / 3.1415926535;
    }

}

geometry_msgs::TransformStamped listenTf(const std::string& DstFrame, const std::string& SrcFrame,
    const ros::Time& Time)
{
    try
    {
        return buffer.lookupTransform(DstFrame, SrcFrame, Time, ros::Duration(1.0));
    }
    catch (tf2::TransformException &e)
    {
        ROS_ERROR("%s", e.what());
        return geometry_msgs::TransformStamped();
    }
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& MsgPose)
{
	auto trans = listenTf("map", base_link, ros::Time(0));

    Json::Value item;
    poseJson["agv_x_axis"] = trans.transform.translation.x + offsets_idiot[0];
    poseJson["agv_y_axis"] = trans.transform.translation.y + offsets_idiot[1];

    tf2::Quaternion quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
        trans.transform.rotation.w);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
  	tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // ROS_INFO("roll:%f , pitch:%f , yaw:%f ",roll,pitch,yaw);
    item["rot"] = yaw;
    //poseJson.clear();
    poseJson["agv_angel"] = angles::to_degrees(yaw);
}

void motionstateCallback(const whi_interfaces::WhiMotionState::ConstPtr& MsgMotion)
{
    int state = MsgMotion->state;
    motionJson["motion"] = state ;
}


void senddataMqtt()
{
    Json::Value sendJson;
    ros::Rate loop_rate(10);
    std::map<std::string,int> taskMap;
    taskMap.insert(std::pair<std::string,int>("robotstart",0));
    taskMap.insert(std::pair<std::string,int>("sucker",0));
    taskMap.insert(std::pair<std::string,int>("probe",0));
    taskMap.insert(std::pair<std::string,int>("sucker_grab",0));

    while (!exit_app.load())
    {
        // get the robot pose
        poseCallback(nullptr);

        sendJson.clear();
        sendJson = jointJson;
        for (const auto& key : poseJson.getMemberNames()) 
        {
            sendJson[key] = poseJson[key];
        }

        if(taskJson["order"] == "pgnd_inspection" && taskJson["subOrder"] == "up_handle_grabbed")
        {
            sendJson["probe"] = 1;
        }
        else
        {
            sendJson["probe"] = 0;
        }
        if(taskJson["order"] == "glass_inspection" && taskJson["subOrder"] == "up_suction_grabbed")
        {
            sendJson["sucker"] = 1;
        }
        else
        {
            sendJson["sucker"] = 0;
        }        
        if(taskJson["order"] == "glass_inspection" && taskJson["subOrder"] == "up_to_safe")
        {
            sendJson["sucker"] = 1;
            sendJson["sucker_grab"] = 1;
        }
        else
        {
            sendJson["sucker_grab"] = 0;
        }
        if(motionJson["motion"] == whi_interfaces::WhiMotionState::STA_STANDBY)
        {
            sendJson["robotstart"] = 0;
        }
        else
        {
            sendJson["robotstart"] = 1;
        }


        if (sendJson.isNull() || sendJson.empty())
        {
            ROS_INFO("sendJson mqtt is empty " );
        }
        else
        {
            Json::FastWriter writer;
	        std::string sendStr = writer.write(sendJson);
            if (myMqtt.getStart())
            {
                myMqtt.mosquittoPublish(mqtttopic, sendStr);
                // ROS_INFO_STREAM("mqtt topic " << mqtttopic << " and data " << sendStr);
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
    nd.getParam("task_state_topic", taskTopic);
    nd.getParam("det_topic", detTopic);
    nd.getParam("post_addr",postAddr);
    ROS_INFO("state_topic:%s , task_topic:%s , det_topic:%s",stateTopic.c_str(),taskTopic.c_str(),detTopic.c_str());   

    signal(SIGINT, signal_handler);

    ros::Subscriber subState = nd.subscribe<whi_interfaces::WhiBattery>(stateTopic, 10, stateCallback);
    ros::Subscriber subTask = nd.subscribe<whi_interfaces::WhiTaskState>(taskTopic, 10, taskCallback);
    ros::Subscriber subDetection = nd.subscribe<whi_interfaces::WhiBoundingBoxes>(detTopic, 10, detectionCallback);
    ros::ServiceServer serviceAction = nd.advertiseService("action", GetAction);

    std::thread senddataTh(senddataFun);
    

//---------------- handle mqtt -------------------------
    std::string mqttaddr;
    int mqttport;
    nd.getParam("mqtt_addr", mqttaddr);
    nd.getParam("mqtt_port", mqttport);
    nd.getParam("mqtt_topic", mqtttopic);

    std::string jointTopic,poseTopic,motionTopic;
    nd.param("joint_topic", jointTopic, std::string("/joint_state"));
    nd.param("pose_topic", poseTopic, std::string("/amcl_pose"));
    nd.param("motion_topic", motionTopic, std::string("/whi_motion_state"));

    ros::Subscriber subJoint = nd.subscribe<sensor_msgs::JointState>(jointTopic, 10, jointCallback);
    ros::Subscriber subMotion = nd.subscribe<whi_interfaces::WhiMotionState>(motionTopic, 10, motionstateCallback);
    //ros::Subscriber subPose = nd.subscribe<geometry_msgs::PoseWithCovarianceStamped>(poseTopic, 10, poseCallback);
    nd.param("base_link", base_link, std::string("base_link"));
    tf_listener = std::make_shared<tf2_ros::TransformListener>(buffer);
    if (!nd.getParam("offsets", offsets_idiot))
    {
        offsets_idiot.resize(2);
    }

    myMqtt.init(mqttaddr, mqttport);
    std::thread senddataMqttTh(senddataMqtt);

    ROS_INFO("Ready to client.");
    ros::spin();
    senddataTh.join();
    senddataMqttTh.join();
    
    return 0;
}
