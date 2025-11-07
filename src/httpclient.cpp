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
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <jsoncpp/json/json.h>
#include <iostream>
#include "whi_interfaces/msg/whi_battery.hpp"
#include "whi_interfaces/msg/whi_motion_state.hpp"
#include "whi_interfaces/msg/whi_bounding_box.hpp"
#include "whi_interfaces/msg/whi_bounding_boxes.hpp"
#include "whi_interfaces/msg/whi_task_state.hpp"
#include <csignal>
#include "whi_custom_interaction_example/mymqttclient.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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

// Global node pointer for callbacks
rclcpp::Node::SharedPtr g_node = nullptr;

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
    rclcpp::Rate rate(ratei);
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
                RCLCPP_INFO(g_node->get_logger(), "%s request timeout !, %d ", getStr.c_str(), gettime);
                return false;
            }

            Json::Value root;
            if (!jsonRoot(getres->body, root))
            {
                RCLCPP_INFO(g_node->get_logger(), "json::parse error!");
                return false;
            }

            int status = root["status"].asInt();
            std::string msgStr = root["msg"].asString();
            if (status == 100)
            {
                RCLCPP_INFO_STREAM(g_node->get_logger(), "request " << getStr << " succeed, with msg: " << msgStr);
                return true;
            }
            else if (status == 101)
            {
                RCLCPP_INFO(g_node->get_logger(), "%s request prohibit , msg:%s", getStr.c_str(), msgStr.c_str());
            }            
            else
            {
                RCLCPP_ERROR_STREAM(g_node->get_logger(), "request " << getStr << " exception, with msg: " << msgStr);
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
void ActionRequest(httplib::Client& Cli, const std::string& Apistr, std_srvs::srv::SetBool::Response::SharedPtr& Res)
{
    int ratei = 1;
    rclcpp::Rate rate(ratei) ;
    int mytimeout = timeout * ratei;
    int gettime = 0;
    httplib::Params params;
    std::string getStr = "/getRequest";
    getStr = "/api/shemt/" + Apistr;
    httplib::Headers headers = { { "Accept", "application/json" } };
    Res->success = false;
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
                RCLCPP_INFO(g_node->get_logger(), "%s request timeout !, %d", getStr.c_str(), gettime);
                Res->message = "time out";
                break;;
            }

            Json::Value root;
            if (!jsonRoot(getres->body, root))
            {
                RCLCPP_INFO(g_node->get_logger(), "json::parse error!");
                Res->message = "parse message from server error";
                break;
            }
            int status = root["status"].asInt();
            std::string msgStr = root["msg"].asString();
            if (status == 101)
            {
                // loop for next valid reponse
                RCLCPP_INFO(g_node->get_logger(), "%s request prohibit , msg:%s",getStr.c_str(), msgStr.c_str());
            }
            else if (status == 100)
            {
                RCLCPP_INFO(g_node->get_logger(), "%s request permit , msg:%s", getStr.c_str(), msgStr.c_str());
                // 如果是place或者reclaim ，需要在动作开始执行前 发送请求
                if (Apistr == "place" || Apistr == "reclaim")
                {
                    bool getStart = StartActionRequst(Cli, Apistr);
                    if (getStart)
                    {
                        Res->success = true;
                        Res->message = msgStr ;
                    }
                    else
                    {
                        msgStr = Apistr + "ing failed";
                        Res->success = false;
                        Res->message = msgStr ;
                    }
                }
                else
                {
                    Res->success = true;
                    Res->message = msgStr ;
                }

                break;
            }
            else if (status == 102)
            {
                Res->success = false;
                Res->message = "response with exception";
                RCLCPP_INFO_STREAM(g_node->get_logger(), Res->message);

                break;
            }
            else
            {
                Res->success = false;
                Res->message = "undefined response";
                RCLCPP_INFO_STREAM(g_node->get_logger(), Res->message);

                break;
            }
        }
        else
        {
            std::cout << "error code: " << getres.error() << std::endl;
            Res->success = false;
            Res->message = "web server connection failed";
            RCLCPP_INFO_STREAM(g_node->get_logger(), Res->message);

            break;
        }
        gettime++;
        rate.sleep();
    }
}

void RequestPlace(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_INFO(g_node->get_logger(), "sending request: place");
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);  
    if (request->data)
    {
        ActionRequest(cli, "place", response);
    }
    RCLCPP_INFO(g_node->get_logger(), "sending back response: [%d]", response->success);
}

void RequestPlaced(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_INFO(g_node->get_logger(), "sending request: placed");
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);      
    if (request->data)
    {
        ActionRequest(cli, "placed", response);
    }
    RCLCPP_INFO(g_node->get_logger(), "sending back response: [%d]", response->success);
}

void RequestReclaim(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_INFO(g_node->get_logger(), "sending request: reclaim");
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);      
    if (request->data)
    {
        ActionRequest(cli, "reclaim", response);
    }
    RCLCPP_INFO(g_node->get_logger(), "sending back response: [%d]", response->success);
}

void RequestReclaimed(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_INFO(g_node->get_logger(), "sending request: reclaimed");
    const char* hostaddr = host.c_str();
    httplib::Client cli(hostaddr, port);
    cli.set_connection_timeout(0, 800000);  
    cli.set_read_timeout(20, 0);      
    if (request->data)
    {
        ActionRequest(cli, "reclaimed", response);
    }
    RCLCPP_INFO(g_node->get_logger(), "sending back response: [%d]", response->success);
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
        rclcpp::shutdown();
    }
}

void stateCallback(const whi_interfaces::msg::WhiBattery::SharedPtr MsgBat)
{
    int battery = MsgBat->soc;
    stateJson["power"] = battery ;
}

void taskCallback(const whi_interfaces::msg::WhiTaskState::SharedPtr MsgTask)
{
    taskJson["order"] = MsgTask->parent_name;
    taskJson["subOrder"] = MsgTask->name;
}

void detectionCallback(const whi_interfaces::msg::WhiBoundingBoxes::SharedPtr MsgDet)
{
    RCLCPP_INFO(g_node->get_logger(), "in detection callback ");
    UpdateDet = true;
    detJson.clear();
    std::vector<whi_interfaces::msg::WhiBoundingBox> detResults(MsgDet->bounding_boxes);
    if(detResults.size() > 0)
    {
        std::string clsname = detResults.front().cls;
        std::string resultStr;
        RCLCPP_INFO(g_node->get_logger(), "clsname is %s",clsname.c_str());
        std::string substr = clsname.substr(0,4);
        RCLCPP_INFO(g_node->get_logger(), "substr is %s",substr.c_str());
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
}


void GetAction(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data)
    {
        Update = true;
        response->success = true;
    }
}

void senddataFun()
{
    const char* hostaddr = posthost.c_str();
    httplib::Client cli(hostaddr, postport);
    cli.set_connection_timeout(0, 800000); // 800 milliseconds
    cli.set_read_timeout(20, 0); // 20 seconds
    Json::Value sendJson;
    rclcpp::Rate loop_rate(1);
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
            // RCLCPP_INFO(g_node->get_logger(), "sendJson is empty");
        }
        else
        {
            Json::FastWriter writer;
	        std::string sendStr = writer.write(sendJson);
            RCLCPP_INFO(g_node->get_logger(), "sendstr data is: %s",sendStr.c_str());
            std::string poststr = "/" + postAddr;
            httplib::Headers headers = { { "content-type", "application/json" } };
            if ( auto res = cli.Post(poststr, headers, sendStr, "application/json"))
            {
                if (res->status == httplib::StatusCode::OK_200)
                {
                    RCLCPP_INFO(g_node->get_logger(), "POST success ,post data is: %s",sendStr.c_str());
                }
            }
            else
            {
                RCLCPP_INFO(g_node->get_logger(), "POST fail ,error ,%d ",res.error());
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
tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
std::shared_ptr<tf2_ros::TransformListener> tf_listener{ nullptr };
std::vector<double> offsets_idiot;

void jointCallback(const sensor_msgs::msg::JointState::SharedPtr MsgJoint)
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

geometry_msgs::msg::TransformStamped listenTf(const std::string& DstFrame, const std::string& SrcFrame,
    const rclcpp::Time& Time)
{
    try
    {
        return buffer.lookupTransform(DstFrame, SrcFrame, Time, rclcpp::Duration::from_seconds(1.0));
    }
    catch (tf2::TransformException &e)
    {
        RCLCPP_ERROR(g_node->get_logger(), "%s", e.what());
        return geometry_msgs::msg::TransformStamped();
    }
}

void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr MsgPose)
{
	auto trans = listenTf("map", base_link, rclcpp::Time(0));

    Json::Value item;
    poseJson["agv_x_axis"] = trans.transform.translation.x + offsets_idiot[0];
    poseJson["agv_y_axis"] = trans.transform.translation.y + offsets_idiot[1];

    tf2::Quaternion quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
        trans.transform.rotation.w);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
  	tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // RCLCPP_INFO(g_node->get_logger(), "roll:%f , pitch:%f , yaw:%f ",roll,pitch,yaw);
    item["rot"] = yaw;
    //poseJson.clear();
    poseJson["agv_angel"] = angles::to_degrees(yaw);
}

void motionstateCallback(const whi_interfaces::msg::WhiMotionState::SharedPtr MsgMotion)
{
    int state = MsgMotion->state;
    //RCLCPP_INFO(g_node->get_logger(), "motion state is %d",state);
    motionJson["motion"] = state ;
}

void senddataMqtt()
{
    Json::Value sendJson;
    rclcpp::Rate loop_rate(10);
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
        if(motionJson["motion"] == whi_interfaces::msg::WhiMotionState::STA_STANDBY)
        {
            sendJson["robotstart"] = 0;
        }
        else
        {
            sendJson["robotstart"] = 1;
        }

        if (sendJson.isNull() || sendJson.empty())
        {
            RCLCPP_INFO(g_node->get_logger(), "sendJson mqtt is empty " );
        }
        else
        {
            Json::FastWriter writer;
	        std::string sendStr = writer.write(sendJson);
            if (myMqtt.getStart())
            {
                myMqtt.mosquittoPublish(mqtttopic, sendStr);
                // RCLCPP_INFO_STREAM(g_node->get_logger(), "mqtt topic " << mqtttopic << " and data " << sendStr);
            }

        }


        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, ""); // for Chinese char: setlocale(LC_CTYPE, "zh_CN.utf8");

    const std::string nodeName("whi_custom_interaction_example");
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared(nodeName);

//---------get request -----------------------
    g_node->declare_parameter("host", "");
    g_node->declare_parameter("port", 0);
    g_node->declare_parameter("timeout", timeout);
    
    host = g_node->get_parameter("host").as_string();
    port = g_node->get_parameter("port").as_int();
    timeout = g_node->get_parameter("timeout").as_int();
    
    RCLCPP_INFO(g_node->get_logger(), "getparam host:%s , port:%d ,timeout:%d",host.c_str(),port,timeout);

    auto servicePlace = g_node->create_service<std_srvs::srv::SetBool>("place", RequestPlace);
    auto servicePlaced = g_node->create_service<std_srvs::srv::SetBool>("placed", RequestPlaced);
    auto serviceReclaim = g_node->create_service<std_srvs::srv::SetBool>("reclaim", RequestReclaim);
    auto serviceReclaimed = g_node->create_service<std_srvs::srv::SetBool>("reclaimed", RequestReclaimed);
    
//---------post states -----------------------

    g_node->declare_parameter("posthost", "");
    g_node->declare_parameter("postport", 0);
    posthost = g_node->get_parameter("posthost").as_string();
    postport = g_node->get_parameter("postport").as_int();
    RCLCPP_INFO(g_node->get_logger(), "post host:%s , post port:%d",posthost.c_str(),postport);   

    std::string stateTopic,taskTopic,detTopic;
    g_node->declare_parameter("state_topic", "");
    g_node->declare_parameter("task_state_topic", "");
    g_node->declare_parameter("det_topic", "");
    g_node->declare_parameter("post_addr", "");
    stateTopic = g_node->get_parameter("state_topic").as_string();
    taskTopic = g_node->get_parameter("task_state_topic").as_string();
    detTopic = g_node->get_parameter("det_topic").as_string();
    postAddr = g_node->get_parameter("post_addr").as_string();
    RCLCPP_INFO(g_node->get_logger(), "state_topic:%s , task_topic:%s , det_topic:%s",stateTopic.c_str(),taskTopic.c_str(),detTopic.c_str());   

    signal(SIGINT, signal_handler);

    auto subState = g_node->create_subscription<whi_interfaces::msg::WhiBattery>(stateTopic, 10, stateCallback);
    auto subTask = g_node->create_subscription<whi_interfaces::msg::WhiTaskState>(taskTopic, 10, taskCallback);
    auto subDetection = g_node->create_subscription<whi_interfaces::msg::WhiBoundingBoxes>(detTopic, 10, detectionCallback);
    auto serviceAction = g_node->create_service<std_srvs::srv::SetBool>("action", GetAction);

    std::thread senddataTh(senddataFun);
    

//---------------- handle mqtt -------------------------
    std::string mqttaddr;
    int mqttport;
    g_node->declare_parameter("mqtt_addr", "");
    g_node->declare_parameter("mqtt_port", 0);
    g_node->declare_parameter("mqtt_topic", "");
    mqttaddr = g_node->get_parameter("mqtt_addr").as_string();
    mqttport = g_node->get_parameter("mqtt_port").as_int();
    mqtttopic = g_node->get_parameter("mqtt_topic").as_string();

    std::string jointTopic,poseTopic,motionTopic;
    g_node->declare_parameter("joint_topic", "/joint_state");
    g_node->declare_parameter("pose_topic", "/amcl_pose");
    g_node->declare_parameter("motion_topic", "/whi_motion_state");
    jointTopic = g_node->get_parameter("joint_topic").as_string();
    poseTopic = g_node->get_parameter("pose_topic").as_string();
    motionTopic = g_node->get_parameter("motion_topic").as_string();

    auto subJoint = g_node->create_subscription<sensor_msgs::msg::JointState>(jointTopic, 10, jointCallback);
    auto subMotion = g_node->create_subscription<whi_interfaces::msg::WhiMotionState>(motionTopic, 10, motionstateCallback);
    //auto subPose = g_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(poseTopic, 10, poseCallback);
    g_node->declare_parameter("base_link", "base_link");
    base_link = g_node->get_parameter("base_link").as_string();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(buffer);
    g_node->declare_parameter("offsets", std::vector<double>{0.0, 0.0});
    offsets_idiot = g_node->get_parameter("offsets").as_double_array();
    if (offsets_idiot.size() != 2)
    {
        offsets_idiot.resize(2);
    }

    myMqtt.init(mqttaddr, mqttport);
    std::thread senddataMqttTh(senddataMqtt);

    RCLCPP_INFO(g_node->get_logger(), "Ready to client.");
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    senddataTh.join();
    senddataMqttTh.join();
    
    return 0;
}