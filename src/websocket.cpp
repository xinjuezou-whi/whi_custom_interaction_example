/******************************************************************
WebSocket robot motion control client for ROS with PCM Audio Support
Connects to WebSocket server and receives velocity commands

Features:
- Connect to WebSocket server as client
- Receive cmd_vel via WebSocket (JSON format)
- Send velocity commands to robot
- Send motion state, odometry, battery data to server
- Safety checks (E-Stop, collision detection)
- Built-in HTTP file receiver server
- PCM Audio playback support via Hikvision SDK with remote file download

Based on original teleop code by Yuhang Su
WebSocket client integration and HTTP file receiver added
PCM Audio functionality with remote download capability

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.
******************************************************************/

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <whi_interfaces/msg/whi_motion_state.hpp>
#include <whi_interfaces/msg/whi_rc_state.hpp>
#include <whi_interfaces/msg/whi_io.hpp>
#include <whi_interfaces/srv/whi_srv_io.hpp>
#include <whi_interfaces/msg/whi_battery.hpp>
#include <whi_interfaces/msg/whi_temperature_humidity.hpp>
#include <serial/serial.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <signal.h>
#include <functional>
#include <atomic>
#include <memory>
#include <jsoncpp/json/json.h>
#include <array>
#include <cstring>
#include <cmath>
#include <fstream>
#include <sstream>
#include <curl/curl.h>
#include <sys/stat.h>
#include <map>
#include <std_msgs/msg/int32.hpp>

#include <iomanip>
#include <ctime>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// HTTP服务器库
#include "whi_custom_interaction_example/httplib.h"

#include "HCNetSDK.h"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// 修改后的多点导航相关结构体定义
struct NavigationPoint {
    double x, y, yaw;
    double delay;  // 到达后的等待时间（秒）
    bool has_detection;  // 是否包含detection指令
    bool detection_enable;  // detection开启或关闭
    std::string video_src;  // 音频文件路径
    bool has_audio;  // 是否包含音频播放
};

// 前向函数声明
std::string getCurrentTimeString();
double calculateDistance(double x1, double y1, double x2, double y2);
void sendCombinedDataToServer();
bool navigateToGoal(double x, double y, double yaw = 0.0);
void cancelNavigation();
void navigationGoalResponseCallback(const NavGoalHandle::SharedPtr& goal_handle);
void navigationFeedbackCallback(NavGoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
void navigationResultCallback(const NavGoalHandle::WrappedResult& result);
// 多点导航函数声明
void startMultiPointNavigation(const std::vector<NavigationPoint>& points, bool repeat = false);
void continueMultiPointNavigation();
void cancelMultiPointNavigation();
void onNavDelayComplete();
// 新增：地图文件下载函数声明
bool downloadMapFile(const std::string& map_url, const std::string& local_filename);

typedef websocketpp::client<websocketpp::config::asio_client> client;

// PCM音频常量定义
#define PCM_FRAME_SIZE          1920    // PCM帧大小：1920字节
#define PCM_FRAME_DURATION_MS   40      // 帧持续时间：40ms

static const char* VERSION = "02.16.3-WebSocket-Client-With-FileReceiver-And-Remote-Audio";
static std::shared_ptr<rclcpp::Node> node = nullptr;
static double linear_min = 0.08;
static double linear_max = 1.0;
static double angular_min = 0.1;
static double angular_max = 0.8;

static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_unstamped;
using Twist = geometry_msgs::msg::TwistStamped;
static rclcpp::Publisher<Twist>::SharedPtr pub_twist;
static geometry_msgs::msg::Twist msg_twist_unstamped;
static Twist msg_twist;

static std::atomic_bool terminating = false;
static std::atomic_bool remote_mode = false;
static std::atomic_bool toggle_estop = false;
static std::atomic_bool toggle_collision = false;
static bool sw_estopped = false;
static bool use_stamped_vel = true;

// WebSocket client
static client ws_client;
static std::shared_ptr<std::thread> ws_thread = nullptr;
static std::atomic_bool ws_connected = false;
static websocketpp::connection_hdl ws_connection_hdl;
static std::atomic_bool ws_should_reconnect = true;
static int ws_reconnect_interval = 5; // 重连间隔（秒）

// HTTP文件接收服务器
static std::unique_ptr<httplib::Server> file_server = nullptr;
static std::shared_ptr<std::thread> file_server_thread = nullptr;
static int file_server_port = 8080;

// 文件上传配置
static std::string upload_server_url = "http://192.168.30.14:5566/sysFileInfo/uploadResultPath"; // 上传地址
static std::string local_files_directory = "/home/nvidia/robot_files";
static std::string audio_files_directory = "/home/nvidia/robot_audio";  // 音频文件目录
static bool file_monitor_enabled = true;
static std::map<std::string, time_t> file_modification_times;

// Modbus IO for lift and light control
static std::unique_ptr<serial::Serial> serial_inst_;
static int device_addr_ = 0x02;
static std::string serial_port_ = "/dev/ttyUART_485_2";
static int baudrate_ = 9600;

// IO addresses
static const int LIFT_UP_ADDR = 18;
static const int LIFT_DOWN_ADDR = 19;
static const int LIGHT_ADDR = 20;

// 统一设备控制变量
static LONG device_user_id_ = -1;           // 统一的设备登录ID
static LONG voice_handle_ = -1;             // 语音句柄
static std::atomic_bool is_voice_active_ = false;
static std::atomic_bool audio_debug_mode_ = false;
static std::string device_ip_ = "192.168.254.7";
static int device_port_ = 8000;
static std::string device_username_ = "admin";  
static std::string device_password_ = "jy100200300";
static int device_channel_ = 1;
static bool ptz_is_moving_ = false;
static DWORD ptz_default_speed_ = 3;  // 默认速度，范围1-7

// PTZ command constants
static const DWORD PTZ_TILT_UP = 21;
static const DWORD PTZ_TILT_DOWN = 22;
static const DWORD PTZ_PAN_LEFT = 23;
static const DWORD PTZ_PAN_RIGHT = 24;
static const DWORD PTZ_LIGHT_PWRON = 2;   // 接通灯光电源
static const DWORD PTZ_WIPER_PWRON = 3;   // 接通雨刷开关

// 预置点相关常量
static const DWORD PTZ_GOTO_PRESET = 39;   // 转到预置点命令

// Data for sending to server
static whi_interfaces::msg::WhiMotionState latest_motion_state;
static nav_msgs::msg::Odometry latest_odom;
static whi_interfaces::msg::WhiBattery latest_battery_data;
static whi_interfaces::msg::WhiTemperatureHumidity latest_temp_humidity_pm25;
static bool temp_humidity_pm25_received = false;

// 距离追踪相关变量
static double accumulated_distance = 0.0;  // 累积行驶距离
static bool is_first_position = true;      // 是否为第一次获取位置
static double last_position_x = 0.0;       // 上一次的X坐标（odom坐标系）
static double last_position_y = 0.0;       // 上一次的Y坐标（odom坐标系）

// 在全局变量声明部分添加tf2相关变量
static std::unique_ptr<tf2_ros::Buffer> tf_buffer = nullptr;
static std::shared_ptr<tf2_ros::TransformListener> tf_listener = nullptr;

// Navigation client
static rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client = nullptr;
static std::atomic_bool nav_active = false;
static NavGoalHandle::SharedPtr current_nav_goal_handle = nullptr;
// 在全局变量声明部分添加
static rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr detect_service_client = nullptr;
static std::atomic_bool detection_active = false;

// 多点导航相关变量
static std::vector<NavigationPoint> nav_points_sequence;
static size_t current_nav_point_index = 0;
static std::atomic_bool multi_point_nav_active = false;
static rclcpp::TimerBase::SharedPtr nav_delay_timer = nullptr;

// 计算两点之间的欧几里得距离
double calculateDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}
// 多点导航统计相关变量
static std::atomic<int> multi_nav_execution_count{0};  // 总执行次数
static std::chrono::steady_clock::time_point multi_nav_start_time;  // 当前执行的开始时间
static std::atomic<bool> multi_nav_timing_active{false};  // 是否正在计时
// 连接和任务统计相关变量
static std::atomic<bool> connection_timing_active{false};  // 是否正在计算连接时间
static std::chrono::steady_clock::time_point connection_start_time;  // 连接开始时间
static std::atomic<int> audio_play_count{0};  // 音频播放次数统计
// 在全局变量声明部分添加重复相关变量
static std::vector<NavigationPoint> original_nav_points_sequence;  // 原始导航序列
static std::atomic_bool nav_repeat_enabled = false;  // 是否启用无限重复
static int current_repeat_iteration = 0;  // 当前重复轮次
//在全局变量声明部分添加新的计数器（在现有计数器后面添加）
static std::atomic<int> total_charge_count{0};      // 总充电次数统计
static std::atomic<int> total_voice_call_count{0};  // 总语音呼叫次数统计

// 在全局变量声明部分添加持久化相关变量
static std::string stats_file_path = "/home/nvidia/robot_stats.json";
static std::mutex stats_file_mutex;  // 保护文件读写的互斥锁
// 在全局变量声明部分添加TaskGuid相关变量
static std::mutex task_guid_mutex;           // 保护TaskGuid的互斥锁
static std::string taskGuid = "";
static double task_start_distance = 0.0;  // 任务开始时的累积距离

// 2. 更新 PersistentStats 结构体（替换现有的结构体）
struct PersistentStats {
    int total_multi_nav_executions = 0;
    int total_audio_plays = 0;
    int total_charge_count = 0;        // 新增：总充电次数
    int total_voice_call_count = 0;    // 新增：总语音呼叫次数
    double total_connection_time = 0.0;
    double total_distance = 0.0;
    std::string last_updated;
};

static PersistentStats persistent_stats;

// 在全局变量声明部分添加一个结构体来存储当前点的音频信息
struct CurrentPointAudioInfo {
    bool has_audio = false;
    std::string video_src = "";
    size_t point_index = 0;
    int iteration = 0;
};

static CurrentPointAudioInfo current_point_audio_info;
static std::mutex audio_info_mutex;
// 在全局变量声明部分添加温度监控相关变量
#ifndef NET_DVR_GET_THERMOMETRYRULE_TEMPERATURE_INFO
#define NET_DVR_GET_THERMOMETRYRULE_TEMPERATURE_INFO 23001
#endif
static std::atomic_bool enable_temperature_monitoring_ = true;
static int temperature_rule_id_ = 1;
static int temperature_interval_ms_ = 2000;
static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub = nullptr;
static rclcpp::TimerBase::SharedPtr temperature_timer = nullptr;
static std::chrono::steady_clock::time_point last_detection_time = std::chrono::steady_clock::now();
static rclcpp::TimerBase::SharedPtr no_detection_timer = nullptr;
static rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ptz_home_service = nullptr;
// 声源定位TCP服务器
static std::unique_ptr<std::thread> sound_server_thread = nullptr;
static std::atomic_bool sound_server_running{false};

// 添加温度获取函数
bool getTemperatureInfo(int rule_id, float& max_temperature)
{
    if (device_user_id_ < 0) {
        RCLCPP_WARN(node->get_logger(), "Device not connected, cannot get temperature info");
        return false;
    }
    
    NET_DVR_THERMOMETRYRULE_TEMPERATURE_INFO temp_info;
    memset(&temp_info, 0, sizeof(NET_DVR_THERMOMETRYRULE_TEMPERATURE_INFO));
    
    DWORD dwReturned = 0;
    
    // 调用API获取温度信息
    BOOL result = NET_DVR_GetDVRConfig(
        device_user_id_,                                           // lUserID
        NET_DVR_GET_THERMOMETRYRULE_TEMPERATURE_INFO,              // dwCommand (23001)
        rule_id,                                                   // lChannel (规则ID)
        &temp_info,                                                // lpOutBuffer
        sizeof(NET_DVR_THERMOMETRYRULE_TEMPERATURE_INFO),          // dwOutBufferSize
        &dwReturned                                                // lpBytesReturned
    );
    
    if (!result) {
        DWORD error = NET_DVR_GetLastError();
        RCLCPP_ERROR(node->get_logger(), 
                    "NET_DVR_GetDVRConfig failed for temperature info, rule ID: %d, error code: %d", 
                    rule_id, error);
        return false;
    }
    
    max_temperature = temp_info.fMaxTemperature;
    
    if (audio_debug_mode_.load()) {
        RCLCPP_DEBUG(node->get_logger(), 
                    "Successfully got temperature info for rule %d, max temp: %.2f°C", 
                    rule_id, max_temperature);
    }
    
    return true;
}
// 发布温度信息函数
void publishTemperatureInfo()
{
    if (!enable_temperature_monitoring_.load() || !temperature_pub) {
        return;
    }
    
    float max_temperature = 0.0f;
    
    if (!getTemperatureInfo(temperature_rule_id_, max_temperature)) {
        return;
    }
    
    // 发布最高温度
    auto temp_msg = std_msgs::msg::Float32();
    temp_msg.data = max_temperature;
    temperature_pub->publish(temp_msg);
    
    //RCLCPP_INFO(node->get_logger(), "Published max temperature: %.2f°C (Rule ID: %d)", 
               //max_temperature, temperature_rule_id_);
}

// 温度定时器回调函数
void temperatureTimerCallback()
{
    publishTemperatureInfo();
}

// 读取持久化统计数据
bool loadPersistentStats() {
    std::lock_guard<std::mutex> lock(stats_file_mutex);
    
    std::ifstream file(stats_file_path);
    if (!file.is_open()) {
        RCLCPP_INFO(node->get_logger(), "Stats file not found, starting with zero statistics");
        return false;
    }
    
    try {
        Json::Value root;
        Json::Reader reader;
        
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        file.close();
        
        if (!reader.parse(content, root)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to parse stats file: %s", 
                        reader.getFormattedErrorMessages().c_str());
            return false;
        }
        
        // 读取各项统计数据
        if (root.isMember("total_multi_nav_executions")) {
            persistent_stats.total_multi_nav_executions = root["total_multi_nav_executions"].asInt();
        }
        if (root.isMember("total_audio_plays")) {
            persistent_stats.total_audio_plays = root["total_audio_plays"].asInt();
        }
        if (root.isMember("total_connection_time")) {
            persistent_stats.total_connection_time = root["total_connection_time"].asDouble();
        }
        if (root.isMember("total_distance")) {
            persistent_stats.total_distance = root["total_distance"].asDouble();
        }
        if (root.isMember("total_charge_count")) {
            persistent_stats.total_charge_count = root["total_charge_count"].asInt();
        }
        if (root.isMember("total_voice_call_count")) {
            persistent_stats.total_voice_call_count = root["total_voice_call_count"].asInt();
        }
        if (root.isMember("last_updated")) {
            persistent_stats.last_updated = root["last_updated"].asString();
        }
        
        // 设置当前运行时的计数器为持久化的值
        multi_nav_execution_count.store(persistent_stats.total_multi_nav_executions);
        audio_play_count.store(persistent_stats.total_audio_plays);
        accumulated_distance = persistent_stats.total_distance;
        total_charge_count.store(persistent_stats.total_charge_count);
        total_voice_call_count.store(persistent_stats.total_voice_call_count);
        
        // 更新日志输出 - 显示分钟
        RCLCPP_INFO(node->get_logger(), 
                   "Loaded persistent statistics: nav_executions=%d, audio_plays=%d, charge_count=%d, voice_calls=%d, total_distance=%.2fm, connection_time=%.2f minutes", 
                   persistent_stats.total_multi_nav_executions,
                   persistent_stats.total_audio_plays,
                   persistent_stats.total_charge_count,
                   persistent_stats.total_voice_call_count,
                   persistent_stats.total_distance,
                   persistent_stats.total_connection_time / 60.0);  // 转换为分钟显示
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error loading stats file: %s", e.what());
        file.close();
        return false;
    }
}

// 保存持久化统计数据
bool savePersistentStats() {
    std::lock_guard<std::mutex> lock(stats_file_mutex);
    
    try {
        // 更新当前统计数据
        persistent_stats.total_multi_nav_executions = multi_nav_execution_count.load();
        persistent_stats.total_audio_plays = audio_play_count.load();
        persistent_stats.total_distance = accumulated_distance;
        persistent_stats.total_charge_count = total_charge_count.load();
        persistent_stats.total_voice_call_count = total_voice_call_count.load();
        
        // 计算总连接时间
        if (connection_timing_active.load()) {
            auto current_time = std::chrono::steady_clock::now();
            double current_session_time = std::chrono::duration<double>(current_time - connection_start_time).count();
            persistent_stats.total_connection_time += current_session_time;
        }
        
        persistent_stats.last_updated = getCurrentTimeString();
        
        // 构建JSON对象
        Json::Value root;
        root["total_multi_nav_executions"] = persistent_stats.total_multi_nav_executions;
        root["total_audio_plays"] = persistent_stats.total_audio_plays;
        root["total_connection_time"] = persistent_stats.total_connection_time;
        root["total_distance"] = persistent_stats.total_distance;
        root["last_updated"] = persistent_stats.last_updated;
        // 在构建JSON对象部分添加：
        root["total_charge_count"] = persistent_stats.total_charge_count;
        root["total_voice_call_count"] = persistent_stats.total_voice_call_count;
        
        // 写入文件
        std::ofstream file(stats_file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to open stats file for writing: %s", stats_file_path.c_str());
            return false;
        }
        
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(root, &file);
        file.close();
        
        RCLCPP_DEBUG(node->get_logger(), "Statistics saved successfully");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error saving stats file: %s", e.what());
        return false;
    }
}

// 统一的设备异常回调函数
void CALLBACK deviceExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
    switch(dwType)
    {
    case EXCEPTION_RECONNECT:
        RCLCPP_INFO(node->get_logger(), "Device reconnecting...");
        break;
    default:
        break;
    }
}

void CALLBACK voicePCMDataCallBack(LONG lVoiceComHandle, char *pRecvDataBuffer, 
                                  DWORD dwBufSize, BYTE byAudioFlag, void* pUser)
{
    if (audio_debug_mode_.load()) {
        RCLCPP_DEBUG(node->get_logger(), "Received PCM data: size=%d", dwBufSize);
    }
}

// 在initDevice函数的最后添加温度监控初始化
bool initDevice()
{
    // 初始化SDK
    NET_DVR_Init();
    
    // 设置连接时间与重连时间
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(10000, true);
    
    // 设置异常消息回调函数
    NET_DVR_SetExceptionCallBack_V30(0, NULL, deviceExceptionCallBack, NULL);
    
    // 改用与第一个代码相同的登录方式
    NET_DVR_DEVICEINFO_V30 struDeviceInfo = {0};
    
    device_user_id_ = NET_DVR_Login_V30(
        (char*)device_ip_.c_str(),
        device_port_,
        (char*)device_username_.c_str(),
        (char*)device_password_.c_str(),
        &struDeviceInfo
    );
    
    if (device_user_id_ < 0)
    {
        RCLCPP_ERROR(node->get_logger(), "Device login failed, error code: %d", NET_DVR_GetLastError());
        return false;
    }
    
    RCLCPP_INFO(node->get_logger(), "Device connected successfully to %s", device_ip_.c_str());
    
    // 初始化温度监控（如果启用）
    if (enable_temperature_monitoring_.load()) {
        //RCLCPP_INFO(node->get_logger(), "Temperature monitoring enabled - Rule ID: %d, Interval: %dms", 
                   //temperature_rule_id_, temperature_interval_ms_);
    }
    
    return true;
}

// 音频控制函数
bool startVoiceTransfer()
{
    if (device_user_id_ < 0) {
        RCLCPP_ERROR(node->get_logger(), "Device not connected");
        return false;
    }
    
    if (is_voice_active_.load()) {
        RCLCPP_WARN(node->get_logger(), "Voice transfer already active");
        return true;
    }

    voice_handle_ = NET_DVR_StartVoiceCom_MR_V30(device_user_id_, device_channel_, voicePCMDataCallBack, nullptr);
    if (voice_handle_ < 0)
    {
        RCLCPP_ERROR(node->get_logger(), "NET_DVR_StartVoiceCom_MR_V30 failed, error code: %d", 
                    NET_DVR_GetLastError());
        return false;
    }
    
    is_voice_active_.store(true);
    RCLCPP_INFO(node->get_logger(), "PCM voice transfer started successfully");

    return true;
}

bool stopVoiceTransfer()
{
    if (voice_handle_ >= 0 && is_voice_active_.load()) {
        if (!NET_DVR_StopVoiceCom(voice_handle_))
        {
            RCLCPP_ERROR(node->get_logger(), "NET_DVR_StopVoiceCom failed, error code: %d", 
                        NET_DVR_GetLastError());
            return false;
        }
        
        voice_handle_ = -1;
        is_voice_active_.store(false);
        RCLCPP_INFO(node->get_logger(), "Voice transfer stopped");
    }
    
    return true;
}

bool sendPCMData(const unsigned char* data, int size)
{
    if (!is_voice_active_.load() || voice_handle_ < 0) {
        RCLCPP_WARN(node->get_logger(), "Voice transfer not active");
        return false;
    }
    
    if (!NET_DVR_VoiceComSendData(voice_handle_, (char*)data, size))
    {
        RCLCPP_ERROR(node->get_logger(), "NET_DVR_VoiceComSendData failed, error code: %d", 
                    NET_DVR_GetLastError());
        return false;
    }
    
    if (audio_debug_mode_.load()) {
        RCLCPP_DEBUG(node->get_logger(), "Sent PCM data: %d bytes", size);
    }
    
    return true;
}

// 获取当前时间字符串 - 修改为新格式
std::string getCurrentTimeString() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y:%m:%d:%H:%M:%S");  // 改为冒号分隔格式
    return oss.str();
}

// 确保目录存在
bool ensureDirectoryExists(const std::string& dir) {
    struct stat st = {0};
    if (stat(dir.c_str(), &st) == -1) {
        if (mkdir(dir.c_str(), 0755) != 0) {
            RCLCPP_ERROR(node->get_logger(), "Failed to create directory: %s", dir.c_str());
            return false;
        }
    }
    return true;
}

// 保存文件到本地
bool saveFile(const std::string& filename, const std::string& content) {
    if (!ensureDirectoryExists(local_files_directory)) {
        return false;
    }
    
    std::string filepath = local_files_directory + "/" + filename;
    std::ofstream file(filepath, std::ios::binary);
    if (!file) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create file: %s", filepath.c_str());
        return false;
    }
    
    file.write(content.c_str(), content.size());
    file.close();
    
    RCLCPP_INFO(node->get_logger(), "File saved: %s (%zu bytes)", filepath.c_str(), content.size());
    return true;
}

// 启动HTTP文件接收服务器
void startFileReceiver() {
    file_server = std::make_unique<httplib::Server>();
    
    // 文件上传接口
    file_server->Post("/upload", [](const httplib::Request& req, httplib::Response& res) {
        auto file = req.get_file_value("file");
        if (file.filename.empty()) {
            res.set_content("{\"success\":false,\"message\":\"No file provided\"}", "application/json");
            res.status = 400;
            return;
        }
        
        RCLCPP_INFO(node->get_logger(), "Receiving file: %s (%zu bytes)", 
                    file.filename.c_str(), file.content.size());
        
        if (saveFile(file.filename, file.content)) {
            res.set_content("{\"success\":true,\"message\":\"File uploaded successfully\"}", "application/json");
        } else {
            res.set_content("{\"success\":false,\"message\":\"Failed to save file\"}", "application/json");
            res.status = 500;
        }
    });
    
    // 设置CORS头 (允许跨域请求)
    file_server->set_pre_routing_handler([](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "Content-Type");
        return httplib::Server::HandlerResponse::Unhandled;
    });
    
    // 处理OPTIONS请求 (CORS预检)
    file_server->Options(".*", [](const httplib::Request&, httplib::Response& res) {
        return;
    });
    
    if (!ensureDirectoryExists(local_files_directory)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create local directory");
        return;
    }
    
    if (!file_server->listen("0.0.0.0", file_server_port)) {
        //RCLCPP_ERROR(node->get_logger(), "Failed to start file receiver server on port %d", file_server_port);
    }
}

// HTTP响应回调函数
size_t WriteCallback(void *contents, size_t size, size_t nmemb, std::string *s) {
    size_t newLength = size * nmemb;
    try {
        s->append((char*)contents, newLength);
    } catch(std::bad_alloc &e) {
        return 0;
    }
    return newLength;
}

// 文件下载回调函数
size_t WriteFileCallback(void *contents, size_t size, size_t nmemb, FILE *stream) {
    return fwrite(contents, size, nmemb, stream);
}

// 下载地图文件（使用现有的POST API方案）
bool downloadMapFile(const std::string& map_url, const std::string& local_filename) {
    // 使用现有的POST下载API端点
    std::string download_url = "http://192.168.30.235:5566/sysFileInfo/download/string";
    std::string local_path = local_files_directory + "/" + local_filename;
  
    // 确保文件目录存在
    if (!ensureDirectoryExists(local_files_directory)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create files directory");
        return false;
    }
    
    CURL *curl;
    FILE *fp;
    CURLcode res;
    struct curl_slist *headers = NULL;
    
    curl = curl_easy_init();
    if (!curl) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize CURL for map download");
        return false;
    }
    
    fp = fopen(local_path.c_str(), "wb");
    if (!fp) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open local file for writing: %s", local_path.c_str());
        curl_easy_cleanup(curl);
        return false;
    }
    
    // 准备JSON数据 - 与音频下载相同的格式，只传文件名
    Json::Value json_data;
    json_data["name"] = local_filename;  // 只传文件名
    
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string json_string = Json::writeString(builder, json_data);
    
    // 设置HTTP头
    headers = curl_slist_append(headers, "Content-Type: application/json");
    
    // 配置CURL选项 - 与音频下载完全相同
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
    curl_easy_setopt(curl, CURLOPT_URL, download_url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_string.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteFileCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    
    res = curl_easy_perform(curl);
    
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    
    // 清理资源
    fclose(fp);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
        RCLCPP_ERROR(node->get_logger(), "CURL map download failed: %s", curl_easy_strerror(res));
        remove(local_path.c_str());
        return false;
    }
    
    if (http_code != 200) {
        RCLCPP_ERROR(node->get_logger(), "HTTP error code: %ld for map download request", http_code);
        remove(local_path.c_str());
        return false;
    }
    
    RCLCPP_INFO(node->get_logger(), "Map file downloaded successfully: %s", local_path.c_str());
    return true;
}

// 修正后的下载函数，使用POST方法下载文件
bool downloadAudioFile(const std::string& remote_path, const std::string& local_filename) {
    // 使用POST下载API端点
    std::string download_url = "http://192.168.30.235:5566/sysFileInfo/download/string";
    std::string local_path = audio_files_directory + "/" + local_filename;
  
    // 确保音频目录存在
    if (!ensureDirectoryExists(audio_files_directory)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create audio directory");
        return false;
    }
    
    CURL *curl;
    FILE *fp;
    CURLcode res;
    struct curl_slist *headers = NULL;
    
    curl = curl_easy_init();
    if (!curl) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize CURL");
        return false;
    }
    
    fp = fopen(local_path.c_str(), "wb");
    if (!fp) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open local file for writing: %s", local_path.c_str());
        curl_easy_cleanup(curl);
        return false;
    }
    
    // 准备JSON数据 - 直接使用文件名
    Json::Value json_data;
    json_data["name"] = local_filename;  // 只传文件名，如：717743559548998.pcm
    
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string json_string = Json::writeString(builder, json_data);
    
    // 设置HTTP头
    headers = curl_slist_append(headers, "Content-Type: application/json");
    
    // 配置CURL选项
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "POST");
    curl_easy_setopt(curl, CURLOPT_URL, download_url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_string.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteFileCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
    
    res = curl_easy_perform(curl);
    
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    
    // 清理资源
    fclose(fp);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
        RCLCPP_ERROR(node->get_logger(), "CURL download failed: %s", curl_easy_strerror(res));
        remove(local_path.c_str());
        return false;
    }
    
    if (http_code != 200) {
        RCLCPP_ERROR(node->get_logger(), "HTTP error code: %ld for download request", http_code);
        remove(local_path.c_str());
        return false;
    }
    
    RCLCPP_INFO(node->get_logger(), "Audio file downloaded successfully: %s", local_path.c_str());
    return true;
}

// 处理PCM音频文件
bool processAudioFile(const std::string& filename)
{

    FILE* m_hStreamFile = fopen(filename.c_str(), "rb");
    if (m_hStreamFile == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Cannot open PCM audio file: %s", filename.c_str());
        return false;
    }
    
    // 停止之前的语音转发
    if (is_voice_active_.load()) {
        stopVoiceTransfer();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // 启动语音转发
    RCLCPP_INFO(node->get_logger(), "Starting PCM voice transfer...");
    voice_handle_ = NET_DVR_StartVoiceCom_MR_V30(device_user_id_, device_channel_, voicePCMDataCallBack, nullptr);
    if (voice_handle_ < 0) {
        DWORD error = NET_DVR_GetLastError();
        RCLCPP_ERROR(node->get_logger(), "NET_DVR_StartVoiceCom_MR_V30 failed, error code: %d", error);
        fclose(m_hStreamFile);
        return false;
    }
    
    is_voice_active_.store(true);
    RCLCPP_INFO(node->get_logger(), "PCM voice transfer started, handle: %ld", voice_handle_);
    
    // 等待语音通道完全建立
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // 严格按照测试代码处理PCM数据
    BYTE pBuf[PCM_FRAME_SIZE];  // 1920字节缓冲区
    BOOL bBufFull = FALSE;
    DWORD dwSize = PCM_FRAME_SIZE;  // 1920
    DWORD dwDataLen = 0;
    int frame_count = 0;
    
    RCLCPP_INFO(node->get_logger(), "Starting PCM audio processing loop...");
    
    while (!terminating.load()) {
        if (!bBufFull) {
            // 每次读取1920字节
            dwDataLen = fread(pBuf, 1, dwSize, m_hStreamFile);
            if (dwDataLen == 0) {
                RCLCPP_INFO(node->get_logger(), "PCM file end reached");
                break;  // 文件结束
            } else {
                if (pBuf != nullptr) {
                    // 直接发送PCM音频数据给设备
                    if (!NET_DVR_VoiceComSendData(voice_handle_, (char*)pBuf, PCM_FRAME_SIZE)) {
                        RCLCPP_ERROR(node->get_logger(), "NET_DVR_VoiceComSendData failed, error: %d", 
                                   NET_DVR_GetLastError());
                        break;
                    }
                    
                    frame_count++;
                    if (audio_debug_mode_.load() || frame_count % 25 == 0) {  // 每秒打印一次
                        //RCLCPP_INFO(node->get_logger(), "Sent PCM frame %d, size: %d bytes", 
                                   //frame_count, dwDataLen);
                    }
                }
                // 关键：40ms延迟
                std::this_thread::sleep_for(std::chrono::milliseconds(PCM_FRAME_DURATION_MS));
            }
        } else {
            break;
        }
    }
    
    fclose(m_hStreamFile);
    
    RCLCPP_INFO(node->get_logger(), "PCM audio processing completed, %d frames sent", frame_count);
    
    return frame_count > 0;
}

// 添加检测服务控制函数
bool callDetectionService(bool enable)
{
    if (!detect_service_client) {
        RCLCPP_ERROR(node->get_logger(), "Detection service client not initialized");
        return false;
    }
    
    if (!detect_service_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "Detection service not available");
        return false;
    }
    
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    
    try {
        auto future = detect_service_client->async_send_request(request);
        
        // 等待响应，设置超时时间
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                detection_active.store(enable);
                RCLCPP_INFO(node->get_logger(), "Detection service %s successfully: %s", 
                           enable ? "enabled" : "disabled", response->message.c_str());
                return true;
            } else {
                RCLCPP_ERROR(node->get_logger(), "Detection service call failed: %s", 
                            response->message.c_str());
                return false;
            }
        } else {
            RCLCPP_ERROR(node->get_logger(), "Detection service call timeout");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception calling detection service: %s", e.what());
        return false;
    }
}

void movePTZWithSpeed(DWORD command, DWORD speed = 0)
{
    if (device_user_id_ < 0) {
        RCLCPP_ERROR(node->get_logger(), "PTZ device not connected");
        return;
    }
    
    // 如果没有指定速度，使用默认速度
    if (speed == 0) {
        speed = ptz_default_speed_;
    }
    
    // 限制速度范围在1-7之间
    speed = std::max(1u, std::min(7u, speed));
    
    // 开始运动 (dwStop = 0) 使用带速度的API
    if (!NET_DVR_PTZControlWithSpeed_Other(device_user_id_, device_channel_, command, 0, speed))
    {
        RCLCPP_ERROR(node->get_logger(), "PTZ move with speed failed, error code: %d", NET_DVR_GetLastError());
        return;
    }
    
    ptz_is_moving_ = true;
    RCLCPP_INFO(node->get_logger(), "PTZ moving with command: %d, speed: %d", command, speed);
}

void stopPTZ()
{
    if (device_user_id_ < 0) {
        return;
    }
    
    if (ptz_is_moving_) {
        // 停止所有可能的运动 (dwStop = 1) 使用带速度的API
        NET_DVR_PTZControlWithSpeed_Other(device_user_id_, device_channel_, PTZ_TILT_UP, 1, ptz_default_speed_);
        NET_DVR_PTZControlWithSpeed_Other(device_user_id_, device_channel_, PTZ_TILT_DOWN, 1, ptz_default_speed_);
        NET_DVR_PTZControlWithSpeed_Other(device_user_id_, device_channel_, PTZ_PAN_LEFT, 1, ptz_default_speed_);
        NET_DVR_PTZControlWithSpeed_Other(device_user_id_, device_channel_, PTZ_PAN_RIGHT, 1, ptz_default_speed_);
        
        ptz_is_moving_ = false;
        RCLCPP_INFO(node->get_logger(), "PTZ stopped");
    }
}

void togglePTZLight(bool on)
{
    if (device_user_id_ < 0) {
        RCLCPP_ERROR(node->get_logger(), "PTZ device not connected");
        return;
    }
    
    if (!NET_DVR_PTZControl_Other(device_user_id_, device_channel_, PTZ_LIGHT_PWRON, on ? 0 : 1))
    {
        RCLCPP_ERROR(node->get_logger(), "PTZ light control failed, error code: %d", NET_DVR_GetLastError());
        return;
    }
    
    RCLCPP_INFO(node->get_logger(), "PTZ light %s", on ? "ON" : "OFF");
}

void togglePTZWiper(bool on)
{
    if (device_user_id_ < 0) {
        RCLCPP_ERROR(node->get_logger(), "PTZ device not connected");
        return;
    }
    
    if (!NET_DVR_PTZControl_Other(device_user_id_, device_channel_, PTZ_WIPER_PWRON, on ? 0 : 1))
    {
        RCLCPP_ERROR(node->get_logger(), "PTZ wiper control failed, error code: %d", NET_DVR_GetLastError());
        return;
    }
    
    RCLCPP_INFO(node->get_logger(), "PTZ wiper %s", on ? "ON" : "OFF");
}

void ptzUp(DWORD speed = 0)
{
    movePTZWithSpeed(PTZ_TILT_UP, speed);
}

void ptzDown(DWORD speed = 0)
{
    movePTZWithSpeed(PTZ_TILT_DOWN, speed);
}

void ptzLeft(DWORD speed = 0)
{
    movePTZWithSpeed(PTZ_PAN_LEFT, speed);
}

void ptzRight(DWORD speed = 0)
{
    movePTZWithSpeed(PTZ_PAN_RIGHT, speed);
}

bool gotoPreset(int preset_index)
{
    if (device_user_id_ < 0) {
        RCLCPP_ERROR(node->get_logger(), "PTZ device not connected");
        return false;
    }
    
    if (preset_index < 1 || preset_index > 255) {
        RCLCPP_ERROR(node->get_logger(), "Invalid preset index: %d (should be 1-255)", preset_index);
        return false;
    }
    
    RCLCPP_INFO(node->get_logger(), "Going to PTZ preset point: %d", preset_index);
    
    if (!NET_DVR_PTZPreset_Other(device_user_id_, device_channel_, PTZ_GOTO_PRESET, preset_index))
    {
        DWORD error_code = NET_DVR_GetLastError();
        RCLCPP_ERROR(node->get_logger(), 
                    "NET_DVR_PTZPreset_Other (GOTO_PRESET) failed, error code: %d", error_code);
        return false;
    }
    
    RCLCPP_INFO(node->get_logger(), "Successfully moved to PTZ preset point: %d", preset_index);
    return true;
}

// 声源定位TCP服务器
void soundSourceServer()
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create sound server socket");
        return;
    }
    
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(10000);
    
    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to bind sound server: %s", strerror(errno));
        close(server_fd);
        return;
    }
    
    listen(server_fd, 1);
    RCLCPP_INFO(node->get_logger(), "Sound source server started on port 10000");
    sound_server_running.store(true);
    
    while (sound_server_running.load() && !terminating.load()) {
        struct timeval tv = {1, 0};
        setsockopt(server_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        
        if (client_fd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;  // 超时，继续等待
            }
            RCLCPP_WARN(node->get_logger(), "Accept failed: %s", strerror(errno));
            continue;
        }
        
        // 获取客户端IP
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        RCLCPP_INFO(node->get_logger(), "Maix connected from %s", client_ip);
        
        // 设置socket选项：保持连接
        int keepalive = 1;
        setsockopt(client_fd, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        
        send(client_fd, "OK\n", 3, 0);
        
        char buffer[256];
        std::string data_buf;
        
        while (sound_server_running.load() && !terminating.load()) {
            memset(buffer, 0, sizeof(buffer));
            int n = recv(client_fd, buffer, sizeof(buffer)-1, 0);
            
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;  // 超时，继续接收
                }
                RCLCPP_WARN(node->get_logger(), "Recv error: %s, disconnecting", strerror(errno));
                break;
            }
            
            if (n == 0) {
                RCLCPP_INFO(node->get_logger(), "Maix disconnected");
                break;
            }
            
            buffer[n] = 0;
            data_buf += buffer;
            
            size_t pos;
            while ((pos = data_buf.find('\n')) != std::string::npos) {
                std::string line = data_buf.substr(0, pos);
                data_buf.erase(0, pos + 1);
                
                // 跳过空行
                if (line.empty()) continue;
                
                size_t comma = line.find(',');
                if (comma != std::string::npos) {
                    try {
                        int angle = std::stoi(line.substr(0, comma));
                        int intensity = std::stoi(line.substr(comma + 1));
                        
                        RCLCPP_DEBUG(node->get_logger(), 
                            "Received: angle=%d°, intensity=%d", angle, intensity);
                        
                        if (intensity > 5) {
                            int preset = ((angle - 1) / 30) + 1;
                            if (preset >= 1 && preset <= 12) {
                                RCLCPP_INFO(node->get_logger(), 
                                    "Sound: %d° intensity:%d -> preset %d", 
                                    angle, intensity, preset);
                                gotoPreset(preset);
                            }
                        }
                        
                        // 发送确认
                        const char* ack = "OK\n";
                        if (send(client_fd, ack, strlen(ack), MSG_NOSIGNAL) < 0) {
                            RCLCPP_WARN(node->get_logger(), "Send ACK failed: %s", strerror(errno));
                            break;
                        }
                    } catch(const std::exception& e) {
                        RCLCPP_WARN(node->get_logger(), "Parse error: %s", e.what());
                    }
                }
            }
        }
        
        close(client_fd);
        RCLCPP_INFO(node->get_logger(), "Client connection closed");
    }
    
    close(server_fd);
    sound_server_running.store(false);
    RCLCPP_INFO(node->get_logger(), "Sound source server stopped");
}

// 添加服务回调函数（可以放在gotoPreset函数之后，约在第625行附近）
void ptzHomeServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request; // 未使用的参数
    
    //RCLCPP_INFO(node->get_logger(), "Received PTZ home position service call");
    
    // 调用预置点34
    if (gotoPreset(34)) {
        response->success = true;
        response->message = "Successfully moved PTZ to home position (preset 34)";
        RCLCPP_INFO(node->get_logger(), "PTZ home service: Success");
    } else {
        response->success = false;
        response->message = "Failed to move PTZ to home position (preset 34)";
        RCLCPP_ERROR(node->get_logger(), "PTZ home service: Failed");
    }
}

// 在cleanupDevice函数中添加温度监控清理
void cleanupDevice()
{
    RCLCPP_INFO(node->get_logger(), "Cleaning up device system");
    
    // 停止温度定时器
    if (temperature_timer) {
        temperature_timer->cancel();
        temperature_timer = nullptr;
    }
    
    // 停止PTZ移动
    stopPTZ();
    
    // 停止音频相关功能
    stopVoiceTransfer();
    NET_DVR_ClientAudioStop();
    
    // 登出设备
    if (device_user_id_ >= 0) {
        NET_DVR_Logout(device_user_id_);
        RCLCPP_INFO(node->get_logger(), "Logged out from device");
        device_user_id_ = -1;
    }
    
    NET_DVR_Cleanup();
    RCLCPP_INFO(node->get_logger(), "Device SDK cleanup completed");
}

void publishVelocity(double linear_x, double angular_z)
{
    // 限制速度范围
    if (fabs(linear_x) > linear_max) {
        linear_x = (linear_x > 0) ? linear_max : -linear_max;
    }
    if (fabs(angular_z) > angular_max) {
        angular_z = (angular_z > 0) ? angular_max : -angular_max;
    }

    msg_twist.header.stamp = node->get_clock()->now();
    msg_twist.twist.linear.x = linear_x;
    msg_twist.twist.angular.z = angular_z;

    if (pub_twist) {
        pub_twist->publish(msg_twist);
    } else {
        msg_twist_unstamped.linear.x = linear_x;
        msg_twist_unstamped.angular.z = angular_z;
        pub_twist_unstamped->publish(msg_twist_unstamped);
    }

    RCLCPP_INFO(node->get_logger(), "WebSocket cmd: linear %.2f, angular %.2f", 
                linear_x, angular_z);
}

// Modbus CRC16 calculation
uint16_t crc16(const uint8_t* Data, size_t Length)
{
    uint16_t crc = 0xffff;
    uint16_t polynomial = 0xa001;

    for (size_t i = 0; i < Length; ++i)
    {
        crc ^= Data[i];
        for (int j = 0; j < 8; ++j)
        {
            if ((crc & 0x0001))
            {
                crc = (crc >> 1) ^ polynomial;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void composeModbusData(int addr, int level, std::array<uint8_t, 8>& data)
{
    data[0] = uint8_t(device_addr_);
    data[1] = 0x05;  // Write single coil
    data[2] = 0;
    data[3] = uint8_t(addr - 1);
    data[4] = 0;
    data[5] = level;
    uint16_t crc = crc16(data.data(), data.size() - 2);
    data[6] = uint8_t(crc);
    data[7] = uint8_t(crc >> 8);
}

void controlIO(int addr, int level)
{
    if (!serial_inst_ || !serial_inst_->isOpen())
    {
        RCLCPP_ERROR(node->get_logger(), "Serial port not open for IO control");
        return;
    }

    std::array<uint8_t, 8> data;
    composeModbusData(addr, level, data);
    
    try
    {
        serial_inst_->write(data.data(), data.size());
        RCLCPP_INFO(node->get_logger(), "IO control: addr=%d, level=%d", addr, level);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send IO command: %s", e.what());
    }
}

void liftUp()
{
    controlIO(LIFT_UP_ADDR, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    controlIO(LIFT_UP_ADDR, 0);
    RCLCPP_INFO(node->get_logger(), "Lift up completed and sent");
}

void liftDown()
{
    controlIO(LIFT_DOWN_ADDR, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    controlIO(LIFT_DOWN_ADDR, 0);
    RCLCPP_INFO(node->get_logger(), "Lift down completed and sent");
}

void toggleLight(bool on)
{
    controlIO(LIGHT_ADDR, on ? 1 : 0);
}

// 四元数转欧拉角函数
void quaternionToEuler(double x, double y, double z, double w, double& roll, double& pitch, double& yaw)
{
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// 修改后的sendCombinedDataToServer函数
void sendCombinedDataToServer()
{
    if (!ws_connected.load()) {
        return;
    }

    try {
        Json::Value json_msg;
        
        auto now = node->get_clock()->now();
        json_msg["timestamp"] = std::to_string(now.seconds()) + "." + std::to_string(now.nanoseconds() % 1000000000);
        json_msg["robotId"] = "001";
        json_msg["success"] = true;
        json_msg["message"] = "Status data";
        json_msg["Cmd"] = "status_update";
        json_msg["type"] = "state";

        // 始终添加TaskGuid字段，即使为空
        std::string current_taskguid;
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            current_taskguid = taskGuid;
        }
        // 添加这行来单独打印TaskGuid
        //RCLCPP_INFO(node->get_logger(), "Current TaskGuid in sendCombinedDataToServer: '%s'", current_taskguid.c_str());
        json_msg["TaskGuid"] = current_taskguid; // 移除if条件判断
        
        Json::Value motion_state;
        motion_state["state"] = latest_motion_state.state;
        json_msg["motion_state"] = motion_state;
        
        Json::Value odometry;

        // 直接使用odom坐标系进行距离计算
        double current_x = latest_odom.pose.pose.position.x;
        double current_y = latest_odom.pose.pose.position.y;
        
        // 计算累积距离（基于odom坐标系）
        if (!is_first_position) {
            double distance_increment = calculateDistance(last_position_x, last_position_y, current_x, current_y);
            
            // 只有当移动距离大于阈值时才累加（避免噪声）
            if (distance_increment > 0.01) {  // 1cm阈值
                accumulated_distance += distance_increment;
            }
        } else {
            is_first_position = false;
        }
        
        // 更新上一次位置
        last_position_x = current_x;
        last_position_y = current_y;

        Json::Value position;
        Json::Value orientation;

        try {
            // 直接查询base_link到map的变换（获取当前最新的变换）
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
            
            // 直接使用map坐标系中的位置
            position["x"] = transform_stamped.transform.translation.x;
            position["y"] = transform_stamped.transform.translation.y;
            position["z"] = transform_stamped.transform.translation.z;
            
            // 转换四元数到欧拉角
            double roll, pitch, yaw;
            quaternionToEuler(
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w,
                roll, pitch, yaw
            );
            
            orientation["roll"] = roll;
            orientation["pitch"] = pitch;
            orientation["yaw"] = yaw;
            
            //RCLCPP_INFO(node->get_logger(), "Using direct base_link->map transform: (%.3f, %.3f, %.3f)", 
                        //position["x"].asDouble(), position["y"].asDouble(), yaw);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, 
                                "Could not transform base_link to map: %s", ex.what());
            
            // 回退方案1：尝试从odom转换到map
            try {
                if (tf_buffer && tf_buffer->canTransform("map", latest_odom.header.frame_id,
                                                    latest_odom.header.stamp, tf2::durationFromSec(0.1))) {
                    // 创建PoseStamped消息包含位置和姿态
                    geometry_msgs::msg::PoseStamped odom_pose;
                    odom_pose.header = latest_odom.header;
                    odom_pose.pose = latest_odom.pose.pose;
                    
                    // 将完整的pose转换到map坐标系
                    geometry_msgs::msg::PoseStamped map_pose = tf_buffer->transform(odom_pose, "map", tf2::durationFromSec(0.1));
                    
                    position["x"] = map_pose.pose.position.x;
                    position["y"] = map_pose.pose.position.y;
                    position["z"] = map_pose.pose.position.z;
                    
                    double roll, pitch, yaw;
                    quaternionToEuler(
                        map_pose.pose.orientation.x,
                        map_pose.pose.orientation.y,
                        map_pose.pose.orientation.z,
                        map_pose.pose.orientation.w,
                        roll, pitch, yaw
                    );
                    
                    orientation["roll"] = roll;
                    orientation["pitch"] = pitch;
                    orientation["yaw"] = yaw;

                } else {
                    // 回退方案2：使用odom坐标（用于距离计算的current_x, current_y）
                    position["x"] = current_x;
                    position["y"] = current_y;
                    position["z"] = latest_odom.pose.pose.position.z;
                    
                    double roll, pitch, yaw;
                    quaternionToEuler(
                        latest_odom.pose.pose.orientation.x,
                        latest_odom.pose.pose.orientation.y,
                        latest_odom.pose.pose.orientation.z,
                        latest_odom.pose.pose.orientation.w,
                        roll, pitch, yaw
                    );
                    
                    orientation["roll"] = roll;
                    orientation["pitch"] = pitch;
                    orientation["yaw"] = yaw;
                    
                }
            } catch (const std::exception& e) {
                // 最终回退：使用odom坐标
                position["x"] = current_x;
                position["y"] = current_y;
                position["z"] = latest_odom.pose.pose.position.z;
                
                double roll, pitch, yaw;
                quaternionToEuler(
                    latest_odom.pose.pose.orientation.x,
                    latest_odom.pose.pose.orientation.y,
                    latest_odom.pose.pose.orientation.z,
                    latest_odom.pose.pose.orientation.w,
                    roll, pitch, yaw
                );
                
                orientation["roll"] = roll;
                orientation["pitch"] = pitch;
                orientation["yaw"] = yaw;

            }
        }
        
        odometry["position"] = position;
        odometry["orientation"] = orientation;
        
        // 添加累积距离信息（基于odom坐标系计算）
        odometry["accumulated_distance"] = accumulated_distance;

        // 速度处理：保持原有代码（速度通常保持在原坐标系）
        Json::Value velocity;
        velocity["linear_x"] = latest_odom.twist.twist.linear.x;
        velocity["linear_y"] = latest_odom.twist.twist.linear.y;
        velocity["linear_z"] = latest_odom.twist.twist.linear.z;
        velocity["angular_x"] = latest_odom.twist.twist.angular.x;
        velocity["angular_y"] = latest_odom.twist.twist.angular.y;
        velocity["angular_z"] = latest_odom.twist.twist.angular.z;
        odometry["velocity"] = velocity;

        json_msg["odometry"] = odometry;
        
        Json::Value battery;
        float battery_percentage = static_cast<float>(latest_battery_data.soc);
        battery_percentage = std::max(0.0f, std::min(100.0f, battery_percentage));
        
        battery["percentage"] = battery_percentage;
        json_msg["battery"] = battery;

        if (temp_humidity_pm25_received) {
            Json::Value environment;
            environment["temperature"] = latest_temp_humidity_pm25.temperature;
            environment["humidity"] = latest_temp_humidity_pm25.humidity;
            environment["pm25"] = latest_temp_humidity_pm25.pm25;
            json_msg["environment"] = environment;
        }
        // 修改统计信息部分
        Json::Value system_statistics;
        
        // 计算当前会话连接时间
        double current_session_time_minutes = 0.0;
        if (connection_timing_active.load()) {
            auto current_time = std::chrono::steady_clock::now();
            double current_session_time_seconds = std::chrono::duration<double>(current_time - connection_start_time).count();
            current_session_time_minutes = current_session_time_seconds / 60.0;  // 转换为分钟
        }
        
        // 计算总连接时间（持久化数据 + 当前会话时间）
        double total_connection_time_minutes = (persistent_stats.total_connection_time / 60.0) + current_session_time_minutes;
        
        system_statistics["current_session_connection_time_minutes"] = current_session_time_minutes;
        system_statistics["total_connection_time_minutes"] = total_connection_time_minutes;  // 改为分钟
        system_statistics["total_multi_point_navigation_executions"] = multi_nav_execution_count.load();
        system_statistics["total_audio_play_count"] = audio_play_count.load();
        system_statistics["total_accumulated_distance_meters"] = accumulated_distance;
        system_statistics["total_charge_count"] = total_charge_count.load();
        system_statistics["total_voice_call_count"] = total_voice_call_count.load();
        system_statistics["stats_last_updated"] = getCurrentTimeString();
        
        // 添加多点导航进度信息（如果正在执行多点导航）
        if (multi_point_nav_active.load()) {
            Json::Value navigation_progress;
            navigation_progress["current_point"] = static_cast<int>(current_nav_point_index + 1);
            navigation_progress["total_points"] = static_cast<int>(nav_points_sequence.size());
            navigation_progress["progress_text"] = std::to_string(current_nav_point_index + 1) + "/" + std::to_string(nav_points_sequence.size());
            navigation_progress["current_iteration"] = current_repeat_iteration + 1;
            navigation_progress["repeat_enabled"] = nav_repeat_enabled.load();
            system_statistics["navigation_progress"] = navigation_progress;
        }
        
        json_msg["system_statistics"] = system_statistics; 
        
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::string json_string = Json::writeString(builder, json_msg);
        
        websocketpp::lib::error_code ec;
        ws_client.send(ws_connection_hdl, json_string, websocketpp::frame::opcode::text, ec);
        
        if (ec) {
            RCLCPP_ERROR(node->get_logger(), "Failed to send combined data: %s", ec.message().c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error sending combined data: %s", e.what());
    }
}

// 修改 on_open 函数，添加连接时间统计
void on_open(websocketpp::connection_hdl hdl)
{
    ws_connected.store(true);
    ws_connection_hdl = hdl;
    
    // 开始统计连接时间
    connection_start_time = std::chrono::steady_clock::now();
    connection_timing_active.store(true);
    
    RCLCPP_INFO(node->get_logger(), "WebSocket connected successfully");
}

// 修改 on_close 函数，停止连接时间统计
void on_close(websocketpp::connection_hdl hdl)
{
    ws_connected.store(false);
    connection_timing_active.store(false);  // 停止计时
    
    RCLCPP_WARN(node->get_logger(), "WebSocket connection closed");
    
    if (!terminating.load()) {
        ws_should_reconnect.store(true);
    }
}

// 修改 on_fail 函数，停止连接时间统计
void on_fail(websocketpp::connection_hdl hdl)
{
    ws_connected.store(false);
    connection_timing_active.store(false);  // 停止计时
    
    RCLCPP_ERROR(node->get_logger(), "WebSocket connection failed");
    
    if (!terminating.load()) {
        ws_should_reconnect.store(true);
    }
}

std::string decodeUnicodeEscapes(const std::string& str) {
    std::string result = str;
    size_t pos = 0;
    
    while ((pos = result.find("\\u0022", pos)) != std::string::npos) {
        result.replace(pos, 6, "\"");
        pos += 1;
    }
    
    pos = 0;
    while ((pos = result.find("\\u0027", pos)) != std::string::npos) {
        result.replace(pos, 6, "'");
        pos += 1;
    }
    
    return result;
}

Json::Value parseDataField(const std::string& data_str) {
    Json::Value data_json;
    
    try {
        std::string decoded_data = decodeUnicodeEscapes(data_str);
        
        Json::Reader reader;
        bool parsing_successful = reader.parse(decoded_data, data_json);
        
        if (!parsing_successful) {
            RCLCPP_ERROR(node->get_logger(), "Failed to parse Data field as JSON: %s", decoded_data.c_str());
            RCLCPP_ERROR(node->get_logger(), "JSON parse error: %s", reader.getFormattedErrorMessages().c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error parsing Data field: %s", e.what());
    }
    
    return data_json;
}

void sendResponse(const std::string& robotId, bool success, const std::string& message = "", const std::string& cmd = "", const std::string& type = "response", const std::string& taskGuid = "")
{
    if (!ws_connected.load()) {
        return;
    }

    try {
        Json::Value response;
        response["robotId"] = robotId;
        response["success"] = success;
        response["message"] = message;
        
        if (!cmd.empty()) {
            response["Cmd"] = cmd;
        }
        
        response["type"] = type;
        // 添加TaskGuid字段
        if (!taskGuid.empty()) {
            response["TaskGuid"] = taskGuid;
        }

        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        std::string response_string = Json::writeString(builder, response);

        websocketpp::lib::error_code ec;
        ws_client.send(ws_connection_hdl, response_string, websocketpp::frame::opcode::text, ec);
        
        if (ec) {
            RCLCPP_ERROR(node->get_logger(), "Failed to send response: %s", ec.message().c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error sending response: %s", e.what());
    }
}

void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
{
    std::string robotId = "unknown";
    bool command_success = false;
    std::string cmd = "";
    std::string error_message = "";

    try {
        std::string payload = msg->get_payload();

        Json::Value root;
        Json::Reader reader;
        bool parsing_successful = reader.parse(payload, root);
        
        if (!parsing_successful) {
            RCLCPP_ERROR(node->get_logger(), "Failed to parse JSON message");
            sendResponse(robotId, false, "Invalid JSON format", "parse_error");
            return;
        }

        if (root.isMember("id")) {
            robotId = root["id"].asString();
        } else if (root.isMember("robotId")) {
            robotId = root["robotId"].asString();
        }
        // 修正：使用正确的大小写和线程安全
        if (root.isMember("TaskGuid")) {  // 改为大写T
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            taskGuid = root["TaskGuid"].asString();  // 改为大写T
            RCLCPP_INFO(node->get_logger(), "Received TaskGuid: %s", taskGuid.c_str());
        }

        // 安全检查部分也要包含TaskGuid
        std::string current_taskguid;
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            current_taskguid = taskGuid;
        }

        if (toggle_estop.load() || sw_estopped) {
            RCLCPP_WARN(node->get_logger(), "E-Stop detected, WebSocket command ignored");
            sendResponse(robotId, false, "E-Stop detected, command ignored", "safety_check");
            return;
        }
        if (toggle_collision.load()) {
            RCLCPP_WARN(node->get_logger(), "Collision detected, WebSocket command ignored");
            sendResponse(robotId, false, "Collision detected, command ignored", "safety_check");
            return;
        }
        if (remote_mode.load()) {
            RCLCPP_WARN(node->get_logger(), "Vehicle in remote control mode, WebSocket command ignored");
            sendResponse(robotId, false, "Remote control mode active, command ignored", "safety_check");
            return;
        }
        // 修正：处理map命令 - 改为大写Cmd
        if (root.isMember("Cmd") && root["Cmd"].asString() == "map") {
            if (root.isMember("Data")) {
                std::string data_str = root["Data"].asString();
                Json::Value data_json = parseDataField(data_str);
                
                if (!data_json.empty() && data_json.isMember("url")) {
                    std::string map_url = data_json["url"].asString();
                    
                    RCLCPP_INFO(node->get_logger(), "Received map download command, URL: %s", map_url.c_str());
                    
                    // 从URL中提取文件名，或生成默认文件名
                    std::string filename;
                    size_t last_slash = map_url.find_last_of("/");
                    if (last_slash != std::string::npos) {
                        filename = map_url.substr(last_slash + 1);
                    } else {
                        // 生成带时间戳的默认文件名
                        auto now = std::chrono::system_clock::now();
                        auto time_t = std::chrono::system_clock::to_time_t(now);
                        auto tm = *std::localtime(&time_t);
                        
                        std::ostringstream oss;
                        oss << "map_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".pgm";
                        filename = oss.str();
                    }
                    
                    // 确保文件名有.pgm扩展名
                    if (filename.find(".pgm") == std::string::npos && filename.find(".PGM") == std::string::npos) {
                        filename += ".pgm";
                    }
                    
                    RCLCPP_INFO(node->get_logger(), "Downloading map file as: %s", filename.c_str());
                    
                    // 在后台线程中下载，避免阻塞WebSocket
                    std::thread download_thread([map_url, filename, robotId, current_taskguid]() {
                        bool download_success = downloadMapFile(map_url, filename);
                        
                        if (download_success) {
                            std::string success_msg = "Map file downloaded successfully: " + filename;
                            sendResponse(robotId, true, success_msg, "map_download", "response", current_taskguid);
                            RCLCPP_INFO(node->get_logger(), "Map download completed: %s", filename.c_str());
                        } else {
                            std::string error_msg = "Failed to download map file: " + filename;
                            sendResponse(robotId, false, error_msg, "map_download_error", "response", current_taskguid);
                            RCLCPP_ERROR(node->get_logger(), "Map download failed: %s", filename.c_str());
                        }
                    });
                    download_thread.detach();
                    
                    // 立即发送下载开始确认
                    sendResponse(robotId, true, "Map download started: " + filename, "map_download_start", "response", current_taskguid);
                    return;
                    
                } else {
                    error_message = "Map command missing URL in data field";
                    RCLCPP_ERROR(node->get_logger(), "Map command missing URL in data field");
                    sendResponse(robotId, false, error_message, "map_error", "response", current_taskguid);
                    return;
                }
            } else {
                error_message = "Map command missing Data field";
                RCLCPP_ERROR(node->get_logger(), "Map command missing Data field");
                sendResponse(robotId, false, error_message, "map_error", "response", current_taskguid);
                return;
            }
        }

        if (root.isMember("Cmd") && root.isMember("Data")) {
            std::string cmd_field = root["Cmd"].asString();
            
            if (cmd_field == "operate") {
                std::string data_str = root["Data"].asString();
                Json::Value data_json = parseDataField(data_str);
                
                if (data_json.empty()) {
                    RCLCPP_ERROR(node->get_logger(), "Data field parsing failed or empty");
                    sendResponse(robotId, false, "Data field parsing failed", "parse_error");
                    return;
                }
                
                bool command_processed = false;

                // PTZ light control
                if (data_json.isMember("ptz_light")) {
                    bool ptz_light_on = data_json["ptz_light"].asBool();
                    RCLCPP_INFO(node->get_logger(), "Processing PTZ light control: %s", ptz_light_on ? "ON" : "OFF");
                    togglePTZLight(ptz_light_on);
                    command_success = true;
                    cmd = "ptz_light";
                    command_processed = true;
                }

                // Light control
                if (data_json.isMember("light")) {
                    bool light_on = data_json["light"].asBool();
                    RCLCPP_INFO(node->get_logger(), "Processing light control: %s", light_on ? "ON" : "OFF");
                    toggleLight(light_on);
                    command_success = true;
                    cmd = "light_control";
                    command_processed = true;
                }

                // PTZ wiper control
                if (data_json.isMember("ptz_wiper")) {
                    bool ptz_wiper_on = data_json["ptz_wiper"].asBool();
                    RCLCPP_INFO(node->get_logger(), "Processing PTZ wiper control: %s", ptz_wiper_on ? "ON" : "OFF");
                    togglePTZWiper(ptz_wiper_on);
                    command_success = true;
                    cmd = "ptz_wiper";
                    command_processed = true;
                }

                // Lift control
                if (data_json.isMember("lift")) {
                    if (data_json["lift"].isString()) {
                        std::string lift_cmd = data_json["lift"].asString();
                        if (lift_cmd == "up") {
                            RCLCPP_INFO(node->get_logger(), "Processing lift up command");
                            liftUp();
                            command_success = true;
                            cmd = "lift_up";
                            command_processed = true;
                        } else if (lift_cmd == "down") {
                            RCLCPP_INFO(node->get_logger(), "Processing lift down command");
                            liftDown();
                            command_success = true;
                            cmd = "lift_down";
                            command_processed = true;
                        }
                    }
                }

                // Movement control
                if (data_json.isMember("linear") || data_json.isMember("angular") ||
                    data_json.isMember("linear_x") || data_json.isMember("angular_z")) {
                    
                    double linear_x = 0.0;
                    double angular_z = 0.0;

                    if (data_json.isMember("linear")) linear_x = data_json["linear"].asDouble();
                    if (data_json.isMember("angular")) angular_z = data_json["angular"].asDouble();
                    if (data_json.isMember("linear_x")) linear_x = data_json["linear_x"].asDouble();
                    if (data_json.isMember("angular_z")) angular_z = data_json["angular_z"].asDouble();

                    publishVelocity(linear_x, angular_z);
                    command_success = true;
                    cmd = "movement";
                    command_processed = true;
                }

                if (data_json.isMember("ptz")) {
                    std::string ptz_cmd = data_json["ptz"].asString();
                    DWORD ptz_speed = ptz_default_speed_;  // 默认速度
                    
                    // 检查是否有速度参数（只支持字符串格式）
                    if (data_json.isMember("ptz_speed")) {
                        try {
                            ptz_speed = static_cast<DWORD>(std::stoi(data_json["ptz_speed"].asString()));
                            // 限制速度范围在1-7之间
                            ptz_speed = std::max(1u, std::min(7u, ptz_speed));
                        } catch (const std::exception& e) {
                            RCLCPP_WARN(node->get_logger(), "Invalid ptz_speed value: %s, using default speed", 
                                    root["ptz_speed"].asString().c_str());
                            ptz_speed = ptz_default_speed_;
                        }
                    }
                    
                    bool ptz_success = false;
                    
                    if (ptz_cmd == "up") {
                        ptzUp(ptz_speed);
                        ptz_success = true;
                    } else if (ptz_cmd == "down") {
                        ptzDown(ptz_speed);
                        ptz_success = true;
                    } else if (ptz_cmd == "left") {
                        ptzLeft(ptz_speed);
                        ptz_success = true;
                    } else if (ptz_cmd == "right") {
                        ptzRight(ptz_speed);
                        ptz_success = true;
                    } else if (ptz_cmd == "stop") {
                        stopPTZ();
                        ptz_success = true;
                    } else if (ptz_cmd.substr(0, 5) == "goto_") {
                        // 处理预置点命令，例如 "goto_1", "goto_5" 等
                        try {
                            int preset_index = std::stoi(ptz_cmd.substr(5));
                            if (gotoPreset(preset_index)) {
                                ptz_success = true;
                            } else {
                                error_message = "Failed to go to preset point";
                            }
                        } catch (const std::exception& e) {
                            error_message = "Invalid preset command format";
                            RCLCPP_ERROR(node->get_logger(), "Invalid preset command: %s, error: %s", ptz_cmd.c_str(), e.what());
                        }
                    } else {
                        error_message = "Unknown PTZ command: " + ptz_cmd;
                        RCLCPP_WARN(node->get_logger(), "Unknown PTZ command: %s", ptz_cmd.c_str());
                    }
                    
                    if (ptz_success) {
                        command_success = true;
                        cmd = "ptz_control";
                        command_processed = true;
                    } else if (error_message.empty()) {
                        error_message = "PTZ command execution failed";
                    }
                }
                
                if (data_json.isMember("navigate")) {
                    Json::Value nav_data = data_json["navigate"];
                    
                    if (nav_data.isString() && nav_data.asString() == "cancel") {
                        // 取消导航（包括单点和多点）
                        RCLCPP_INFO(node->get_logger(), "Processing navigation cancel command");
                        cancelMultiPointNavigation();  // 这会同时取消单点和多点导航
                        command_success = true;
                        cmd = "navigate_cancel";
                        command_processed = true;
                    }
                    else if (nav_data.isObject() && nav_data.isMember("points") && nav_data["points"].isArray()) {
                        // 多点导航
                        Json::Value points_array = nav_data["points"];
                        std::vector<NavigationPoint> points;
                        
                        // 解析repeat参数 - 简化为布尔值控制
                        bool repeat_enabled = false;
                        if (nav_data.isMember("repeat")) {
                            if (nav_data["repeat"].isBool()) {
                                repeat_enabled = nav_data["repeat"].asBool();
                            } else if (nav_data["repeat"].isString()) {
                                std::string repeat_str = nav_data["repeat"].asString();
                                repeat_enabled = (repeat_str == "true" || repeat_str == "1");
                            }
                        }
                        
                        for (const auto& point_json : points_array) {
                            if (point_json.isMember("x") && point_json.isMember("y")) {
                                NavigationPoint point;
                                point.x = point_json["x"].asDouble();
                                point.y = point_json["y"].asDouble();
                                point.yaw = point_json.isMember("yaw") ? point_json["yaw"].asDouble() : 0.0;
                                point.delay = point_json.isMember("delay") ? point_json["delay"].asDouble() : 0.0;
                                
                                // 处理detection字段
                                if (point_json.isMember("detection")) {
                                    point.has_detection = true;
                                    point.detection_enable = point_json["detection"].asBool();
                                    RCLCPP_INFO(node->get_logger(), "Point (%.2f, %.2f) includes detection: %s", 
                                            point.x, point.y, point.detection_enable ? "ENABLE" : "DISABLE");
                                } else {
                                    point.has_detection = false;
                                    point.detection_enable = false;
                                }
                                
                                // 处理videoSrc字段（音频播放）
                                if (point_json.isMember("videoSrc") && !point_json["videoSrc"].asString().empty()) {
                                    point.has_audio = true;
                                    point.video_src = point_json["videoSrc"].asString();
                                    RCLCPP_INFO(node->get_logger(), "Point (%.2f, %.2f) includes audio: %s", 
                                            point.x, point.y, point.video_src.c_str());
                                } else {
                                    point.has_audio = false;
                                    point.video_src = "";
                                }
                                
                                points.push_back(point);
                            } else {
                                RCLCPP_WARN(node->get_logger(), "Skipping invalid point - missing x or y coordinate");
                            }
                        }
                        
                        if (!points.empty()) {
                            startMultiPointNavigation(points, repeat_enabled);
                            command_success = true;
                            cmd = "multi_navigate";
                            
                            std::string success_msg = "Multi-point navigation started with " + std::to_string(points.size()) + " points";
                            if (repeat_enabled) {
                                success_msg += " (infinite repeat enabled)";
                            }
                            RCLCPP_INFO(node->get_logger(), "%s", success_msg.c_str());
                        } else {
                            error_message = "No valid points found in multi-point navigation";
                            RCLCPP_ERROR(node->get_logger(), "No valid points found in multi-point navigation");
                        }
                        command_processed = true;
                    }
                    else if (nav_data.isObject() && nav_data.isMember("x") && nav_data.isMember("y")) {
                        // 单点导航（原有逻辑保持不变）
                        double target_x = nav_data["x"].asDouble();
                        double target_y = nav_data["y"].asDouble();
                        double target_yaw = nav_data.isMember("yaw") ? nav_data["yaw"].asDouble() : 0.0;
                        
                        // 取消多点导航如果正在进行
                        if (multi_point_nav_active.load()) {
                            cancelMultiPointNavigation();
                        }
                        
                        if (navigateToGoal(target_x, target_y, target_yaw)) {
                            command_success = true;
                            cmd = "navigate_to_pose";
                            RCLCPP_INFO(node->get_logger(), "Single point navigation command processed successfully: (%.2f, %.2f, %.2f)", 
                                    target_x, target_y, target_yaw);
                        } else {
                            error_message = "Failed to send navigation goal";
                            RCLCPP_ERROR(node->get_logger(), "Failed to process navigation command");
                        }
                        command_processed = true;
                    }
                    else {
                        error_message = "Invalid navigation command format";
                        RCLCPP_ERROR(node->get_logger(), "Invalid navigation command format");
                        command_processed = true;
                    }
                }

                // Send response
                if (command_processed) {
                    std::string response_message = command_success ? "Command executed successfully" : error_message;
                    sendResponse(robotId, command_success, response_message, cmd, "response", current_taskguid);
                } else {
                    RCLCPP_WARN(node->get_logger(), "No valid command found in Data field");
                    sendResponse(robotId, false, "No valid command found in Data field", "unknown", "response", current_taskguid);
                }
                return;
            }
        }

        RCLCPP_WARN(node->get_logger(), "Unknown message format");
        sendResponse(robotId, false, "Unknown message format", "unknown", "response", current_taskguid);

    } catch (const std::exception& e) {
        std::string current_taskguid;
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            current_taskguid = taskGuid;
        }
        RCLCPP_ERROR(node->get_logger(), "Error processing WebSocket message: %s", e.what());
        sendResponse(robotId, false, "Internal processing error", "error", "response", current_taskguid);
    }
}

void subCallbackMotionState(const whi_interfaces::msg::WhiMotionState::SharedPtr MotionState)
{
    latest_motion_state = *MotionState;

    if (MotionState->state == whi_interfaces::msg::WhiMotionState::STA_ESTOP) {
        publishVelocity(0.0, 0.0);
        
        if (!toggle_estop.load()) {
            RCLCPP_WARN(node->get_logger(), "E-Stop detected - stopping robot");
        }
        toggle_estop.store(true);
    } else if (MotionState->state == whi_interfaces::msg::WhiMotionState::STA_STANDBY) {
        toggle_estop.store(false);
    }

    if (MotionState->state == whi_interfaces::msg::WhiMotionState::STA_CRITICAL_COLLISION) {
        publishVelocity(0.0, 0.0);
        
        if (!toggle_collision.load()) {
            RCLCPP_WARN(node->get_logger(), "Collision detected - stopping robot");
        }
        toggle_collision.store(true);
    } else if (MotionState->state == whi_interfaces::msg::WhiMotionState::STA_CRITICAL_COLLISION_CLEAR) {
        toggle_collision.store(false);
    }
}

void subCallbackOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    latest_odom = *odom;
}

void subCallbackBatteryData(const whi_interfaces::msg::WhiBattery::SharedPtr battery)
{
    latest_battery_data = *battery;
}

void subCallbackTempHumidityPM25(const whi_interfaces::msg::WhiTemperatureHumidity::SharedPtr msg)
{
    latest_temp_humidity_pm25 = *msg;
    temp_humidity_pm25_received = true;
}

// 检查无检测超时的函数
void checkNoDetectionTimeout()
{
    auto now = std::chrono::steady_clock::now();
    auto time_since_last = std::chrono::duration_cast<std::chrono::seconds>(now - last_detection_time);
    
    if (time_since_last.count() >= 20) {
        RCLCPP_INFO(node->get_logger(), "20秒内无烟雾/火源检测，调用34号预置点");
        gotoPreset(34);
        
        // 重置时间，避免重复调用
        last_detection_time = now;
    }
}

// 修改现有的回调函数名和内容
void detectionTriggerCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    int preset_point = msg->data;
    
    // 更新最后检测时间
    last_detection_time = std::chrono::steady_clock::now();
    
    // 检查预置点有效性（1-12）
    if (preset_point < 1 || preset_point > 12) {
        RCLCPP_WARN(node->get_logger(), 
            "无效的预置点编号: %d (有效范围: 1-12)", preset_point);
        return;
    }
    
    // 计算流索引和角度范围
    int stream_index = (preset_point - 1) / 3;
    int angle_start = (preset_point - 1) * 30;
    int angle_end = angle_start + 30;
    
    RCLCPP_ERROR(node->get_logger(), 
        "流%d检测到烟雾/火源 [%d°-%d°]，调用预置点%d", 
        stream_index, angle_start, angle_end, preset_point);
    
    if (gotoPreset(preset_point)) {
        RCLCPP_INFO(node->get_logger(), "预置点%d调用成功", preset_point);
    } else {
        RCLCPP_ERROR(node->get_logger(), "预置点%d调用失败", preset_point);
    }
}

void subCallbackSwEstop(const std_msgs::msg::Bool::SharedPtr Msg)
{
    sw_estopped = Msg->data;
    if (sw_estopped) {
        publishVelocity(0.0, 0.0);
        RCLCPP_WARN(node->get_logger(), "Software E-Stop activated");
    }
}

void subCallbackRcState(const whi_interfaces::msg::WhiRcState::SharedPtr RcState)
{
    if (RcState->state == whi_interfaces::msg::WhiRcState::STA_ACTIVE)
    {
        if (!remote_mode.load())
        {
            publishVelocity(0.0, 0.0);
            RCLCPP_WARN(node->get_logger(), "Control taken over by remote");
        }
        remote_mode.store(true);
    }
    else if (RcState->state == whi_interfaces::msg::WhiRcState::STA_INACTIVE)
    {
        remote_mode.store(false);
    }
}

// 修改后的 sendMultiNavStatistics 函数
void sendMultiNavStatistics(double execution_time_seconds, bool completed_successfully, const std::string& taskGuid = "")
{
    // 立即保存统计数据到文件
    savePersistentStats();
    
    if (!ws_connected.load()) {
        return;
    }

    try {
        Json::Value stats_msg;
        auto now = node->get_clock()->now();
        stats_msg["timestamp"] = std::to_string(now.seconds()) + "." + std::to_string(now.nanoseconds() % 1000000000);
        stats_msg["robotId"] = "001";
        stats_msg["type"] = "multi_nav_statistics";
        stats_msg["Cmd"] = "multi_nav_stats";
        stats_msg["success"] = completed_successfully;
        // 添加TaskGuid字段
        if (!taskGuid.empty()) {
            stats_msg["TaskGuid"] = taskGuid;
        }
        
        // 计算本次任务的里程
        double task_total_mile = accumulated_distance - task_start_distance;
        
        // 统计数据
        Json::Value statistics;
        statistics["execution_time_seconds"] = execution_time_seconds;
        statistics["total_execution_count"] = multi_nav_execution_count.load();
        statistics["completed_successfully"] = completed_successfully;
        statistics["total_mile"] = task_total_mile;  // 新增：本次任务里程（米）
        
        // 新增：当前电量信息
        Json::Value battery_info;
        float battery_percentage = static_cast<float>(latest_battery_data.soc);
        battery_percentage = std::max(0.0f, std::min(100.0f, battery_percentage));
        battery_info["percentage"] = battery_percentage;
        statistics["battery"] = battery_info;
        
        stats_msg["statistics"] = statistics;
        stats_msg["message"] = completed_successfully ? 
            "Multi-point navigation completed successfully" : 
            "Multi-point navigation was canceled or failed";

        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        std::string stats_string = Json::writeString(builder, stats_msg);
        websocketpp::lib::error_code ec;
        ws_client.send(ws_connection_hdl, stats_string, websocketpp::frame::opcode::text, ec);
        
        if (ec) {
            RCLCPP_ERROR(node->get_logger(), "Failed to send multi-nav statistics: %s", ec.message().c_str());
        } else {
            RCLCPP_INFO(node->get_logger(), "Multi-nav statistics sent and saved: %.2fs execution time, %.2fm task distance, %d total executions, battery: %.1f%%", 
                       execution_time_seconds, task_total_mile, multi_nav_execution_count.load(), battery_percentage);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error sending multi-nav statistics: %s", e.what());
    }
}

// 修改后的 startMultiPointNavigation 函数（需要记录任务开始时的距离）
void startMultiPointNavigation(const std::vector<NavigationPoint>& points, bool repeat)
{
    if (points.empty()) {
        std::string current_taskguid;
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            current_taskguid = taskGuid;
        }
        RCLCPP_ERROR(node->get_logger(), "Empty navigation points sequence");
        sendResponse("001", false, "Empty navigation points sequence", "multi_navigate", "response", current_taskguid);
        return;
    }
    
    // 取消当前的单点或多点导航
    cancelMultiPointNavigation();
    
    // 保存原始序列和重复设置
    original_nav_points_sequence = points;
    nav_points_sequence = points;
    nav_repeat_enabled.store(repeat);
    current_repeat_iteration = 0;
    
    current_nav_point_index = 0;
    multi_point_nav_active.store(true);
    
    // 记录任务开始时的累积距离
    task_start_distance = accumulated_distance;
    
    // 开始计时和计数
    multi_nav_start_time = std::chrono::steady_clock::now();
    multi_nav_timing_active.store(true);
    multi_nav_execution_count.fetch_add(1);  // 增加执行次数
    
    std::string repeat_info = repeat ? " (infinite repeat)" : "";
    
    RCLCPP_INFO(node->get_logger(), "Starting multi-point navigation with %zu points%s (Execution #%d, starting distance: %.2fm)", 
                points.size(), repeat_info.c_str(), multi_nav_execution_count.load(), task_start_distance);
    
    // 开始导航到第一个点
    continueMultiPointNavigation();
}
// 修改 continueMultiPointNavigation 函数中的完成部分
void continueMultiPointNavigation()
{
    if (!multi_point_nav_active.load()) {
        return;
    }
    
    // 检查当前轮次是否完成
    if (current_nav_point_index >= nav_points_sequence.size()) {
        // 当前轮次完成，检查是否需要重复
        if (nav_repeat_enabled.load()) {
            current_repeat_iteration++;
            current_nav_point_index = 0;
            
            RCLCPP_INFO(node->get_logger(), "Starting repeat iteration %d (infinite loop)", 
                       current_repeat_iteration);
            
            // 发送重复开始反馈
            Json::Value repeat_msg;
            repeat_msg["robotId"] = "001";
            repeat_msg["type"] = "multi_nav_repeat_start";
            repeat_msg["success"] = true;
            repeat_msg["current_iteration"] = current_repeat_iteration;
            repeat_msg["progress_text"] = "1/" + std::to_string(nav_points_sequence.size()) + " - Iteration " + std::to_string(current_repeat_iteration);
            repeat_msg["message"] = "Starting repeat iteration " + std::to_string(current_repeat_iteration) + " (infinite loop)";
            
            if (ws_connected.load()) {
                Json::StreamWriterBuilder builder;
                builder["indentation"] = "";
                std::string repeat_string = Json::writeString(builder, repeat_msg);
                websocketpp::lib::error_code ec;
                ws_client.send(ws_connection_hdl, repeat_string, websocketpp::frame::opcode::text, ec);
            }
            
            // 继续处理第一个点
            continueMultiPointNavigation();
            return;
        }
        
        // 序列完全完成（非重复模式）
        double execution_time = 0.0;
        if (multi_nav_timing_active.load()) {
            auto end_time = std::chrono::steady_clock::now();
            execution_time = std::chrono::duration<double>(end_time - multi_nav_start_time).count();
            multi_nav_timing_active.store(false);
        }
        
        multi_point_nav_active.store(false);
        nav_repeat_enabled.store(false);

        std::string current_taskguid;
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            current_taskguid = taskGuid;
        }
        
        std::string completion_message = "Multi-point navigation sequence completed";
        if (current_repeat_iteration > 0) {
            completion_message += " after " + std::to_string(current_repeat_iteration) + " iterations";
        }
        completion_message += " in " + std::to_string(execution_time) + " seconds";
        
        RCLCPP_INFO(node->get_logger(), "%s (Execution #%d)", 
                   completion_message.c_str(), multi_nav_execution_count.load());
        
        // 发送完成响应
        sendResponse("001", true, completion_message, "multi_navigate_complete", "response", current_taskguid);
        
        // 发送统计数据
        sendMultiNavStatistics(execution_time, true, current_taskguid);
        
        // 任务完成后清零TaskGuid
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            taskGuid.clear();
        }
        
        return;
    }
    
    const auto& point = nav_points_sequence[current_nav_point_index];
    
    // 创建进度文本，如 "3/10"
    std::string progress_text = std::to_string(current_nav_point_index + 1) + "/" + std::to_string(nav_points_sequence.size());
    if (current_repeat_iteration > 0) {
        progress_text += " - Iteration " + std::to_string(current_repeat_iteration + 1);
    }
    
    RCLCPP_INFO(node->get_logger(), "Processing point %s: (%.2f, %.2f, %.2f), delay: %.1fs", 
                progress_text.c_str(), point.x, point.y, point.yaw, point.delay);
    
    // 发送导航开始进度反馈
    Json::Value progress_msg;
    progress_msg["robotId"] = "001";
    progress_msg["type"] = "multi_nav_point_start";
    progress_msg["success"] = true;
    progress_msg["point_index"] = static_cast<int>(current_nav_point_index + 1);
    progress_msg["total_points"] = static_cast<int>(nav_points_sequence.size());
    progress_msg["progress_text"] = progress_text;
    progress_msg["current_iteration"] = current_repeat_iteration + 1;
    progress_msg["point_coordinates"] = Json::Value(Json::objectValue);
    progress_msg["point_coordinates"]["x"] = point.x;
    progress_msg["point_coordinates"]["y"] = point.y;
    progress_msg["point_coordinates"]["yaw"] = point.yaw;
    progress_msg["message"] = "Starting navigation to point " + progress_text;
    
    if (ws_connected.load()) {
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "";
        std::string progress_string = Json::writeString(builder, progress_msg);
        websocketpp::lib::error_code ec;
        ws_client.send(ws_connection_hdl, progress_string, websocketpp::frame::opcode::text, ec);
    }
    
    // 如果有detection指令，先执行detection服务
    if (point.has_detection) {
        RCLCPP_INFO(node->get_logger(), "Point %s includes detection command: %s", 
                   progress_text.c_str(), point.detection_enable ? "ENABLE" : "DISABLE");
        
        if (callDetectionService(point.detection_enable)) {
            RCLCPP_INFO(node->get_logger(), "Detection service %s successfully for point %s", 
                       point.detection_enable ? "enabled" : "disabled", progress_text.c_str());
            
            // 发送detection执行成功的反馈
            Json::Value feedback_msg;
            feedback_msg["robotId"] = "001";
            feedback_msg["type"] = "multi_nav_detection_feedback";
            feedback_msg["success"] = true;
            feedback_msg["point_index"] = static_cast<int>(current_nav_point_index + 1);
            feedback_msg["progress_text"] = progress_text;
            feedback_msg["iteration"] = current_repeat_iteration + 1;
            feedback_msg["detection_status"] = point.detection_enable ? "enabled" : "disabled";
            feedback_msg["message"] = "Detection service executed successfully for point " + progress_text;
            
            if (ws_connected.load()) {
                Json::StreamWriterBuilder builder;
                builder["indentation"] = "";
                std::string feedback_string = Json::writeString(builder, feedback_msg);
                websocketpp::lib::error_code ec;
                ws_client.send(ws_connection_hdl, feedback_string, websocketpp::frame::opcode::text, ec);
            }
        } else {
            RCLCPP_WARN(node->get_logger(), "Failed to control detection service for point %s, continuing navigation", 
                       progress_text.c_str());
        }
        
        // 添加小延迟确保detection服务执行完成
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // 保存当前点的音频信息，但不立即播放
    {
        std::lock_guard<std::mutex> lock(audio_info_mutex);
        current_point_audio_info.has_audio = point.has_audio;
        current_point_audio_info.video_src = point.video_src;
        current_point_audio_info.point_index = current_nav_point_index + 1;
        current_point_audio_info.iteration = current_repeat_iteration + 1;
    }
    
    if (point.has_audio && !point.video_src.empty()) {
        RCLCPP_INFO(node->get_logger(), "Point %s includes audio playback: %s (will start after navigation begins)", 
                   progress_text.c_str(), point.video_src.c_str());
    }
    
    // 导航到当前点
    if (!navigateToGoal(point.x, point.y, point.yaw)) {
        std::string current_taskguid;
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            current_taskguid = taskGuid;
        }
        RCLCPP_ERROR(node->get_logger(), "Failed to start navigation to point %s", progress_text.c_str());
        multi_point_nav_active.store(false);
        nav_repeat_enabled.store(false);
        sendResponse("001", false, "Failed to navigate to point " + progress_text, "multi_navigate_error");
    }
}

// 处理音频播放的独立函数
void processPointAudio(const std::string& video_src, size_t point_index, int iteration)
{
    std::string filename;
    if (video_src.find("Upload/Default/") == 0) {
        // 如果video_src是 "Upload/Default/717743559548998.pcm"
        filename = video_src.substr(15); // 去掉 "Upload/Default/" 前缀
    } else {
        // 如果video_src就是文件名 "717743559548998.pcm"
        filename = video_src;
    }
    
    // 在外层作用域定义local_path
    std::string local_path = audio_files_directory + "/" + filename;
    
    // 下载并播放音频文件
    if (downloadAudioFile(video_src, filename)) {
        // 增加音频播放计数
        audio_play_count.fetch_add(1);
        
        // 发送音频播放开始反馈
        Json::Value audio_feedback;
        audio_feedback["robotId"] = "001";
        audio_feedback["type"] = "multi_nav_audio_feedback";
        audio_feedback["success"] = true;
        audio_feedback["point_index"] = static_cast<int>(point_index);
        audio_feedback["iteration"] = iteration;
        audio_feedback["audio_file"] = filename;
        audio_feedback["message"] = "Audio playback started after navigation began";
        
        if (ws_connected.load()) {
            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string feedback_string = Json::writeString(builder, audio_feedback);
            websocketpp::lib::error_code ec;
            ws_client.send(ws_connection_hdl, feedback_string, websocketpp::frame::opcode::text, ec);
        }
        
        // 在后台线程中处理音频文件
        std::thread audio_thread([local_path, point_index]() {
            if (processAudioFile(local_path)) {
                // 音频播放完成后立即保存统计
                savePersistentStats();
                RCLCPP_INFO(node->get_logger(), "Audio playback completed and stats saved for point %zu: %s", 
                        point_index, local_path.c_str());
            } else {
                RCLCPP_ERROR(node->get_logger(), "Audio playback failed for point %zu: %s", 
                            point_index, local_path.c_str());
            }
        });
        audio_thread.detach();
        
        RCLCPP_INFO(node->get_logger(), "Started playing audio file for point %zu AFTER navigation began: %s -> %s (Total plays: %d)", 
                point_index, video_src.c_str(), local_path.c_str(), audio_play_count.load());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to download audio file for point %zu: %s", 
                    point_index, video_src.c_str());
    }
}

// 修改 cancelMultiPointNavigation 函数
void cancelMultiPointNavigation()
{
    if (multi_point_nav_active.load() || nav_repeat_enabled.load()) {
        // 计算执行时间
        double execution_time = 0.0;
        if (multi_nav_timing_active.load()) {
            auto end_time = std::chrono::steady_clock::now();
            execution_time = std::chrono::duration<double>(end_time - multi_nav_start_time).count();
            multi_nav_timing_active.store(false);
        }
        
        multi_point_nav_active.store(false);
        nav_repeat_enabled.store(false);
        nav_points_sequence.clear();
        original_nav_points_sequence.clear();
        current_nav_point_index = 0;
        current_repeat_iteration = 0;
        
        // 停止延迟定时器
        if (nav_delay_timer) {
            nav_delay_timer->cancel();
            nav_delay_timer = nullptr;
        }
        
        // 获取当前TaskGuid
        std::string current_taskguid;
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            current_taskguid = taskGuid;
        }
        
        std::string cancel_message = "Multi-point navigation canceled after " + std::to_string(execution_time) + " seconds";
        if (current_repeat_iteration > 0) {
            cancel_message += " (during iteration " + std::to_string(current_repeat_iteration + 1) + ")";
        }
        
        RCLCPP_INFO(node->get_logger(), "%s (Execution #%d)", 
                   cancel_message.c_str(), multi_nav_execution_count.load());
        
        // 发送取消的统计数据（包含里程和电量）
        sendMultiNavStatistics(execution_time, false, current_taskguid);
        // 任务取消后清零TaskGuid
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            taskGuid.clear();
        }
    }
    
    // 取消当前导航
    cancelNavigation();
}

void onNavDelayComplete()
{
    // 延迟完成，继续下一个点
    current_nav_point_index++;
    continueMultiPointNavigation();
}

// 修改后的 navigationGoalResponseCallback 函数
void navigationGoalResponseCallback(const NavGoalHandle::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Navigation goal was rejected");
        nav_active.store(false);
        return;
    }
    
    current_nav_goal_handle = goal_handle;
    RCLCPP_INFO(node->get_logger(), "Navigation goal accepted, robot started moving");
    
    // 导航目标被接受后，检查是否需要播放音频
    {
        std::lock_guard<std::mutex> lock(audio_info_mutex);
        if (current_point_audio_info.has_audio && !current_point_audio_info.video_src.empty()) {
            // 在小延迟后开始音频播放，确保机器人已经开始移动
            std::thread delayed_audio_thread([info = current_point_audio_info]() {
                // 等待1秒确保机器人已经开始移动
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                
                // 检查导航是否仍然活跃（避免在取消后播放音频）
                if (nav_active.load() && multi_point_nav_active.load()) {
                    RCLCPP_INFO(node->get_logger(), "Robot has started moving to point %zu, now starting audio playback", 
                               info.point_index);
                    processPointAudio(info.video_src, info.point_index, info.iteration);
                } else {
                    RCLCPP_INFO(node->get_logger(), "Navigation was canceled, skipping audio playback for point %zu", 
                               info.point_index);
                }
            });
            delayed_audio_thread.detach();
            
            // 清除音频信息
            current_point_audio_info.has_audio = false;
            current_point_audio_info.video_src = "";
        }
    }
}

void navigationFeedbackCallback(NavGoalHandle::SharedPtr, 
                               const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    // 可以在这里处理导航反馈，比如发送进度到WebSocket
    if (ws_connected.load()) {
        try {
            Json::Value response;
            response["robotId"] = "001";
            response["type"] = "navigation_feedback";
            response["Cmd"] = "navigate_feedback";
            response["success"] = true;
            
            Json::Value pose;
            pose["x"] = feedback->current_pose.pose.position.x;
            pose["y"] = feedback->current_pose.pose.position.y;
            response["current_pose"] = pose;
            
            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string response_string = Json::writeString(builder, response);
            
            websocketpp::lib::error_code ec;
            ws_client.send(ws_connection_hdl, response_string, websocketpp::frame::opcode::text, ec);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Error sending navigation feedback: %s", e.what());
        }
    }
}

// 修改 navigationResultCallback 函数中的多点导航完成部分
void navigationResultCallback(const NavGoalHandle::WrappedResult& result)
{
    nav_active.store(false);
    current_nav_goal_handle = nullptr;
    
    std::string status;
    bool success = false;
    
    switch(result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            status = "Navigation succeeded";
            success = true;
            RCLCPP_INFO(node->get_logger(), "Navigation goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            status = "Navigation aborted";
            RCLCPP_WARN(node->get_logger(), "Navigation goal aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            status = "Navigation canceled";
            RCLCPP_INFO(node->get_logger(), "Navigation goal canceled");
            break;
        default:
            status = "Navigation failed";
            RCLCPP_ERROR(node->get_logger(), "Navigation goal failed");
            break;
    }
    
    // 处理多点导航的情况
    if (multi_point_nav_active.load() && success) {
        // 创建进度文本
        std::string progress_text = std::to_string(current_nav_point_index + 1) + "/" + std::to_string(nav_points_sequence.size());
        if (current_repeat_iteration > 0) {
            progress_text += " - Iteration " + std::to_string(current_repeat_iteration + 1);
        }
        
        // 当前点导航成功，发送进度反馈
        Json::Value progress_msg;
        progress_msg["robotId"] = "001";
        progress_msg["type"] = "multi_nav_progress";
        progress_msg["success"] = true;
        progress_msg["point_index"] = static_cast<int>(current_nav_point_index + 1);
        progress_msg["total_points"] = static_cast<int>(nav_points_sequence.size());
        progress_msg["progress_text"] = progress_text;
        progress_msg["current_iteration"] = current_repeat_iteration + 1;
        progress_msg["message"] = "Reached point " + progress_text;
        
        if (ws_connected.load()) {
            Json::StreamWriterBuilder builder;
            builder["indentation"] = "";
            std::string progress_string = Json::writeString(builder, progress_msg);
            websocketpp::lib::error_code ec;
            ws_client.send(ws_connection_hdl, progress_string, websocketpp::frame::opcode::text, ec);
        }
        
        // 检查是否需要延迟
        if (current_nav_point_index < nav_points_sequence.size()) {
            const auto& current_point = nav_points_sequence[current_nav_point_index];
            
            if (current_point.delay > 0.0) {
                RCLCPP_INFO(node->get_logger(), "Reached point %s, waiting %.1f seconds before next point", 
                           progress_text.c_str(), current_point.delay);
                
                // 创建延迟定时器
                nav_delay_timer = node->create_wall_timer(
                    std::chrono::duration<double>(current_point.delay),
                    [&]() {
                        nav_delay_timer = nullptr;  // 清除定时器
                        onNavDelayComplete();
                    }
                );
            } else {
                // 无延迟，直接继续下一个点
                current_nav_point_index++;
                continueMultiPointNavigation();
            }
        }
        return;  // 多点导航中，不发送单独的结果响应
    } else if (multi_point_nav_active.load() && !success) {
        // 多点导航中某个点失败
        std::string progress_text = std::to_string(current_nav_point_index + 1) + "/" + std::to_string(nav_points_sequence.size());
        if (current_repeat_iteration > 0) {
            progress_text += " - Iteration " + std::to_string(current_repeat_iteration + 1);
        }
        
        double execution_time = 0.0;
        if (multi_nav_timing_active.load()) {
            auto end_time = std::chrono::steady_clock::now();
            execution_time = std::chrono::duration<double>(end_time - multi_nav_start_time).count();
            multi_nav_timing_active.store(false);
        }
        
        multi_point_nav_active.store(false);
        
        std::string current_taskguid;
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            current_taskguid = taskGuid;
        }
        
        RCLCPP_ERROR(node->get_logger(), "Multi-point navigation failed at point %s after %.2f seconds (Execution #%d)", 
                    progress_text.c_str(), execution_time, multi_nav_execution_count.load());
        
        sendResponse("001", false, "Multi-point navigation failed at point " + progress_text + ": " + status, "multi_navigate_error", "response", current_taskguid);
        
        // 发送失败的统计数据
        sendMultiNavStatistics(execution_time, false, current_taskguid);
        
        // 多点导航失败后清零TaskGuid
        {
            std::lock_guard<std::mutex> lock(task_guid_mutex);
            taskGuid.clear();
        }
        
        return;
    }
    
    // 单点导航的原有逻辑
    std::string current_taskguid;
    {
        std::lock_guard<std::mutex> lock(task_guid_mutex);
        current_taskguid = taskGuid;
    }
    
    sendResponse("001", success, status, "navigate_result", "response", current_taskguid);
}

// 导航到目标点
bool navigateToGoal(double x, double y, double yaw)
{
    if (!nav_client) {
        RCLCPP_ERROR(node->get_logger(), "Navigation client not initialized");
        return false;
    }
    
    if (!nav_client->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node->get_logger(), "Navigation action server not available");
        return false;
    }
    
    // 取消当前导航
    if (nav_active.load()) {
        cancelNavigation();
    }
    
    NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node->get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;
    
    // 设置朝向（四元数）
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = navigationGoalResponseCallback;
    send_goal_options.feedback_callback = navigationFeedbackCallback;
    send_goal_options.result_callback = navigationResultCallback;
    
    nav_client->async_send_goal(goal_msg, send_goal_options);
    nav_active.store(true);
    
    RCLCPP_INFO(node->get_logger(), "Navigation goal sent: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    return true;
}

// 取消导航
void cancelNavigation()
{
    if (nav_active.load() && nav_client) {
        nav_client->async_cancel_all_goals();
        nav_active.store(false);
        RCLCPP_INFO(node->get_logger(), "Navigation canceled");
    }
}

void runWebSocketClient(const std::string& uri)
{
    while (!terminating.load()) {
        try {
            RCLCPP_INFO(node->get_logger(), "Attempting to connect to WebSocket: %s", uri.c_str());
            
            ws_client.clear_access_channels(websocketpp::log::alevel::all);
            ws_client.set_access_channels(websocketpp::log::alevel::connect | 
                                        websocketpp::log::alevel::disconnect | 
                                        websocketpp::log::alevel::app);
            ws_client.set_error_channels(websocketpp::log::elevel::all);

            ws_client.init_asio();

            ws_client.set_open_handler(&on_open);
            ws_client.set_close_handler(&on_close);
            ws_client.set_fail_handler(&on_fail);
            ws_client.set_message_handler(&on_message);

            websocketpp::lib::error_code ec;
            client::connection_ptr con = ws_client.get_connection(uri, ec);
            
            if (ec) {
                RCLCPP_ERROR(node->get_logger(), "Could not create connection: %s", ec.message().c_str());
            } else {
                ws_client.connect(con);
                ws_should_reconnect.store(false);
                
                ws_client.run();
            }

        } catch (websocketpp::exception const & e) {
            RCLCPP_ERROR(node->get_logger(), "WebSocket error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Client error: %s", e.what());
        }

        if (terminating.load()) {
            break;
        }

        if (ws_should_reconnect.load()) {
            RCLCPP_WARN(node->get_logger(), "WebSocket disconnected, attempting to reconnect in %d seconds...", ws_reconnect_interval);
            
            for (int i = 0; i < ws_reconnect_interval && !terminating.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            try {
                ws_client.stop();
                ws_client.reset();
            } catch (...) {
                // 忽略重置时的异常
            }
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "WebSocket client thread exiting");
}

std::function<void(int)> functionWrapper;
void sigintHandler(int Signal)
{
    functionWrapper(Signal);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("whi_websocket_teleop_client_with_file_receiver_and_remote_audio");
    // 在node创建之后添加
    nav_client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");
    no_detection_timer = node->create_wall_timer(std::chrono::seconds(5), checkNoDetectionTimeout);
    // 启动声源定位服务器
    sound_server_thread = std::make_unique<std::thread>(soundSourceServer);

    // 等待动作服务器
    RCLCPP_INFO(node->get_logger(), "Waiting for navigation action server...");
    if (!nav_client->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_WARN(node->get_logger(), "Navigation action server not available");
    } else {
        RCLCPP_INFO(node->get_logger(), "Navigation action server connected");
    }
	// 在 node 创建之后添加
	detect_service_client = node->create_client<std_srvs::srv::SetBool>("/setup_detect_service");

	// 等待检测服务
	RCLCPP_INFO(node->get_logger(), "Waiting for detection service...");
	if (!detect_service_client->wait_for_service(std::chrono::seconds(5))) {
		RCLCPP_WARN(node->get_logger(), "Detection service not available");
	} else {
		RCLCPP_INFO(node->get_logger(), "Detection service connected");
	}

    ptz_home_service = node->create_service<std_srvs::srv::Trigger>(
    "/ptz_home",
    ptzHomeServiceCallback
    );
    //RCLCPP_INFO(node->get_logger(), "PTZ home position service created: /ptz_home");

    // 初始化tf2 buffer和listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 创建定期保存统计数据的定时器（每5分钟保存一次）
    auto saveStatsTimer = node->create_wall_timer(
        std::chrono::minutes(5),
        []() {
            savePersistentStats();
        }
    );

    signal(SIGINT, sigintHandler);
    functionWrapper = [&](int) {
        std::cout << "Shutting down WebSocket client with file receiver and remote audio support..." << std::endl;

        // 保存最终的统计数据
        savePersistentStats();
        publishVelocity(0.0, 0.0);

        if (serial_inst_ && serial_inst_->isOpen())
        {
            serial_inst_->close();
        }

        cleanupDevice();

        terminating.store(true);
        ws_should_reconnect.store(false);
        
        try {
            ws_client.stop();
        } catch (...) {
            // 忽略停止时的异常
        }
        
        if (ws_thread && ws_thread->joinable()) {
            ws_thread->join();
        }

        if (file_server) {
            file_server->stop();
        }
        
        if (file_server_thread && file_server_thread->joinable()) {
            file_server_thread->join();
        }
       
       // 在程序退出时清理（在functionWrapper中添加）
       if (no_detection_timer) {
           no_detection_timer->cancel();
           no_detection_timer = nullptr;
       }
        // 在cleanup部分添加服务清理（functionWrapper中，约在第2340行）
        if (ptz_home_service) {
            ptz_home_service.reset();
        }
        // 停止声源定位服务器
        sound_server_running.store(false);
        if (sound_server_thread && sound_server_thread->joinable()) {
            sound_server_thread->join();
        }
        // 清理导航
        if (nav_active.load()) {
            cancelNavigation();
        }
        nav_client.reset();
        // 在 "清理导航" 注释部分添加：
        if (multi_point_nav_active.load()) {
            cancelMultiPointNavigation();
        }
        if (nav_delay_timer) {
            nav_delay_timer->cancel();
            nav_delay_timer = nullptr;
        }
        // 清理tf2资源
        tf_listener.reset();
        tf_buffer.reset();
		detect_service_client.reset();

        node.reset();

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    };

    // 参数设置
    std::string ws_uri = "ws://114.55.55.192:9000/robot?robotId=001";
    double frequency = 5.0;
    double combined_data_send_frequency = 1.0;
    std::string stateTopic, swEstopTopic, rcStateTopic, odomTopic, batteryTopic, tempHumidityPM25Topic;
    
    node->declare_parameter("websocket_uri", ws_uri);
    node->get_parameter("websocket_uri", ws_uri);
    node->declare_parameter("command_frequency", 5.0);
    node->get_parameter("command_frequency", frequency);
    node->declare_parameter("combined_data_send_frequency", 1.0);
    node->get_parameter("combined_data_send_frequency", combined_data_send_frequency);
    node->declare_parameter("websocket_reconnect_interval", 5);
    node->get_parameter("websocket_reconnect_interval", ws_reconnect_interval);
    node->declare_parameter("local_files_directory", std::string("/home/nvidia/robot_files"));
    node->get_parameter("local_files_directory", local_files_directory);
    node->declare_parameter("audio_files_directory", std::string("/home/nvidia/robot_audio"));
    node->get_parameter("audio_files_directory", audio_files_directory);
    node->declare_parameter("file_monitor_enabled", true);
    node->get_parameter("file_monitor_enabled", file_monitor_enabled);
    node->declare_parameter("file_server_port", 8080);
    node->get_parameter("file_server_port", file_server_port);
    // 在参数设置部分添加统计文件路径参数
    node->declare_parameter("stats_file_path", std::string("/home/nvidia/robot_stats.json"));
    node->get_parameter("stats_file_path", stats_file_path);
    
    // 统一的设备参数
    node->declare_parameter("device.ip", std::string("192.168.254.5"));
    node->get_parameter("device.ip", device_ip_);
    node->declare_parameter("device.port", 8000);
    node->get_parameter("device.port", device_port_);
    node->declare_parameter("device.username", std::string("admin"));
    node->get_parameter("device.username", device_username_);
    node->declare_parameter("device.password", std::string("jy100200300"));
    node->get_parameter("device.password", device_password_);
    node->declare_parameter("device.channel", 1);
    node->get_parameter("device.channel", device_channel_);
    node->declare_parameter("temperature.enable_monitoring", true);
    bool temp_enable;
    node->get_parameter("temperature.enable_monitoring", temp_enable);
    enable_temperature_monitoring_.store(temp_enable);
    node->declare_parameter("temperature.rule_id", 1);
    node->get_parameter("temperature.rule_id", temperature_rule_id_);
    node->declare_parameter("temperature.interval_ms", 2000);
    node->get_parameter("temperature.interval_ms", temperature_interval_ms_);
    
    // PTZ控制参数
    node->declare_parameter("ptz.default_speed", 3);
    int temp_speed;
    node->get_parameter("ptz.default_speed", temp_speed);
    ptz_default_speed_ = static_cast<DWORD>(std::max(1, std::min(7, temp_speed)));
    
    node->declare_parameter("motion_state_topic", std::string("motion_state"));
    node->get_parameter("motion_state_topic", stateTopic);
    node->declare_parameter("odom_topic", std::string("odom"));
    node->get_parameter("odom_topic", odomTopic);
    node->declare_parameter("battery_topic", std::string("battery_data"));
    node->get_parameter("battery_topic", batteryTopic);
    node->declare_parameter("temp_humidity_pm25_topic", std::string("temperature_humidity"));
    node->get_parameter("temp_humidity_pm25_topic", tempHumidityPM25Topic);
    node->declare_parameter("sw_estop_topic", std::string("estop"));
    node->get_parameter("sw_estop_topic", swEstopTopic);
    node->declare_parameter("rc_state_topic", std::string("rc_state"));
    node->get_parameter("rc_state_topic", rcStateTopic);
    node->declare_parameter("use_stamped_vel", use_stamped_vel);
    node->get_parameter("use_stamped_vel", use_stamped_vel);
    node->declare_parameter("linear.min", 0.08);
    node->get_parameter("linear.min", linear_min);
    node->declare_parameter("linear.max", 1.0);
    node->get_parameter("linear.max", linear_max);
    node->declare_parameter("angular.min", 0.1);
    node->get_parameter("angular.min", angular_min);
    node->declare_parameter("angular.max", 0.8);
    node->get_parameter("angular.max", angular_max);
    
    // Modbus IO 参数
    node->declare_parameter("modbus.device_addr", 0x02);
    node->get_parameter("modbus.device_addr", device_addr_);
    node->declare_parameter("modbus.serial_port", std::string("/dev/ttyUART_485_2"));
    node->get_parameter("modbus.serial_port", serial_port_);
    node->declare_parameter("modbus.baudrate", 9600);
    node->get_parameter("modbus.baudrate", baudrate_);

    linear_min = fabs(linear_min);
    linear_max = fabs(linear_max);
    angular_min = fabs(angular_min);
    angular_max = fabs(angular_max);

    try
    {
        serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, 
                                                       serial::Timeout::simpleTimeout(500));
        if (serial_inst_->isOpen())
        {
            RCLCPP_INFO(node->get_logger(), "Modbus serial port opened successfully: %s", serial_port_.c_str());
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed to open Modbus serial port: %s", serial_port_.c_str());
        }
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open serial %s: %s", serial_port_.c_str(), e.what());
    }

    // 创建发布者
    if (use_stamped_vel) {
        pub_twist = node->create_publisher<Twist>("cmd_vel", 50);
    } else {
        pub_twist_unstamped = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 50);
    }
    // 创建温度发布者
    if (enable_temperature_monitoring_.load()) {
        temperature_pub = node->create_publisher<std_msgs::msg::Float32>("max_temperature", 10);
    }

    // 使用统一的设备初始化
    if (initDevice())
    {
        RCLCPP_INFO(node->get_logger(), "Device (PTZ + Audio + Temperature) initialized successfully");
        
        // 启动温度监控定时器（如果启用）
        if (enable_temperature_monitoring_.load() && temperature_pub) {
            temperature_timer = node->create_wall_timer(
                std::chrono::milliseconds(temperature_interval_ms_),
                temperatureTimerCallback
            );
            RCLCPP_INFO(node->get_logger(), "Temperature monitoring timer started - interval: %dms", temperature_interval_ms_);
        }
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Device initialization failed, PTZ, audio and temperature functions will be disabled");
    }
    
    // 创建订阅者
    rclcpp::Subscription<whi_interfaces::msg::WhiMotionState>::SharedPtr subMotionState;
    if (!stateTopic.empty()) {
        subMotionState = node->create_subscription<whi_interfaces::msg::WhiMotionState>(
            stateTopic, 10, subCallbackMotionState);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    if (!odomTopic.empty()) {
        subOdom = node->create_subscription<nav_msgs::msg::Odometry>(
            odomTopic, 10, subCallbackOdom);
    }

    rclcpp::Subscription<whi_interfaces::msg::WhiBattery>::SharedPtr subBatteryData;
    if (!batteryTopic.empty()) {
        subBatteryData = node->create_subscription<whi_interfaces::msg::WhiBattery>(
            batteryTopic, 10, subCallbackBatteryData);
    }

    rclcpp::Subscription<whi_interfaces::msg::WhiTemperatureHumidity>::SharedPtr subTempHumidityPM25;
    if (!tempHumidityPM25Topic.empty()) {
        subTempHumidityPM25 = node->create_subscription<whi_interfaces::msg::WhiTemperatureHumidity>(
            tempHumidityPM25Topic, 10, subCallbackTempHumidityPM25);
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subSwEstop;
    if (!swEstopTopic.empty()) {
        subSwEstop = node->create_subscription<std_msgs::msg::Bool>(
            swEstopTopic, 10, subCallbackSwEstop);
    }

    rclcpp::Subscription<whi_interfaces::msg::WhiRcState>::SharedPtr subRcState;
    if (!rcStateTopic.empty()) {
        subRcState = node->create_subscription<whi_interfaces::msg::WhiRcState>(
            rcStateTopic, 10, subCallbackRcState);
    }
    // 在创建其他订阅者的地方添加这个订阅者
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr detection_trigger_sub;
    detection_trigger_sub = node->create_subscription<std_msgs::msg::Int32>(
        "/fire_trigger", 10, detectionTriggerCallback);

    curl_global_init(CURL_GLOBAL_DEFAULT);

    file_server_thread = std::make_shared<std::thread>(startFileReceiver);

    ws_thread = std::make_shared<std::thread>([ws_uri]() {
        runWebSocketClient(ws_uri);
    });

    auto loopPub = [&]() {
        if (toggle_estop.load() || toggle_collision.load() || sw_estopped) {
            publishVelocity(0.0, 0.0);
        }
    };

    auto period = std::chrono::duration<double>(1.0 / frequency);
    auto timer = node->create_wall_timer(period, loopPub);

    auto sendCombinedData = [&]() {
        if (ws_connected.load()) {
            sendCombinedDataToServer();
        }
    };
    auto combinedDataPeriod = std::chrono::duration<double>(1.0 / combined_data_send_frequency);
    auto combinedDataTimer = node->create_wall_timer(combinedDataPeriod, sendCombinedData);

    auto connectionMonitor = [&]() {
        if (!ws_connected.load()) {
            publishVelocity(0.0, 0.0);
        }
    };
    auto connectionTimer = node->create_wall_timer(std::chrono::seconds(1), connectionMonitor);

    rclcpp::spin(node);
    // 程序正常结束时也保存统计数据
    savePersistentStats();

    curl_global_cleanup();

    return 0;
}