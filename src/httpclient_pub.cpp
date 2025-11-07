#include "rclcpp/rclcpp.hpp"
#include "whi_interfaces/msg/whi_bounding_box.hpp"
#include "whi_interfaces/msg/whi_bounding_boxes.hpp"

#include <sstream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto n = rclcpp::Node::make_shared("talker");
  
  auto mystate_pub = n->create_publisher<whi_interfaces::msg::WhiBoundingBoxes>("/myState", 10);
  
  rclcpp::Rate loop_rate(1);
  
  int count = 0;
  while (rclcpp::ok())
  {
    whi_interfaces::msg::WhiBoundingBoxes msg;
    
    std::vector<whi_interfaces::msg::WhiBoundingBox> detboxV;
    whi_interfaces::msg::WhiBoundingBox onedet;
    onedet.cls = "cls1";
    onedet.state = "90";
    msg.bounding_boxes.push_back(onedet);
    onedet.cls = "cls2";
    onedet.state = "80";       
    msg.bounding_boxes.push_back(onedet);
    onedet.cls = "cls3";
    onedet.state = "70";       
    msg.bounding_boxes.push_back(onedet);
         
    //msg.bounding_boxes = detboxV;
    
    RCLCPP_INFO(n->get_logger(), "start publish ");
    
    mystate_pub->publish(msg);
    
    rclcpp::spin_some(n);
    
    loop_rate.sleep();
    ++count;
  }
  
  rclcpp::shutdown();
  return 0;
}