#include "ros/ros.h"
#include "whi_interfaces/WhiBoundingBox.h"
#include "whi_interfaces/WhiBoundingBoxes.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher mystate_pub = n.advertise<whi_interfaces::WhiBoundingBoxes>("/myState", 10);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    whi_interfaces::WhiBoundingBoxes msg;

    std::vector<whi_interfaces::WhiBoundingBox> detboxV;
    whi_interfaces::WhiBoundingBox onedet;
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

    ROS_INFO("start publish ");

    mystate_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}