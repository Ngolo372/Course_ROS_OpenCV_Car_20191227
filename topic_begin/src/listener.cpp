#include "ros/ros.h"
#include "std_msgs/String.h"
#include "duckietown_msgs/Twist2DStamped.h"
#include "std_msgs/Float32.h"

void chatterCallback(const duckietown_msgs::Twist2DStamped& msg)
{
    ROS_INFO("I heard: v = [%f]", msg.v);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("duckiebot7/joy_mapper_node/car_cmd", 1000, chatterCallback);
    ros::spin();
    return 0;
}
