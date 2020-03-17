#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"

#include "duckietown_msgs/Twist2DStamped.h"

#include <sstream>
#include <iostream>

#include <termio.h>
#include <stdio.h>

void chatterCallback(const duckietown_msgs::Twist2DStamped& msg)
{
    ROS_INFO("I heard v = : [%f]", msg.v);
}

class person
{
private:
    /* data */
    ros::NodeHandle n;
    ros::Publisher chatter_pub;
    ros::Subscriber chatter_sub;
public:
    person();
    ~person();
    void publish(duckietown_msgs::Twist2DStamped msg);
};

person::person()
{
    
    chatter_pub = n.advertise<duckietown_msgs::Twist2DStamped>("duckiebot7/joy_mapper_node/car_cmd", 1000);
    chatter_sub = n.subscribe("duckiebot7/joy_mapper_node/car_cmd", 1000, chatterCallback);

}

person::~person()
{
}

void person::publish(duckietown_msgs::Twist2DStamped msg)
{
    chatter_pub.publish(msg);
}

int scanKeyboard();

int main(int argc, char *argv[])
{
    /* code */
    ros::init(argc, argv, "talker");
    person Ren;
    ros::Rate loop_rate(1000);

    while (ros::ok())
    {

        /* code */
        bool ready = false;

        std_msgs::Header header;
        std_msgs::UInt32 seq;
        std_msgs::Time stamp;
        std_msgs::String frame_id;
        std_msgs::Float32 v, omega;
        
        frame_id.data = '\' \'';
        header.frame_id = frame_id.data;
        header.seq = seq.data;
        header.stamp = stamp.data;
        
        duckietown_msgs::Twist2DStamped msg;

        int key = scanKeyboard();
        std::cout << key << std::endl;

		//ready
        if(key == 27 || key == 91){
            ready = true;
        }

		// forward
        else if(key == 65){
            v.data = 4;
            omega.data = 0;
            msg.header = header;
            msg.v = v.data;
            msg.omega = omega.data;        
            Ren.publish(msg);
            ros::spinOnce();            
        }

        // back
		else if(key == 66){
            v.data = -4;
            omega.data = 0;
            msg.header = header;
            msg.v = v.data;
            msg.omega = omega.data;   
            Ren.publish(msg);
            ros::spinOnce();            
        }

		//left
        else if(key == 67){
            v.data = 0;
            omega.data = 4;
            msg.header = header;
            msg.v = v.data;
            msg.omega = omega.data; 
            Ren.publish(msg);
            key = 10;           
            ros::spinOnce(); 
        }

		//right
        else if(key == 68){
            v.data = 0;
            omega.data = -4;
            msg.header = header;
            msg.v = v.data;
            msg.omega = omega.data;          
            Ren.publish(msg);
            ros::spinOnce();            
        }

		//stop
        else if(key == 10){
            ready = false;
            v.data = 0;
            omega.data = 0;
            msg.header = header;
            msg.v = v.data;
            msg.omega = omega.data;             
            Ren.publish(msg);
            key = 10;           
            ros::spinOnce();            
        }

		//invalid keys
        else{
            ready = false;
        }
        
    }
    
    return 0;
}

//scan the keybord
int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    
    in = getchar();
    
    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}