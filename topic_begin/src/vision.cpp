#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "duckietown_msgs/Twist2DStamped.h"
#include "duckietown_msgs/Pose2DStamped.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <cmath>
#include <termio.h>

#define disSize 0.0
#define kp_v 2000.0
#define ki_v 0.0
#define kd_v 0.0
#define kp_a 0.001
#define ki_a 0.0
#define kd_a 0.0
#define d_Angle 2.5

#define turn_L 3.1
#define turn_R -3.2
#define v_Low 0.16
#define a_Low 0.44
#define v_Max 0.2
#define c_Periods 3
#define v_Turn 0.8
#define turn_StrL -0.5
#define turn_StrR 0.5
#define v_Tiny 0.3

static char cmd = 't';
double angle;
double dst_Angle = 0.0;
static double signArea = 0.0;
static double lastArea = 0.0;
static cv::Point2d mc = cv::Point2d(0.0, 0.0);
static bool tran = false;
static bool leftflag = false;
static bool rightflag = false;
static int w_period = 0;

void chatterCallback(const sensor_msgs::Image& msg);
void display(cv::Mat &im, cv::Mat &bbox);
int scanKeyboard();

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("duckiebot7/image_rect", 1, chatterCallback);
    ros::Publisher pub = n.advertise<duckietown_msgs::Twist2DStamped>("duckiebot7/joy_mapper_node/car_cmd", 1000);


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

    ros::Rate loop_rate(1);

    int period = c_Periods;

    while (ros::ok()) 
    {
        if(1)
        {
            std::cout << "cmd = " << cmd << std::endl;
            switch (cmd)
            {
            // null
            case 't':
                break;
				
			// avoidance
            case 'w':
                switch (w_period)
                {
                    case 0:case 7:
                        v.data = 0;
                        omega.data = turn_L;
                        msg.header = header;
                        msg.v = v.data;
                        msg.omega = omega.data;  
                    break;

                    case 3:case 4:
                        v.data = v_Low * 45000.0 / signArea;
                        omega.data = a_Low;
                        msg.header = header;
                        msg.v = v.data;
                        msg.omega = omega.data;
                    break;

                    case 2:case 5:
                        v.data = 0;
                        omega.data = turn_R;
                        msg.header = header;
                        msg.v = v.data;
                        msg.omega = omega.data;   
                    break;

                    case 1:case 6:case 8:
                        v.data = v_Low * 5;
                        omega.data = a_Low;
                        msg.header = header;
                        msg.v = v.data;
                        msg.omega = omega.data;
                    break;

                    case 9:
                        v.data = 0;
                        omega.data = 0;
                        msg.header = header;
                        msg.v = v.data;
                        msg.omega = omega.data;
                    break;
                }
                if(w_period < 9)w_period++;
                else w_period = 9;
                break;
				
            // forward slowly
            case 'a':
                v.data = v_Low;
                omega.data = a_Low;
                msg.header = header;
                msg.v = v.data;
                msg.omega = omega.data;
                break;
				
            // forward slowly
            case 'd':
                v.data = v_Low;
                omega.data = a_Low;
                msg.header = header;
                msg.v = v.data;
                msg.omega = omega.data;
                cmd = 'r';
                break;
				
            // stop
            case 's':
                v.data = 0;
                omega.data = 0;
                msg.header = header;
                msg.v = v.data;
                msg.omega = omega.data; 
                break;
				
            // pause
            case 'p':
                v.data = 0;
                omega.data = 0;
                msg.header = header;
                msg.v = v.data;
                msg.omega = omega.data; 
                // std::cout << "period = " << period << std::endl;
                // if(period > 0)period --;
                // else if(period == 0) cmd = 'r';   
                if(!tran)cmd = 'a'; 
                if(tran)cmd = 'd';          
                break;
				
            // turn left
            case 'l':
                if(signArea < disSize)
                {
                    v.data = std::min(kp_v / signArea, v_Max);
                    omega.data = 0;
                    msg.header = header;
                    msg.v = v.data;
                    msg.omega = omega.data; 
                    leftflag = true;  
                }
                else
                {
                    v.data = 0;
                    omega.data = turn_L;
                    msg.header = header;
                    msg.v = v.data;
                    msg.omega = omega.data;  
                    std::cout << tran << std::endl;                  
                    cmd ='p';
                    period = c_Periods;
                    leftflag = true;
                }      
                break;
				
            // turn right
            case 'r':
                if(signArea < disSize)
                {
                    v.data = std::min(kp_v / signArea, v_Max);
                    omega.data = 0;
                    msg.header = header;
                    msg.v = v.data;
                    msg.omega = omega.data; 
                }
                else
                {
                    v.data = 0;
                    omega.data = turn_R;
                    msg.header = header;
                    msg.v = v.data;
                    msg.omega = omega.data;   
                    period = -1;        
                    cmd ='p';
                    rightflag = true;
                }  
                break;
			
            // follow
            case 'f':
				// too close
                if(signArea > 18000.0)
                {
                    v.data = 0;
                    omega.data = 0;
                    msg.header = header;
                    msg.v = v.data;
                    msg.omega = omega.data; 
                    period = c_Periods;  
                }
                else
                {
					// middle
                    if(mc.x > 300 && mc.x < 340)
                    {
                        v.data = std::min(kp_v / signArea, v_Max);
                        omega.data = a_Low;
                        msg.header = header;
                        msg.v = v.data;
                        msg.omega = omega.data;  
                        period = c_Periods;  
                        cmd = 's';
                    }
                    else if(mc.x <= 300)
                    {
                        v.data = std::min(kp_v / signArea, v_Max);
                        omega.data = a_Low + std::min(1.0, 0.01 * (280 - mc.x));
                        msg.header = header;
                        msg.v = v.data;
                        msg.omega = omega.data;  
                        period = c_Periods; 
                        cmd = 's';
                    }
                    else if(mc.x >= 340)
                    {
                        v.data = std::min(kp_v / signArea, v_Max);
                        omega.data =  std::max(-1.0, -0.01 * (mc.x - 340));
                        msg.header = header;
                        msg.v = v.data;
                        msg.omega = omega.data;  
                        period = c_Periods; 
                        cmd = 's';
                    }
                }
                break;
            default:
                break;
            }
            std::cout << "cmd = " << cmd << "," << msg.v << "," << msg.omega << std::endl;
            pub.publish(msg);
        }
        ros::spinOnce();  
        loop_rate.sleep();
        }
        
    return 0;
}

void chatterCallback(const sensor_msgs::Image& msg)
{
    cv_bridge::CvImagePtr cvCopyPtr;
	// give a copy
    try
    {
        cvCopyPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const std::exception& e)
    {
        std::cerr << "CV Bridge ERROR" << e.what() << '\n';
        return;
    }

    cv::Mat src_image = cvCopyPtr-> image;

    cv::Mat target_image = src_image;

	// increase contrast
    for( int y = 0; y < src_image.rows; y++ )
    {
        for( int x = 0; x < src_image.cols; x++ )
        {
            for( int c = 0; c < 3; c++ )
            {
                target_image.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>( 1.05*( src_image.at<cv::Vec3b>(y,x)[c] ) + 0 );
            }
        }
    }

	// get QR Code , calculate size of area and the Centroid 
    cv::cvtColor(target_image, target_image, cv::COLOR_BGR2GRAY);
    cv::QRCodeDetector qrDecoder;
    cv::Mat bbox, rectifiedImage;
    std::string data = qrDecoder.detectAndDecode(target_image, bbox, rectifiedImage);
    cv::Moments mu;

	// get Code 
    if(data.length() > 0)
    {
        std::cout << "Decoded Data : " << data << std::endl;

        if(data == "stop")
        {
            cmd = 's';
            tran = false;    
        }
        else if(data == "left")
        {
            if(!leftflag)cmd = 'l';
            tran = false;
        }
        else if(data == "right")
        {
            if(!rightflag)cmd = 'r';
            tran = false;
        }
        else if(data == "leader")
        {
            cmd = 'f';
            tran = true;
        }
        else if(data == "start")
        {
            cmd = 'w';
        }
        lastArea = signArea;
        signArea = cv::contourArea(bbox);
        mu = cv::moments(bbox);
        mc.x = mu.m10 / mu.m00;
        mc.y = mu.m01 / mu.m00;
        ROS_INFO("I heard: (size, pose) = [%f,%f]", signArea, mc.x);
     
    }
	
	//no Code 
    else 
    {
        std::cout << "QR Code not detected, pose_angle = [" << angle << "]" << std::endl;
    }
}

// display the rectified QR Code
void display(cv::Mat &im, cv::Mat &bbox)
{
  int n = bbox.rows;
  for(int i = 0 ; i < n ; i++)
  {
    cv::line(im, cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), cv::Point2i(bbox.at<float>((i+1) % n,0), bbox.at<float>((i+1) % n,1)), cv::Scalar(255,0,0), 3);
  }
  cv::imshow("Result", im);
}
