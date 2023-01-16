#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include<vector>
#include<cv_bridge/cv_bridge.h>
#include<opencv/cv.h>
#include"opencv/cv.hpp"
#include <math.h>
#include <queue>
#include <image_transport/image_transport.h>
#include<iostream>
#include <std_msgs/Float64.h>
using namespace std;
using namespace cv;
class perception
{
private:
    //sensor_msgs::Image rgb_data;
    vector<int> rgb_r,rgb_g,rgb_b;
    int image_height,image_width;
    queue<Mat> q_rgb;
    Mat rpg_data;

    //tf2_ros::Buffer tf_buffer_;
    //tf2_ros::TransformListener tf_listener_;  //!< @brief tf listener
public:
    perception();
    ~perception();

    void callbackRGBData(const sensor_msgs::Image::ConstPtr & msg);
    void callback_depth(const sensor_msgs::Image::ConstPtr& msg);
    void callbackTimerControl(const ros::TimerEvent & event);
    void cvtest();
    ros::NodeHandle nh_;
    ros::Subscriber sub_rgb_;
    ros::Subscriber sub_depthl_;
    ros::Timer timer_control_;
    pair<std_msgs::Float64,std_msgs::Float64> x_y_location;

};


