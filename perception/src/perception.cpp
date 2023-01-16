#include"perception.h"

perception::perception()
{

    image_height = 0;
    image_height = 0;

     sub_rgb_ =  nh_.subscribe("/camera/color/image_raw", 1, &perception::callbackRGBData, this);
     
     sub_depthl_ = nh_.subscribe("/camera/depth/image_rect_raw", 1000, &perception::callback_depth, this);

    timer_control_ = nh_.createTimer(ros::Duration(1.0 / 30), &perception::callbackTimerControl, this);
    rpg_data.push_back(0);
}

perception::~perception()
{

}

void perception::cvtest(){
   
    imshow("img", rpg_data);
    	waitKey(0);
}

void perception::callbackTimerControl(const ros::TimerEvent & event){
     
       // imshow("img", rpg_data); 
}

void perception::callbackRGBData(const sensor_msgs::Image::ConstPtr & msg){

    int height,width;
    cv_bridge::CvImagePtr cv_ptr_rgb;
   height = msg->height;
   width = msg->width;
    cv_ptr_rgb = cv_bridge::toCvCopy(msg,"bgr8");
    rpg_data = cv_ptr_rgb->image;
    /*1280*720*/
    //cout<<"height"<<" "<<height <<endl;
    //cout<<"width"<<" "<<width <<endl;
  /********rgb_data = {B,G,R}**********/
   /* for(int i =0;i<100;i++){
      rpg_data.at<Vec3b>(height/2+i,width/2+i) = {0,0,255};
      //cout<<"r_data"<<" "<<rpg_data.at<Vec3b>(height/2+i,width/2+i) <<endl;
    }*/
    rpg_data.at<Vec3b>(height/2,width/2) = {0,0,255};
      imshow("img", rpg_data);
    	waitKey(1);
    q_rgb.push(rpg_data);
    Mat rgb_preb; //rgb->gray
    cvtColor(rpg_data,rgb_preb,CV_BGR2GRAY); //rgb->gray
     //imshow("img", rgb_preb);
    //	waitKey(1);
   q_rgb.pop();
}

 void perception::callback_depth(const sensor_msgs::Image::ConstPtr& msg)
  {
    //PUBLISHED_MESSAGE_TYPE output;
    //.... do something with the input and generate the output...
    //cout<<"depth_image"<<endl;
    cv_bridge::CvImagePtr cv_ptr_depth;
    cv_ptr_depth = cv_bridge::toCvCopy(msg);
    Mat depth = cv_ptr_depth->image;
    int height,width;
    height = msg->height;
    width = msg->height;
    cout<<"height"<<" "<<height <<endl;
    cout<<"width"<<" "<<width <<endl;
    std_msgs::Float64 z_location;
    //cout<<"times_"<<""<<"     "<<"depth_data"<<" "<<depth.at<double>(height/2,width/2) <<endl;
   // float d ;
   // ushort dist = depth.at<ushort>(height/2,width/2)/1000;
    //d = float(dist)/1000;
    //cout<<"times_"<<""<<"     "<<"depth_data"<<" "<<d <<endl;
    //z_location.data = depth.at<double>(x_y_location.first.data,x_y_location.second.data);
   /*  for(int i =0;i<100;i++){
      //depth.at<double>(height/2+i,width/2+i)= -100;
      cout<<"times_"<<i<<"     "<<"depth_data"<<" "<<depth.at<double>(height/2+i,width/2+i) <<endl;
    }*/
    z_location.data = depth.at<double>(height/2,width/2);
    //cout<<z_location.data<<endl;
    geometry_msgs::Pose output;

    // rs2::config cfg;
    // cfg.enable_stream(RS2_STREAM_DEPTH);
    // cfg.enable_stream(RS2_STREAM_COLOR);
    // pipe.start(cfg);
    // rs2::frameset frameset = pipe.wait_for_frames(); 
    // intrinsics  _color_intrin;

    //circle(rgb,center , 6, Scalar(0,0,255), 1);
    imshow("depth",depth);
    waitKey(1);

    output.position.x = x_y_location.first.data;
    output.position.y = x_y_location.second.data;
    output.position.z = z_location.data; 
    //cout<<"z"<<output.position.z<<endl;
    //pub_1.publish(output);
  }

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "perception");
  perception node;
 // node.cvtest();
  ros::spin();
  return 0;
}
