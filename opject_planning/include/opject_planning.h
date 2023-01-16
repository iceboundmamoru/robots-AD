#include<ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include<vector>
#include <math.h>
#include <queue>
#include<iostream>
#include <std_msgs/Float64.h>
#include "casadi/casadi.hpp"
#include "util/Guidance.h"
#include"util/GpsPosition.h"
#include"util/ScenePt.h"
#include"util/VehicleInfo.h"
#include"util/Controller_t.h"
#include <matrix.h>


#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535
using namespace std;
using namespace casadi;
class opject_planning
{
public:
//template <typename T>
    opject_planning();
    ~opject_planning();
    bool isinit=false;
private:
   //ros 
    ros::NodeHandle nh;
    ros::Subscriber sub_guide;
    ros::Subscriber sub_bestpositionl;
    ros::Subscriber sub_vehicle_info;
    ros::Timer timer_control;
   //vehicle
   double Steer_Ratio = 23.58;
   double wheel_base = 4;
   //opt 
    DVec<double> x0,xf,u0,uf;
    Vec3<double> current_pos;
    double current_velo;
    int idx_vehicle_pos_in_lane;
    //others
    int lane_state;
    // topic pointer
    shared_ptr<util::Guidance> vector_map_ptr ;
    shared_ptr<util::GpsPosition> current_pose_ptr ;
    shared_ptr<util::VehicleInfo> vehicle_info_ptr ;
    shared_ptr<util::Controller_t> vehicle_ctrl_ptr ;
    
    
    void CallbackTimerControl(const ros::TimerEvent & event);
    void CallbackVectorMap(const util::Guidance::ConstPtr& map_ptr);
    void CallbackLocation(const util::GpsPosition::ConstPtr& pos_ptr);
    void CallbackVehicleInfo(const util::VehicleInfo::ConstPtr& vinfo_ptr);
    void PubCmd(const util::Controller_t cmd);
    bool CheckInit();
    void GetX0(int m);
    void GetF0(int m );
    void GetXf(DVec<double> Xf);
    void GetBeginState(int idx_closest);
    void GetFinalState(int idx_final);
    int CalcVehicleIdxInMapFrame(const DVec<double> cur_pos,const util::Guidance  lane_ptr);
    void GetVehileCurPos();
    void GetVehileCurVelo();
    void OptimizeMotion(const DVec<double> & X0, DVec<double> Xf);
    double normalizeRadian(const double angle);
    int GetCurrentState( int lane_cur,const util::Guidance lane_ptr);
    MX VehicleDyn(const MX& x, const MX& u);
};


