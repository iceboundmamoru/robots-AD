#include"opject_planning.h"

opject_planning::opject_planning()
{
 


    timer_control = nh.createTimer(ros::Duration(1.0 / 30), &opject_planning::CallbackTimerControl, this);
    sub_guide = nh.subscribe("/yjkj_vector_map/guidance", 1000, &opject_planning::CallbackVectorMap, this);
    sub_bestpositionl = nh.subscribe("/minibus/bestposition", 1000, &opject_planning::CallbackLocation, this);
    sub_vehicle_info = nh.subscribe("/minibus/vehicle/info", 1000, &opject_planning::CallbackVehicleInfo, this);
}

opject_planning::~opject_planning()
{

}

void opject_planning::CallbackTimerControl(const ros::TimerEvent & event){
     
       // imshow("img", rpg_data); 
       if(!CheckInit()&!isinit){
        ROS_ERROR("Init  fault");
        return ;
       }else{
        isinit = true;
       }
       GetVehileCurPos(); 
       GetVehileCurVelo(); 
       idx_vehicle_pos_in_lane=CalcVehicleIdxInMapFrame(current_pos,*vector_map_ptr); 
       GetX0(4);
       GetF0(2);
}

bool opject_planning::CheckInit(){
  if((!current_pose_ptr)||(!vector_map_ptr)||(!vehicle_info_ptr)){
    return false;
  }else{
    return true;
  }
}

void opject_planning::GetVehileCurPos(){
    current_pos[0] = current_pose_ptr->gaussX*0.01;
    current_pos[1] = current_pose_ptr->gaussY*0.01;
    current_pos[2] = current_pose_ptr->azimuth;
    ROS_INFO("current_pos_x = %f",current_pos[0]);
    ROS_INFO("current_pos_y = %f",current_pos[1]);
    ROS_INFO("current_pos_theta = %f",current_pos[2]);
}

void opject_planning::GetVehileCurVelo(){
    current_velo = vehicle_info_ptr->Wheel_Speed_ABS/3.6;
    ROS_INFO("current_velo_m/s = %f",current_velo);
}

int opject_planning::GetCurrentState(int lane_cur, const util::Guidance lane_ptr){
  lane_state = lane_ptr.scene_pts[lane_cur].type;
  return lane_state;
}

int opject_planning::CalcVehicleIdxInMapFrame(const DVec<double> cur_pos, const util::Guidance lane_ptr){
  int idx_,lane_idx_;
  util::Lane lane_;
 double min_dist_squared = numeric_limits<double>::max();
  lane_idx_ = lane_ptr.curr_idx - 1;
  ROS_INFO("lane_idx_ = %d",lane_idx_);
  ROS_INFO("lane_idx_x= %f", lane_ptr.lanes[lane_idx_].points[0].x);
  lane_ = lane_ptr.lanes[lane_idx_];
  for(uint i =0;i<lane_.points.size();i++){
    const double dx = cur_pos[0] - lane_.points[i].x;
    const double dy = cur_pos[1] - lane_.points[i].y;
    const double dist_squared = dx * dx + dy *dy ;
    const double err_yaw = normalizeRadian(cur_pos[2] -atan(dy/dx));
    if (std::fabs(err_yaw) > (M_PI / 3.0)) {
      continue;
    }
    if (dist_squared < min_dist_squared) {
      min_dist_squared = dist_squared;
      idx_ = i;
    }
  }
  return idx_;
}

void opject_planning::GetX0(int m){
  x0.resize(m) ;
  x0.setZero();
 // x0<< current_pos[0], current_pos[1], current_pos[2],current_velo;
  x0<< 1, 2, 3,4;
 cout<<x0<<endl;
  //ROS_INFO("x0_size = %f",x0.size); 
  /*ROS_INFO("current_pos_x = %f",x0(0));
  ROS_INFO("current_pos_y = %f",x0(1));
  ROS_INFO("current_pos_theta = %f",x0(2));
  ROS_INFO("current_velo = %f",x0(3));*/
}

void opject_planning::GetF0(int m){
  u0[0] = (vehicle_info_ptr->EPS_Ang_Stat_SteeringAngle/Steer_Ratio)*DEG2RAD;
  u0[1] = (vehicle_info_ptr->LongitudinalAcceleration);
  ROS_INFO("LongitudinalAcceleration = %f",u0[1]);
}

double opject_planning::normalizeRadian(const double angle)
{
  double n_angle = std::fmod(angle, 2 * M_PI);
  n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
  return n_angle;
}

void opject_planning::CallbackVectorMap(const util::Guidance::ConstPtr& map_ptr){
  vector_map_ptr =  std::make_shared<util::Guidance>(*map_ptr);
}

void opject_planning::CallbackLocation(const util::GpsPosition::ConstPtr& pos_ptr){
  current_pose_ptr =  std::make_shared<util::GpsPosition>(*pos_ptr);
  //ROS_INFO("junction_direction = %d",current_pose_ptr_->gaussX);
}

void opject_planning::CallbackVehicleInfo(const util::VehicleInfo::ConstPtr& vinfo_ptr){
  vehicle_info_ptr =  std::make_shared<util::VehicleInfo>(*vinfo_ptr);
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "opject_planning");
  opject_planning node;
 // node.cvtest();
  ros::spin();
  return 0;
}
