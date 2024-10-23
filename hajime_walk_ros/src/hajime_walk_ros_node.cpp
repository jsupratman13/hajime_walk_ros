#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <hajime_walk_msgs/HajimeWalk.h>
#include <hajime_walk_msgs/HajimeMotion.h>

#include "hr46/cntr.hpp"

class HajimeWalkRosNode
{
public:
  HajimeWalkRosNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh_(nh), private_nh_(private_nh)
  {
    cntr_ = std::make_unique<hr46::Cntr>();

    std::string motion_path;
    private_nh_.param<std::string>("motion_path", motion_path, "motions");
    cntr_->setMotionPath(motion_path);

    loadEEPROMParams();

    motion_flag_ = false;
    sub_walk_ = nh_.subscribe("/hajime_walk/walk", 10, &HajimeWalkRosNode::walkCallback, this);
    sub_cancel_ = nh_.subscribe("/hajime_walk/cancel", 10, &HajimeWalkRosNode::cancelCallback, this);
    sub_motion_ = nh_.subscribe("/hajime_walk/motion", 10, &HajimeWalkRosNode::motionCallback, this);
    sub_imu_ = nh_.subscribe("/imu/data", 1000, &HajimeWalkRosNode::imuCallback, this);
    joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_position_controller/command", 1);
  };

  void update()
  {
    if (!motion_flag_)
    {
      cntr_->setCommand(cmd_type_, walk_cmd_.num_step, walk_cmd_.stride_th, walk_cmd_.stride_x, walk_cmd_.period,
                        walk_cmd_.stride_y);
    }
    cntr_->cntr();

    std_msgs::Float64MultiArray multi_rad;
    multi_rad.data = cntr_->getJoints();
    joint_pub_.publish(multi_rad);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_walk_;
  ros::Subscriber sub_cancel_;
  ros::Subscriber sub_motion_;
  ros::Subscriber sub_imu_;
  ros::Publisher joint_pub_;
  hr46::CntrUniquePtr cntr_;

  bool motion_flag_;
  hajime_walk_msgs::HajimeWalk walk_cmd_;
  char cmd_type_;

  void motionCallback(const hajime_walk_msgs::HajimeMotion::ConstPtr& msg)
  {
    motion_flag_ = true;
    cntr_->setCommand('M', msg->motion_id, 0, 0, msg->num_repeat, 0);
  };
  void cancelCallback(const std_msgs::Empty::ConstPtr& /* msg */)
  {
    motion_flag_ = false;
    cmd_type_ = 'C';
  };
  void walkCallback(const hajime_walk_msgs::HajimeWalk::ConstPtr& msg)
  {
    motion_flag_ = false;
    cmd_type_ = 'A';
    walk_cmd_ = *msg;
  };
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    hr46::st_xv_acc xv_acc;
    xv_acc.acc_data1 = -msg->linear_acceleration.x;
    xv_acc.acc_data2 = -msg->linear_acceleration.y;
    xv_acc.acc_data3 = -msg->linear_acceleration.z;

    const float radian_to_degree = 180.0 / M_PI;
    hr46::st_xv_gyro xv_gyro;
    xv_gyro.gyro_data1 = msg->angular_velocity.x * radian_to_degree;
    xv_gyro.gyro_data2 = msg->angular_velocity.y * radian_to_degree;
    xv_gyro.gyro_data3 = msg->angular_velocity.z * radian_to_degree;
    xv_gyro.quaternion[0] = msg->orientation.w;
    xv_gyro.quaternion[1] = msg->orientation.x;
    xv_gyro.quaternion[2] = msg->orientation.y;
    xv_gyro.quaternion[3] = msg->orientation.z;
    xv_gyro.gyro_roll = xv_gyro.gyro_roll2 = roll * radian_to_degree;
    xv_gyro.gyro_pitch = xv_gyro.gyro_pitch2 = -pitch * radian_to_degree;
    xv_gyro.gyro_yaw = xv_gyro.gyro_yaw2 = yaw * radian_to_degree;

    cntr_->setImuData(xv_acc, xv_gyro);
  };

  void loadEEPROMParams()
  {
    std::map<std::string, double> param;

    double odometry_correct_para_x;
    double odometry_correct_para_y;
    private_nh_.param<double>("eeprom_list/odometry_correct_para_x", odometry_correct_para_x, 0.0);
    private_nh_.param<double>("eeprom_list/odometry_correct_para_y", odometry_correct_para_y, 0.0);
    param["odometry_correct_para_x"] = odometry_correct_para_x;
    param["odometry_correct_para_y"] = odometry_correct_para_y;

    hr46::st_xp_acc xp_acc;
    private_nh_.getParam("eeprom_list/xp_acc/acc_k1", xp_acc.acc_k1);
    private_nh_.getParam("eeprom_list/xp_acc/acc_k2", xp_acc.acc_k2);
    private_nh_.getParam("eeprom_list/xp_acc/acc_k3", xp_acc.acc_k3);
    private_nh_.getParam("eeprom_list/xp_acc/ad_volt_offset1", xp_acc.ad_volt_offset1);
    private_nh_.getParam("eeprom_list/xp_acc/ad_volt_offset2", xp_acc.ad_volt_offset2);
    private_nh_.getParam("eeprom_list/xp_acc/ad_volt_offset3", xp_acc.ad_volt_offset3);
    private_nh_.getParam("eeprom_list/xp_acc/t1", xp_acc.t1);
    private_nh_.getParam("eeprom_list/xp_acc/t2", xp_acc.t2);
    private_nh_.getParam("eeprom_list/xp_acc/fall_fwd", xp_acc.fall_fwd);
    private_nh_.getParam("eeprom_list/xp_acc/fall_bwd", xp_acc.fall_bwd);
    private_nh_.getParam("eeprom_list/xp_acc/fall_right", xp_acc.fall_right);
    private_nh_.getParam("eeprom_list/xp_acc/fall_left", xp_acc.fall_left);
    private_nh_.getParam("eeprom_list/xp_acc/fall_check_time", xp_acc.fall_check_time);
    private_nh_.getParam("eeprom_list/xp_acc/fall_pitch", xp_acc.fall_pitch);
    private_nh_.getParam("eeprom_list/xp_acc/fall_roll", xp_acc.fall_roll);
    private_nh_.getParam("eeprom_list/xp_acc/fall_pitch_oblique", xp_acc.fall_pitch_oblique);
    private_nh_.getParam("eeprom_list/xp_acc/fall_roll_oblique", xp_acc.fall_roll_oblique);

    hr46::st_xp_gyro xp_gyro;
    private_nh_.getParam("eeprom_list/xp_gyro/kp1_foot", xp_gyro.kp1_foot);
    private_nh_.getParam("eeprom_list/xp_gyro/kp2_foot", xp_gyro.kp2_foot);
    private_nh_.getParam("eeprom_list/xp_gyro/kp1_hip", xp_gyro.kp1_hip);
    private_nh_.getParam("eeprom_list/xp_gyro/kp2_hip", xp_gyro.kp2_hip);
    private_nh_.getParam("eeprom_list/xp_gyro/kp1_arm", xp_gyro.kp1_arm);
    private_nh_.getParam("eeprom_list/xp_gyro/kp2_arm", xp_gyro.kp2_arm);
    private_nh_.getParam("eeprom_list/xp_gyro/kp2_waist", xp_gyro.kp2_waist);
    private_nh_.getParam("eeprom_list/xp_gyro/kp3_waist", xp_gyro.kp3_waist);
    private_nh_.getParam("eeprom_list/xp_gyro/gyro_k1", xp_gyro.gyro_k1);
    private_nh_.getParam("eeprom_list/xp_gyro/gyro_k2", xp_gyro.gyro_k2);
    private_nh_.getParam("eeprom_list/xp_gyro/gyro_k3", xp_gyro.gyro_k3);
    private_nh_.getParam("eeprom_list/xp_gyro/ad_volt_offset1", xp_gyro.ad_volt_offset1);
    private_nh_.getParam("eeprom_list/xp_gyro/ad_volt_offset2", xp_gyro.ad_volt_offset2);
    private_nh_.getParam("eeprom_list/xp_gyro/ad_volt_offset3", xp_gyro.ad_volt_offset3);
    private_nh_.getParam("eeprom_list/xp_gyro/t1", xp_gyro.t1);
    private_nh_.getParam("eeprom_list/xp_gyro/t2", xp_gyro.t2);
    private_nh_.getParam("eeprom_list/xp_gyro/gyro_data3_flt2_t1", xp_gyro.gyro_data3_flt2_t1);
    private_nh_.getParam("eeprom_list/xp_gyro/yaw_cntl_gain", xp_gyro.yaw_cntl_gain);
    private_nh_.getParam("eeprom_list/xp_gyro/yaw_cntl_dead", xp_gyro.yaw_cntl_dead);
    private_nh_.getParam("eeprom_list/xp_gyro/yaw_cntl_theta", xp_gyro.yaw_cntl_theta);
    private_nh_.getParam("eeprom_list/xp_gyro/gyro_omega", xp_gyro.gyro_omega);
    private_nh_.getParam("eeprom_list/xp_gyro/fall_roll_deg1", xp_gyro.fall_roll_deg1);
    private_nh_.getParam("eeprom_list/xp_gyro/fall_pitch_deg1", xp_gyro.fall_pitch_deg1);

    hr46::st_flag_gyro flag_gyro;
    float flag_gyro_fall_cntl;
    private_nh_.param<float>("eeprom_list/flag_gyro/fall_cntl", flag_gyro_fall_cntl);
    flag_gyro.fall_cntl = static_cast<short>(flag_gyro_fall_cntl);
    float flag_gyro_zero;
    private_nh_.param<float>("eeprom_list/flag_gyro/zero", flag_gyro_zero);
    flag_gyro.zero = static_cast<short>(flag_gyro_zero);

    hr46::st_xp_mv_straight xp_mv_straight;
    private_nh_.getParam("eeprom_list/xp_mv_straight/time", xp_mv_straight.time);
    private_nh_.getParam("eeprom_list/xp_mv_straight/z3", xp_mv_straight.z3);
    private_nh_.getParam("eeprom_list/xp_mv_straight/arm_sh_pitch", xp_mv_straight.arm_sh_pitch);
    private_nh_.getParam("eeprom_list/xp_mv_straight/arm_sh_roll", xp_mv_straight.arm_sh_roll);
    private_nh_.getParam("eeprom_list/xp_mv_straight/arm_el_yaw", xp_mv_straight.arm_el_yaw);
    private_nh_.getParam("eeprom_list/xp_mv_straight/arm_el_pitch", xp_mv_straight.arm_el_pitch);

    hr46::st_xp_mv_ready xp_mv_ready;
    private_nh_.getParam("eeprom_list/xp_mv_ready/time", xp_mv_ready.time);
    private_nh_.getParam("eeprom_list/xp_mv_ready/z3", xp_mv_ready.z3);
    private_nh_.getParam("eeprom_list/xp_mv_ready/arm_sh_pitch", xp_mv_ready.arm_sh_pitch);
    private_nh_.getParam("eeprom_list/xp_mv_ready/arm_sh_roll", xp_mv_ready.arm_sh_roll);
    private_nh_.getParam("eeprom_list/xp_mv_ready/arm_el_yaw", xp_mv_ready.arm_el_yaw);
    private_nh_.getParam("eeprom_list/xp_mv_ready/arm_el_pitch", xp_mv_ready.arm_el_pitch);
    private_nh_.getParam("eeprom_list/xp_mv_ready/pitch", xp_mv_ready.pitch);

    hr46::st_xp_mv_walk xp_mv_walk;
    float xp_mv_walk_num;
    private_nh_.getParam("eeprom_list/xp_mv_walk/num", xp_mv_walk_num);
    xp_mv_walk.num = static_cast<long>(xp_mv_walk_num);
    private_nh_.getParam("eeprom_list/xp_mv_walk/h_cog", xp_mv_walk.h_cog);
    private_nh_.getParam("eeprom_list/xp_mv_walk/time", xp_mv_walk.time);
    private_nh_.getParam("eeprom_list/xp_mv_walk/x_fwd_swg", xp_mv_walk.x_fwd_swg);
    private_nh_.getParam("eeprom_list/xp_mv_walk/x_fwd_spt", xp_mv_walk.x_fwd_spt);
    private_nh_.getParam("eeprom_list/xp_mv_walk/x_bwd_swg", xp_mv_walk.x_bwd_swg);
    private_nh_.getParam("eeprom_list/xp_mv_walk/x_bwd_spt", xp_mv_walk.x_bwd_spt);
    private_nh_.getParam("eeprom_list/xp_mv_walk/y_swg", xp_mv_walk.y_swg);
    private_nh_.getParam("eeprom_list/xp_mv_walk/y_spt", xp_mv_walk.y_spt);
    private_nh_.getParam("eeprom_list/xp_mv_walk/theta", xp_mv_walk.theta);
    private_nh_.getParam("eeprom_list/xp_mv_walk/z", xp_mv_walk.z);
    private_nh_.getParam("eeprom_list/xp_mv_walk/y_balance", xp_mv_walk.y_balance);
    private_nh_.getParam("eeprom_list/xp_mv_walk/hip_roll", xp_mv_walk.hip_roll);
    private_nh_.getParam("eeprom_list/xp_mv_walk/x_fwd_pitch", xp_mv_walk.x_fwd_pitch);
    private_nh_.getParam("eeprom_list/xp_mv_walk/x_bwd_pitch", xp_mv_walk.x_bwd_pitch);
    private_nh_.getParam("eeprom_list/xp_mv_walk/arm_sh_pitch", xp_mv_walk.arm_sh_pitch);
    private_nh_.getParam("eeprom_list/xp_mv_walk/start_time_k1", xp_mv_walk.start_time_k1);
    private_nh_.getParam("eeprom_list/xp_mv_walk/start_zmp_k1", xp_mv_walk.start_zmp_k1);
    // private_nh_.getParam("eeprom_list/xp_mv_walk/start_time_k2", xp_mv_walk.start_time_k2);
    private_nh_.getParam("eeprom_list/xp_mv_walk/foot_cntl_p", xp_mv_walk.foot_cntl_p);
    private_nh_.getParam("eeprom_list/xp_mv_walk/foot_cntl_r", xp_mv_walk.foot_cntl_r);
    private_nh_.getParam("eeprom_list/xp_mv_walk/sidestep_time_k", xp_mv_walk.sidestep_time_k);
    private_nh_.getParam("eeprom_list/xp_mv_walk/sidestep_roll", xp_mv_walk.sidestep_roll);
    private_nh_.getParam("eeprom_list/xp_mv_walk/y_wide", xp_mv_walk.y_wide);
    private_nh_.getParam("eeprom_list/xp_mv_walk/time_dutyfactor", xp_mv_walk.time_dutyfactor);
    private_nh_.getParam("eeprom_list/xp_mv_walk/x_fwd_acc_pitch", xp_mv_walk.x_fwd_acc_pitch);
    private_nh_.getParam("eeprom_list/xp_mv_walk/x_bwd_acc_pitch", xp_mv_walk.x_bwd_acc_pitch);
    private_nh_.getParam("eeprom_list/xp_mv_walk/accurate_x_percent_dlim", xp_mv_walk.accurate_x_percent_dlim);
    private_nh_.getParam("eeprom_list/xp_mv_walk/accurate_y_percent_dlim", xp_mv_walk.accurate_y_percent_dlim);
    private_nh_.getParam("eeprom_list/xp_mv_walk/accurate_th_percent_dlim", xp_mv_walk.accurate_th_percent_dlim);
    private_nh_.getParam("eeprom_list/xp_mv_walk/arm_el_pitch", xp_mv_walk.arm_el_pitch);

    hr46::st_xp_dlim_wait xp_dlim_wait_x;
    private_nh_.getParam("eeprom_list/xp_dlim_wait_x/dlim", xp_dlim_wait_x.dlim);
    private_nh_.getParam("eeprom_list/xp_dlim_wait_x/wait_time", xp_dlim_wait_x.wait_time);

    hr46::st_xp_dlim_wait xp_dlim_wait_y;
    private_nh_.getParam("eeprom_list/xp_dlim_wait_y/dlim", xp_dlim_wait_y.dlim);
    private_nh_.getParam("eeprom_list/xp_dlim_wait_y/wait_time", xp_dlim_wait_y.wait_time);

    hr46::st_xp_dlim_wait xp_dlim_wait_theta;
    private_nh_.getParam("eeprom_list/xp_dlim_wait_theta/dlim", xp_dlim_wait_theta.dlim);
    private_nh_.getParam("eeprom_list/xp_dlim_wait_theta/wait_time", xp_dlim_wait_theta.wait_time);

    hr46::st_xp_dlim_wait xp_dlim_wait_pitch;
    private_nh_.getParam("eeprom_list/xp_dlim_wait_pitch/dlim", xp_dlim_wait_pitch.dlim);

    cntr_->setEEPROM(param, xp_acc, xp_gyro, flag_gyro, xp_mv_straight, xp_mv_ready, xp_mv_walk, xp_dlim_wait_x,
                     xp_dlim_wait_y, xp_dlim_wait_theta, xp_dlim_wait_pitch);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "hajime_walk_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto node = HajimeWalkRosNode(nh, private_nh);
  ros::Rate rate(100);  // 10 ms

  while (ros::ok())
  {
    node.update();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
