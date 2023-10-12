// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_serial_driver
{
#define PI 3.1415926535f
#define GRAVITY 9.78
class solve_angle
{
public:
  typedef unsigned char uint8_t;
  enum ARMOR_ID {
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
  };

  enum ARMOR_NUM { ARMOR_NUM_BALANCE = 2, ARMOR_NUM_OUTPOST = 3, ARMOR_NUM_NORMAL = 4 };

  enum BULLET_TYPE { BULLET_17 = 0, BULLET_42 = 1 };

  //设置参数
  struct SolveTrajectoryParams
  {
    float k;  //弹道系数

    //自身参数
    enum BULLET_TYPE bullet_type;  //自身机器人类型 0-步兵 1-英雄
    float current_v = 14;          //当前弹速
    float current_pitch;           //当前pitch
    float current_yaw;             //当前yaw

    //目标参数
    float xw;                  //ROS坐标系下的x
    float yw;                  //ROS坐标系下的y
    float zw;                  //ROS坐标系下的z
    float vxw;                 //ROS坐标系下的vx
    float vyw;                 //ROS坐标系下的vy
    float vzw;                 //ROS坐标系下的vz
    float tar_yaw;             //目标yaw
    float v_yaw;               //目标yaw速度
    float r1;                  //目标中心到前后装甲板的距离
    float r2;                  //目标中心到左右装甲板的距离
    float dz;                  //另一对装甲板的相对于被跟踪装甲板的高度差
    int bias_time;             //偏置时间
    float s_bias;              //枪口前推的距离
    float z_bias;              //yaw轴电机到枪口水平面的垂直距离
    enum ARMOR_ID armor_id;    //装甲板类型  0-outpost 6-guard 7-base
                               //1-英雄 2-工程 3-4-5-步兵
    enum ARMOR_NUM armor_num;  //装甲板数字  2-balance 3-outpost 4-normal
  };

  //用于存储目标装甲板的信息
  struct tar_pos
  {
    float x;    //装甲板在世界坐标系下的x
    float y;    //装甲板在世界坐标系下的y
    float z;    //装甲板在世界坐标系下的z
    float yaw;  //装甲板坐标系相对于世界坐标系的yaw角
  };

  //单方向空气阻力模型
  float monoDirectionalAirResistanceModel(float s, float v, float angle);
  //完全空气阻力模型
  // float completeAirResistanceModel(float s, float v, float angle);
  //pitch弹道补偿
  float pitchTrajectoryCompensation(float s, float y, float v);
  //根据最优决策得出被击打装甲板 自动解算弹道
  bool autoSolveTrajectory(float * pitch, float * yaw, float * aim_x, float * aim_y, float * aim_z);

  struct SolveTrajectoryParams st;
  struct tar_pos tar_position[4];  //最多只有四块装甲板
  float t = 0.5f;                  // 飞行时间
};
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void reopenPort();

  void setParam(const rclcpp::Parameter & param);

  void resetTracker();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_param_ = false;
  uint8_t previous_receive_color_ = 0;
  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  // Broadcast tf from odom to gimbal_link
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  solve_angle solve;
  bool is_hero;
  std::thread receive_thread_;
};

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
