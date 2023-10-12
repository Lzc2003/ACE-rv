// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
float solve_angle::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
  float z;
  //t为给定v与angle时的飞行时间
  t = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
  //z为给定v与angle时的高度
  z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return z;
}

/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float solve_angle::pitchTrajectoryCompensation(float s, float z, float v)
{
  float z_temp, z_actual, dz;
  float angle_pitch;
  int i = 0;
  z_temp = z;
  // iteration
  for (i = 0; i < 20; i++) {
    angle_pitch = atan2(z_temp, s);  // rad
    z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
    dz = 0.3 * (z - z_actual);
    z_temp = z_temp + dz;
    if (fabsf(dz) < 0.00001) {
      break;
    }
  }
  return angle_pitch;
}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
bool solve_angle::autoSolveTrajectory(
  float * pitch, float * yaw, float * aim_x, float * aim_y, float * aim_z)
{
  // 线性预测
  float timeDelay = st.bias_time / 1000.0 + t;
  st.tar_yaw += st.v_yaw * timeDelay;

  //计算四块装甲板的位置
  //装甲板id顺序，以四块装甲板为例，逆时针编号
  //      2
  //   3     1
  //      0
  int use_1 = 1;
  int i = 0;
  int idx = 0;  // 选择的装甲板
  //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
  if (st.armor_num == ARMOR_NUM_BALANCE) {
    for (i = 0; i < 2; i++) {
      float tmp_yaw = st.tar_yaw + i * PI;
      float r = st.r1;
      tar_position[i].x = st.xw - r * cos(tmp_yaw);
      tar_position[i].y = st.yw - r * sin(tmp_yaw);
      tar_position[i].z = st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);

    //因为是平衡步兵 只需判断两块装甲板即可
    float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw);
    if (temp_yaw_diff < yaw_diff_min) {
      yaw_diff_min = temp_yaw_diff;
      idx = 1;
    }

  } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
    for (i = 0; i < 3; i++) {
      float tmp_yaw = st.tar_yaw + i * 2.0 * PI / 3.0;  // 2/3PI
      float r = (st.r1 + st.r2) / 2;                    //理论上r1=r2 这里取个平均值
      tar_position[i].x = st.xw - r * cos(tmp_yaw);
      tar_position[i].y = st.yw - r * sin(tmp_yaw);
      tar_position[i].z = st.zw;
      tar_position[i].yaw = tmp_yaw;
    }

    //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用

  } else {
    for (i = 0; i < 4; i++) {
      float tmp_yaw = st.tar_yaw + i * PI / 2.0;
      float r = use_1 ? st.r1 : st.r2;
      tar_position[i].x = st.xw - r * cos(tmp_yaw);
      tar_position[i].y = st.yw - r * sin(tmp_yaw);
      tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
      tar_position[i].yaw = tmp_yaw;
      use_1 = !use_1;
    }
    std::cout << "装甲板0:" << tar_position[0].yaw << std::endl;
    std::cout << "装甲板1:" << tar_position[1].yaw << std::endl;
    std::cout << "装甲板2:" << tar_position[2].yaw << std::endl;
    std::cout << "装甲板3:" << tar_position[3].yaw << std::endl;
    //2种常见决策方案：
    //1.计算枪管到目标装甲板yaw最小的那个装甲板
    //2.计算距离最近的装甲板
    std::vector<float> distans_;
    std::vector<int> num;
    //计算距离最近的装甲板
    for (i = 0; i < 4; i++) {
      float temp_dis_diff =
        sqrt(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y);
      distans_.push_back(temp_dis_diff);
      num.push_back(i);
    }
    std::sort(num.begin(), num.end(), [&](const auto & i, const auto & j) {
      return distans_[i] < distans_[j];
    });
    idx = num[0];
    for (i = 0; i < 4; i++) {
      if (tar_position[num[i]].yaw < 1 && tar_position[num[i]].yaw > -1) {
        idx = num[i];
        break;
      }
    }
  }

  *aim_z = tar_position[idx].z + st.vzw * timeDelay;
  *aim_x = tar_position[idx].x + st.vxw * timeDelay;
  *aim_y = tar_position[idx].y + st.vyw * timeDelay;
  //这里符号给错了
  *pitch = -pitchTrajectoryCompensation(
    sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)) - st.s_bias, *aim_z + st.z_bias, st.current_v);
  //*pitch = -atan(*aim_z/sqrt(((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y))));
  *yaw = (float)(atan2(*aim_y, *aim_x));
  return true;
}

// 从坐标轴正向看向原点，逆时针方向为正

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
  // is_hero = this->declare_parameter("is_hero", false);
  is_hero = declare_parameter<bool>("is_hero", false);
  getParams();
  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0xFF) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivePacket packet = fromVector(data);
        if (packet.checksum == 0xFE) {
          if (!initial_set_param_ || packet.detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", packet.detect_color));
            previous_receive_color_ = packet.detect_color;
          }

          if (packet.reset_tracker) {
            resetTracker();
          }

          geometry_msgs::msg::TransformStamped t;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          t.header.frame_id = "odom";
          t.child_frame_id = "gimbal_link";
          tf2::Quaternion q;
          q.setRPY(
            packet.pitch * 3.1415926 / 180, -packet.roll * 3.1415926 / 180,
            packet.yaw * 3.1415926 / 180);
          t.transform.rotation = tf2::toMsg(q);
          tf_broadcaster_->sendTransform(t);

          solve.st.current_pitch = -packet.roll;
          solve.st.current_yaw = packet.yaw;
          solve.st.current_v = packet.speed;
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    float aim_x = 0, aim_y = 0, aim_z = 0;  // aim point 落点，传回上位机用于可视化
    float pitch = 0;                        //输出控制量 pitch绝对角度 弧度
    float yaw = 0;                          //输出控制量 yaw绝对角度 弧度
    packet.reserved = 1;
    packet.tracking = msg->tracking;
    solve.st.armor_id = solve_angle::ARMOR_ID(id_unit8_map.at(msg->id));
    solve.st.armor_num = solve_angle::ARMOR_NUM(msg->armors_num);

    is_hero = this->get_parameter("is_hero").as_bool();
    if (is_hero) {
      solve.st.bullet_type = solve_angle::BULLET_TYPE::BULLET_42;
      solve.st.k = 0.092;
    } else {
      solve.st.bullet_type = solve_angle::BULLET_TYPE::BULLET_17;
      solve.st.k = 0.038;
    }

    solve.st.xw = msg->position.x;
    solve.st.yw = msg->position.y;
    solve.st.zw = msg->position.z;
    solve.st.vxw = msg->velocity.x;
    solve.st.vyw = msg->velocity.y;
    solve.st.vzw = msg->velocity.z;
    solve.st.tar_yaw = msg->yaw;
    solve.st.v_yaw = msg->v_yaw;
    solve.st.r1 = msg->radius_1;
    solve.st.r2 = msg->radius_2;
    solve.st.dz = msg->dz;
    solve.st.bias_time = 0;
    solve.st.s_bias = 0.13;
    solve.st.z_bias = 0.07;
    std::cout << solve.st.current_v << std::endl;
    packet.checksum = 0xFE;
    packet.fire_control = !msg->is_jump;
    std::cout << "调变了!!!!!!!!!"<< !msg->is_jump << std::endl;
    if (msg->tracking && solve.autoSolveTrajectory(&pitch, &yaw, &aim_x, &aim_y, &aim_z)) {
      if (abs(aim_x) > 0.01) {
        aiming_point_.header.stamp = this->now();
        aiming_point_.pose.position.x = aim_x;
        aiming_point_.pose.position.y = aim_y;
        aiming_point_.pose.position.z = aim_z;
        marker_pub_->publish(aiming_point_);
      }
      if (std::isnan(pitch) || std::isnan(yaw)) {
        RCLCPP_WARN(this->get_logger(), "nnnnnnnnnnnnnnnnnnaaaaaaaaaaaannnnnnnnnnnnan");
        std::cout << "弹道系数" << solve.st.k << std::endl;
        std::cout << "发送pitch:" << pitch << std::endl;
        std::cout << "发送yaw:" << yaw << std::endl;

        std_msgs::msg::Float64 latency;
        latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
        RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
        latency_pub_->publish(latency);
        return;
      }
      std::cout << "弹道系数" << solve.st.k << std::endl;
      std::cout << "发送pitch:" << pitch << std::endl;
      std::cout << "发送yaw:" << yaw << std::endl;
      packet.pitch = pitch;
      packet.yaw = yaw;
      std::vector<uint8_t> data = toVector(packet);

      serial_driver_->port()->send(data);
      std::cout << "发射！" << std::endl;
    }else if(!msg->tracking)
    {
	std::cout << "调变了!!!!!!!!!"<< !msg->is_jump << std::endl;
	std::cout << "is_tracking?" << msg->tracking << std::endl;
        std::vector<uint8_t> data = toVector(packet);
	serial_driver_->port()->send(data);
        std::cout << "发射！" << std::endl;
    }
    std::cout << "is_tracking?" << msg->tracking << std::endl;
    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    std::cout << "耗时" <<latency.data << std::endl;
    latency_pub_->publish(latency);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
