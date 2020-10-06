/*
 * agv_motionbox_client.cpp
 *
 *  Created on: 2020年6月14日
 *      Author: kcchang
 */
#include <vector>
#include <stdio.h>
#include <util_crc.h>
#include <ros/ros.h>

#include <agv_motionbox_client.h>

#define hi_dbg 1

namespace agv {

MotionBoxClient::MotionBoxClient(const std::string &port, const int baudrate,
    const int read_timeout_ms, const double wheel_radius, SickEncoderDriver *sick_encoder,
    DigitalIODriver *fork_driver)
    : port_(port), baudrate_(baudrate), timeout_(read_timeout_ms), wheel_radius_(wheel_radius)
{
  sick_encoder_ = sick_encoder;
  fork_driver_ = fork_driver;
}

MotionBoxClient::~MotionBoxClient()
{
}

bool MotionBoxClient::start()
{
  if (serial_.isOpen()) {
    serial_.close();
  }

  try {
    serial_.setPort(port_);
    serial_.setBaudrate(baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
    serial_.setTimeout(to);
    serial_.open();
  }
  catch (serial::IOException& e) {
#if hi_dbg
    std::cout << "[" << class_name_ << "] Unable to open port " << port_;
#else
    ROS_ERROR_STREAM("[" << class_name_ << "] Unable to open port " << port_);
#endif
    return false;
  }

  return true;
}

bool MotionBoxClient::stop()
{
  // todo: 停車、進入手動模式
  action_ = Action::MANUAL;
  // 發送封包
  send_packet();

  serial_.close();

  return true;
}

bool MotionBoxClient::switch_to_agv()
{
  action_ = Action::STOP;
  return true;
}

bool MotionBoxClient::switch_to_manual()
{
  action_ = Action::MANUAL;
  return true;
}

bool MotionBoxClient::set_trac_angvel(double angvel)
{
  target_trac_angvel_ = angvel;
  return true;
}

bool MotionBoxClient::get_trac_angvel(double *angvel)
{
  // todo : 明確方向
  int rpm = sick_encoder_->get_wheel_rpm() / gear_ratio_;  // 舵輪 trac rpm
  *angvel = rpm_to_rads(rpm);

  return true;
}

bool MotionBoxClient::set_steer_rad(double rad)
{
  target_steer_rad_ = rad;
  return true;
}

bool MotionBoxClient::get_steer_rad(double *rad)
{
  // todo : 明確方向
  *rad = sick_encoder_->get_steering_position();
  return true;
}

bool MotionBoxClient::set_fork_velocity(double vel)
{
  target_fork_vel_ = vel;
  return true;
}

bool MotionBoxClient::get_fork_position(double *pos)
{
  *pos = sick_encoder_->get_fork_position() / 1000.0;  // unit m
  return true;
}

bool MotionBoxClient::send_packet()
{
  try {
    double cur_trac_angvel;
    double cur_steer_rad;

    if (!get_trac_angvel(&cur_trac_angvel)) {
#if hi_dbg
#else
      ROS_ERROR_STREAM_THROTTLE(1,
          "[" << class_name_ << "] send_packet()::get_trac_angvel() failed !");
#endif
      return false;  // 不送封包，數秒後，讓 motion box 自動 timeout 停止
    }

    if (!get_steer_rad(&cur_steer_rad)) {
#if hi_dbg
#else
      ROS_ERROR_STREAM_THROTTLE(1,
          "[" << class_name_ << "] send_packet()::get_steer_rad() failed !");
#endif
      return false;  // 不送封包，數秒後，讓 motion box 自動 timeout 停止
    }

    std::vector<uint8_t> pkt = pack_packet(action_, cur_trac_angvel, cur_steer_rad,
        target_trac_angvel_, target_steer_rad_);
    size_t success = serial_.write(pkt);

    if (success < pkt.size()) {
#if hi_dbg
#else
      ROS_ERROR_STREAM_THROTTLE(1,
          "[" << class_name_ << "] send_packet()::serial_.write() failed !");
#endif
    }
  }
  catch (std::exception &err) {
#if hi_dbg
#else
    ROS_ERROR_STREAM("[" << class_name_ << "] send_packet() failed!");
    ROS_ERROR_STREAM("[" << class_name_ << "]" << err.what());
#endif
    return false;
  }

  return true;
}

double MotionBoxClient::rads_to_rpm(const double rads)
{
  return rads * (30 / M_PI);
}

double MotionBoxClient::rpm_to_rads(const double rpm)
{
  return rpm * (M_PI / 30);
}

uint8_t MotionBoxClient::get_low_byte(double value)
{
  uint16_t value_byte = static_cast<uint16_t>(value);
  return static_cast<uint8_t>(value_byte & 0x00FF);
}

uint8_t MotionBoxClient::get_high_byte(double value)
{
  uint16_t value_byte = static_cast<uint16_t>(value);
  return static_cast<uint8_t>((value_byte & 0xFF00) >> 8);
}

std::vector<uint8_t> MotionBoxClient::pack_packet(Action fbb, double curr_trac, double curr_rotate,
    double target_trac, double target_steer)
{
  std::vector<uint8_t> cmd_vector;

  InnovatiCmd cmd;
  cmd.id = default_id_;
  cmd.fc = default_fc_;
  cmd.cmd_data_len = default_len_;
  cmd.fbb_lo = static_cast<uint8_t>(fbb);
  cmd.fbb_hi = zero_byte_;
  cmd.trac_lo = get_low_byte(curr_trac);
  cmd.trac_hi = get_high_byte(curr_trac);
  cmd.steer_lo = get_low_byte(curr_rotate);
  cmd.steer_hi = get_high_byte(curr_rotate);
  cmd.tar_trac_lo = get_low_byte(target_trac);
  cmd.tar_trac_hi = get_high_byte(target_trac);
  cmd.tar_rotat_lo = get_low_byte(target_steer);
  cmd.tar_rotat_hi = get_high_byte(target_steer);

  cmd_vector.push_back(cmd.id);
  cmd_vector.push_back(cmd.fc);
  cmd_vector.push_back(cmd.cmd_data_len);
  cmd_vector.push_back(cmd.fbb_lo);
  cmd_vector.push_back(cmd.fbb_hi);
  cmd_vector.push_back(cmd.trac_lo);
  cmd_vector.push_back(cmd.trac_hi);
  cmd_vector.push_back(cmd.steer_lo);
  cmd_vector.push_back(cmd.steer_hi);
  cmd_vector.push_back(cmd.tar_trac_lo);
  cmd_vector.push_back(cmd.tar_trac_hi);
  cmd_vector.push_back(cmd.tar_rotat_lo);
  cmd_vector.push_back(cmd.tar_rotat_hi);

  uint16_t crc = util_crc_calc(cmd_vector.data(), static_cast<uint16_t>(cmd_vector.size()));

  cmd.crc_lo = crc & 0x00FF;
  cmd.crc_hi = (crc & 0xFF00) >> 8;
  cmd.cmd_end = default_cmd_end_;

  cmd_vector.push_back(cmd.crc_lo);
  cmd_vector.push_back(cmd.crc_hi);
  cmd_vector.push_back(cmd.cmd_end);

  return cmd_vector;
}

/// 測試用
void MotionBoxClient::scan_degree()
{
  const double idle_trac_code = 4000;
  std::vector<double> codes;
  std::vector<double> degs;

  // 給 encoder 0 ~ 4000 ~ 8000, 取對應角度值
  for (double steer_code = 0; steer_code < 8000; steer_code += 200) {
    std::vector<uint8_t> pkt = pack_packet(MotionBoxClient::Action::FORWARD, idle_trac_code, steer_code, idle_trac_code,
        steer_code);
    serial_.write(pkt);

    // 停 2 sec
#if hi_dbg
#else
    ros::Duration period(2);
    period.sleep();
#endif

    // 取出 encoder 角度
    double rad, deg;
    get_steer_rad(&rad);
    deg = 180.0 * rad / M_PI;

    // 收集數據
    codes.push_back(steer_code);
    degs.push_back(deg);
  }

  // 印出存檔
  FILE *fp = fopen("/tmp/deg_scan.txt", "w");
  for (unsigned int i = 0; i < codes.size(); i++) {
    fprintf(fp, "%.0lf, %.0lf\n", codes.at(i), degs.at(i));
#if hi_dbg
#else
    ROS_INFO("%.0lf, %.0lf\n", codes.at(i), degs.at(i));
#endif
  }
  fclose(fp);

#if hi_dbg
  printf("Finished..!!");
#else
  ROS_INFO("Finished..!!");
#endif
}

}  // namespace agv
