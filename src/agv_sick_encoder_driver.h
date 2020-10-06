/*
 * agv_steering_encoder_driver.h
 *
 *  Created on: 2019年12月16日
 *      Author: kerwin.lo
 */

#ifndef AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_SICK_ENCODER_DRIVER_H_
#define AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_SICK_ENCODER_DRIVER_H_

#include <string>

namespace agv {

class SickEncoderDriver {
public:
  explicit SickEncoderDriver(const std::string &can_port);
  ~SickEncoderDriver();

  bool start(const std::string &can_port);
  double get_steering_position();
  double get_fork_position();
  double get_wheel_velocity();
  int get_wheel_rpm();
  bool is_healthy();

private:
  bool healthy_ = false;  // 紀錄最近一次存取 encoder 是否正常 (正常則本物件 healthy_ == true)
};
}  // namespace agv

#endif  // AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_SICK_ENCODER_DRIVER_H_
