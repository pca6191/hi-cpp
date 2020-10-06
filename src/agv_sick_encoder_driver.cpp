/*
 * agv_steering_encoder_driver.cpp
 *
 *  Created on: 2019年12月16日
 *      Author: kerwin.lo
 */
#include <agv_sick_encoder_driver.h>

namespace agv {

SickEncoderDriver::SickEncoderDriver(const std::string &can_port)
{
  (void)can_port;
}

SickEncoderDriver::~SickEncoderDriver()
{
}

bool SickEncoderDriver::is_healthy()
{
  return healthy_;
}

bool SickEncoderDriver::start(const std::string &can_port)
{
  healthy_ = true;

  return healthy_;
}

double SickEncoderDriver::get_steering_position()
{
  double angle = 0.0;
  return angle;
}

double SickEncoderDriver::get_fork_position()
{
  double position = 0.0;
  return position;
}

int SickEncoderDriver::get_wheel_rpm()
{
    return 0;
}

double SickEncoderDriver::get_wheel_velocity()
{
  return 0.0;
}
}  // namespace agv
