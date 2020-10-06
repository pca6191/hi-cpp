/*
 * agv_digital_io_driver.h
 *
 *  Created on: 2019年12月17日
 *      Author: luther.wu
 */

#ifndef AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_DIGITAL_IO_DRIVER_H_
#define AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_DIGITAL_IO_DRIVER_H_

#include <stdlib.h>
#include <stdio.h>
#include <string>

namespace agv {

typedef unsigned char BYTE;

class DigitalIODriver {
public:
  explicit DigitalIODriver();
  ~DigitalIODriver();

  void fork_stop();
  void fork_up_by_fast_gear();
  void fork_up_by_slow_gear();
  void fork_down();

  bool is_healthy();

  static constexpr const char *class_name_ = "agv_digital_io_driver";

private:
  bool healthy_ = false;  // 紀錄最近一次存取 encoder 是否正常 (正常則本物件 healthy_ == true)
};

}  // namespace agv

#endif  // AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_DIGITAL_IO_DRIVER_H_
