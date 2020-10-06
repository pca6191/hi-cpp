/*
 * agv_pid_controller.h
 *
 *  Created on: 2019年12月07日
 *      Author: kc.chang
 */
#ifndef LIB_AGV_COMMON_INCLUDE_AGV_PID_CONTROLLER_H_
#define LIB_AGV_COMMON_INCLUDE_AGV_PID_CONTROLLER_H_

#include <string>
#include <vector>

namespace agv {
class PIDController {
public:
  PIDController(const double pgain, const double igain, const double dgain, const int win_size);

  ~PIDController();

  void set_error(double err);
  double get_output();

  void set_p_gain(double gain);
  void set_i_gain(double gain);
  void set_d_gain(double gain);

private:
  double p_gain_ = 1.0;
  double i_gain_ = 0.0;
  double d_gain_ = 0.0;

  int win_size_ = 0;
  int last_sample_pos_ = 0;
  std::vector<double> samples_;  // size = win_size
  bool samples_full_ = false;
};

}  // namespace agv
#endif  // LIB_AGV_COMMON_INCLUDE_AGV_PID_CONTROLLER_H_
