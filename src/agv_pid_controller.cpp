/*
 * agv_pid_controller.h
 *
 *  Created on: 2019年12月07日
 *      Author: kc.chang
 */

#include <agv_pid_controller.h>

namespace agv {
PIDController::PIDController(const double pgain, const double igain, const double dgain, const int win_size)
  : samples_(win_size, 0.0)
{
  p_gain_ = pgain;
  i_gain_ = igain;
  d_gain_ = dgain;

  win_size_ = win_size;
  samples_full_ = false;
}

PIDController::~PIDController()
{
}

void PIDController::set_error(double err)
{
  if (!samples_full_) {
    samples_[last_sample_pos_] = err;
    last_sample_pos_++;  // 指向待放置處
    if (last_sample_pos_ == win_size_) {  // buffer 剛好滿
      samples_full_ = true;
    }

    return;
  }

  // buffer 已經滿, 先將 samples 往 [0] 挪動
  for (int i = 1; i < win_size_; i++) {
    samples_[i - 1] = samples_[i];
  }

  samples_[last_sample_pos_ - 1] = err;
}

double PIDController::get_output()
{
  double p_err = 0.0;
  double i_err = 0.0;
  double d_err = 0.0;

  if (last_sample_pos_ == 0) {  // 沒東西
    return 0.0;
  }

  // count p error
  p_err = samples_[last_sample_pos_ - 1] * p_gain_;

  // count i error
  for (int i = 0; i < last_sample_pos_; i++) {
    i_err += samples_[i];
  }
  i_err *= i_gain_;

  // count d error
  if (last_sample_pos_ > 1) {
    d_err = samples_[last_sample_pos_ - 1] - samples_[last_sample_pos_ - 2];
    d_err *= d_gain_;
  }

  return (p_err + i_err + d_err);
}

void PIDController::set_p_gain(double gain)
{
  p_gain_ = gain;
}

void PIDController::set_i_gain(double gain)
{
  i_gain_ = gain;
}

void PIDController::set_d_gain(double gain)
{
  d_gain_ = gain;
}

}  // namespace agv
