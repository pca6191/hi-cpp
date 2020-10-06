/*
 * cil_digital_motor.cpp
 *
 *  Created on: 2020年5月30日
 *      Author: kc.chang
 */
#include <cil_digital_motor.h>
#include <chrono>
#include <math.h>
#include <stdio.h>


namespace cil {

DigitalMotor::DigitalMotor(const double time_const, const double vo_to_out_ratio)
    : time_constant_(time_const), vo_to_out_ratio_(vo_to_out_ratio)
{
}

void DigitalMotor::set_voltage(double v)
{
  if (v == v_cmd_) {
    return;  // 電壓沒變化，抱持現況充放電
  }

  v_cmd_ = v;
  vc_start_ = vc_;  // 以現況重新指定電容初始電壓

  cycle_ = 0;

  // printf("v_cmd = %lf, vc_start = %lf\n", v_cmd_, vc_start_);  // KC_DBG
}

void DigitalMotor::update()
{
  cycle_++;
}

double DigitalMotor::get_output()
{
  double t = cycle_;
  double rc = time_constant_;
  // 處理充電情況
  if (v_cmd_ > vc_start_) {
    vc_ = vc_start_ + (v_cmd_ - vc_start_) * (1.0 - exp(-t / rc));
  }
  // 處理放電情況
  else {
    vc_ = v_cmd_ + (vc_start_ - v_cmd_) * exp(-t / rc);
  }
  // Vc 轉成輸出物理量
  double out = vc_ * vo_to_out_ratio_;

//  printf("v_cmd_(%lf), vc_start_(%lf), t(%lf), rc(%lf)\n", v_cmd_, vc_start_, t, rc);
//  printf("out(%lf) = vc_(%lf)*vo_to_out_ratio(%lf)\n", out, vc_, vo_to_out_ratio_);

  return out;
}

}  // namespace cil


