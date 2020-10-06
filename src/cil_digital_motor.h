/*
 * cil_digital_motor.h
 *
 *  Created on: 2020年6月20日
 *      Author: kc.chang
 */

#ifndef CIL_LAYER_INCLUDE_CIL_DIGITAL_MOTOR_H_
#define CIL_LAYER_INCLUDE_CIL_DIGITAL_MOTOR_H_
#include <sys/time.h>

#include <stdint.h>

/// @file 以 RC 電路，及其時間常數控制電容電壓升降的特性，
///       模擬馬達響應：
///          Vin --- R ---+-----Vc
///                       |
///                       C
///                       |
///                      GND
///       - Vin 表示設入馬達電壓，單位 mv
///       - Vc  表示轉速，放大縮小一定比例，可代表轉速
namespace cil {

class DigitalMotor {
public:
  /// time_const_: 仿 RC 電路充電常數, 單位 cycle 數
  /// Vo = Vin(1- e^-t/RC), t 為時間常數 (單位 cycle), Vo 為電容輸出電壓
  ///
  /// 假設 Vo:1V 代表 100 rpm, 則 vo_to_out_ratio 要設為 100.0 (rpm/v);
  /// 假設 Vo:1V 代表 10 deg,  則 vo_to_out_ratio 要設為 10.0 (deg/v);
  DigitalMotor(const double time_const_, const double vo_to_out_ratio);

  /// 設定馬達的輸入電壓指令 (單位 V)，後續 Vo 要追這個電壓
  void set_voltage(double v);

  /// 讀取馬達作動後，搭配的編碼器輸出的物理量，單位依照 vo_to_out_ratio_ 規範
  /// ex. rpm or deg or ...
  double get_output();

  /// 每呼叫一次，cycle 往前計數一次
  void update();

private:
  // Vc = Vc-start + (Vcmd-Vc-start)(1 - exp(-t/time_const)) , when (Vcmd-Vc-start) > 0, 充電
  // Vc = Vcmd + (Vcmd-Vc-start)(1 - exp(-t/time_const)) , when (Vcmd-Vc-start) < 0, 放電
  double v_cmd_ = 0.0;  // 輸入電壓 (單位 V)
  double vc_start_ = 0.0;  // 輸入電壓有變化當下的輸出電壓 Vc (單位 V)
  double vc_ = 0.0;

  /// 紀錄電壓變化當下時間點 (cycle 點)
  int cycle_ = 0;

  const double time_constant_;
  const double vo_to_out_ratio_;
};

}  // namespace cil


#endif /* CIL_LAYER_INCLUDE_CIL_DIGITAL_MOTOR_H_ */
