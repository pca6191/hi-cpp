/*
 * agv_fork_stopper.h
 *
 *  Created on: 2020年4月10日
 *      Author: kc.chang
 */

#ifndef AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_FORK_STOPPER_H_
#define AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_FORK_STOPPER_H_
#include <string>
#include <vector>

namespace agv {

class ForkStopper {
public:
  /// database_filename: 紀錄歷史滑行時間的檔案名稱
  explicit ForkStopper(std::string database_filename);

  /// 設定要移動範圍 (單位 m)
  void set_moving_range(double from_m, double to_m);

  /// 設定目前 (時刻、高度) 用來估算時間
  void update_current_height(double time_sec, double height);

  /// 傳出目前是否在下令停止-減速滑行 過程中
  bool stopping_now();

  /// 下停止命令的高度點
  void begin_stop_height(double height);

  /// 設定抵達高度
  void final_arrived_height(double height);

private:
  /// 取得預估的滑行距離
  double predict_skid_distance();

  /// 從歷史紀錄估算滑行時間
  double evaluate_skid_time();

  /// 除存該次運動真正的滑行時間
  double save_skid_time();

  const std::string database_filename_;
  const double tolorance_ = 0.005;
  bool enalbed_ = false;
  double moving_from_ = 0.0;
  double moving_to_ = 0.0;
  std::vector<double> velocity_;
  double previous_time_ = 0.0;
  double stop_height_ = 0.0;
  double previous_height_ = 0.0;
  double arrived_height_ = 0.0;
  double speed_when_stopping_ = 0;  // 下令停止當下的速率
};

}  // namespace agv

#endif /* AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_FORK_STOPPER_H_ */
