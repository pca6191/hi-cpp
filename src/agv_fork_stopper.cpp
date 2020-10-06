/*
 * agv_fork_stopper.cpp
 *
 *  Created on: 2020年4月10日
 *      Author: kc.chang
 */
#include <numeric>
#include <agv_fork_stopper.h>

namespace agv {

ForkStopper::ForkStopper(std::string database_filename)
    : database_filename_(database_filename)
{
  (void)database_filename_;
}

void ForkStopper::set_moving_range(double from_m, double to_m)
{
  moving_from_ = from_m;
  moving_to_ = to_m;
  enalbed_ = true;

  // 初始化速度紀錄
  previous_time_ = 0.0;
  previous_height_ = moving_from_;
  velocity_.push_back(0.0);

  // todo: 其他 initial ?
}

void ForkStopper::update_current_height(double time_sec, double height)
{
  if (!enalbed_) {
    return;  // 避免 velocity_ vector 無限制擴大
  }

  // 計算速度
  double dt = time_sec - previous_time_;  // 加上微小 1ns , 避免除以零
  double dh = height - previous_height_;

  if (dt == 0.0) {
    return;
  }

  // 紀錄速率備用. 上升為正, 下降為負.
  velocity_.push_back(dh/dt);

  // 備份時間位置下一輪使用
  previous_time_ = time_sec;
  previous_height_ = height;
}

bool ForkStopper::stopping_now()
{
  double half_height = (moving_from_ + moving_to_) / 2;

  if (moving_from_ < moving_to_) {  // 如果上升前半段
    if (previous_height_ < half_height) {
      return false;
    }
    // 估算下令停止的高度位置
    double h0 = moving_to_ - predict_skid_distance();

    if (previous_height_ > h0) {
      return true;
    }
  }
  else {  // 如果下降前半段
    if (previous_height_ > half_height) {
      return false;
    }  // else do the following
    // 估算下令停止的高度位置
    double h0 = moving_to_ - predict_skid_distance();

    if (previous_height_ < h0) {
      return true;
    }
  }

  return false;
}

double ForkStopper::evaluate_skid_time()
{
  // todo: 從歷史紀錄擬合
  return 0.1;
}

double ForkStopper::predict_skid_distance()
{
  if (velocity_.size() <= 1) {
    return 0.0;  // 防止下面除以 0
  }

  double dt = evaluate_skid_time();
  double v = std::accumulate(velocity_.begin(), velocity_.end(), 0.0);
  v /= (velocity_.size() - 1);  // 第一筆速度為 dummy 0.0, 故減 1

  return v * dt;
}

void ForkStopper::begin_stop_height(double height)
{
  stop_height_ = height;
  speed_when_stopping_ = std::accumulate(velocity_.begin(), velocity_.end(), 0.0);
  speed_when_stopping_ /= (velocity_.size() - 1);  // 第一筆速度為 dummy 0.0, 故減 1
}

void ForkStopper::final_arrived_height(double height)
{
  arrived_height_ = height;

  // 各式紀錄歸零
  velocity_.clear();
  enalbed_ = false;

  save_skid_time();
}

double ForkStopper::save_skid_time()
{
  double dh = arrived_height_- stop_height_;
  dh = (dh > 0) ? dh : -dh;  // 取速率

  double t = dh / speed_when_stopping_;

  // todo save to database
  return t;
}

}  // namespace agv

