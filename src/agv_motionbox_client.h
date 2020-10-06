/*
 * agv_motionbox_client.h
 *
 *  Created on: 2020年6月14日
 *      Author: kc.chang
 */

#ifndef AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_MOTIONBOX_CLIENT_H_
#define AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_MOTIONBOX_CLIENT_H_
#include <memory>
#include <vector>
#include <string>

#include <serial/serial.h>
#include <agv_sick_encoder_driver.h>
#include <agv_digital_io_driver.h>

namespace agv {
class MotionBoxClient {
public:
  using uptr = std::unique_ptr<MotionBoxClient>;

  static uptr create_instance(const std::string &port, const int baudrate,
      const int read_timeout_ms, const double wheel_radius, SickEncoderDriver *sick_encoder,
      DigitalIODriver *fork_driver)
  {
    return std::make_unique<MotionBoxClient>(port, baudrate, read_timeout_ms, wheel_radius,
        sick_encoder, fork_driver);
  }

  MotionBoxClient(const std::string &port, const int baudrate, const int read_timeout_ms,
      const double wheel_radius, SickEncoderDriver *sick_encoder, DigitalIODriver *fork_driver);
  ~MotionBoxClient();

  /// 啟動本物件
  bool start();
  /// 停止本物件
  bool stop();
  /// 切到自動車模式
  bool switch_to_agv();
  /// 切到手控模式
  bool switch_to_manual();
  /// 設定目標輪子角度 (單位: rad/s)
  bool set_trac_angvel(double angvel);
  /// 取得目前輪子角速度 (單位: rad/s)
  bool get_trac_angvel(double *angvel);
  /// 設定目標輪子擺角 (單位: rad)
  bool set_steer_rad(double rad);
  /// 取得目前輪子擺角 (單位: rad)
  bool get_steer_rad(double *rad);
  /// 設定目標貨叉升降速度 (單位： m/s)
  bool set_fork_velocity(double vel);
  /// 取得目前貨叉高度 (單位: m)
  bool get_fork_position(double *pos);
  /// 將本物件內存的速度、高度、操作模式..配成封包送出
  bool send_packet();

  /// 測試用: -90 ~ 90 度間逐步調整，紀錄對應的 encoder 角度
  /// 要搭配特殊 motion box FW
  void scan_degree();

private:
  enum class Action {
    INIT = 0,
    FORWARD = 1,
    BACKWARD = 2,
    STOP = 3,
    MANUAL = 4
  };

  struct InnovatiCmd {
    uint8_t id;
    uint8_t fc;
    uint8_t cmd_data_len;
    uint8_t fbb_lo;
    uint8_t fbb_hi;
    uint8_t trac_lo;
    uint8_t trac_hi;
    uint8_t steer_lo;
    uint8_t steer_hi;
    uint8_t tar_trac_lo;
    uint8_t tar_trac_hi;
    uint8_t tar_rotat_lo;
    uint8_t tar_rotat_hi;
    uint8_t crc_lo;
    uint8_t crc_hi;
    uint8_t cmd_end;
  };

  double rads_to_rpm(const double rads);  // rad/s 轉 rpm
  double rpm_to_rads(const double rpm);  // rpm 轉 rads

  uint8_t get_low_byte(double value);
  uint8_t get_high_byte(double value);
  std::vector<uint8_t> pack_packet(Action fbb, double curr_trac, double curr_steer, double target_trac,
      double target_steer);

  const std::string class_name_ = "MotionBoxClient";

  Action action_ = Action::STOP;
  double target_trac_angvel_ = 0.0;
  double target_steer_rad_ = 0.0;
  double target_fork_vel_ = 0.0;

  serial::Serial serial_;
  const std::string port_ = "";
  const uint32_t baudrate_;
  const int timeout_;

  const double wheel_radius_;

  const uint8_t default_id_ = 0x01;
  const uint8_t default_fc_ = 0x01;
  const uint8_t default_len_ = 0x05;
  const uint8_t default_cmd_end_ = 0x0A;
  const uint8_t zero_byte_ = 0x00;

  // encoder 轉 18.636 圈，相當於舵輪 trac 轉 1 圈
  const double gear_ratio_ = 18.636;

  SickEncoderDriver *sick_encoder_;
  DigitalIODriver *fork_driver_;
};

}  // namespace agv



#endif /* AGV_CONTROLS_AGV_NOVEL_CONTROL_INCLUDE_AGV_MOTIONBOX_CLIENT_H_ */
