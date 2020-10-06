
//============================================================================
// Name        : hi-cpp.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#if 1
#include <string>
#include <iostream>

#include "eeprom.h"

using namespace std;

int main()
{
  cout << "EEPROM Emulation ...!" << endl;

  EE_FormatPage(0);
  EE_Write(0x00, 0xAA);

  return 0;
}

#endif

#if 0
#include <string>
#include <iostream>
#include <vector>
#include <util_crc.h>

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
  uint8_t rotat_lo;
  uint8_t rotat_hi;
  uint8_t tar_trac_lo;
  uint8_t tar_trac_hi;
  uint8_t tar_rotat_lo;
  uint8_t tar_rotat_hi;
  uint8_t crc_lo;
  uint8_t crc_hi;
  uint8_t cmd_end;
};

uint8_t default_id_ = 0x01;
uint8_t default_fc_ = 0x01;
uint8_t default_len_ = 0x05;
uint8_t default_cmd_end_ = 0x0A;
uint8_t zero_byte_ = 0x00;

uint8_t get_low_byte(double value)
{
  uint16_t value_byte = static_cast<uint16_t>(value);
  return static_cast<uint8_t>(value_byte & 0x00FF);
}

uint8_t get_high_byte(double value)
{
  uint16_t value_byte = static_cast<uint16_t>(value);
  return static_cast<uint8_t>((value_byte & 0xFF00) >> 8);
}

std::vector<uint8_t> pack_cmd_pkg(Action fbb, double curr_trac, double curr_rotate,
    double target_trac, double target_rotate)
{
  std::vector<uint8_t> cmd_vector;

  InnovatiCmd cmd;
  cmd.id = default_id_;
  cmd.fc = default_fc_;
  cmd.cmd_data_len = default_len_;
  cmd.fbb_lo = static_cast<uint8_t>(fbb);
  cmd.fbb_hi = zero_byte_;
  cmd.trac_lo = get_low_byte(curr_trac);
  cmd.trac_hi = get_high_byte(curr_trac);
  cmd.rotat_lo = get_low_byte(curr_rotate);
  cmd.rotat_hi = get_high_byte(curr_rotate);
  cmd.tar_trac_lo = get_low_byte(target_trac);
  cmd.tar_trac_hi = get_high_byte(target_trac);
  cmd.tar_rotat_lo = get_low_byte(target_rotate);
  cmd.tar_rotat_hi = get_high_byte(target_rotate);

  cmd_vector.push_back(cmd.id);
  cmd_vector.push_back(cmd.fc);
  cmd_vector.push_back(cmd.cmd_data_len);
  cmd_vector.push_back(cmd.fbb_lo);
  cmd_vector.push_back(cmd.fbb_hi);
  cmd_vector.push_back(cmd.trac_lo);
  cmd_vector.push_back(cmd.trac_hi);
  cmd_vector.push_back(cmd.rotat_lo);
  cmd_vector.push_back(cmd.rotat_hi);
  cmd_vector.push_back(cmd.tar_trac_lo);
  cmd_vector.push_back(cmd.tar_trac_hi);
  cmd_vector.push_back(cmd.tar_rotat_lo);
  cmd_vector.push_back(cmd.tar_rotat_hi);

  uint16_t crc = agv::util_crc_calc(cmd_vector.data(), static_cast<uint16_t>(cmd_vector.size()));

  cmd.crc_lo = crc & 0x00FF;
  cmd.crc_hi = (crc & 0xFF00) >> 8;
  cmd.cmd_end = default_cmd_end_;

  cmd_vector.push_back(cmd.crc_lo);
  cmd_vector.push_back(cmd.crc_hi);
  cmd_vector.push_back(cmd.cmd_end);

  return cmd_vector;
}

int main()
{
  // "resetmcu H" packet
  default_fc_ = 0x02;
  std::vector<uint8_t> cmd = pack_cmd_pkg(Action::STOP, 0.0, 0.0, 0.0, 0.0);

  for (size_t i = 0; i < cmd.size(); i++) {
  // printf("0x%02X, ", cmd[i]);
  // printf("%02X ", cmd[i]);
    printf("\\x%02X", cmd[i]);
  }

  return 0;
}

#endif

#if 0
#include <string>
#include <iostream>

int min(int &x, int &y)
{
  return x = (x < y) ? x : y;
}

int main()
{
  int a = 9, b = 1;

  std::cout  << "min(a, b) = " << min(a, b) << std::endl;
  std::cout  << "a= " << a << std::endl;

  return 0;
}

#endif

#if 0
#include <string>
#include <iostream>
int get_distance(std::string line)
{
  // line format: "ms:1234,mm:123\n"
  std::string mm = "";
  std::size_t p;
  std::size_t q;

  p = line.find("mm:");
  if (p == std::string::npos) {
    return 255;  // 無窮遠
  }
  p += 3;  // pos after ':'

  q = line.find("\n");
  if (q == std::string::npos) {
    return 255;  // 字串不全
  }

  mm = line.substr(p, q - p);

  return std::stoi(mm);
}

int main()
{
  // std::cout <<  get_distance("ms:123,mm:456\n");
  std::cout <<  get_distance("ms:123,mm:456");
  return 0;
}
#endif

#if 0
#include <stdio.h>

int script_param_[4] = {0, 0, 0, 80};
double last_steering_angle_ = 0.0;

void write_fork()
{
  // 封包內角度方向：左手定則. 輸入 DAC code: 0 ~ 8191, 遞增 80 (約 2 度)
  // ROS_INFO(">>>> Start steer test ...");
  const int period = 70;  // 單位 1/50Hz
  static int steer_code = 0;  // 要帶入封包的值
  int d_code = script_param_[3];  // 80;  // 遞增 ex. 80 (約 2 度)
  static int cycle = period;  // =period: 一開始就進入 state machine

  double curr_rotate = -last_steering_angle_ * 100;  // 左手定則，封包座標

  if (cycle == period) {  // delay 週期到了，才調整設定值
    static int state = 0;
    cycle = 0;

    switch (state) {
      case 0:  // 就定位, FW 啟動 DAC = 0, 分梯次每 30 度遞增到初始位置，避免 zapi 鎖住
        static int start_code = script_param_[1];  // user 輸入的起始值
        static int init_code = 0;
        init_code += 1000;  // 將近 30 度
        if (init_code > start_code) {
          steer_code = start_code;
          state = 1;
          printf("reset steer finished !!\n");
        }
        else {
          steer_code = init_code;
        }
        break;

      case 1:  // 遞增、遞減調角度
        steer_code += d_code;
        break;

      case 2:  // 收尾
        break;
    }

    // 限制範圍
    steer_code = (steer_code > 12000) ? 12000: steer_code;
    steer_code = (steer_code < 0) ? 0: steer_code;

    printf("code,curr_cdeg,%d,%.0lf\n", steer_code, curr_rotate);
  }



  printf("    enc_steer = %lf\n", steer_code / 100.0);  // unit: 無，純作為 enc_steer 變化的參考基準

//  std::vector < uint8_t > cmd_vector = pack_cmd_pkg(Action::FORWARD, 4600,  // 4600, 速度中位，不會轉  // curr_trac
//      0,  // dont care, curr_rotate
//      4600,  // 4600, 速度中位，不會轉  // target_trac
//      script_param_[1]);  // target_rotate
//
//  send_package (cmd_vector);
//
  cycle++;
}

void write_fork_jump()
{
  // 封包內角度方向：左手定則. 輸入 DAC code: 0 ~ 8191, 遞增 80 (約 2 度)
  // ROS_INFO(">>>> Start steer test ...");
  const int period = 70 * 3;  // 單位 1/50Hz
  static int steer_code = 0;  // 要帶入封包的值
  static int cycle = period;  // =period: 一開始就進入 state machine

  double curr_rotate = -last_steering_angle_ * 100;  // 左手定則，封包座標

  if (cycle == period) {  // delay 週期到了，才調整設定值
    static int state = 0;
    cycle = 0;

    switch (state) {
      case 0:  // 就定位, FW 啟動 DAC = 0, 分梯次每 30 度遞增到初始位置，避免 zapi 鎖住
        static int start_code = script_param_[1];  // user 輸入的起始值
        static int init_code = 0;
        init_code += 1000;  // 將近 30 度
        if (init_code > start_code) {
          steer_code = start_code;
          state = 1;
          printf("reset steer finished !!\n");
        }
        else {
          steer_code = init_code;
        }
        break;

      case 1:  // 遞增、遞減調角度
        static int d_code = script_param_[3];  // 80;  // 遞增 ex. 80 (約 2 度)
        static int count = 1;
        steer_code += d_code * count;
        count++;
        state = 2;
        break;

      case 2:  // 回到 start code
        steer_code = start_code;
        state = 1;
        break;
    }

    // 限制範圍
    steer_code = (steer_code > 12000) ? 12000: steer_code;
    steer_code = (steer_code < 0) ? 0: steer_code;

    printf("code,curr_cdeg,%d,%.0lf\n", steer_code, curr_rotate);
  }



 // printf("    enc_steer = %lf\n", steer_code / 100.0);  // unit: 無，純作為 enc_steer 變化的參考基準

//  std::vector < uint8_t > cmd_vector = pack_cmd_pkg(Action::FORWARD, 4600,  // 4600, 速度中位，不會轉  // curr_trac
//      0,  // dont care, curr_rotate
//      4600,  // 4600, 速度中位，不會轉  // target_trac
//      script_param_[1]);  // target_rotate
//
//  send_package (cmd_vector);
//
  cycle++;
}

int main()
{
   for (int i = 0; i < 10000; i++) {
     last_steering_angle_ = i;
     // write_fork();
     write_fork_jump();
   }

   printf("the end !!\n");

   return 0;
}
#endif

#if 0
#include <stdio.h>
#include <time.h>

int main()
{
  char buf[80];
  time_t time_seconds = time(0);
  struct tm now_time;
  localtime_r(&time_seconds, &now_time);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &now_time);

  return 0;
}
#endif

#if 0
#include <vector>
#include <algorithm>
#include <stdio.h>

#define ROS_INFO printf

bool start()
{
  std::vector<double> diff_up;
  std::vector<double> diff_down;
  double fork_allow_tolerance_ = 1.0;

  const char *class_name_ = "hi";

  for (double i = 1.0; i <= 5.0; i += 1.0) {
    diff_up.push_back(i / 10);
    diff_down.push_back(-i / 10);
  }

  // 去頭去尾 (頭尾可能是 boundary 位置，沒有滑動效應)
  if (diff_up.size() > 0) {
    diff_up.erase(diff_up.begin());
  }

  if (diff_down.size() > 0) {
    diff_down.erase(diff_down.end() - 1);
  }

  // 收集位置
  for (auto d : diff_up) {
    ROS_INFO("[%s] start(): diff_up %lf", class_name_, d);
  }

  for (auto d : diff_down) {
    ROS_INFO("[%s] start(): diff_down %lf", class_name_, d);
  }

  // 平均後，取滑動距離
  double avg_up = 0.0;
  double avg_dn = 0.0;

  if (diff_up.empty() || diff_down.empty()) {
    ROS_ERROR("[%s] start(): failed, up or down targets can't less than one.", class_name_);
  }
  avg_up = std::accumulate(diff_up.begin(), diff_up.end(), 0.0) / diff_up.size();
  avg_dn = std::accumulate(diff_down.begin(), diff_down.end(), 0.0) / diff_down.size();

  ROS_INFO("[%s] start(): average up/down (m): %lf/%lf", class_name_, avg_up, avg_dn);

  double slide_up = fork_allow_tolerance_ + avg_up;  // ex. tolerance 1.0 cm, avg_up 0.2 cm ==> 滑動 1.2 cm
  double slide_dn = fork_allow_tolerance_ + avg_dn;
  ROS_INFO("[%s] start(): slide up/down (m): %lf/%lf", class_name_, slide_up, slide_dn);

  // 寫入 yaml file
  return true;
}

int main()
{
  start();
  return 0;
}
#endif

#if 0
#include <string>
#include <iostream>
#include <stdio.h>
void angle_inc(const double &res)
{
  const double min = -90.0;
  const double max = 90.0;

  for (double ang = min; ang <= max; ang += res) {
    printf("{(0.0, 0.0, 0.0), (0.0, %.1lf, 0.0), 2.0}\n", ang);
  }

  printf("{(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 2.0}\n");
}

void angle_dec(const double &res)
{
  const double min = -90.0;
  const double max = 90.0;

  for (double ang = max; ang >= min; ang -= res) {
    printf("{(0.0, 0.0, 0.0), (0.0, %.1lf, 0.0), 2.0}\n", ang);
  }

  printf("{(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 2.0}\n");
}

void angle_jump_inc(const double &res)
{
#if 0
  const double min = -80.0;
  const double max = 80.0;
#else
  const double min = 0.0;
  const double max = 80.0;
#endif

  // 先打到底
  printf("{(0.0, 0.0, 0.0), (0.0, %.1lf, 0.0), 3.0}\n", min);

  // 變化遞增
  for (double d_ang = res ; (min + d_ang) <= 90.0; d_ang += res) {
    printf("{(0.0, 0.0, 0.0), (0.0, %.1lf, 0.0), 3.0}\n", (min + d_ang));
    printf("{(0.0, 0.0, 0.0), (0.0, %.1lf, 0.0), 3.0}\n", min);
  }

  printf("{(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 3.0}\n");
}

void angle_jump_dec(const double &res)
{
#if 0
  const double min = -80.0;
  const double max = 80.0;
#else
  const double min = -80.0;
  const double max = 0.0;
#endif

  // 先打到頂
  printf("{(0.0, 0.0, 0.0), (0.0, %.1lf, 0.0), 3.0}\n", max);

  // 變化遞增
  for (double d_ang = res ; (max - d_ang) >= -90.0; d_ang += res) {
    printf("{(0.0, 0.0, 0.0), (0.0, %.1lf, 0.0), 3.0}\n", (max - d_ang));
    printf("{(0.0, 0.0, 0.0), (0.0, %.1lf, 0.0), 3.0}\n", max);
  }

  printf("{(0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 3.0}\n");
}

int main()
{
  // angle_inc(3.0);
  // angle_dec(3.0);
  // angle_inc(15.0);
  // angle_dec(15.0);
  angle_jump_inc(15);
  printf("\n\n");
  angle_jump_dec(15);

  return 0;
}
#endif

#if 0
#include <string>
#include <vector>
#include <iostream>

std::vector<std::string> split_string(const std::string& s, char delimiter)
{
  std::vector<std::string> out;

  std::string::size_type stop = 0;
  while (stop != std::string::npos) {
    const std::string::size_type start = s.find_first_not_of(delimiter, stop);
    if (start == std::string::npos)
      break;
    stop = s.find_first_of(delimiter, start + 1);
    out.emplace_back(s.substr(start, stop == std::string::npos ? std::string::npos : (stop - start)));
  }

  return out;
}

int main()
{
  // 解出 linear, angular 數值, 字串如：
  std::string line = "{(0.2, 0.2, 0.0),(0.0, 0.05, 0.05), 1.0}";
  std::string csv = "";
  std::size_t p;
  std::size_t q;

  p = line.find("(");
  q = line.find(")");
  csv = line.substr(p + 1, q - p - 1);

  p = line.find_first_of("(", q + 1);
  q = line.find_first_of(")", q + 1);
  csv = csv + "," + line.substr(p + 1, q - p - 1);

  p = line.find(",", q + 1);
  q = line.find("}", q + 1);
  csv = csv + "," + line.substr(p + 1, q - p - 1);

  std::cout << csv << std::endl;

  std::vector<std::string> str = split_string(csv, ',');

  for (auto s : str) {
    std::cout << "[" << s << "]";
  }
}

#endif

#if 0
#include <vector>
#include <stdio.h>
#include <stdint.h>
#include <util_crc.h>

const uint8_t default_id_ = 0x01;
const uint8_t default_fc_ = 0x01;
const uint8_t default_len_ = 0x05;
const uint8_t default_cmd_end_ = 0x0A;
const uint8_t zero_byte_ = 0x00;

uint8_t get_low_byte(double value)
{
  uint16_t value_byte = static_cast<uint16_t>(value);
  return static_cast<uint8_t>(value_byte & 0x00FF);
}

uint8_t get_high_byte(double value)
{
  uint16_t value_byte = static_cast<uint16_t>(value);
  return static_cast<uint8_t>((value_byte & 0xFF00) >> 8);
}

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

std::vector<uint8_t> pack_packet(Action fbb, double curr_trac, double curr_rotate,
    double target_trac, double target_steer)
{
  std::vector<uint8_t> cmd_vector;

  InnovatiCmd cmd;
  cmd.id = default_id_;
  cmd.fc = default_fc_;
  cmd.cmd_data_len = default_len_;
  cmd.fbb_lo = static_cast<uint8_t>(fbb);
  cmd.fbb_hi = zero_byte_;
  cmd.trac_lo = get_low_byte(curr_trac);
  cmd.trac_hi = get_high_byte(curr_trac);
  cmd.steer_lo = get_low_byte(curr_rotate);
  cmd.steer_hi = get_high_byte(curr_rotate);
  cmd.tar_trac_lo = get_low_byte(target_trac);
  cmd.tar_trac_hi = get_high_byte(target_trac);
  cmd.tar_rotat_lo = get_low_byte(target_steer);
  cmd.tar_rotat_hi = get_high_byte(target_steer);

  cmd_vector.push_back(cmd.id);
  cmd_vector.push_back(cmd.fc);
  cmd_vector.push_back(cmd.cmd_data_len);
  cmd_vector.push_back(cmd.fbb_lo);
  cmd_vector.push_back(cmd.fbb_hi);
  cmd_vector.push_back(cmd.trac_lo);
  cmd_vector.push_back(cmd.trac_hi);
  cmd_vector.push_back(cmd.steer_lo);
  cmd_vector.push_back(cmd.steer_hi);
  cmd_vector.push_back(cmd.tar_trac_lo);
  cmd_vector.push_back(cmd.tar_trac_hi);
  cmd_vector.push_back(cmd.tar_rotat_lo);
  cmd_vector.push_back(cmd.tar_rotat_hi);

  uint16_t crc = agv::util_crc_calc(cmd_vector.data(), static_cast<uint16_t>(cmd_vector.size()));

  cmd.crc_lo = crc & 0x00FF;
  cmd.crc_hi = (crc & 0xFF00) >> 8;
  cmd.cmd_end = default_cmd_end_;

  cmd_vector.push_back(cmd.crc_lo);
  cmd_vector.push_back(cmd.crc_hi);
  cmd_vector.push_back(cmd.cmd_end);

  return cmd_vector;
}

int main()
{
  double curr_trac = 360;  // 360 m/h = 0.1 m/s
  double curr_rotate = 3000;  // = 30 deg x 100
  double target_trac = curr_trac;
  double target_steer = curr_rotate;

  auto pkt = pack_packet(Action::FORWARD, curr_trac, curr_rotate, target_trac, target_steer);

  // 印出封包 raw data
  printf("raw data of packet:\n");
  for (int i = 0; i < pkt.size(); i++) {
    printf("0x%02X, ", pkt[i]);
  }

  return 0;
}

#endif

#if 0
#include <agv_motionbox_client.h>
#include <agv_sick_encoder_driver.h>
#include <agv_digital_io_driver.h>
#include <unistd.h>

bool start()
{
  agv::SickEncoderDriver encoder("can0");
  agv::DigitalIODriver digital_io;
  agv::MotionBoxClient mbx_client("/dev/ttyS0", 115200, 200, 1.4543, &encoder, &digital_io);

  if (!mbx_client.start()) {
    printf(">>>> mbx_client.start() failed !");
    return false;
  }

  while (1) {
    mbx_client.scan_degree();
    break;
    usleep(100000);
  }

  return false;
}

int main()
{
  start();
  return 0;
}
#endif

#if 0
#include <iostream>
#include <cil_digital_motor.h>
#include <unistd.h>
#include <agv_pid_controller.h>
#include <math.h>
#include <cil_digital_motor.h>

agv::PIDController pid_ctrl_(1.0, 1.0, 0.0, 4);
const double degree_ = 180 / M_PI;
const double rad_ = M_PI / 180;

cil::DigitalMotor dmotor(10.0, 1.0);

double to_degree(double rad)
{
  return rad * degree_;  // unit degree
}

double to_rad(double deg)
{
  return deg * rad_;  // unit rad
}

const double steer_deadzone_tolerance_ = 0.5;

double get_pushed_angle(double target, double cur)
{
  static double pushed_ang = 0.0;  // rad
  static double pre_pushed_ang = 0.0;
  double e = target - cur;  // error : rad

  pid_ctrl_.set_error(e);  // unit: rad
  pushed_ang = target;  // rad

  if ((e < steer_deadzone_tolerance_) && (e > -steer_deadzone_tolerance_)) {
    //  do nothing, 不補償，維持上一次設定值
    pushed_ang = pre_pushed_ang;
  }
  else  {  // 目前控制值過高、過低
    pushed_ang = target + pid_ctrl_.get_output();
    pre_pushed_ang = pushed_ang;
//
//    printf(">>> target= %lf, cur= %lf, e = %lf, pushed_ang = %lf, pre_ang = %lf",
//        to_degree(target), to_degree(cur), to_degree(e), to_degree(pushed_ang),
//        to_degree(pre_pushed_ang));
  }

  // todo: 限制角度

  return pushed_ang;
}

double get_steer_angle()
{
  dmotor.update();

  return dmotor.get_output();
}

int main()
{
#define CYC_RANGE 100

  double target = -50.0;
  // dmotor.set_voltage(5);

  FILE *fp = fopen("/tmp/crv", "w");

  for (int i = 0; i < CYC_RANGE; i++) {
    if(i == 50) {
      target = 0;
    }

    double cur = get_steer_angle();
    double pushed_cmd = get_pushed_angle(target, cur);

    if(pushed_cmd > 90) { pushed_cmd = 90; }
    if(pushed_cmd < -90) { pushed_cmd = -90; }

    pushed_cmd = 4*((int)(pushed_cmd/4));
    dmotor.set_voltage(pushed_cmd);
    printf("%d, %lf\n", i, cur);

    fprintf(fp, "%d, %lf, %lf, %lf, %lf\n", i, pushed_cmd, cur, target + 0.5, target - 0.5);
  }

  fclose(fp);
  return 0;
}

#endif

#if 0
#include <iostream>
#include <cil_digital_motor.h>
#include <unistd.h>

int main()
{
  cil::Motor m(1, 1);

#if 0
  // 輸入 5 V , 每 0.1 sec 輸出
  m.set_voltage(5.0);

  double dt = 0.1;
  for (double t = 0.0; t < 5.0; t += dt) {
    double vo = m.get_output();
    printf("%.4lf, %.4lf\n", t, vo);
    usleep(dt*1000000.0);
  }

  // 輸入 0 V 放電
  m.set_voltage(0.0);

  dt = 0.1;
  for (double t = 0.0; t < 5.0; t += dt) {
    double vo = m.get_output();
    printf("%.4lf, %.4lf\n", t, vo);
    usleep(dt*1000000.0);
  }
#endif

#if 1
  // 輸入 0 ~ 5 V step up , 每 0.1 sec 遞增 0.1 V
  m.set_voltage(0.0);

  double dv = 0.1;
  double dt = 0.1;
  for (double t = 0.0, v = 0.0; t < 5.0; t += dt, v += dv) {
    m.set_voltage(v);
    usleep(dt*1000000.0);

    double vo = m.get_output();
    printf("%.4lf, %.4lf, %.4lf\n", t, v, vo);
    ;
  }
#endif
}

#endif

#if 0  // 未完，待繼續
#include <iostream>
#include <agv_fork_stopper.h>

using namespace std;
vector<double> height_m;
vector<double> time_s;

/// 1200 kg 上升速度 50 mm/s, 在 50 Hz w/r 頻率下，剛好 1 mm/Hz.
/// 準備數據 0 ~ 2200 mm, 也就是 2200 samples
void prepare_1200kg_up_samples(vector<double> &h_mm, vector<double> &t_s)
{
  const double period = 1.0 / 50.0;  // unit sec
  const double dis_per_period = 1.0;  // unit mm
  const double height_range = 2200;  // unit mm

  h_mm.clear();
  t_s.clear();
  for (double h = 0.0, t = 0.0; h < height_range; h += dis_per_period, t += period) {
    h_mm.push_back(h/1000.0);  // to (m)
    t_s.push_back(t);
  }
}

/// 1200 kg 上升速度 50 mm/s, 在 50 Hz w/r 頻率下，剛好 1 mm/Hz.
/// 準備數據 0 ~ 2200 mm, 也就是 2200 samples
void prepare_1200kg_down_samples(vector<double> &h_mm, vector<double> &t_s)
{
  const double period = 1.0 / 50.0;  // unit sec
  const double dis_per_period = 1.0;  // unit mm
  const double height_range = 2200;  // unit mm

  h_mm.clear();
  t_s.clear();
  for (double h = height_range, t = 0.0; h >= 0.0; h -= dis_per_period, t += period) {
    h_mm.push_back(h/1000.0);  // to (m)
    t_s.push_back(t);
  }
}


int main()
{
  const double from_m = 0.0;
  const double to_m = 2.0;

  agv::ForkStopper stopper("skid_time.db");

  prepare_1200kg_up_samples(height_m, time_s);

  stopper.set_moving_range(from_m, to_m);

  // 模擬上升中
  int skid_period_num = 9;  // 假設滑行 9 period == 9 mm
  bool is_set_stop = false;
  for (unsigned int i = 0; i < height_m.size(); i++) {
    stopper.update_current_height(time_s[i], height_m[i]);

    if (stopper.stopping_now()) {
      if (!is_set_stop) {
        is_set_stop = true;
        stopper.begin_stop_height(height_m[i]);
      }
      // 下令 stop
      skid_period_num--;
    }

    if (skid_period_num == 0) {
      stopper.final_arrived_height(height_m[i]);
      break;
    }
  }

  prepare_1200kg_down_samples(height_m, time_s);
  stopper.set_moving_range(to_m, 0.15);
  // 模擬下降中
  skid_period_num = 9;  // 假設滑行 9 period == 9 mm
  is_set_stop = false;
  for (unsigned int i = 0; i < height_m.size(); i++) {
    stopper.update_current_height(time_s[i], height_m[i]);

    if (stopper.stopping_now()) {
      if (!is_set_stop) {
        is_set_stop = true;
        stopper.begin_stop_height(height_m[i]);
      }
      // 下令 stop
      skid_period_num--;
    }

    if (skid_period_num == 0) {
      stopper.final_arrived_height(height_m[i]);
      break;
    }
  }

  return 0;
}
#endif

#if 0
#include <stdint.h>
#include <stdio.h>
#include <functional>

#include <string>
#include <vector>

class PIDController {
public:
  PIDController(){}
  PIDController(const double pgain, const double igain, const double dgain, const int win_size);

  ~PIDController();

  void set_error(double err);
  double get_output();

  void set_p_gain(double gain);
  void set_i_gain(double gain);
  void set_d_gain(double gain);
  void set_winsize(int size);  // 單位：sample 數

private:
  double p_gain_ = 1.0;
  double i_gain_ = 0.0;
  double d_gain_ = 0.0;

  int win_size_ = 0;
  int last_sample_pos_ = 0;
  std::vector<double> samples_;  // size = win_size
  bool samples_full_ = false;
};

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
  printf("hi");
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

void PIDController::set_winsize(int size)
{
  win_size_ = size;
}

int main()
{
  PIDController pid_ctrl_(1.0,2.0,3.0,4);
  double cmd = 0;
  double vel = 0;

  pid_ctrl_.set_p_gain(0);
  pid_ctrl_.set_i_gain(1);
  pid_ctrl_.set_d_gain(0);
  pid_ctrl_.set_winsize(10);

  pid_ctrl_.set_error(cmd - vel);  // unit: rad/s
}

#endif



#if 0
bool wait_for_condition(std::function<bool()> condition)
{
  while (1) {
    if (condition()) {
      return true;
    }
  }

  return false;
}

bool foo()
{
  return true;
}

int main()
{

  // wait_for_condition([]() -> bool {return foo();});
  char a[] = "AHM36A-**C*14x12";

  for (int i = 0; a[i] != '\0'; i++) {
    printf("%X ", a[i]);
  }

  return 0;
}

#endif

#if 0
#include <stdint.h>
#include <stdio.h>

#define CRC_HI      0xFF
#define CRC_LO      0xFF
// ---------------------------------------------------------------------------
// The following lookup tables used to quickly calculate CRC of incoming and
// outgoing Modbus data blocks. They're large, but this is the fastest way.
// ---------------------------------------------------------------------------
static const uint8_t crchiarray_m[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t crcloarray_m[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/*
 * @brief  This function calculates the 16-bit CRC of the ncount bytes pointed to by pdata.
 * @param  pdata : Pointer to the data
 *         ncount  : Number of bytes
 * @return CRC value
 */
uint16_t util_crc_calc(uint8_t *pdata, uint16_t ncount)
{
    uint8_t crchi = CRC_HI; //high byte of CRC initialized
    uint8_t crclo = CRC_LO; //low byte of CRC initialized
    uint8_t index = 0;      //will index into CRC lookup table
    uint16_t crc = 0;       //放置運算出的結果
    uint16_t x = 0;         //pilot2_KC_0060, 宣告於此，使得 PC 非 C99 mode 可以 compile.

    for(x = ncount; x > 0; x--) //pilot2_KC_0060 //step through each byte of data
    {
        index = crchi ^ *pdata++; //calculate the CRC
        crchi = crclo ^ crchiarray_m[index];
        crclo = crcloarray_m[index];
    }

//  crc = (crchi << 8) + crclo;
    crc = (crclo << 8) + crchi; //交換高低位元
    return crc;
}

int main() {
  unsigned char data[16] = {
      0x01, 0x01, 0x05, 0x00, 0x00, 0xAE, 0x01, 0xAE, 0x01, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x0A
  };

  uint16_t crc = util_crc_calc(data, 13);

  printf("0x%04X\n", crc);

  return 0;
}
#endif

#if 0
#include <stdint.h>
#include <stdio.h>

struct CanFrame {
  uint32_t id = 0;
  uint8_t len = 0;
  uint8_t flags = 0;
  uint8_t data[64];
};

int main() {
  int i = 0xFFFFFFA3;
  printf("%X %d", -i, i);
  CanFrame cf;
  CanFrame df;

  cf.id = 1;
  cf.len = 2;
  cf.flags = 3;
  for(int i=0; i< 64; i++)
  {
    cf.data[i] = i+100;
  }

  df = cf;

  for (int i = 0; i < 64; i++) {
    printf("%d\t", df.data[i]);
  }

}


#endif


#if 0
#include <thread>
#include <iostream>
#include <unistd.h>

class A {
public:
  ~A();
  void start();

  std::thread read_thread_;
  bool stop = true;
private:
  void thd_func();
};

A::~A()
{
  stop = true;
  read_thread_.join();
}

void A::start()
{
  stop = false;
  read_thread_ = std::thread([this] {thd_func();});
}

void A::thd_func()
{
  static int i;

  while(!stop) {
    i++;
    std::cout << i << std::endl;
  }

  std::cout << "stop thread !!" << std::endl;
}

void thd_fx()

{
  static int i;

  while (true) {
    i++;
    std::cout << i << std::endl;
  }
}

int main()
{
  A a;

  a.start();

  usleep(1000000);

  return 0;
}
#endif

#if 0
#include <iostream>
#include <ios>

int main()
{
  int a = 0x64;
  std::cout << std::hex << a << " hi" << std::endl;

  return 0;
}

#endif

#if 0

#ifndef SOCKETCAN_CPP_EXPORT_H
#define SOCKETCAN_CPP_EXPORT_H

#ifdef SOCKETCAN_CPP_STATIC_DEFINE
#  define SOCKETCAN_CPP_EXPORT
#  define SOCKETCAN_CPP_NO_EXPORT
#else
#  ifndef SOCKETCAN_CPP_EXPORT
#    ifdef socketcan_cpp_EXPORTS
        /* We are building this library */
#      define SOCKETCAN_CPP_EXPORT
#    else
        /* We are using this library */
#      define SOCKETCAN_CPP_EXPORT
#    endif
#  endif

#  ifndef SOCKETCAN_CPP_NO_EXPORT
#    define SOCKETCAN_CPP_NO_EXPORT
#  endif
#endif

#ifndef SOCKETCAN_CPP_DEPRECATED
#  define SOCKETCAN_CPP_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef SOCKETCAN_CPP_DEPRECATED_EXPORT
#  define SOCKETCAN_CPP_DEPRECATED_EXPORT SOCKETCAN_CPP_EXPORT SOCKETCAN_CPP_DEPRECATED
#endif

#ifndef SOCKETCAN_CPP_DEPRECATED_NO_EXPORT
#  define SOCKETCAN_CPP_DEPRECATED_NO_EXPORT SOCKETCAN_CPP_NO_EXPORT SOCKETCAN_CPP_DEPRECATED
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define SOCKETCAN_CPP_NO_DEPRECATED
#endif

#define HAVE_SOCKETCAN_HEADERS

#endif
//============================================================================
//#pragma once

#include <string>
//#include <socketcan_cpp/socketcan_cpp_export.h>

#ifndef HAVE_SOCKETCAN_HEADERS
#define CAN_MTU 0
#define CANFD_MTU 1
#else
#include <linux/can.h>
#endif

namespace scpp
{

    enum SocketMode
    {
        MODE_CAN_MTU = CAN_MTU,
        MODE_CANFD_MTU = CANFD_MTU
    };

    struct CanFrame
    {
        uint32_t id = 0;
        uint8_t len = 0;
        uint8_t flags = 0;
        uint8_t data[64];

    };

    enum SocketCanStatus
    {
        STATUS_OK = 1 << 0,
        STATUS_SOCKET_CREATE_ERROR = 1 << 2,
        STATUS_INTERFACE_NAME_TO_IDX_ERROR = 1 << 3,
        STATUS_MTU_ERROR = 1 << 4, /// maximum transfer unit
        STATUS_CANFD_NOT_SUPPORTED = 1 << 5, /// Flexible data-rate is not supported on this interface
        STATUS_ENABLE_FD_SUPPORT_ERROR = 1 << 6, /// Error on enabling fexible-data-rate support
        STATUS_WRITE_ERROR = 1 << 7,
        STATUS_READ_ERROR = 1 << 8,
        STATUS_BIND_ERROR = 1 << 9,
    };

    class SocketCan
    {
    public:
        SOCKETCAN_CPP_EXPORT SocketCan();
        SOCKETCAN_CPP_EXPORT SocketCan(const SocketCan &) = delete;
        SOCKETCAN_CPP_EXPORT SocketCan & operator=(const SocketCan &) = delete;
        SOCKETCAN_CPP_EXPORT SocketCanStatus open(const std::string & can_interface, int32_t read_timeout_ms = 3, SocketMode mode = MODE_CAN_MTU);
        SOCKETCAN_CPP_EXPORT SocketCanStatus write(const CanFrame & msg);
        SOCKETCAN_CPP_EXPORT SocketCanStatus read(CanFrame & msg);
        SOCKETCAN_CPP_EXPORT SocketCanStatus close();
        SOCKETCAN_CPP_EXPORT const std::string & interfaceName() const;
        SOCKETCAN_CPP_EXPORT ~SocketCan();
    private:
        int m_socket = -1;
        int32_t m_read_timeout_ms = 3;
        std::string m_interface;
        SocketMode m_socket_mode;
    };
}

//=================================================================================================
//#include "../include/socketcan_cpp/socketcan_cpp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef HAVE_SOCKETCAN_HEADERS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can/raw.h>
/* CAN DLC to real data length conversion helpers */

static const unsigned char dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7,
          8, 12, 16, 20, 24, 32, 48, 64};

/* get data length from can_dlc with sanitized can_dlc */
unsigned char can_dlc2len(unsigned char can_dlc)
{
  return dlc2len[can_dlc & 0x0F];
}

static const unsigned char len2dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,    /* 0 - 8 */
          9, 9, 9, 9,       /* 9 - 12 */
          10, 10, 10, 10,       /* 13 - 16 */
          11, 11, 11, 11,       /* 17 - 20 */
          12, 12, 12, 12,       /* 21 - 24 */
          13, 13, 13, 13, 13, 13, 13, 13,   /* 25 - 32 */
          14, 14, 14, 14, 14, 14, 14, 14,   /* 33 - 40 */
          14, 14, 14, 14, 14, 14, 14, 14,   /* 41 - 48 */
          15, 15, 15, 15, 15, 15, 15, 15,   /* 49 - 56 */
          15, 15, 15, 15, 15, 15, 15, 15};  /* 57 - 64 */

/* map the sanitized data length to an appropriate data length code */
unsigned char can_len2dlc(unsigned char len)
{
  if (len > 64)
    return 0xF;

  return len2dlc[len];
}

#endif

namespace scpp
{
    SocketCan::SocketCan()
    {
    }
    SocketCanStatus SocketCan::open(const std::string & can_interface, int32_t read_timeout_ms, SocketMode mode)
    {
        m_interface = can_interface;
        m_socket_mode = mode;
        m_read_timeout_ms = read_timeout_ms;
#ifdef HAVE_SOCKETCAN_HEADERS

        /* open socket */
        if ((m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("socket");
            return STATUS_SOCKET_CREATE_ERROR;
        }
        int mtu, enable_canfd = 1;
        struct sockaddr_can addr;
        struct ifreq ifr;

        strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
        if (!ifr.ifr_ifindex) {
            perror("if_nametoindex");
            return STATUS_INTERFACE_NAME_TO_IDX_ERROR;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (mode == MODE_CANFD_MTU)
        {
            /* check if the frame fits into the CAN netdevice */
            if (ioctl(m_socket, SIOCGIFMTU, &ifr) < 0) {
                perror("SIOCGIFMTU");
                return STATUS_MTU_ERROR;
            }
            mtu = ifr.ifr_mtu;

            if (mtu != CANFD_MTU) {
                return STATUS_CANFD_NOT_SUPPORTED;
            }

            /* interface is ok - try to switch the socket into CAN FD mode */
            if (setsockopt(m_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                &enable_canfd, sizeof(enable_canfd)))
            {

                return STATUS_ENABLE_FD_SUPPORT_ERROR;
            }

        }

        //const int timestamping_flags = (SOF_TIMESTAMPING_SOFTWARE |
        //    SOF_TIMESTAMPING_RX_SOFTWARE |
        //    SOF_TIMESTAMPING_RAW_HARDWARE);

        //if (setsockopt(m_socket, SOL_SOCKET, SO_TIMESTAMPING,
        //    &timestamping_flags, sizeof(timestamping_flags)) < 0) {
        //    perror("setsockopt SO_TIMESTAMPING is not supported by your Linux kernel");
        //}

        ///* disable default receive filter on this RAW socket */
        ///* This is obsolete as we do not read from the socket at all, but for */
        ///* this reason we can remove the receive list in the Kernel to save a */
        ///* little (really a very little!) CPU usage.                          */
        //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

// LINUX
        struct timeval tv;
        tv.tv_sec = 0;  /* 30 Secs Timeout */
        tv.tv_usec = m_read_timeout_ms * 1000;  // Not init'ing this can cause strange errors
        setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));

        if (bind(m_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("bind");
            return STATUS_BIND_ERROR;
        }
#else
        printf("Your operating system does not support socket can! \n");
#endif
        return STATUS_OK;
    }
    SocketCanStatus SocketCan::write(const CanFrame & msg)
    {
#ifdef HAVE_SOCKETCAN_HEADERS
        struct canfd_frame frame;
        memset(&frame, 0, sizeof(frame)); /* init CAN FD frame, e.g. LEN = 0 */
        //convert CanFrame to canfd_frame
        frame.can_id = msg.id;
        frame.len = msg.len;
        frame.flags = msg.flags;
        memcpy(frame.data, msg.data, msg.len);

        if (m_socket_mode == MODE_CANFD_MTU)
        {
            /* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
            frame.len = can_dlc2len(can_len2dlc(frame.len));
        }
        /* send frame */
        if (::write(m_socket, &frame, int(m_socket_mode)) != int(m_socket_mode)) {
            perror("write");
            return STATUS_WRITE_ERROR;
        }
#else
        printf("Your operating system does not support socket can! \n");
#endif
        return STATUS_OK;
    }
    SocketCanStatus SocketCan::read(CanFrame & msg)
    {
#ifdef HAVE_SOCKETCAN_HEADERS
        struct canfd_frame frame;

        // Read in a CAN frame
        auto num_bytes = ::read(m_socket, &frame, CANFD_MTU);
        if (num_bytes != CAN_MTU && num_bytes != CANFD_MTU)
        {
            //perror("Can read error");
            return STATUS_READ_ERROR;
        }

        msg.id = frame.can_id;
        msg.len = frame.len;
        msg.flags = frame.flags;
        memcpy(msg.data, frame.data, frame.len);
#else
        printf("Your operating system does not support socket can! \n");
#endif
        return STATUS_OK;
    }
    SocketCanStatus SocketCan::close()
    {
#ifdef HAVE_SOCKETCAN_HEADERS
        ::close(m_socket);
#endif
        return STATUS_OK;
    }
    const std::string & SocketCan::interfaceName() const
    {
        return m_interface;
    }
    SocketCan::~SocketCan()
    {
        close();
    }
}

//=================================================================================================
//#include "socketcan_cpp/socketcan_cpp.h"
#include <string>
#include <iostream>

int main()
{
    scpp::SocketCan sockat_can;
    if (sockat_can.open("can0") == scpp::STATUS_OK)
    {
    for (int j = 0; j < 20000; ++j)
    {
#if 0 // KC+DBG
        scpp::CanFrame fr;

        while(sockat_can.read(fr) == scpp::STATUS_OK)
        {
            printf("len %d byte, id: %d, data: %02x %02x %02x %02x %02x %02x %02x %02x  \n", fr.len, fr.id,
                fr.data[0], fr.data[1], fr.data[2], fr.data[3],
                fr.data[4], fr.data[5], fr.data[6], fr.data[7]);
        }
#endif

        scpp::CanFrame cf_to_write;

        cf_to_write.id = 123;
        cf_to_write.len = 8;
        for (int i = 0; i < 8; ++i)
            cf_to_write.data[i] = j & (254);
        auto write_sc_status = sockat_can.write(cf_to_write);
        if (write_sc_status != scpp::STATUS_OK)
            printf("something went wrong on socket write, error code : %d \n", int32_t(write_sc_status));
        else
            printf("Message was written to the socket \n");
    }
    }
    else
    {
        printf("Cannot open can socket!");
    }
    return 0;
}



#endif

#if 0
#include <iostream>
#include <memory>
#include <utility>

class Base {
public:
  virtual ~Base(){
    std::cout << "base dctor" << std::endl;
  }
};

class Child {
public:
  ~Child()
  {
    std::c  << "child dctor" << std::endl;
  }
};

int main()
{
  std::unique_ptr<Base> a;
  a = std::move(std::make_unique<Child>());
  //Child c;

  return 0;
}
#endif

#if 0
#include <iostream>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <chrono>
#include <future>


std::vector<int> data;
std::condition_variable cv;
std::mutex data_mutex;
bool flag = false;

bool dumpsd()
{
  std::unique_lock<std::mutex> lock(data_mutex);
  cv.wait_for(lock, std::chrono::seconds(1), [] {return (flag == true);});
  for (const auto v : data) {
    std::cout << v << '\n';
  }

  return true;
}

int main()
{
  auto result = std::async(std::launch::async, dumpsd);
  for (int i = 0; i < 10; ++i) {
    data.emplace_back(i);
  }
  flag = true;
  cv.notify_one();

  // do something
  result.wait();
  return 0;
}
#endif

#if 0
#include <iostream>
#include <sstream>
#include <set>
using namespace std;

int main ()
{
  std::set<std::string> my_set;
    string tmps;
    istringstream is ("the dellimiter is the space");
    while (is.good ()) {
        is >> tmps;
        //cout << tmps << "\n";
      my_set.insert(tmps);
    }

    if(my_set.count("the") > 0)
     cout << "ok";
    else
      cout << "NG";

    return 0;
}

#endif

#if 0
#include <string>
#include <iostream>
#include <map>

class BatteryInfo {
public:
  double voltage_;
  double current_;
  int remaining_;
};
//-------------------------------------------------------

class BatteryUpdator {
public:
  virtual bool update(BatteryInfo *info) = 0;
  virtual ~BatteryUpdator() = default;
private:
};

class FakeBatteryUpdator : public BatteryUpdator {
public:
  bool update(BatteryInfo *info)
  {
    // do .....
    info->voltage_ = 27.0;

    return true;
  }
};

class LiFeBatteryUpdator : public BatteryUpdator {
public:
  bool update(BatteryInfo *info)
  {
    // do read uart .......
    info->voltage_ = 28.0;

    return true;
  }
};
//-------------------------------------------------------

class Battery {
public:
  Battery(BatteryUpdator *bat_updator)
  {
    battery_updator = bat_updator;
  }

  bool update_device()
  {
    return battery_updator->update(&battery_info_);
  }

  void pub_info() {
    // pub battery_info_ --> (voltage, current ...)
    std::cout << "voltage " << battery_info_.voltage_ << std::endl;
  }

private:
  BatteryInfo battery_info_;
  BatteryUpdator *battery_updator;
};
//-------------------------------------------------------

int main()
{
  // 假設從 launch file 得到 battery_type = "fake"
  std::string battery_type("fake");

  FakeBatteryUpdator fake_updator;
  LiFeBatteryUpdator life_updator;

  std::map<std::string, BatteryUpdator*> updators {
    {"fake", &fake_updator}, {"life", &life_updator}
  };

  Battery battery(updators[battery_type]);

  while(true) {
    battery.update_device ();
    battery.pub_info();
  }
}

#endif

#if 0
#include <iostream>

int y =5;

int &min(int &x)
{
  int z = 10;
  return (x < y) ? x : z;
}

int main()
{
  int a = 10;
  int &tmp = min(a) ;//= 3;

  tmp = 99;

  std::cout  << "tmp= " << tmp << std::endl;

  return 0;
}
#endif

#if 0
#include <iostream>

int &min(int &x, int &y)
{
  return (x < y) ? x : y;
}

int main()
{
  int a = 1, b = 10;

  min(a,b) = 3;

  std::cout  << "a= " << a << std::endl;
  std::cout  << "min(a, b) = " << min(a, b) << std::endl;

  return 0;
}
#endif

#if 0
#include <iostream>
#include <vector>
#include <string>

class Person {
public:
  std::string name;
  int age;
};

int main()
{
  std::vector<Person> people {{"Bill", 5}, {"John", 10}};

  std::cout << people[0].name << "is " << people[0].age << " years old." << std::endl;

  std::cout << people[1].name << "is " << people[1].age << " years old." << std::endl;

  return 0;
}
#endif

#if 0
#include <iostream>
using namespace std;

class Parent{
public:
     virtual void say() {
           cout<<"I am Parent" ;
     }
};

class Children:public Parent{
public:
     virtual void say() override {
           cout<<"I am Children" ;
     }
};

int main()
{
  cout << "!!!Hello World!!!" << endl;  // prints !!!Hello World!!!

  Children c;
  Parent &p =  c;
  p.say() ;

  return 0;
}
#endif
