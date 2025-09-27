#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace {
constexpr double kCountToRad = 2.0 * M_PI / 4096.0;
constexpr double kCurrentPerCount = 0.00269;  // [A/LSB] XH430/540 공통

using json = nlohmann::json;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using rclcpp::Duration;
using rclcpp::Time;

Matrix4d dh(double theta, double d, double a, double alpha)
{
  const double cth = std::cos(theta);
  const double sth = std::sin(theta);
  const double cal = std::cos(alpha);
  const double sal = std::sin(alpha);

  Matrix4d T = Matrix4d::Identity();
  T <<  cth, -sth * cal,  sth * sal, a * cth,
        sth,  cth * cal, -cth * sal, a * sth,
          0,         sal,        cal,       d,
          0,           0,          0,       1;
  return T;
}

struct DHParameters {
  double d1{};
  double a2{};
  double a3{};
  double a4{};
  double alpha1{};
  double theta2_offset{};
  Vector3d gravity{0.0, 0.0, -9.81};
};

struct LinkInertialData {
  double mass{0.0};
  Vector3d com{0.0, 0.0, 0.0};
};

struct Tunables {
  double control_frequency_hz = 100.0;         // 제어 주기 [Hz]
  double integral_freeze_sec = 0.2;            // 상태 전환 시 I항 동결 시간
  double v_max_cnt_s = 1800.0;                 // 경로 속도 상한 [cnt/s]
  double a_max_cnt_s2 = 9000.0;                // 경로 가속 상한 [cnt/s^2]
  double corner_ratio = 0.55;                  // 코너 속도 비율 (v_corner = ratio * v_max)
  double v_ref_eps = 50.0;                     // FSM 기준: 참조 속도 임계 [cnt/s]
  double v_meas_eps = 30.0;                    // FSM 기준: 실측 속도 임계 [cnt/s]
  double hold_deadband = 20.0;                 // HOLD 데드밴드 [cnt]
  double hold_ki_scale = 0.2;                  // HOLD 모드 KI 배율
  double hold_i_leak_rate = 0.001;             // HOLD 모드 I 누수율 [1/s]
  double bias_alpha = 0.3;                     // HOLD 바이어스 학습 이득 [1/s]
  double bias_max = 120.0;                     // HOLD 바이어스 제한 [LSB]
  double vel_lpf_a = 0.85;                     // 속도 저역통과 계수
  double dterm_limit = 300.0;                  // D항 클램프 [LSB]
  double aw_gain = 0.05;                       // Anti-windup 보상 gain
  double integral_limit = 2500.0;              // I항 클램프
  std::array<int32_t, 5> current_limit{1000, 1000, 1000, 700, 700};  // [LSB]
  std::array<float, 4> kp{1.0f, 1.7f, 1.8f, 1.4f};
  std::array<float, 4> ki{0.00f, 0.01f, 0.01f, 0.01f};
  std::array<float, 4> kd{0.015f, 0.015f, 0.02f, 0.010f};
  float kp_dxl5 = 0.8f;
  float ki_dxl5 = 0.0f;
  float kd_dxl5 = 0.02f;
};

struct SegmentProfile {
  double distance = 0.0;   // 최대 변위 (스칼라 진행) [cnt]
  double v_start = 0.0;    // 시작 속도 [cnt/s]
  double v_end = 0.0;      // 종료 속도 [cnt/s]
  double v_peak = 0.0;     // 중간 최대 속도 [cnt/s]
  double t_acc = 0.0;      // 가속 구간 길이 [s]
  double t_flat = 0.0;     // 정속 구간 길이 [s]
  double t_dec = 0.0;      // 감속 구간 길이 [s]
  double duration = 0.0;   // 전체 세그먼트 길이 [s]
};

struct TrajectorySegment {
  Eigen::Vector4d start = Eigen::Vector4d::Zero();
  Eigen::Vector4d goal = Eigen::Vector4d::Zero();
  Eigen::Vector4d delta = Eigen::Vector4d::Zero();
  SegmentProfile profile;
};

DHParameters load_dh_parameters(const std::string &config_dir)
{
  std::ifstream file(config_dir + "/gravity_dh_param.json");
  if (!file.is_open()) {
    throw std::runtime_error("gravity_dh_param.json 파일을 열 수 없습니다");
  }

  json j;
  file >> j;

  DHParameters params;
  const auto &joints = j["gravity_dh_parameters"]["joints"];

  for (const auto &joint : joints) {
    const int motor_id = joint.at("motor_id");
    const auto &dh = joint.at("dh_params");
    switch (motor_id) {
      case 1:
        params.d1 = dh.at("d");
        params.alpha1 = dh.at("alpha");
        break;
      case 2:
        params.a2 = dh.at("a");
        if (dh.contains("theta_offset")) {
          params.theta2_offset = dh.at("theta_offset");
        }
        break;
      case 3:
        params.a3 = dh.at("a");
        break;
      case 4:
        params.a4 = dh.at("a");
        break;
      default:
        break;
    }
  }

  const auto &gvec = j["gravity_dh_parameters"]["gravity_vector"];
  params.gravity.x() = gvec.at("x");
  params.gravity.y() = gvec.at("y");
  params.gravity.z() = gvec.at("z");

  return params;
}

std::vector<LinkInertialData> load_inertial_data(const std::string &config_dir, std::array<double, 4> &k_gff)
{
  std::ifstream file(config_dir + "/link_inertial.json");
  if (!file.is_open()) {
    throw std::runtime_error("link_inertial.json 파일을 열 수 없습니다");
  }

  json j;
  file >> j;

  std::vector<LinkInertialData> links;
  const auto &root = j.at("link_inertial_properties").at("links");
  links.reserve(root.size());
  for (const auto &link : root) {
    LinkInertialData data;
    data.mass = link.at("mass");
    const auto &com = link.at("center_of_mass");
    data.com.x() = com.at("x");
    data.com.y() = com.at("y");
    data.com.z() = com.at("z");
    links.push_back(data);
  }

  if (j.contains("gravity_compensation")) {
    const auto &gff = j["gravity_compensation"]["gravity_feedforward_gains"];
    k_gff[0] = gff.value("joint_1", 1.0);
    k_gff[1] = gff.value("joint_2", 1.0);
    k_gff[2] = gff.value("joint_3", 1.0);
    k_gff[3] = gff.value("joint_4", 1.0);
  }
  return links;
}

class TestMotorConnectNode : public rclcpp::Node {
public:
  TestMotorConnectNode()
  : rclcpp::Node("test_motor_connect"),
    tunables_(),
    dh_params_(),
    port_handler_(dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0")),
    packet_handler_(dynamixel::PacketHandler::getPacketHandler(2.0))
  {
    try {
      const std::string default_config = this->declare_parameter<std::string>(
        "config_path", "/home/pc/soomac_ws/src/dongsoo_description/config");
      config_path_ = default_config;

      dh_params_ = load_dh_parameters(config_path_);
      auto links = load_inertial_data(config_path_, k_gff_);
      if (links.size() < link_data_.size()) {
        throw std::runtime_error("링크 관성 데이터가 충분하지 않습니다");
      }
      std::copy_n(links.begin(), link_data_.size(), link_data_.begin());
    } catch (const std::exception &e) {
      RCLCPP_FATAL(get_logger(), "설정 파일 로드 실패: %s", e.what());
      throw;
    }

    initialise_mechanics();
    initialise_transport();
    initialise_state();

    current_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/motor/current", 10);
    velocity_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/motor/velocity", 10);
    position_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/motor/position", 10);

    path_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/motor/command_path", rclcpp::QoS(10),
      std::bind(&TestMotorConnectNode::path_callback, this, std::placeholders::_1));

    position_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/motor/command_position", rclcpp::QoS(10),
      std::bind(&TestMotorConnectNode::position_callback, this, std::placeholders::_1));

    dxl5_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/motor/command_position_dxl5", rclcpp::QoS(10),
      std::bind(&TestMotorConnectNode::dxl5_callback, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration<double>(1.0 / tunables_.control_frequency_hz);
    control_timer_ = create_wall_timer(timer_period, std::bind(&TestMotorConnectNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "test_motor_connect 노드 초기화 완료");
  }

  ~TestMotorConnectNode() override
  {
    try {
      if (port_handler_ && port_handler_->is_using_) {
        for (auto id : dxl_ids_) {
          packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, 0);
        }
        port_handler_->closePort();
      }
    } catch (...) {
    }
  }

private:
  enum class Mode { HOLD = 0, MOVE = 1 };

  void initialise_mechanics()
  {
    N_ = {272.5, 272.5, 272.5, 353.5};
    eta_ = {0.80, 0.70, 0.60, 0.70};

    // Kt는 쉽게 조정할 수 있도록 별도 계산.
    Kt_ = { compute_motor_torque_constant(9.2, N_[0], eta_[0], 2.4),
            compute_motor_torque_constant(9.2, N_[1], eta_[1], 2.4),
            compute_motor_torque_constant(9.2, N_[2], eta_[2], 2.4),
            compute_motor_torque_constant(4.1, N_[3], eta_[3], 1.18) };
  }

  static double compute_motor_torque_constant(double stall_torque_Nm,
                                               double gear_ratio,
                                               double efficiency,
                                               double rated_current_A)
  {
    const double motor_side_torque = stall_torque_Nm / (gear_ratio * efficiency);
    return motor_side_torque / rated_current_A;  // [Nm/A]
  }

  void initialise_transport()
  {
    if (!port_handler_->openPort()) {
      throw std::runtime_error("Dynamixel 포트를 열 수 없습니다");
    }
    if (!port_handler_->setBaudRate(3000000)) {
      throw std::runtime_error("Baudrate 설정 실패");
    }

    group_bulk_read_ = std::make_shared<dynamixel::GroupBulkRead>(port_handler_, packet_handler_);
    group_sync_write_ = std::make_shared<dynamixel::GroupSyncWrite>(port_handler_, packet_handler_, ADDR_GOAL_CURRENT, 2);

    const uint16_t addr_block = ADDR_PRESENT_CURRENT;
    const uint16_t len_block = LEN_PRESENT_CURRENT + LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;

    for (size_t index = 0; index < dxl_ids_.size(); ++index) {
      const uint8_t id = dxl_ids_[index];

      int comm = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
      if (comm != COMM_SUCCESS) {
        RCLCPP_WARN(get_logger(), "ID %u OperatingMode 설정 실패: %s", id, packet_handler_->getTxRxResult(comm));
      }

      const uint16_t limit_raw = static_cast<uint16_t>(std::clamp<int32_t>(tunables_.current_limit[index], 0, 1188));
      comm = packet_handler_->write2ByteTxRx(port_handler_, id, ADDR_CURRENT_LIMIT, limit_raw);
      if (comm != COMM_SUCCESS) {
        RCLCPP_WARN(get_logger(), "ID %u CurrentLimit 설정 실패: %s", id, packet_handler_->getTxRxResult(comm));
      }

      comm = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
      if (comm != COMM_SUCCESS) {
        RCLCPP_WARN(get_logger(), "ID %u TorqueEnable 실패: %s", id, packet_handler_->getTxRxResult(comm));
      }

      if (!group_bulk_read_->addParam(id, addr_block, len_block)) {
        throw std::runtime_error("BulkRead 파라미터 등록 실패 (ID=" + std::to_string(id) + ")");
      }
    }
  }

  void initialise_state()
  {
    auto init_read = [&]() -> bool {
      int comm = group_bulk_read_->txRxPacket();
      if (comm != COMM_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "초기 BulkRead 실패: %s", packet_handler_->getTxRxResult(comm));
        return false;
      }

      for (size_t i = 0; i < dxl_ids_.size(); ++i) {
        const uint8_t id = dxl_ids_[i];
        const int32_t pos = read_present_position(id);
        present_position_[i] = pos;
        desired_position_cnt_[i] = static_cast<double>(pos);
        desired_slew_cnt_[i] = static_cast<double>(pos);
        bias_ff_[i] = 0.0;
      }

      dxl5_target_cnt_ = present_position_[4];
      last_tick_time_ = now();
      return true;
    };

    int retry = 0;
    while (retry < 5 && !init_read()) {
      rclcpp::sleep_for(std::chrono::milliseconds(50));
      ++retry;
    }
    if (retry == 5) {
      throw std::runtime_error("초기 상태 읽기에 실패했습니다");
    }

    mode_ = Mode::HOLD;
    integrator_freeze_until_ = now();
  }

  int32_t read_present_position(uint8_t id) const
  {
    const uint32_t raw = group_bulk_read_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    int32_t value = static_cast<int32_t>(raw);
    if (value & 0x80000000) {
      value = value - 0x100000000LL;
    }
    return value;
  }

  int32_t read_present_velocity(uint8_t id) const
  {
    const uint32_t raw = group_bulk_read_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
    int32_t value = static_cast<int32_t>(raw);
    if (value & 0x80000000) {
      value = value - 0x100000000LL;
    }
    return value;
  }

  int16_t read_present_current(uint8_t id) const
  {
    const uint32_t raw = group_bulk_read_->getData(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
    return static_cast<int16_t>(raw & 0xFFFF);
  }

  void control_loop()
  {
    const Time now_time = now();
    double dt = (now_time - last_tick_time_).seconds();
    last_tick_time_ = now_time;
    if (dt <= 0.0 || dt > 0.1) {
      dt = 1.0 / tunables_.control_frequency_hz;
    }

    if (!read_feedback()) {
      if (++consecutive_read_fail_ >= watchdog_limit_) {
        watchdog_trip("BulkRead 실패 누적");
      }
      return;
    }
    consecutive_read_fail_ = 0;

    update_velocity_filters(dt);
    update_trajectory(dt);
    update_mode(now_time);
    apply_control(dt, now_time);
    publish_feedback();
  }

  bool read_feedback()
  {
    int comm = group_bulk_read_->txRxPacket();
    if (comm != COMM_SUCCESS) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "BulkRead 실패: %s",
                            packet_handler_->getTxRxResult(comm));
      return false;
    }

    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      const uint8_t id = dxl_ids_[i];
      const int32_t pos = read_present_position(id);
      const int32_t vel = read_present_velocity(id);
      const int16_t cur = read_present_current(id);

      present_position_[i] = pos;
      present_velocity_raw_[i] = static_cast<double>(vel);
      present_current_raw_[i] = static_cast<double>(cur);
    }
    return true;
  }

  void update_velocity_filters(double /* dt */)
  {
    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      const double vel_raw = present_velocity_raw_[i];
      velocity_filtered_[i] = tunables_.vel_lpf_a * velocity_filtered_[i]
        + (1.0 - tunables_.vel_lpf_a) * vel_raw;

    }
  }

  void update_trajectory(double dt)
  {
    if (segments_.empty() || current_segment_index_ >= segments_.size()) {
      traj_active_ = false;
      desired_velocity_cnt_.fill(0.0);
      return;
    }

    traj_elapsed_ += dt;

    while (current_segment_index_ < segments_.size()) {
      const auto &seg = segments_[current_segment_index_];
      if (traj_elapsed_ <= seg.profile.duration + 1e-6) {
        break;
      }
      traj_elapsed_ -= seg.profile.duration;
      ++current_segment_index_;
    }

    if (current_segment_index_ >= segments_.size()) {
      traj_active_ = false;
      desired_position_cnt_ = target_position_cnt_;
      desired_slew_cnt_ = target_position_cnt_;
      desired_velocity_cnt_.fill(0.0);
      return;
    }

    traj_active_ = true;
    const auto &seg = segments_[current_segment_index_];
    const auto &profile = seg.profile;

    const double t = std::clamp(traj_elapsed_, 0.0, profile.duration);
    double sigma = 0.0;
    double sigma_dot = 0.0;

    if (profile.distance < 1e-6) {
      sigma = 0.0;
      sigma_dot = 0.0;
    } else {
      const double a_acc = profile.t_acc > 0.0 ? (profile.v_peak - profile.v_start) / profile.t_acc : 0.0;
      const double a_dec = profile.t_dec > 0.0 ? (profile.v_peak - profile.v_end) / profile.t_dec : 0.0;
      const double d_acc = 0.5 * (profile.v_start + profile.v_peak) * profile.t_acc;

      if (t < profile.t_acc) {
        sigma = profile.v_start * t + 0.5 * a_acc * t * t;
        sigma_dot = profile.v_start + a_acc * t;
      } else if (t < profile.t_acc + profile.t_flat) {
        const double t_flat = t - profile.t_acc;
        sigma = d_acc + profile.v_peak * t_flat;
        sigma_dot = profile.v_peak;
      } else {
        const double t_dec = t - (profile.t_acc + profile.t_flat);
        const double d_dec = 0.5 * (profile.v_peak + profile.v_end) * profile.t_dec;
        sigma = profile.distance - (d_dec - (profile.v_peak * t_dec - 0.5 * a_dec * t_dec * t_dec));
        sigma_dot = profile.v_peak - a_dec * t_dec;
      }
    }

    const double s = profile.distance > 1e-9 ? sigma / profile.distance : 1.0;
    const double s_dot = profile.distance > 1e-9 ? sigma_dot / profile.distance : 0.0;

    Eigen::Vector4d q_ref = seg.start + seg.delta * s;
    Eigen::Vector4d qd_ref = seg.delta * s_dot;

    for (size_t i = 0; i < 4; ++i) {
      desired_slew_cnt_[i] = q_ref(i);
      desired_velocity_cnt_[i] = qd_ref(i);
    }

    desired_position_cnt_ = target_position_cnt_;
  }

  void update_mode(const Time &now_time)
  {
    const double max_ref_vel = *std::max_element(desired_velocity_cnt_.begin(), desired_velocity_cnt_.end(),
      [](double a, double b) { return std::fabs(a) < std::fabs(b); });

    const double max_meas_vel = *std::max_element(velocity_filtered_.begin(), velocity_filtered_.end(),
      [](double a, double b) { return std::fabs(a) < std::fabs(b); });

    const bool moving_ref = traj_active_ || std::fabs(max_ref_vel) > tunables_.v_ref_eps;
    const bool moving_meas = std::fabs(max_meas_vel) > tunables_.v_meas_eps;

    const Mode new_mode = (moving_ref || moving_meas) ? Mode::MOVE : Mode::HOLD;
    if (new_mode != mode_) {
      mode_ = new_mode;
      integrator_freeze_until_ = now_time + Duration::from_seconds(tunables_.integral_freeze_sec);
      if (mode_ == Mode::MOVE) {
        bias_ff_.fill(0.0);
      }
    }
  }

  void apply_control(double dt, const Time &now_time)
  {
    Eigen::Vector4d tau_g = compute_gravity_torque();
    for (size_t i = 0; i < 4; ++i) {
      tau_g(i) *= k_gff_[i];
    }

    group_sync_write_->clearParam();

    const bool integrator_locked = now_time < integrator_freeze_until_;

    for (size_t axis = 0; axis < 4; ++axis) {
      const double pos_meas = static_cast<double>(present_position_[axis]);
      const double pos_ref = desired_slew_cnt_[axis];
      double error = pos_ref - pos_meas;

      if (mode_ == Mode::HOLD && std::fabs(error) <= tunables_.hold_deadband) {
        error = 0.0;
      }

      const double vel_meas = velocity_filtered_[axis];
      const double d_term = std::clamp(-static_cast<double>(tunables_.kd[axis]) * vel_meas,
                                       -tunables_.dterm_limit,
                                       tunables_.dterm_limit);

      const double ff_raw = torque_to_current_raw(tau_g(axis), axis);
      const double pre = tunables_.kp[axis] * error + d_term + ff_raw + bias_ff_[axis];

      const double current_limit = static_cast<double>(tunables_.current_limit[axis]);
      const double cmd_clamped = std::clamp(pre, -current_limit, current_limit);

      if (!integrator_locked) {
        const double sat_error = cmd_clamped - pre;
        integral_error_[axis] += error * dt + tunables_.aw_gain * sat_error * dt;
        integral_error_[axis] = std::clamp(integral_error_[axis],
                                           -tunables_.integral_limit,
                                           tunables_.integral_limit);
      }

      double ki_eff = tunables_.ki[axis];
      if (mode_ == Mode::HOLD) {
        ki_eff *= tunables_.hold_ki_scale;
        const double leak = std::clamp(tunables_.hold_i_leak_rate * dt, 0.0, 1.0);
        integral_error_[axis] *= (1.0 - leak);
        bias_ff_[axis] += tunables_.bias_alpha * error * dt;
        bias_ff_[axis] = std::clamp(bias_ff_[axis], -tunables_.bias_max, tunables_.bias_max);
      }

      const double cmd = std::clamp(tunables_.kp[axis] * error + ki_eff * integral_error_[axis]
                                     + d_term + ff_raw + bias_ff_[axis],
                                     -current_limit, current_limit);

      const int16_t cmd_raw = static_cast<int16_t>(std::lround(cmd));
      goal_current_bufs_[axis][0] = static_cast<uint8_t>(cmd_raw & 0xFF);
      goal_current_bufs_[axis][1] = static_cast<uint8_t>((cmd_raw >> 8) & 0xFF);

      if (!group_sync_write_->addParam(dxl_ids_[axis], goal_current_bufs_[axis].data())) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "SyncWrite addParam 실패 (ID=%u)", dxl_ids_[axis]);
      }
    }

    // 5번 축: 단순 PID (중력 보상 없음)
    {
      const double error = static_cast<double>(dxl5_target_cnt_) - static_cast<double>(present_position_[4]);
      if (!integrator_locked) {
        integral_dxl5_ += error * dt;
        integral_dxl5_ = std::clamp(integral_dxl5_, -tunables_.integral_limit, tunables_.integral_limit);
      }
      const double d_term = static_cast<double>(tunables_.kd_dxl5) * velocity_filtered_[4];
      const double cmd = std::clamp(tunables_.kp_dxl5 * error + tunables_.ki_dxl5 * integral_dxl5_ - d_term,
                                    -static_cast<double>(tunables_.current_limit[4]),
                                    static_cast<double>(tunables_.current_limit[4]));
      const int16_t cmd_raw = static_cast<int16_t>(std::lround(cmd));
      goal_current_bufs_[4][0] = static_cast<uint8_t>(cmd_raw & 0xFF);
      goal_current_bufs_[4][1] = static_cast<uint8_t>((cmd_raw >> 8) & 0xFF);
      group_sync_write_->addParam(dxl_ids_[4], goal_current_bufs_[4].data());
    }

    const int comm = group_sync_write_->txPacket();
    if (comm != COMM_SUCCESS) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "SyncWrite 실패: %s",
                            packet_handler_->getTxRxResult(comm));
      if (++consecutive_write_fail_ >= watchdog_limit_) {
        watchdog_trip("SyncWrite 실패 누적");
      }
    } else {
      consecutive_write_fail_ = 0;
    }

  }

  Eigen::Vector4d compute_gravity_torque() const
  {
    std::array<double, 4> q_rad{};
    for (size_t i = 0; i < 4; ++i) {
      const double count = static_cast<double>(present_position_[i]) - zero_count_[i];
      q_rad[i] = sign_[i] * count * kCountToRad;
    }

    const double q1 = q_rad[0];
    const double q2 = q_rad[1] + dh_params_.theta2_offset;
    const double q3 = q_rad[2];
    const double q4 = q_rad[3];

    Matrix4d T01 = dh(q1, dh_params_.d1, 0.0, dh_params_.alpha1);
    Matrix4d T12 = dh(q2, 0.0, dh_params_.a2, 0.0);
    Matrix4d T23 = dh(q3, 0.0, dh_params_.a3, 0.0);
    Matrix4d T34 = dh(q4, 0.0, dh_params_.a4, 0.0);

    Matrix4d T02 = T01 * T12;
    Matrix4d T03 = T02 * T23;
    Matrix4d T04 = T03 * T34;

    const Vector3d p0 = Vector3d::Zero();
    const Vector3d z0 = Vector3d::UnitZ();
    const Vector3d p1 = T01.block<3, 1>(0, 3);
    const Vector3d z1 = T01.block<3, 1>(0, 2);
    const Vector3d p2 = T02.block<3, 1>(0, 3);
    const Vector3d z2 = T02.block<3, 1>(0, 2);
    const Vector3d p3 = T03.block<3, 1>(0, 3);
    const Vector3d z3 = T03.block<3, 1>(0, 2);

    const auto com_to_world = [&](size_t idx, const Matrix4d &T) -> Vector3d {
      Vector4d local(link_data_[idx].com.x(), link_data_[idx].com.y(), link_data_[idx].com.z(), 1.0);
      return (T * local).head<3>();
    };

    const Vector3d c1 = com_to_world(0, T01);
    const Vector3d c2 = com_to_world(1, T02);
    const Vector3d c3 = com_to_world(2, T03);
    const Vector3d c4 = com_to_world(3, T04);

    Eigen::Matrix<double, 3, 4> Jv1 = Eigen::Matrix<double, 3, 4>::Zero();
    Eigen::Matrix<double, 3, 4> Jv2 = Eigen::Matrix<double, 3, 4>::Zero();
    Eigen::Matrix<double, 3, 4> Jv3 = Eigen::Matrix<double, 3, 4>::Zero();
    Eigen::Matrix<double, 3, 4> Jv4 = Eigen::Matrix<double, 3, 4>::Zero();

    Jv1.col(0) = z0.cross(c1 - p0);

    Jv2.col(0) = z0.cross(c2 - p0);
    Jv2.col(1) = z1.cross(c2 - p1);

    Jv3.col(0) = z0.cross(c3 - p0);
    Jv3.col(1) = z1.cross(c3 - p1);
    Jv3.col(2) = z2.cross(c3 - p2);

    Jv4.col(0) = z0.cross(c4 - p0);
    Jv4.col(1) = z1.cross(c4 - p1);
    Jv4.col(2) = z2.cross(c4 - p2);
    Jv4.col(3) = z3.cross(c4 - p3);

    Eigen::Vector4d tau = Eigen::Vector4d::Zero();
    tau += Jv1.transpose() * (link_data_[0].mass * dh_params_.gravity);
    tau += Jv2.transpose() * (link_data_[1].mass * dh_params_.gravity);
    tau += Jv3.transpose() * (link_data_[2].mass * dh_params_.gravity);
    tau += Jv4.transpose() * (link_data_[3].mass * dh_params_.gravity);
    return tau;
  }

  double torque_to_current_raw(double torque_Nm, size_t axis) const
  {
    const double motor_current_A = torque_Nm / (Kt_[axis] * N_[axis] * eta_[axis] + 1e-9);
    return motor_current_A / kCurrentPerCount;
  }

  void publish_feedback()
  {
    std_msgs::msg::Float32MultiArray current_msg;
    std_msgs::msg::Float32MultiArray velocity_msg;
    std_msgs::msg::Int32MultiArray position_msg;

    current_msg.data.reserve(dxl_ids_.size());
    velocity_msg.data.reserve(dxl_ids_.size());
    position_msg.data.reserve(dxl_ids_.size());

    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      current_msg.data.push_back(static_cast<float>(present_current_raw_[i] * kCurrentPerCount));
      velocity_msg.data.push_back(static_cast<float>(velocity_filtered_[i]));
      position_msg.data.push_back(present_position_[i]);
    }

    current_pub_->publish(current_msg);
    velocity_pub_->publish(velocity_msg);
    position_pub_->publish(position_msg);
  }

  void path_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty() || msg->data.size() % 4 != 0) {
      RCLCPP_WARN(get_logger(), "경로 데이터의 크기가 4의 배수가 아닙니다");
      return;
    }

    std::vector<Eigen::Vector4d> waypoints;
    waypoints.reserve(msg->data.size() / 4 + 1);

    Eigen::Vector4d start;
    for (size_t i = 0; i < 4; ++i) {
      start(i) = desired_slew_cnt_[i];
    }
    waypoints.push_back(start);

    for (size_t i = 0; i < msg->data.size(); i += 4) {
      Eigen::Vector4d wp;
      for (size_t axis = 0; axis < 4; ++axis) {
        wp(static_cast<Eigen::Index>(axis)) = static_cast<double>(msg->data[i + axis]);
      }
      waypoints.push_back(wp);
    }

    construct_trajectory(waypoints);
  }

  void position_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN(get_logger(), "단일 목표 길이가 4 미만입니다");
      return;
    }

    std::vector<Eigen::Vector4d> waypoints;
    waypoints.reserve(2);

    Eigen::Vector4d start;
    for (size_t i = 0; i < 4; ++i) {
      start(i) = desired_slew_cnt_[i];
    }
    waypoints.push_back(start);

    Eigen::Vector4d goal;
    for (size_t i = 0; i < 4; ++i) {
      goal(i) = static_cast<double>(msg->data[i]);
    }
    waypoints.push_back(goal);

    construct_trajectory(waypoints);
  }

  void dxl5_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "DXL5 명령이 비어 있습니다");
      return;
    }
    dxl5_target_cnt_ = msg->data[0];
  }

  void construct_trajectory(const std::vector<Eigen::Vector4d> &waypoints)
  {
    if (waypoints.size() < 2) {
      return;
    }

    segments_.clear();
    segments_.reserve(waypoints.size() - 1);

    std::vector<double> distances;
    distances.reserve(waypoints.size() - 1);

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      const Eigen::Vector4d delta = waypoints[i + 1] - waypoints[i];
      const double dist = delta.cwiseAbs().maxCoeff();
      distances.push_back(dist);
    }

    const double v_corner_max = tunables_.corner_ratio * tunables_.v_max_cnt_s;

    std::vector<double> corner_speeds(waypoints.size(), 0.0);
    for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
      const double d_prev = distances[i - 1];
      const double d_next = distances[i];
      const double d_corner = std::min(d_prev, d_next);
      double v_corner = std::sqrt(std::max(0.0, d_corner * tunables_.a_max_cnt_s2));
      v_corner = std::min(v_corner, v_corner_max);
      corner_speeds[i] = v_corner;
    }

    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      TrajectorySegment seg;
      seg.start = waypoints[i];
      seg.goal = waypoints[i + 1];
      seg.delta = seg.goal - seg.start;
      seg.profile.distance = distances[i];
      seg.profile.v_start = corner_speeds[i];
      seg.profile.v_end = corner_speeds[i + 1];
      seg.profile = build_segment_profile(seg.profile);
      segments_.push_back(seg);
    }

    current_segment_index_ = 0;
    traj_elapsed_ = 0.0;
    traj_active_ = true;

    for (size_t i = 0; i < 4; ++i) {
      target_position_cnt_[i] = waypoints.back()(static_cast<Eigen::Index>(i));
    }
  }

  SegmentProfile build_segment_profile(SegmentProfile profile) const
  {
    const double dist = profile.distance;
    const double v0 = profile.v_start;
    const double vf = profile.v_end;
    const double v_max = tunables_.v_max_cnt_s;
    const double a = tunables_.a_max_cnt_s2;

    if (dist < 1e-6) {
      profile.v_peak = v0;
      profile.t_acc = 0.0;
      profile.t_flat = 0.0;
      profile.t_dec = 0.0;
      profile.duration = 0.0;
      return profile;
    }

    double v_peak = v_max;
    double t_acc = std::max(0.0, (v_peak - v0) / a);
    double t_dec = std::max(0.0, (v_peak - vf) / a);
    double d_acc = 0.5 * (v0 + v_peak) * t_acc;
    double d_dec = 0.5 * (v_peak + vf) * t_dec;

    if (d_acc + d_dec > dist) {
      const double term = std::max(0.0, a * dist + 0.5 * (v0 * v0 + vf * vf));
      v_peak = std::sqrt(term);
      v_peak = std::min(v_peak, v_max);
      v_peak = std::max(v_peak, std::max(v0, vf));
      t_acc = v_peak > v0 ? (v_peak - v0) / a : 0.0;
      t_dec = v_peak > vf ? (v_peak - vf) / a : 0.0;
      d_acc = 0.5 * (v0 + v_peak) * t_acc;
      d_dec = 0.5 * (v_peak + vf) * t_dec;
    }

    double d_flat = dist - (d_acc + d_dec);
    if (d_flat < 0.0) {
      d_flat = 0.0;
    }
    const double t_flat = v_peak > 1e-6 ? d_flat / v_peak : 0.0;

    profile.v_peak = v_peak;
    profile.t_acc = t_acc;
    profile.t_flat = t_flat;
    profile.t_dec = t_dec;
    profile.duration = t_acc + t_flat + t_dec;

    return profile;
  }

  void watchdog_trip(const std::string &reason)
  {
    if (watchdog_tripped_) {
      return;
    }
    watchdog_tripped_ = true;
    RCLCPP_ERROR(get_logger(), "워치독 발동: %s", reason.c_str());
    for (auto id : dxl_ids_) {
      packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, 0);
    }
  }

private:
  Tunables tunables_;
  DHParameters dh_params_;
  std::string config_path_;

  std::array<LinkInertialData, 4> link_data_{};
  std::array<double, 4> k_gff_{1.0, 1.0, 1.0, 1.0};

  std::array<double, 4> N_{};
  std::array<double, 4> eta_{};
  std::array<double, 4> Kt_{};

  std::array<int32_t, 4> zero_count_{0, 0, 0, 0};
  std::array<int32_t, 4> sign_{1, 1, 1, 1};

  const std::array<uint8_t, 5> dxl_ids_{1, 2, 3, 4, 5};

  static constexpr uint16_t ADDR_OPERATING_MODE = 11;
  static constexpr uint8_t CURRENT_CONTROL_MODE = 0;
  static constexpr uint16_t ADDR_CURRENT_LIMIT = 38;
  static constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
  static constexpr uint16_t ADDR_GOAL_CURRENT = 102;
  static constexpr uint16_t ADDR_PRESENT_CURRENT = 126;
  static constexpr uint16_t ADDR_PRESENT_VELOCITY = 128;
  static constexpr uint16_t ADDR_PRESENT_POSITION = 132;
  static constexpr uint16_t LEN_PRESENT_CURRENT = 2;
  static constexpr uint16_t LEN_PRESENT_VELOCITY = 4;
  static constexpr uint16_t LEN_PRESENT_POSITION = 4;
  static constexpr uint8_t TORQUE_ENABLE = 1;

  dynamixel::PortHandler *port_handler_;
  dynamixel::PacketHandler *packet_handler_;
  std::shared_ptr<dynamixel::GroupBulkRead> group_bulk_read_;
  std::shared_ptr<dynamixel::GroupSyncWrite> group_sync_write_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr position_pub_;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr dxl5_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::array<int32_t, 5> present_position_{};
  std::array<double, 5> present_velocity_raw_{};
  std::array<double, 5> present_current_raw_{};
  std::array<double, 5> velocity_filtered_{};

  std::array<double, 4> desired_position_cnt_{};
  std::array<double, 4> desired_slew_cnt_{};
  std::array<double, 4> desired_velocity_cnt_{};
  std::array<double, 4> target_position_cnt_{};
  std::array<double, 4> integral_error_{};
  std::array<double, 5> bias_ff_{};
  std::array<std::array<uint8_t, 2>, 5> goal_current_bufs_{};

  int dxl5_target_cnt_{0};
  double integral_dxl5_{0.0};

  std::vector<TrajectorySegment> segments_;
  size_t current_segment_index_{0};
  double traj_elapsed_{0.0};
  bool traj_active_{false};

  Mode mode_{Mode::HOLD};
  Time integrator_freeze_until_;
  Time last_tick_time_;

  int consecutive_read_fail_{0};
  int consecutive_write_fail_{0};
  static constexpr int watchdog_limit_ = 3;
  bool watchdog_tripped_{false};

};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TestMotorConnectNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("test_motor_connect"), "노드 초기화 실패: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
