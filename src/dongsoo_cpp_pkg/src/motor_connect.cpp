#include <chrono>
#include <memory>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>
#include <signal.h>
#include <fstream>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <Eigen/Dense>
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using namespace std::chrono_literals;
using dynamixel::PortHandler;
using dynamixel::PacketHandler;

static inline Matrix4d dh(double th, double d, double a, double al) {
  Matrix4d T = Matrix4d::Identity();
  const double cth = std::cos(th), sth = std::sin(th);
  const double cal = std::cos(al), sal = std::sin(al);
  T <<  cth, -sth*cal,  sth*sal, a*cth,
        sth,  cth*cal, -cth*sal, a*sth,
          0,       sal,      cal,     d,
          0,         0,        0,     1;
  return T;
}

using json = nlohmann::json;

struct DHParameters {
  double d1, a2, a3, a4, alpha1, theta2_offset;
  Vector3d gravity_vector;
};

struct LinkInertialData {
  double mass;
  Vector3d com;
};

DHParameters loadDHParameters(const std::string& config_path) {
  std::ifstream file(config_path + "/gravity_dh_param.json");
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open gravity_dh_param.json");
  }

  json j;
  file >> j;

  DHParameters params;

  const auto& joints = j["gravity_dh_parameters"]["joints"];

  for (const auto& joint : joints) {
    int motor_id = joint["motor_id"];
    const auto& dh = joint["dh_params"];

    if (motor_id == 1) {
      params.d1 = dh["d"];
      params.alpha1 = dh["alpha"];
    } else if (motor_id == 2) {
      params.a2 = dh["a"];
      if (dh.contains("theta_offset")) {
        params.theta2_offset = dh["theta_offset"];
      }
    } else if (motor_id == 3) {
      params.a3 = dh["a"];
    } else if (motor_id == 4) {
      params.a4 = dh["a"];
    }
  }

  const auto& gvec = j["gravity_dh_parameters"]["gravity_vector"];
  params.gravity_vector.x() = gvec["x"];
  params.gravity_vector.y() = gvec["y"];
  params.gravity_vector.z() = gvec["z"];

  return params;
}

std::vector<LinkInertialData> loadLinkInertialData(const std::string& config_path) {
  std::ifstream file(config_path + "/link_inertial.json");
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open link_inertial.json");
  }

  json j;
  file >> j;

  std::vector<LinkInertialData> links;

  for (const auto& link : j["links"]) {
    LinkInertialData link_data;
    link_data.mass = link["inertial"]["mass"];

    const auto& com = link["inertial"]["center_of_mass"];
    link_data.com.x() = com["x"];
    link_data.com.y() = com["y"];
    link_data.com.z() = com["z"];

    links.push_back(link_data);
  }

  return links;
}

class TestDxlCurrentNode : public rclcpp::Node
{
public:
  TestDxlCurrentNode()
  : Node("test_dxl_current_node"),
    device_name_("/dev/ttyUSB0"),
    protocol_version_(2.0),
    dxl_ids_{1, 2, 3, 4, 5},

    ADDR_OPERATING_MODE(11), CURRENT_CONTROL_MODE(0),
    ADDR_TORQUE_ENABLE(64), TORQUE_ENABLE(1),
    ADDR_PRESENT_CURRENT(126), LEN_PRESENT_CURRENT(2),
    ADDR_PRESENT_VELOCITY(128), LEN_PRESENT_VELOCITY(4),
    ADDR_PRESENT_POSITION(132), LEN_PRESENT_POSITION(4),
    ADDR_GOAL_CURRENT(102),
    ADDR_CURRENT_LIMIT(38), LEN_CURRENT_LIMIT(2),

    KP_POS_GAINS({1.0f, 1.8f, 1.8f, 1.6f}),
    KI_POS_GAINS({0.00f, 0.05f, 0.05f, 0.01f}),
    KD_POS_GAINS({0.02f, 0.015f, 0.02f, 0.005f}),

    CURRENT_LIMIT_SOFT(1200),

    MAX_INTEGRAL_ERROR(2000.0f),

    dt(0.005f),

    K_GFF({1.2, 1.6, 1.55, 1.5})
  {
    try {
      std::string config_path = "/home/pc/soomac_ws/src/dongsoo_description/config";

      DHParameters dh_params = loadDHParameters(config_path);
      d1_ = dh_params.d1;
      a2_ = dh_params.a2;
      a3_ = dh_params.a3;
      a4_ = dh_params.a4;
      alpha1_ = dh_params.alpha1;
      theta2_offset_ = dh_params.theta2_offset;
      gvec_ = dh_params.gravity_vector;

      std::vector<LinkInertialData> link_data = loadLinkInertialData(config_path);
      for (size_t i = 0; i < 4 && i < link_data.size(); ++i) {
        link_[i].m = link_data[i].mass;
        link_[i].com = link_data[i].com;
      }

      RCLCPP_INFO(get_logger(), "JSON 설정 파일 로드 완료");

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "JSON 설정 파일 로드 실패: %s", e.what());
      rclcpp::shutdown();
      return;
    }
    port_handler_   = PortHandler::getPortHandler(device_name_.c_str());
    packet_handler_ = PacketHandler::getPacketHandler(protocol_version_);

    group_bulk_read_ = std::make_shared<dynamixel::GroupBulkRead>(port_handler_, packet_handler_);
    group_sync_write_ = std::make_shared<dynamixel::GroupSyncWrite>(port_handler_, packet_handler_, ADDR_GOAL_CURRENT, 2);

    if (!port_handler_->openPort() || !port_handler_->setBaudRate(3000000)) {
      RCLCPP_ERROR(get_logger(), "포트 열기/baudrate 실패");
      rclcpp::shutdown();
      return;
    }

    POS_COUNT_PER_REV_ = 4096.0;
    COUNT2RAD_ = 2.0 * M_PI / POS_COUNT_PER_REV_;

    N_ = {272.5, 272.5, 272.5, 353.5};
    eta_ = {0.80, 0.70, 0.60, 0.70};
    Kt_[0] = 9.2 / (272.5 * eta_[0] * 2.4);
    Kt_[1] = 9.2 / (272.5 * eta_[1] * 2.4);
    Kt_[2] = 9.2 / (272.5 * eta_[2] * 2.4);
    Kt_[3] = 4.1 / (353.5 * eta_[3] * 1.18);

    const uint16_t addr_block = ADDR_PRESENT_CURRENT;
    const uint16_t len_block  = LEN_PRESENT_CURRENT + LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;

    for (auto id : dxl_ids_) {
      int comm = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
      if (comm != COMM_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "ID %u OperatingMode 설정 실패: %s", id, packet_handler_->getTxRxResult(comm));
      }

      uint16_t limit_raw;
      if (id == 4) {
        limit_raw = static_cast<uint16_t>(std::clamp(600, 0, 689));
      } else {
        limit_raw = static_cast<uint16_t>(std::clamp(1000, 0, 1188));
      }
      comm = packet_handler_->write2ByteTxRx(port_handler_, id, ADDR_CURRENT_LIMIT, limit_raw);
      if (comm != COMM_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "ID %u CurrentLimit(38) 설정 실패: %s", id, packet_handler_->getTxRxResult(comm));
      }

      comm = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
      if (comm != COMM_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "ID %u TorqueEnable 실패: %s", id, packet_handler_->getTxRxResult(comm));
      }

      if (!group_bulk_read_->addParam(id, addr_block, len_block)) {
        RCLCPP_ERROR(get_logger(), "ID %u BulkRead 등록 실패", id);
      }
    }

    publisher_current_  = create_publisher<std_msgs::msg::Float32MultiArray>("motor/current", 10);
    publisher_velocity_ = create_publisher<std_msgs::msg::Float32MultiArray>("motor/velocity", 10);
    publisher_position_ = create_publisher<std_msgs::msg::Int32MultiArray>("motor/position", 10);

    desired_pos_.resize(4);
    desired_pos_slew_.resize(4);
    last_pos_error_.assign(4, 0.0f);
    last_pos_count_meas_.assign(4, 0.0f);
    integral_error_.assign(4, 0.0f);

    desired_dxl5_pos_ = 0.0f;
    last_dxl5_pos_error_ = 0.0f;
    integral_dxl5_error_ = 0.0f;

    goal_current_bufs_.resize(dxl_ids_.size());
    for (auto &b : goal_current_bufs_) b = {0, 0};

    group_bulk_read_->txRxPacket();
    for (size_t i = 0; i < 4; ++i) {
      uint32_t raw_p = group_bulk_read_->getData(dxl_ids_[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      int32_t sp = static_cast<int32_t>(raw_p);
      if (sp & 0x80000000) sp -= 0x100000000;
      desired_pos_[i] = static_cast<float>(sp);
      desired_pos_slew_[i] = static_cast<float>(sp);
      last_pos_count_meas_[i] = static_cast<float>(sp);
    }

    uint32_t raw_p5 = group_bulk_read_->getData(5, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    int32_t sp5 = static_cast<int32_t>(raw_p5);
    if (sp5 & 0x80000000) sp5 -= 0x100000000;
    desired_dxl5_pos_ = static_cast<float>(sp5);

    subscription_position_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "motor/command_position", 10,
      std::bind(&TestDxlCurrentNode::positionCallback, this, std::placeholders::_1)
    );

    subscription_dxl5_position_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "motor/command_dxl5_position", 10,
      std::bind(&TestDxlCurrentNode::dxl5PositionCallback, this, std::placeholders::_1)
    );

    timer_ = create_wall_timer(std::chrono::duration<double>(dt),
                               std::bind(&TestDxlCurrentNode::readAndPublish, this));

    RCLCPP_INFO(get_logger(), ":: Test Motor Connected! (개선된 버전) ::");
  }

  ~TestDxlCurrentNode() {
    RCLCPP_INFO(get_logger(), "모터 안전 종료 중...");
    if (port_handler_ && packet_handler_) {
      for (auto id : dxl_ids_) {
        int comm = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_TORQUE_ENABLE, 0);
        if (comm != COMM_SUCCESS) {
          RCLCPP_WARN(get_logger(), "ID %u 토크 비활성화 실패: %s", id, packet_handler_->getTxRxResult(comm));
        } else {
          RCLCPP_INFO(get_logger(), "ID %u 토크 비활성화 완료", id);
        }
      }
      port_handler_->closePort();
    }
    RCLCPP_INFO(get_logger(), "모터 안전 종료 완료");
  }

private:
  double d1_, a2_, a3_, a4_, alpha1_, theta2_offset_;

  struct LinkInertial { double m; Vector3d com; };
  std::array<LinkInertial,4> link_;

  Vector3d gvec_;

  double POS_COUNT_PER_REV_ = 4096.0;
  double COUNT2RAD_ = 2.0 * M_PI / 4096.0;

  std::array<int,4>      sign_{ {+1, +1, +1, +1} };
  std::array<int32_t,4>  zero_count_{ {0, 0, 0, 0} };

  std::array<double,4> A_per_count_{ {0.00269, 0.00269, 0.00269, 0.00269} };
  std::array<double,4> N_{ {272.5, 272.5, 272.5, 353.5} };
  std::array<double,4> eta_{ {0.70, 0.70, 0.70, 0.70} };
  std::array<double,4> Kt_;

  Eigen::Vector4d computeGravityTorqueNm(const std::array<double,4>& q_rad) {
    Matrix4d T01 = dh(q_rad[0], d1_, 0.0, alpha1_);
    Matrix4d T12 = dh(q_rad[1] + theta2_offset_, 0.0, a2_, 0.0);
    Matrix4d T23 = dh(q_rad[2], 0.0, a3_, 0.0);
    Matrix4d T34 = dh(q_rad[3], 0.0, a4_, 0.0);
    Matrix4d T02 = T01 * T12;
    Matrix4d T03 = T02 * T23;
    Matrix4d T04 = T03 * T34;

    Vector3d p0 = Vector3d::Zero();
    Vector3d z0 = Vector3d(0,0,1);
    Vector3d p1 = T01.block<3,1>(0,3);
    Vector3d z1 = T01.block<3,1>(0,2);
    Vector3d p2 = T02.block<3,1>(0,3);
    Vector3d z2 = T02.block<3,1>(0,2);
    Vector3d p3 = T03.block<3,1>(0,3);
    Vector3d z3 = T03.block<3,1>(0,2);

    Vector4d c1_h(link_[0].com.x(), link_[0].com.y(), link_[0].com.z(), 1.0);
    Vector4d c2_h(link_[1].com.x(), link_[1].com.y(), link_[1].com.z(), 1.0);
    Vector4d c3_h(link_[2].com.x(), link_[2].com.y(), link_[2].com.z(), 1.0);
    Vector4d c4_h(link_[3].com.x(), link_[3].com.y(), link_[3].com.z(), 1.0);
    Vector3d pc1 = (T01 * c1_h).head<3>();
    Vector3d pc2 = (T02 * c2_h).head<3>();
    Vector3d pc3 = (T03 * c3_h).head<3>();
    Vector3d pc4 = (T04 * c4_h).head<3>();

    Eigen::Matrix<double,3,4> Jv1 = Eigen::Matrix<double,3,4>::Zero();
    Jv1.col(0) = z0.cross(pc1 - p0);

    Eigen::Matrix<double,3,4> Jv2 = Eigen::Matrix<double,3,4>::Zero();
    Jv2.col(0) = z0.cross(pc2 - p0);
    Jv2.col(1) = z1.cross(pc2 - p1);

    Eigen::Matrix<double,3,4> Jv3 = Eigen::Matrix<double,3,4>::Zero();
    Jv3.col(0) = z0.cross(pc3 - p0);
    Jv3.col(1) = z1.cross(pc3 - p1);
    Jv3.col(2) = z2.cross(pc3 - p2);

    Eigen::Matrix<double,3,4> Jv4 = Eigen::Matrix<double,3,4>::Zero();
    Jv4.col(0) = z0.cross(pc4 - p0);
    Jv4.col(1) = z1.cross(pc4 - p1);
    Jv4.col(2) = z2.cross(pc4 - p2);
    Jv4.col(3) = z3.cross(pc4 - p3);

    Vector4d tau = Vector4d::Zero();
    tau += Jv1.transpose() * (link_[0].m * gvec_);
    tau += Jv2.transpose() * (link_[1].m * gvec_);
    tau += Jv3.transpose() * (link_[2].m * gvec_);
    tau += Jv4.transpose() * (link_[3].m * gvec_);
    return tau;
  }

  inline int32_t torqueNm_to_currentRaw(double tau, int axis) const {
    const double I   = tau / (Kt_[axis] * N_[axis] * eta_[axis]);
    const double raw = I / A_per_count_[axis];
    return static_cast<int32_t>(std::lround(raw));
  }

  void positionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 4) {
      for (size_t i = 0; i < 4; ++i) {
        float new_target = static_cast<float>(msg->data[i]);
        if (std::fabs(new_target - desired_pos_slew_[i]) > 200.0f) {
          integral_error_[i] = 0.0f;
          last_pos_count_meas_[i] = 0.0f;
          RCLCPP_INFO(get_logger(), "큰 목표 변경 감지 - 축 %zu I항 리셋", i);
        }
        desired_pos_[i] = new_target;
      }
      RCLCPP_INFO(get_logger(), "Position command received (DoM + 램핑 + AW + I리셋)");
    }
  }

  void dxl5PositionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 1) {
      desired_dxl5_pos_ = static_cast<float>(msg->data[0]);
      RCLCPP_INFO(get_logger(), "DXL5 Position command received: %d count", msg->data[0]);
    }
  }

  void readAndPublish() {
    std_msgs::msg::Float32MultiArray msg_cur, msg_vel;
    std_msgs::msg::Int32MultiArray msg_pos;

    int comm = group_bulk_read_->txRxPacket();
    if (comm != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Bulk Read 실패: %s", packet_handler_->getTxRxResult(comm));
      return;
    }

    std::vector<float> pos_count(dxl_ids_.size(), 0.0f);
    std::vector<float> vel_count(dxl_ids_.size(), 0.0f);

    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      const auto id = dxl_ids_[i];

      uint32_t raw_c = group_bulk_read_->getData(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
      float current_raw = static_cast<float>(static_cast<int16_t>(raw_c & 0xFFFF));
      msg_cur.data.push_back(current_raw);

      uint32_t raw_v = group_bulk_read_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
      int32_t sv = static_cast<int32_t>(raw_v);
      if (sv & 0x80000000) sv -= 0x100000000;
      float vel_raw = static_cast<float>(sv);
      msg_vel.data.push_back(vel_raw);
      vel_count[i] = vel_raw;

      uint32_t raw_p = group_bulk_read_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      int32_t sp = static_cast<int32_t>(raw_p);
      if (sp & 0x80000000) sp -= 0x100000000;
      msg_pos.data.push_back(sp);
      pos_count[i] = static_cast<float>(sp);
    }

    const float V_MAX_CNT_PER_SEC = 3000.0f;
    const float MAX_STEP = V_MAX_CNT_PER_SEC * dt;

    for (int i = 0; i < 4; ++i) {
      float err_d = desired_pos_[i] - desired_pos_slew_[i];
      float step = std::clamp(err_d, -MAX_STEP, +MAX_STEP);
      desired_pos_slew_[i] += step;
    }

    std::array<double,4> q_rad = {
      sign_[0] * ( (static_cast<int32_t>(pos_count[0]) - zero_count_[0]) * COUNT2RAD_ ),
      sign_[1] * ( (static_cast<int32_t>(pos_count[1]) - zero_count_[1]) * COUNT2RAD_ ),
      sign_[2] * ( (static_cast<int32_t>(pos_count[2]) - zero_count_[2]) * COUNT2RAD_ ),
      sign_[3] * ( (static_cast<int32_t>(pos_count[3]) - zero_count_[3]) * COUNT2RAD_ )
    };

    Eigen::Vector4d tau_g = computeGravityTorqueNm(q_rad);

    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
      RCLCPP_INFO(get_logger(), "=== 개선된 제어 상태 모니터링 ===");
      RCLCPP_INFO(get_logger(), "DH Params - d1:%.3f, a2:%.3f, a3:%.3f, a4:%.3f",
                  d1_, a2_, a3_, a4_);
      RCLCPP_INFO(get_logger(), "Gravity Vec - [%.3f, %.3f, %.3f]",
                  gvec_.x(), gvec_.y(), gvec_.z());
      RCLCPP_INFO(get_logger(), "Joint Angles[rad] - [%.3f, %.3f, %.3f, %.3f]",
                  q_rad[0], q_rad[1], q_rad[2], q_rad[3]);
      RCLCPP_INFO(get_logger(), "Slewed Targets[cnt] - [%.1f, %.1f, %.1f, %.1f]",
                  desired_pos_slew_[0], desired_pos_slew_[1], desired_pos_slew_[2], desired_pos_slew_[3]);
      RCLCPP_INFO(get_logger(), "Position Errors[cnt] - [%.1f, %.1f, %.1f, %.1f]",
                  desired_pos_slew_[0]-pos_count[0], desired_pos_slew_[1]-pos_count[1],
                  desired_pos_slew_[2]-pos_count[2], desired_pos_slew_[3]-pos_count[3]);
      last_log_time = now;
    }

    for (int i=0; i<4; ++i) tau_g[i] *= K_GFF[i];

    std::array<int32_t,4> tau_g_raw = {
      torqueNm_to_currentRaw(tau_g[0], 0),
      torqueNm_to_currentRaw(tau_g[1], 1),
      torqueNm_to_currentRaw(tau_g[2], 2),
      torqueNm_to_currentRaw(tau_g[3], 3)
    };

    group_sync_write_->clearParam();

    for (size_t i = 0; i < 4; ++i) {
      float pos_err = desired_pos_slew_[i] - pos_count[i];

      float vel_meas = (pos_count[i] - last_pos_count_meas_[i]) / dt;
      last_pos_count_meas_[i] = pos_count[i];
      float dterm = -KD_POS_GAINS[i] * vel_meas;

      const float D_LIMIT = 300.0f;
      dterm = std::clamp(dterm, -D_LIMIT, +D_LIMIT);

      float pid_raw = KP_POS_GAINS[i] * pos_err + dterm;

      float ff_raw = static_cast<float>(tau_g_raw[i]);
      float pre_sat_cmd = pid_raw + ff_raw;

      float limit;
      if (dxl_ids_[i] == 4) {
        limit = 500.0f;
      } else {
        limit = 900.0f;
      }

      float cmd_raw_clamped = std::clamp(pre_sat_cmd, -limit, +limit);

      const float K_AW = 0.05f;
      float saturation_error = cmd_raw_clamped - pre_sat_cmd;

      integral_error_[i] += pos_err * dt + K_AW * saturation_error * dt;
      integral_error_[i] = std::clamp(integral_error_[i], -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);

      if (std::fabs(pre_sat_cmd) > 0.95f * limit) {
        integral_error_[i] *= 0.99f;
      }

      float cmd_raw = pid_raw + KI_POS_GAINS[i] * integral_error_[i] + ff_raw;
      cmd_raw = std::clamp(cmd_raw, -limit, +limit);

      int32_t goal_current = static_cast<int32_t>(std::lround(cmd_raw));
      goal_current_bufs_[i][0] = static_cast<uint8_t>( goal_current & 0xFF);
      goal_current_bufs_[i][1] = static_cast<uint8_t>((goal_current >> 8) & 0xFF);

      if (!group_sync_write_->addParam(dxl_ids_[i], goal_current_bufs_[i].data())) {
        RCLCPP_ERROR(get_logger(), "ID %u SyncWrite addParam 실패", dxl_ids_[i]);
      }
    }

    float dxl5_pos_err = desired_dxl5_pos_ - pos_count[4];
    float dxl5_dpos = (dxl5_pos_err - last_dxl5_pos_error_) / dt;

    integral_dxl5_error_ += dxl5_pos_err * dt;
    integral_dxl5_error_ = std::clamp(integral_dxl5_error_, -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);

    last_dxl5_pos_error_ = dxl5_pos_err;

    float dxl5_cmd_raw = KP_DXL5_GAIN * dxl5_pos_err +
                         KI_DXL5_GAIN * integral_dxl5_error_ +
                         KD_DXL5_GAIN * dxl5_dpos;

    dxl5_cmd_raw = std::clamp(dxl5_cmd_raw, -900.0f, +900.0f);

    int32_t dxl5_goal_current = static_cast<int32_t>(std::lround(dxl5_cmd_raw));
    goal_current_bufs_[4][0] = static_cast<uint8_t>( dxl5_goal_current & 0xFF);
    goal_current_bufs_[4][1] = static_cast<uint8_t>((dxl5_goal_current >> 8) & 0xFF);

    if (!group_sync_write_->addParam(5, goal_current_bufs_[4].data())) {
      RCLCPP_ERROR(get_logger(), "ID 5 SyncWrite addParam 실패");
    }

    int wres = group_sync_write_->txPacket();
    if (wres != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Sync Write 실패: %s", packet_handler_->getTxRxResult(wres));
    }

    publisher_current_->publish(msg_cur);
    publisher_velocity_->publish(msg_vel);
    publisher_position_->publish(msg_pos);
  }

  std::string device_name_;
  double protocol_version_;
  std::vector<uint8_t> dxl_ids_;
  PortHandler* port_handler_ = nullptr;
  PacketHandler* packet_handler_ = nullptr;

  std::shared_ptr<dynamixel::GroupBulkRead>  group_bulk_read_;
  std::shared_ptr<dynamixel::GroupSyncWrite> group_sync_write_;

  std::vector<std::array<uint8_t,2>> goal_current_bufs_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_current_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_velocity_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_position_;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_position_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_dxl5_position_;
  rclcpp::TimerBase::SharedPtr timer_;

  const uint16_t ADDR_OPERATING_MODE;
  const uint8_t  CURRENT_CONTROL_MODE;
  const uint16_t ADDR_TORQUE_ENABLE;
  const uint8_t  TORQUE_ENABLE;
  const uint16_t ADDR_PRESENT_CURRENT;
  const uint16_t LEN_PRESENT_CURRENT;
  const uint16_t ADDR_PRESENT_VELOCITY;
  const uint16_t LEN_PRESENT_VELOCITY;
  const uint16_t ADDR_PRESENT_POSITION;
  const uint16_t LEN_PRESENT_POSITION;
  const uint16_t ADDR_GOAL_CURRENT;
  const uint16_t ADDR_CURRENT_LIMIT;
  const uint16_t LEN_CURRENT_LIMIT;

  const std::array<float,4> KP_POS_GAINS;
  const std::array<float,4> KI_POS_GAINS;
  const std::array<float,4> KD_POS_GAINS;

  const float KP_DXL5_GAIN = 0.5f;
  const float KI_DXL5_GAIN = 0.0f;
  const float KD_DXL5_GAIN = 0.012f;
  const int   CURRENT_LIMIT_SOFT;
  const float MAX_INTEGRAL_ERROR;
  const float dt;

  const std::array<double,4> K_GFF;

  std::vector<float> desired_pos_;
  std::vector<float> desired_pos_slew_;
  std::vector<float> last_pos_error_;
  std::vector<float> last_pos_count_meas_;
  std::vector<float> integral_error_;

  float desired_dxl5_pos_;
  float last_dxl5_pos_error_;
  float integral_dxl5_error_;
};

std::shared_ptr<TestDxlCurrentNode> g_test_node = nullptr;

void signalHandler(int signum) {
  RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "시그널 %d 수신, 안전하게 종료합니다...", signum);
  if (g_test_node) {
    rclcpp::shutdown();
  }
}

int main(int argc, char** argv) {
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  rclcpp::init(argc, argv);
  g_test_node = std::make_shared<TestDxlCurrentNode>();

  try {
    rclcpp::spin(g_test_node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "예외 발생: %s", e.what());
  }

  g_test_node.reset();
  rclcpp::shutdown();
  return 0;
}