// src/motor_connect.cpp
// ------------------------------------------------------------
// XH540-V270-R (24V, RS-485) 3자유도 로봇팔: 전류모드 + 중력보상(피드포워드)
// - BulkRead(현재값) + SyncWrite(목표 전류)
// - 중력보상 τg(q) = Σ Jv_i(q)^T (m_i * g)
// - τ[Nm] -> I[A] -> raw(LSB) 변환 반영 (XH540-V270-R 스펙)
// ------------------------------------------------------------

#include <chrono>
#include <memory>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>   // std::clamp
#include <cmath>       // std::round
#include <string>
#include <signal.h>    // signal handling

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"  // C++ Dynamixel SDK

#include <Eigen/Dense>
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using namespace std::chrono_literals;
using dynamixel::PortHandler;
using dynamixel::PacketHandler;

// -------------------- 유틸: 표준 DH 변환 --------------------
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

class DxlCurrentNode : public rclcpp::Node
{
public:
  DxlCurrentNode()
  : Node("dxl_current_node"),
    device_name_("/dev/ttyUSB0"),
    protocol_version_(2.0),
    // 4자유도 로봇팔: ID 1,2,3: XH540-V270-R, ID 4: XH430-V350-R
    dxl_ids_{1, 2, 3, 4},

    // ---- Control Table (X 시리즈 공통, V270 페이지 확인) ----
    ADDR_OPERATING_MODE(11), CURRENT_CONTROL_MODE(0),
    ADDR_TORQUE_ENABLE(64), TORQUE_ENABLE(1),
    ADDR_PRESENT_CURRENT(126), LEN_PRESENT_CURRENT(2),
    ADDR_PRESENT_VELOCITY(128), LEN_PRESENT_VELOCITY(4),
    ADDR_PRESENT_POSITION(132), LEN_PRESENT_POSITION(4),
    ADDR_GOAL_CURRENT(102),
    ADDR_CURRENT_LIMIT(38), LEN_CURRENT_LIMIT(2),

    // ---- PID 게인 (출력 단위: "전류 raw 카운트") ----
    // 입력은 position/velocity raw(count) 단위이므로 게인은 raw->raw 스케일입니다.
    // 2,3축은 처짐 보상을 위해 더 높은 게인 적용
    KP_POS_GAINS({0.5f, 1.25f, 1.0f, 0.5f}), 
    KI_POS_GAINS({0.0f, 0.01f, 0.01f, 0.000f}),  // I 게인 (초기값 0.0)
    KD_POS_GAINS({0.012f, 0.015f, 0.012f, 0.012f}),

    // ---- 전류 리밋(soft clamp + 레지스터(38) 설정) ----
    // XH540-V270-R: Current Limit(38) 범위 0~1188 (3.2A)
    // XH430-V350-R: Current Limit(38) 범위 0~689 (1.85A)
    // 벤치 시작: 각각 900(2.42A), 500(1.35A) 정도 권장
    CURRENT_LIMIT_SOFT(1200),

    // ---- 적분 제한 (Anti-windup) ----
    MAX_INTEGRAL_ERROR(10000.0f),  // 적분 누적 최대값 [count*sec]

    // ---- 제어 주기[s] ----
    dt(0.005f),

    K_GFF({1.00, 1.0, 1.0, 1.0})
  {
    // 포트/패킷 핸들러
    port_handler_   = PortHandler::getPortHandler(device_name_.c_str());
    packet_handler_ = PacketHandler::getPacketHandler(protocol_version_);

    group_bulk_read_ = std::make_shared<dynamixel::GroupBulkRead>(port_handler_, packet_handler_);
    // 주의: Goal Current는 모든 축에서 동일한 주소/길이 -> SyncWrite가 적합
    group_sync_write_ = std::make_shared<dynamixel::GroupSyncWrite>(port_handler_, packet_handler_, ADDR_GOAL_CURRENT, 2);

    if (!port_handler_->openPort() || !port_handler_->setBaudRate(3000000)) {
      RCLCPP_ERROR(get_logger(), "포트 열기/baudrate 실패");
      rclcpp::shutdown();
      return;
    }

    // ---- 모터 스펙 반영(필수 상수) ----
    // 엔코더 해상도: 4096 cnt/rev -> rad 변환
    POS_COUNT_PER_REV_ = 4096.0;
    COUNT2RAD_ = 2.0 * M_PI / POS_COUNT_PER_REV_;
    // Current 단위: XH540-V270 = 2.69mA/LSB, XH430-V350 = 2.69mA/LSB (동일)
    // A_per_count_ 배열은 선언에서 초기화됨

    // 기어비(출력축): XH540(ID1-3) = 272.5, XH430(ID4) = 353.5
    N_ = {272.5, 272.5, 272.5, 353.5};
    // 효율(조정): 2,3축 처짐 보상을 위해 낮게 설정
    eta_ = {0.80, 0.70, 0.60, 0.80};
    // 토크상수 Kt_motor [Nm/A] (모터 측 토크상수 계산)
    // XH540-V270: Stall 9.2Nm @ 2.4A -> 기어비/효율 고려
    // XH430-V350: Stall 4.1Nm @ 1.18A -> 기어비/효율 고려
    Kt_[0] = 9.2 / (272.5 * eta_[0] * 2.4);  // XH540 ID1
    Kt_[1] = 9.2 / (272.5 * eta_[1] * 2.4);  // XH540 ID2  
    Kt_[2] = 9.2 / (272.5 * eta_[2] * 2.4);  // XH540 ID3
    Kt_[3] = 4.1 / (353.5 * eta_[3] * 1.18); // XH430 ID4

    // ---- 모드/토크온/리밋/벌크리드 등록 ----
    const uint16_t addr_block = ADDR_PRESENT_CURRENT;
    const uint16_t len_block  = LEN_PRESENT_CURRENT + LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;

    for (auto id : dxl_ids_) {
      int comm = packet_handler_->write1ByteTxRx(port_handler_, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
      if (comm != COMM_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "ID %u OperatingMode 설정 실패: %s", id, packet_handler_->getTxRxResult(comm));
      }

      // 하드 리밋(레지스터 38) 설정: 모터별 차등 적용
      uint16_t limit_raw;
      if (id == 4) {
        // XH430-V350-R: 0~689
        limit_raw = static_cast<uint16_t>(std::clamp(500, 0, 689)); // 벤치용 500
      } else {
        // XH540-V270-R: 0~1188
        limit_raw = static_cast<uint16_t>(std::clamp(900, 0, 1188)); // 벤치용 900
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

    // 퍼블리셔
    publisher_current_  = create_publisher<std_msgs::msg::Float32MultiArray>("motor/current", 10);
    publisher_velocity_ = create_publisher<std_msgs::msg::Float32MultiArray>("motor/velocity", 10);
    publisher_position_ = create_publisher<std_msgs::msg::Float32MultiArray>("motor/position", 10);

    // 벡터 초기화
    desired_pos_.resize(dxl_ids_.size());
    last_pos_error_.assign(dxl_ids_.size(), 0.0f);
    integral_error_.assign(dxl_ids_.size(), 0.0f);  // 적분 오차 초기화

    // SyncWrite 파라미터 버퍼(포인터 수명 문제 방지용) - 각 축 2바이트
    goal_current_bufs_.resize(dxl_ids_.size());
    for (auto &b : goal_current_bufs_) b = {0, 0};

    // 초기 위치 홀드
    group_bulk_read_->txRxPacket();
    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      uint32_t raw_p = group_bulk_read_->getData(dxl_ids_[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      int32_t sp = static_cast<int32_t>(raw_p);
      if (sp & 0x80000000) sp -= 0x100000000;
      desired_pos_[i] = static_cast<float>(sp);  // count
    }

    // 서브스크립션
    subscription_position_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "motor/command_position", 10,
      std::bind(&DxlCurrentNode::positionCallback, this, std::placeholders::_1)
    );

    // 타이머
    timer_ = create_wall_timer(std::chrono::duration<double>(dt),
                               std::bind(&DxlCurrentNode::readAndPublish, this));

    RCLCPP_INFO(get_logger(), ":: Motor Connected! (XH540-V270-R) ::");
  }

  ~DxlCurrentNode() {
    // 안전한 종료: 모든 모터 토크 비활성화
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
  // -------------------- 로봇/동역학 파라미터 --------------------
  // 4자유도 DH (mm -> m 변환, rad) - 그리퍼 장착으로 변경
  // 0->1: theta1, d1=115.75mm, a1=0, alpha1=pi/2
  // 1->2: theta2 + pi/2, d2=0, a2=250.0mm, alpha2=0
  // 2->3: theta3, d3=0, a3=250.0mm, alpha3=0
  // 3->H: theta4, d4=0, a4=215.9mm, alpha4=0 (그리퍼 포함)
  const double d1_ = 0.11575;
  const double a2_ = 0.250;
  const double a3_ = 0.250;
  const double a4_ = 0.2159;
  const double alpha1_ = M_PI_2;
  const double theta2_offset_ = M_PI_2;

  // 질량/COM (각 링크 프레임 기준, 단위: kg, m)
  // CAD 데이터에서 g 단위를 kg로, mm 단위를 m로 변환 - 그리퍼 장착 반영
  struct LinkInertial { double m; Vector3d com; };
  std::array<LinkInertial,4> link_ = {{
    {0.243781, Vector3d(0.000136, -0.000537, 0.092417)},  // Link1: 2번 v2 (243.781g)
    {0.277998, Vector3d(0.0, -0.000361, 0.182326)},       // Link2: 3번 v4 (277.998g)
    {0.195515, Vector3d(0.0, -0.000161, 0.163246)},       // Link3: 4번 v5 (195.515g)
    {0.346561, Vector3d(-0.003365, -0.00013, 0.108851)}   // Link4: 5번 + Gripper + Camera (346.561g)
  }};

  // 중력벡터 g (베이스 프레임)
  // TODO(선택): 베이스가 기울면 IMU로 gvec_ 갱신
  const Vector3d gvec_ = Vector3d(0, 0, -9.81);

  // 엔코더 count -> rad (런타임 설정)
  double POS_COUNT_PER_REV_ = 4096.0;
  double COUNT2RAD_ = 2.0 * M_PI / 4096.0;

  // 조인트 부호/영점 보정 (필수 캘리브레이션)
  // +전류가 +q로 도는 방향이 되도록 sign_ 설정, 영점은 기계적 기준 포즈에서 측정
  std::array<int,4>      sign_{ {+1, +1, +1, +1} };
  std::array<int32_t,4>  zero_count_{ {0, 0, 0, 0} };

  // 전류 단위/전달계/토크상수 (런타임 설정)
  std::array<double,4> A_per_count_{ {0.00269, 0.00269, 0.00269, 0.00269} }; // [A/LSB] 모터별 동일
  std::array<double,4> N_{ {272.5, 272.5, 272.5, 353.5} }; // 기어비
  std::array<double,4> eta_{ {0.80, 0.70, 0.60, 0.80} };  // 효율(초기치)
  std::array<double,4> Kt_; // 모터측 Kt [Nm/A] (생성자에서 계산)

  // -------------------- 중력보상 계산 --------------------
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
    return tau; // [Nm]
  }

  inline int32_t torqueNm_to_currentRaw(double tau, int axis) const {
    const double I   = tau / (Kt_[axis] * N_[axis] * eta_[axis]);  // [A]
    const double raw = I / A_per_count_[axis];                     // [LSB]
    // Goal Current(102)는 signed 16-bit. 클램프는 호출부에서 수행.
    return static_cast<int32_t>(std::lround(raw));
  }

  // -------------------- 콜백/메인 루프 --------------------
  void positionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= dxl_ids_.size()) {
      for (size_t i = 0; i < dxl_ids_.size(); ++i) {
        desired_pos_[i] = msg->data[i]; // count
        // 새로운 목표 위치 설정 시 적분 오차 리셋 (선택사항)
        // integral_error_[i] = 0.0f;  // 주석 처리: 연속적인 적분 유지
      }
      RCLCPP_INFO(get_logger(), "Position command received: PID control");
    }
  }

  void readAndPublish() {
    std_msgs::msg::Float32MultiArray msg_cur, msg_vel, msg_pos;

    int comm = group_bulk_read_->txRxPacket();
    if (comm != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Bulk Read 실패: %s", packet_handler_->getTxRxResult(comm));
      return;
    }

    std::vector<float> pos_count(dxl_ids_.size(), 0.0f);
    std::vector<float> vel_count(dxl_ids_.size(), 0.0f);

    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      const auto id = dxl_ids_[i];

      // Present Current (2 bytes, signed)
      uint32_t raw_c = group_bulk_read_->getData(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
      float current_raw = static_cast<float>(static_cast<int16_t>(raw_c & 0xFFFF));
      msg_cur.data.push_back(current_raw);

      // Present Velocity (4 bytes, signed)
      uint32_t raw_v = group_bulk_read_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
      int32_t sv = static_cast<int32_t>(raw_v);
      if (sv & 0x80000000) sv -= 0x100000000;
      float vel_raw = static_cast<float>(sv);
      msg_vel.data.push_back(vel_raw);
      vel_count[i] = vel_raw;

      // Present Position (4 bytes, signed)
      uint32_t raw_p = group_bulk_read_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      int32_t sp = static_cast<int32_t>(raw_p);
      if (sp & 0x80000000) sp -= 0x100000000;
      float pos_raw = static_cast<float>(sp);
      msg_pos.data.push_back(pos_raw);
      pos_count[i] = pos_raw;
    }

    // q(rad) 4축 구성
    std::array<double,4> q_rad = {
      sign_[0] * ( (static_cast<int32_t>(pos_count[0]) - zero_count_[0]) * COUNT2RAD_ ),
      sign_[1] * ( (static_cast<int32_t>(pos_count[1]) - zero_count_[1]) * COUNT2RAD_ ),
      sign_[2] * ( (static_cast<int32_t>(pos_count[2]) - zero_count_[2]) * COUNT2RAD_ ),
      sign_[3] * ( (static_cast<int32_t>(pos_count[3]) - zero_count_[3]) * COUNT2RAD_ )
    };

    // τg[Nm] & 전류 raw 산출
    Eigen::Vector4d tau_g = computeGravityTorqueNm(q_rad);
    
    // 중력보상 디버깅 로그 (5초마다 출력)
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
      RCLCPP_INFO(get_logger(), "Gravity Debug - q[rad]: [%.3f, %.3f, %.3f, %.3f]", 
                  q_rad[0], q_rad[1], q_rad[2], q_rad[3]);
      RCLCPP_INFO(get_logger(), "Gravity Debug - tau_g[Nm]: [%.3f, %.3f, %.3f, %.3f]", 
                  tau_g[0], tau_g[1], tau_g[2], tau_g[3]);
      RCLCPP_INFO(get_logger(), "Gravity Debug - pos_err[cnt]: [%.1f, %.1f, %.1f, %.1f]", 
                  desired_pos_[0]-pos_count[0], desired_pos_[1]-pos_count[1], 
                  desired_pos_[2]-pos_count[2], desired_pos_[3]-pos_count[3]);
      RCLCPP_INFO(get_logger(), "Gravity Debug - tau_g_raw[LSB]: [%d, %d, %d, %d]", 
                  torqueNm_to_currentRaw(tau_g[0]*K_GFF[0], 0),
                  torqueNm_to_currentRaw(tau_g[1]*K_GFF[1], 1),
                  torqueNm_to_currentRaw(tau_g[2]*K_GFF[2], 2),
                  torqueNm_to_currentRaw(tau_g[3]*K_GFF[3], 3));
      last_log_time = now;
    }

    for (int i=0; i<4; ++i) tau_g[i] *= K_GFF[i];

    std::array<int32_t,4> tau_g_raw = {
      torqueNm_to_currentRaw(tau_g[0], 0),
      torqueNm_to_currentRaw(tau_g[1], 1),
      torqueNm_to_currentRaw(tau_g[2], 2),
      torqueNm_to_currentRaw(tau_g[3], 3)
    };

    // SyncWrite 파라미터 초기화
    group_sync_write_->clearParam();

    for (size_t i = 0; i < dxl_ids_.size(); ++i) {
      // PID 제어: 출력 단위 = "전류 raw"
      float pos_err = desired_pos_[i] - pos_count[i];
      float dpos = (pos_err - last_pos_error_[i]) / dt;
      
      // 적분 항 계산 (Anti-windup 포함)
      integral_error_[i] += pos_err * dt;
      integral_error_[i] = std::clamp(integral_error_[i], -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);
      
      last_pos_error_[i] = pos_err;
      
      float cmd_raw = KP_POS_GAINS[i] * pos_err + 
                      KI_POS_GAINS[i] * integral_error_[i] +  // I 항 추가
                      KD_POS_GAINS[i] * dpos;

      // 중력보상(전류 raw) 합산
      if (i < 4) cmd_raw += static_cast<float>(tau_g_raw[i]);


      // 소프트 리밋 (모터별 차등 적용)
      float limit;
      if (dxl_ids_[i] == 4) {
        limit = 500.0f; // XH430-V350-R
      } else {
        limit = 900.0f; // XH540-V270-R
      }
      cmd_raw = std::clamp(cmd_raw, -limit, +limit);

      // SyncWrite 버퍼(멤버에 유지: 포인터 수명 문제 방지)
      int32_t goal_current = static_cast<int32_t>(std::lround(cmd_raw));
      goal_current_bufs_[i][0] = static_cast<uint8_t>( goal_current & 0xFF);
      goal_current_bufs_[i][1] = static_cast<uint8_t>((goal_current >> 8) & 0xFF);

      if (!group_sync_write_->addParam(dxl_ids_[i], goal_current_bufs_[i].data())) {
        RCLCPP_ERROR(get_logger(), "ID %u SyncWrite addParam 실패", dxl_ids_[i]);
      }
    }

    // 전송
    int wres = group_sync_write_->txPacket();
    if (wres != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Sync Write 실패: %s", packet_handler_->getTxRxResult(wres));
    }

    // 퍼블리시
    publisher_current_->publish(msg_cur);
    publisher_velocity_->publish(msg_vel);
    publisher_position_->publish(msg_pos);
  }

  // -------------------- 멤버 --------------------
  std::string device_name_;
  double protocol_version_;
  std::vector<uint8_t> dxl_ids_;
  PortHandler* port_handler_ = nullptr;
  PacketHandler* packet_handler_ = nullptr;

  std::shared_ptr<dynamixel::GroupBulkRead>  group_bulk_read_;
  std::shared_ptr<dynamixel::GroupSyncWrite> group_sync_write_;

  // SyncWrite 데이터 보관 버퍼(각 ID 2바이트) - 포인터 수명 보장
  std::vector<std::array<uint8_t,2>> goal_current_bufs_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_current_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_velocity_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_position_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_position_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 제어테이블 주소
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

  // PID 게인/리밋/주기 (축별)
  const std::array<float,4> KP_POS_GAINS;
  const std::array<float,4> KI_POS_GAINS;
  const std::array<float,4> KD_POS_GAINS;
  const int   CURRENT_LIMIT_SOFT; // raw
  const float MAX_INTEGRAL_ERROR; // 적분 제한값
  const float dt;

  const std::array<double,4> K_GFF;
  
 

  std::vector<float> desired_pos_;
  std::vector<float> last_pos_error_;
  std::vector<float> integral_error_;  // 적분 오차 누적
};

// 시그널 핸들러: 안전한 종료를 위한 플래그
std::shared_ptr<DxlCurrentNode> g_node = nullptr;

void signalHandler(int signum) {
  RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "시그널 %d 수신, 안전하게 종료합니다...", signum);
  if (g_node) {
    rclcpp::shutdown();
  }
}

int main(int argc, char** argv) {
  // 시그널 핸들러 등록
  signal(SIGINT, signalHandler);   // Ctrl+C
  signal(SIGTERM, signalHandler);  // 종료 시그널
  
  rclcpp::init(argc, argv);
  g_node = std::make_shared<DxlCurrentNode>();
  
  try {
    rclcpp::spin(g_node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "예외 발생: %s", e.what());
  }
  
  g_node.reset(); // 명시적으로 소멸자 호출
  rclcpp::shutdown();
  return 0;
}