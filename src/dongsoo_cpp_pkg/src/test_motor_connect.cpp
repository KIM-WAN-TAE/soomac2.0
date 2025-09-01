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
#include <fstream>     // JSON file reading
#include <nlohmann/json.hpp>  // nlohmann JSON library

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

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

// -------------------- nlohmann JSON 파싱 함수들 --------------------
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
  
  // ==> 수정사항.md 문제점 해결 1: 정식 JSON 파싱 사용 <==
  // 기존: 라인 기반 파싱으로 중력벡터 파싱 실패 (FF≈0)
  // 개선: nlohmann::json으로 구조적 접근
  const auto& joints = j["gravity_dh_parameters"]["joints"];
  
  // ==> 수정사항.md 문제점 해결 2: d1 덤어쓰기 버그 수정 <==
  // 기존: 모든 "d" 키를 처리하다가 Joint 2,3,4의 "d":0.0이 d1을 덤어씀
  // 개선: motor_id로 명시적 구분
  for (const auto& joint : joints) {
    int motor_id = joint["motor_id"];
    const auto& dh = joint["dh_params"];
    
    if (motor_id == 1) {
      params.d1 = dh["d"];           // 올바른 d1=0.11575 유지
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
  
  // ==> 수정사항.md 문제점 해결 3: 중력벡터 정확한 파싱 <==
  // 기존: line.find("gravity") && line.find("x") 동시 조건으로 파싱 실패
  // 개선: 구조적 경로로 직접 접근
  const auto& gvec = j["gravity_dh_parameters"]["gravity_vector"];
  params.gravity_vector.x() = gvec["x"];  // 0.0
  params.gravity_vector.y() = gvec["y"];  // 0.0
  params.gravity_vector.z() = gvec["z"];  // -9.81 (올바른 값)
  
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
  
  // 정확한 JSON 구조로 파싱
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

class DxlCurrentNode : public rclcpp::Node
{
public:
  DxlCurrentNode()
  : Node("dxl_current_node"),
    device_name_("/dev/ttyUSB0"),
    protocol_version_(2.0),
    // 4자유도 로봇팔: ID 1,2,3: XH540-V270-R, ID 4: XH430-V350-R, ID 5: 별도 제어용
    dxl_ids_{1, 2, 3, 4, 5},

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
    // 2,3축은 처짐 보상을 위해 더 높은 게인 적용 (개선된 버전)
    KP_POS_GAINS({0.8f, 1.5f, 1.2f, 1.0f}), 
    KI_POS_GAINS({0.05f, 0.08f, 0.06f, 0.03f}),  // I 게인 강화 (정상상태 오차 감소)
    KD_POS_GAINS({0.02f, 0.025f, 0.02f, 0.015f}),

    // ---- 전류 리밋(soft clamp + 레지스터(38) 설정) ----
    // XH540-V270-R: Current Limit(38) 범위 0~1188 (3.2A)
    // XH430-V350-R: Current Limit(38) 범위 0~689 (1.85A)
    // 벤치 시작: 각각 900(2.42A), 500(1.35A) 정도 권장
    CURRENT_LIMIT_SOFT(1200),

    // ---- 적분 제한 (Anti-windup) 강화 ----
    MAX_INTEGRAL_ERROR(5000.0f),  // 적분 누적 최대값 감소 [count*sec]

    // ---- 제어 주기[s] ----
    dt(0.005f),

    // ==> 수정사항.md 문제점 해결 8: 중력보상 게인 조정 <==
    // 기존: K_GFF={1.0, 1.0, 1.0, 1.0} - 균등한 게인
    // 개선: 2,3축 강화로 중력보상 효과 극대화
    K_GFF({1.2, 1.45, 1.4, 1.2})  // 중력보상 게인 조정 (2,3축 강화)
  {
    // ---- JSON 설정 파일 로드 ----
    try {
      std::string config_path = "/home/pc/soomac_ws/src/dongsoo_description/config";
      
      // DH 파라미터 로드
      DHParameters dh_params = loadDHParameters(config_path);
      d1_ = dh_params.d1;
      a2_ = dh_params.a2;
      a3_ = dh_params.a3;
      a4_ = dh_params.a4;
      alpha1_ = dh_params.alpha1;
      theta2_offset_ = dh_params.theta2_offset;
      gvec_ = dh_params.gravity_vector;
      
      // 링크 관성 정보 로드
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
    eta_ = {0.80, 0.70, 0.60, 0.70};
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
    publisher_position_ = create_publisher<std_msgs::msg::Int32MultiArray>("motor/position", 10);

    // 벡터 초기화 (4개 모터용)
    desired_pos_.resize(4);  // 1,2,3,4번 모터만
    last_pos_error_.assign(4, 0.0f);
    integral_error_.assign(4, 0.0f);  // 적분 오차 초기화

    // 5번 모터 별도 초기화
    desired_dxl5_pos_ = 0.0f;
    last_dxl5_pos_error_ = 0.0f;
    integral_dxl5_error_ = 0.0f;

    // SyncWrite 파라미터 버퍼(포인터 수명 문제 방지용) - 각 축 2바이트
    goal_current_bufs_.resize(dxl_ids_.size());
    for (auto &b : goal_current_bufs_) b = {0, 0};

    // 초기 위치 홀드 (1,2,3,4번 모터)
    group_bulk_read_->txRxPacket();
    for (size_t i = 0; i < 4; ++i) {
      uint32_t raw_p = group_bulk_read_->getData(dxl_ids_[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      int32_t sp = static_cast<int32_t>(raw_p);
      if (sp & 0x80000000) sp -= 0x100000000;
      desired_pos_[i] = static_cast<float>(sp);  // count
    }

    // 5번 모터 초기 위치 홀드
    uint32_t raw_p5 = group_bulk_read_->getData(5, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    int32_t sp5 = static_cast<int32_t>(raw_p5);
    if (sp5 & 0x80000000) sp5 -= 0x100000000;
    desired_dxl5_pos_ = static_cast<float>(sp5);

    // 서브스크립션
    subscription_position_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "motor/command_position", 10,
      std::bind(&DxlCurrentNode::positionCallback, this, std::placeholders::_1)
    );

    // 5번 모터 별도 서브스크립션
    subscription_dxl5_position_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "motor/command_dxl5_position", 10,
      std::bind(&DxlCurrentNode::dxl5PositionCallback, this, std::placeholders::_1)
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
  // -------------------- 로봇/동역학 파라미터 (JSON에서 로드) --------------------
  // DH parameters loaded from JSON
  double d1_, a2_, a3_, a4_, alpha1_, theta2_offset_;
  
  // Link inertial properties loaded from JSON
  struct LinkInertial { double m; Vector3d com; };
  std::array<LinkInertial,4> link_;
  
  // Gravity vector loaded from JSON
  Vector3d gvec_;

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
  void positionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 4) {
      for (size_t i = 0; i < 4; ++i) {
        desired_pos_[i] = static_cast<float>(msg->data[i]); // int to float for internal processing
        // 새로운 목표 위치 설정 시 적분 오차 리셋 (선택사항)
        // integral_error_[i] = 0.0f;  // 주석 처리: 연속적인 적분 유지
      }
      RCLCPP_INFO(get_logger(), "Position command received (motors 1-4): PID control");
    }
  }

  void dxl5PositionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 1) {
      desired_dxl5_pos_ = static_cast<float>(msg->data[0]); // int to float for internal processing
      // 새로운 목표 위치 설정 시 적분 오차 리셋 (선택사항)
      // integral_dxl5_error_ = 0.0f;  // 주석 처리: 연속적인 적분 유지
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
      msg_pos.data.push_back(sp);  // int32로 직접 퍼블리시
      pos_count[i] = static_cast<float>(sp);  // 내부 계산용은 float 유지
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
    
    // ==> 수정사항.md 문제점 해결 6: 강화된 디버깅 로그 <==
    // 기존: 기본 디버깅 정보만 출력
    // 개선: d1, gravity_vector, FF 전류, 적분항 상태까지 모니터링
    // 중력보상 및 제어 품질 디버깅 로그 (5초마다 출력)
    static auto last_log_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
      RCLCPP_INFO(get_logger(), "=== 제어 상태 모니터링 (nlohmann JSON 버전) ===");
      RCLCPP_INFO(get_logger(), "DH Params - d1:%.3f, a2:%.3f, a3:%.3f, a4:%.3f, alpha1:%.3f", 
                  d1_, a2_, a3_, a4_, alpha1_);
      RCLCPP_INFO(get_logger(), "Gravity Vec - [%.3f, %.3f, %.3f]", 
                  gvec_.x(), gvec_.y(), gvec_.z());
      RCLCPP_INFO(get_logger(), "Joint Angles[rad] - [%.3f, %.3f, %.3f, %.3f]", 
                  q_rad[0], q_rad[1], q_rad[2], q_rad[3]);
      RCLCPP_INFO(get_logger(), "Gravity Torque[Nm] - [%.3f, %.3f, %.3f, %.3f]", 
                  tau_g[0], tau_g[1], tau_g[2], tau_g[3]);
      RCLCPP_INFO(get_logger(), "Position Errors[cnt] - [%.1f, %.1f, %.1f, %.1f]", 
                  desired_pos_[0]-pos_count[0], desired_pos_[1]-pos_count[1], 
                  desired_pos_[2]-pos_count[2], desired_pos_[3]-pos_count[3]);
      RCLCPP_INFO(get_logger(), "FF Current[LSB] - [%d, %d, %d, %d]", 
                  torqueNm_to_currentRaw(tau_g[0]*K_GFF[0], 0),
                  torqueNm_to_currentRaw(tau_g[1]*K_GFF[1], 1),
                  torqueNm_to_currentRaw(tau_g[2]*K_GFF[2], 2),
                  torqueNm_to_currentRaw(tau_g[3]*K_GFF[3], 3));
      RCLCPP_INFO(get_logger(), "Integral Terms - [%.1f, %.1f, %.1f, %.1f]", 
                  integral_error_[0], integral_error_[1], integral_error_[2], integral_error_[3]);
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

    // 1,2,3,4번 모터 제어 (개선된 버전: 강화된 Anti-windup)
    for (size_t i = 0; i < 4; ++i) {
      // PID 제어: 출력 단위 = "전류 raw"
      float pos_err = desired_pos_[i] - pos_count[i];
      float dpos = (pos_err - last_pos_error_[i]) / dt;
      
      // P, D 항 먼저 계산
      float pid_raw = KP_POS_GAINS[i] * pos_err + KD_POS_GAINS[i] * dpos;
      
      // 중력보상 개별 적용 (포화 전에 미리 합산)
      float ff_raw = static_cast<float>(tau_g_raw[i]);
      float pre_sat_cmd = pid_raw + ff_raw;
      
      // 소프트 리밋 (모터별 차등 적용)
      float limit;
      if (dxl_ids_[i] == 4) {
        limit = 500.0f; // XH430-V350-R
      } else {
        limit = 900.0f; // XH540-V270-R
      }
      
      // Back-calculation Anti-windup: 포화 전후 차이를 I 항에서 보상
      float cmd_raw_clamped = std::clamp(pre_sat_cmd, -limit, +limit);
      float saturation_error = cmd_raw_clamped - pre_sat_cmd;
      
      // 적분 항 계산 (Back-calculation Anti-windup 적용)
      integral_error_[i] += pos_err * dt + (saturation_error * 0.1f / KI_POS_GAINS[i]); // Back-calculation factor
      integral_error_[i] = std::clamp(integral_error_[i], -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);
      
      // I 항 추가 및 최종 명령값 계산
      float cmd_raw = pid_raw + KI_POS_GAINS[i] * integral_error_[i] + ff_raw;
      cmd_raw = std::clamp(cmd_raw, -limit, +limit);
      
      last_pos_error_[i] = pos_err;

      // SyncWrite 버퍼(멤버에 유지: 포인터 수명 문제 방지)
      int32_t goal_current = static_cast<int32_t>(std::lround(cmd_raw));
      goal_current_bufs_[i][0] = static_cast<uint8_t>( goal_current & 0xFF);
      goal_current_bufs_[i][1] = static_cast<uint8_t>((goal_current >> 8) & 0xFF);

      if (!group_sync_write_->addParam(dxl_ids_[i], goal_current_bufs_[i].data())) {
        RCLCPP_ERROR(get_logger(), "ID %u SyncWrite addParam 실패", dxl_ids_[i]);
      }
    }

    // 5번 모터 별도 제어 (중력보상 없이 순수 PID만)
    float dxl5_pos_err = desired_dxl5_pos_ - pos_count[4];  // pos_count[4] = ID5 position
    float dxl5_dpos = (dxl5_pos_err - last_dxl5_pos_error_) / dt;
    
    // 적분 항 계산 (Anti-windup 포함)
    integral_dxl5_error_ += dxl5_pos_err * dt;
    integral_dxl5_error_ = std::clamp(integral_dxl5_error_, -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);
    
    last_dxl5_pos_error_ = dxl5_pos_err;
    
    float dxl5_cmd_raw = KP_DXL5_GAIN * dxl5_pos_err + 
                         KI_DXL5_GAIN * integral_dxl5_error_ +
                         KD_DXL5_GAIN * dxl5_dpos;

    // 5번 모터 소프트 리밋 (기본값 사용)
    dxl5_cmd_raw = std::clamp(dxl5_cmd_raw, -900.0f, +900.0f);

    // 5번 모터 SyncWrite 버퍼
    int32_t dxl5_goal_current = static_cast<int32_t>(std::lround(dxl5_cmd_raw));
    goal_current_bufs_[4][0] = static_cast<uint8_t>( dxl5_goal_current & 0xFF);
    goal_current_bufs_[4][1] = static_cast<uint8_t>((dxl5_goal_current >> 8) & 0xFF);

    if (!group_sync_write_->addParam(5, goal_current_bufs_[4].data())) {
      RCLCPP_ERROR(get_logger(), "ID 5 SyncWrite addParam 실패");
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
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_position_;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_position_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_dxl5_position_;
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

  // 5번 모터용 PID 게인
  const float KP_DXL5_GAIN = 0.5f;
  const float KI_DXL5_GAIN = 0.0f;
  const float KD_DXL5_GAIN = 0.012f;
  const int   CURRENT_LIMIT_SOFT; // raw
  const float MAX_INTEGRAL_ERROR; // 적분 제한값
  const float dt;

  const std::array<double,4> K_GFF;

  std::vector<float> desired_pos_;
  std::vector<float> last_pos_error_;
  std::vector<float> integral_error_;  // 적분 오차 누적

  // 5번 모터 별도 제어용 변수
  float desired_dxl5_pos_;
  float last_dxl5_pos_error_;
  float integral_dxl5_error_;
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