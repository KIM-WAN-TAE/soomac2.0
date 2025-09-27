// motor_connect.cpp
#include <chrono>
#include <memory>
#include <vector>
#include <deque>
#include <array>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <string>
#include <signal.h>
#include <fstream>
#include <optional>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include <Eigen/Dense>
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using json = nlohmann::json;

using namespace std::chrono_literals;
using dynamixel::PortHandler;
using dynamixel::PacketHandler;

// ---------- Utilities ----------
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

// ---------- Model from JSON ----------
struct DHParameters {
  double d1=0, a2=0, a3=0, a4=0, alpha1=0, theta2_offset=0;
  Vector3d gravity_vector{0,0,-9.81};
};
struct LinkInertialData { double mass=0; Vector3d com{0,0,0}; };

static DHParameters loadDHParameters(const std::string& path) {
  std::ifstream f(path + "/gravity_dh_param.json");
  if(!f.is_open()) throw std::runtime_error("open gravity_dh_param.json fail");
  json j; f >> j;
  DHParameters p;
  const auto& root = j["gravity_dh_parameters"];
  for (const auto& joint : root["joints"]) {
    int mid = joint["motor_id"];
    const auto& dhp = joint["dh_params"];
    if (mid==1) { p.d1 = dhp["d"]; p.alpha1 = dhp["alpha"]; }
    else if (mid==2) { p.a2 = dhp["a"]; if (dhp.contains("theta_offset")) p.theta2_offset = dhp["theta_offset"]; }
    else if (mid==3) { p.a3 = dhp["a"]; }
    else if (mid==4) { p.a4 = dhp["a"]; }
  }
  const auto& g = root["gravity_vector"];
  p.gravity_vector = Vector3d(double(g["x"]), double(g["y"]), double(g["z"]));
  return p;
}

static std::vector<LinkInertialData> loadLinkInertialData(const std::string& path) {
  std::ifstream f(path + "/link_inertial.json");
  if(!f.is_open()) throw std::runtime_error("open link_inertial.json fail");
  json j; f >> j;
  std::vector<LinkInertialData> out;
  for (const auto& link : j["link_inertial_properties"]["links"]) {
    LinkInertialData L;
    L.mass = link["mass"];
    const auto& c = link["center_of_mass"];
    L.com = Vector3d(double(c["x"]), double(c["y"]), double(c["z"]));
    out.push_back(L);
  }
  return out;
}

// ---------- Node ----------
class GravityArmNode : public rclcpp::Node {
public:
  GravityArmNode()
  : Node("gravity_arm_node"),
    device_name_("/dev/ttyUSB0"),
    protocol_version_(2.0),
    dxl_ids_{1,2,3,4,5},
    // Control table
    ADDR_OPERATING_MODE(11), CURRENT_CONTROL_MODE(0),
    ADDR_TORQUE_ENABLE(64), TORQUE_ENABLE(1),
    ADDR_PRESENT_CURRENT(126), LEN_PRESENT_CURRENT(2),
    ADDR_PRESENT_VELOCITY(128), LEN_PRESENT_VELOCITY(4),
    ADDR_PRESENT_POSITION(132), LEN_PRESENT_POSITION(4),
    ADDR_GOAL_CURRENT(102),
    ADDR_CURRENT_LIMIT(38), LEN_CURRENT_LIMIT(2)
  {
    // -------- Hyper-parameters (edit here) --------
    hp_.dt_nominal = 0.01; // 100 Hz
    // PID gains (counts-domain -> current LSB)
    hp_.Kp = {1.0f, 1.8f, 1.8f, 1.6f};
    hp_.Ki = {0.00f, 0.005f, 0.005f, 0.001f};
    hp_.Kd = {0.002f, 0.0015f, 0.002f, 0.005f};
    // Derivative filter and clamp
    hp_.vel_lpf_a = 0.85f;    // 0.8~0.9
    hp_.d_clamp = 300.0f;     // LSB
    // Anti-windup
    hp_.K_aw = 0.05f;
    hp_.I_max = 2000.0f;
    // Gravity FF scale
    hp_.K_gff = {1.0, 1.0, 1.0, 1.0};
    // Current limits (raw LSB). Will be clamped by model max.
    hp_.limit_joint_raw = {1000, 1000, 1000, 700}; // joint5 uses same 700 later
    // FSM thresholds
    hp_.v_ref_eps = 50.0f;   // cnt/s
    hp_.v_meas_eps = 30.0f;  // cnt/s
    hp_.i_freeze_sec = 0.2f; // I freeze after mode switch
    // HOLD mode settings
    hp_.e_hold_db = 20.0f;       // counts
    hp_.ki_hold_scale = 0.2f;    // Ki scale in HOLD
    hp_.i_leak = 0.001f;         // per second
    hp_.bias_alpha = 0.3f;       // per second
    hp_.bias_max = 120.0f;       // LSB
    // PTP limits
    hp_.v_max = 1800.0;     // cnt/s
    hp_.a_max = 9000.0;     // cnt/s^2
    // Motor/gear params
    POS_COUNT_PER_REV_ = 4096.0;
    COUNT2RAD_ = 2.0 * M_PI / POS_COUNT_PER_REV_;
    N_   = {272.5, 272.5, 272.5, 353.5};
    eta_ = {0.80,   0.70,   0.60,   0.70};
    // A_per_count (A/LSB): XH540(1–3): 0.00269 A/LSB, XH430(4–5): 0.00134 A/LSB
    A_per_count_ = {0.00269, 0.00269, 0.00269, 0.00134};
    // Kt (Nm/A) from datasheets/empirical (keep editable)
    Kt_[0] = 9.2 / (N_[0] * eta_[0] * 2.4);
    Kt_[1] = 9.2 / (N_[1] * eta_[1] * 2.4);
    Kt_[2] = 9.2 / (N_[2] * eta_[2] * 2.4);
    Kt_[3] = 4.1 / (N_[3] * eta_[3] * 1.18);

    // -------- Load JSONs --------
    try {
      std::string cfg = declare_parameter<std::string>("config_path",
        "/home/pc/soomac_ws/src/dongsoo_description/config");
      dh_ = loadDHParameters(cfg);
      auto links = loadLinkInertialData(cfg);
      for (size_t i=0;i<4 && i<links.size();++i) { link_[i].m=links[i].mass; link_[i].com=links[i].com; }
      RCLCPP_INFO(get_logger(), "Loaded DH & inertial JSON");
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "JSON load failed: %s", e.what());
      throw;
    }

    // -------- DXL Init --------
    port_   = PortHandler::getPortHandler(device_name_.c_str());
    ph_     = PacketHandler::getPacketHandler(protocol_version_);
    gbr_    = std::make_shared<dynamixel::GroupBulkRead>(port_, ph_);
    gsw_    = std::make_shared<dynamixel::GroupSyncWrite>(port_, ph_, ADDR_GOAL_CURRENT, 2);

    if(!port_->openPort() || !port_->setBaudRate(3000000)) {
      RCLCPP_FATAL(get_logger(), "Port open or baudrate failed");
      throw std::runtime_error("DXL port");
    }

    // sign_ / zero_count_ as ROS params (runtime adjustable)
    for(int i=0;i<4;++i){
      sign_[i] = declare_parameter<int>("joint_sign_"+std::to_string(i+1), +1);
      zero_count_[i] = declare_parameter<int>("joint_zero_"+std::to_string(i+1), 0);
    }
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GravityArmNode::onParamSet, this, std::placeholders::_1));

    // Operating mode, Current Limit, Torque Enable, BulkRead registration
    const uint16_t addr_block = ADDR_PRESENT_CURRENT;
    const uint16_t len_block  = LEN_PRESENT_CURRENT + LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION;

    for (auto id : dxl_ids_) {
      int comm = ph_->write1ByteTxRx(port_, id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE);
      if (comm != COMM_SUCCESS) RCLCPP_WARN(get_logger(), "ID %u opmode set fail: %s", id, ph_->getTxRxResult(comm));

      // CurrentLimit per model
      uint16_t hw_max = (id<=3) ? 1188 : 689; // XH540 vs XH430
      uint16_t req = (id==5) ? 700 : (id==4 ? 700 : hp_.limit_joint_raw[std::min<size_t>(id-1,3)]);
      uint16_t limit_raw = std::min<uint16_t>(req, hw_max);
      comm = ph_->write2ByteTxRx(port_, id, ADDR_CURRENT_LIMIT, limit_raw);
      if (comm != COMM_SUCCESS) RCLCPP_WARN(get_logger(), "ID %u current limit set fail: %s", id, ph_->getTxRxResult(comm));

      comm = ph_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
      if (comm != COMM_SUCCESS) RCLCPP_WARN(get_logger(), "ID %u torque enable fail: %s", id, ph_->getTxRxResult(comm));

      if (!gbr_->addParam(id, addr_block, len_block)) RCLCPP_ERROR(get_logger(), "ID %u bulk add fail", id);
    }

    // -------- ROS IO --------
    pub_cur_  = create_publisher<std_msgs::msg::Float32MultiArray>("/motor/current", 10);
    pub_vel_  = create_publisher<std_msgs::msg::Float32MultiArray>("/motor/velocity", 10);
    pub_pos_  = create_publisher<std_msgs::msg::Int32MultiArray>("/motor/position", 10);

    sub_path_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/motor/command_path", 10, std::bind(&GravityArmNode::pathCallback, this, std::placeholders::_1));

    sub_pos_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/motor/command_position", 10, std::bind(&GravityArmNode::singlePosCallback, this, std::placeholders::_1));

    sub_dxl5_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/motor/command_position_dxl5", 10, std::bind(&GravityArmNode::dxl5Callback, this, std::placeholders::_1));

    // -------- State init --------
    desired_pos_.assign(4, 0.f);
    qref_.assign(4, 0.f);
    qref_prev_.assign(4, 0.f);
    vel_filt_.assign(4, 0.f);
    last_pos_meas_.assign(4, 0.f);
    I_.assign(4, 0.f);
    bias_ff_.assign(4, 0.f);
    i_freeze_t_.assign(4, 0.f);

    goal_bufs_.resize(dxl_ids_.size());
    for(auto& b: goal_bufs_) b = {0,0};

    // Prime first read and set refs to current
    if (gbr_->txRxPacket()!=COMM_SUCCESS) RCLCPP_WARN(get_logger(),"Initial bulk read failed");
    std::vector<float> pos_now(5,0.f);
    for(size_t i=0;i<dxl_ids_.size();++i){
      uint32_t raw_p = gbr_->getData(dxl_ids_[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      int32_t sp = (int32_t)raw_p; if (sp & 0x80000000) sp -= 0x100000000;
      pos_now[i] = (float)sp;
    }
    for(int i=0;i<4;++i){ desired_pos_[i]=qref_[i]=qref_prev_[i]=last_pos_meas_[i]=pos_now[i]; }
    desired_dxl5_pos_ = pos_now[4];

    mode_ = HOLD;
    traj_active_ = false;

    // -------- Timer --------
    last_tick_ = now();
    timer_ = create_wall_timer(std::chrono::duration<double>(hp_.dt_nominal),
              std::bind(&GravityArmNode::loop, this));

    RCLCPP_INFO(get_logger(), "GravityArmNode ready @100Hz");
  }

  ~GravityArmNode() override {
    RCLCPP_INFO(get_logger(), "Safe shutdown...");
    if (port_ && ph_) {
      for (auto id : dxl_ids_) {
        ph_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, 0);
      }
      port_->closePort();
    }
  }

private:
  // ---------- Hyper params ----------
  struct Hyper {
    double dt_nominal{};
    std::array<float,4> Kp, Ki, Kd;
    float vel_lpf_a, d_clamp;
    float K_aw, I_max;
    std::array<double,4> K_gff;
    std::array<uint16_t,4> limit_joint_raw;
    float v_ref_eps, v_meas_eps, i_freeze_sec;
    float e_hold_db, ki_hold_scale, i_leak, bias_alpha, bias_max;
    double v_max, a_max;
  } hp_;

  // ---------- PTP segment ----------
  struct Segment {
    std::array<float,4> q0, q1; // joint end-points (counts)
    std::array<double,4> dir_ratio; // Δq_i / D
    double D=0; // max distance among joints
    // entry/exit speeds at the path space
    double v_in=0, v_out=0, v_max=0, a=0;
    // durations
    double t1=0, t2=0, t3=0, T=0, v_peak=0;
    rclcpp::Time t0;
    bool triangular=false;
  };

  // ---------- Callbacks ----------
  void pathCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
    if (msg->data.size()%4 != 0) {
      RCLCPP_WARN(get_logger(), "command_path length not multiple of 4");
      return;
    }
    for(size_t i=0;i<msg->data.size(); i+=4){
      std::array<float,4> q{
        (float)msg->data[i+0], (float)msg->data[i+1],
        (float)msg->data[i+2], (float)msg->data[i+3]};
      path_.push_back(q);
    }
    // if idle, build plan starting from current measured position
    if (!traj_active_ && path_.size()>=1) buildPlanFromCurrent();
  }

  void singlePosCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
    if (msg->data.size()<4) return;
    std::array<float,4> q{
      (float)msg->data[0], (float)msg->data[1],
      (float)msg->data[2], (float)msg->data[3]};
    path_.clear();
    path_.push_back(q);
    buildPlanFromCurrent();
  }

  void dxl5Callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
    if (msg->data.empty()) return;
    desired_dxl5_pos_ = (float)msg->data[0];
  }

  // ---------- Planning ----------
  void buildPlanFromCurrent(){
    // Start at current measured joint positions to avoid initial bump
    std::array<float,4> qstart{};
    for(int i=0;i<4;++i) qstart[i] = last_pos_meas_[i];

    // Form waypoints: qstart -> each queued path point in order
    std::vector<std::array<float,4>> wps;
    wps.push_back(qstart);
    while(!path_.empty()){ wps.push_back(path_.front()); path_.pop_front(); }

    // Compute segments with corner speeds
    segs_.clear();
    const size_t M = wps.size();
    if (M<2) return;

    // First pass: compute D per segment
    std::vector<double> D(M-1,0.0);
    for(size_t k=0;k<M-1;++k){
      double dmax = 0.0;
      for(int i=0;i<4;++i) dmax = std::max(dmax, static_cast<double>(std::fabs(wps[k+1][i]-wps[k][i])));
      D[k] = dmax;
    }

    // Corner speeds: start/end zero, middle limited by distance and a_max
    std::vector<double> v_corner(M, 0.0);
    v_corner[0] = 0.0; v_corner[M-1]=0.0;
    for(size_t k=1;k<M-1;++k){
      double d_in = D[k-1], d_out = D[k];
      // Simple heuristic
      double v_allow = std::sqrt(std::max(0.0, hp_.a_max * std::min(d_in, d_out)));
      v_corner[k] = std::min(hp_.v_max, v_allow);
    }

    // Build segments with entry/exit speeds
    for(size_t k=0;k<M-1;++k){
      Segment s;
      s.q0 = wps[k]; s.q1 = wps[k+1];
      s.D = D[k];
      if (s.D < 1e-6) continue; // skip zero-length
      for(int i=0;i<4;++i){
        s.dir_ratio[i] = (s.q1[i]-s.q0[i]) / s.D; // may be negative
      }
      s.v_in  = v_corner[k];
      s.v_out = v_corner[k+1];
      s.v_max = hp_.v_max;
      s.a     = hp_.a_max;

      // Solve durations
      solveProfile(s);
      s.t0 = now();
      segs_.push_back(s);
    }
    if (!segs_.empty()){
      active_seg_ = segs_.front();
      segs_.pop_front();
      t_ref_start_ = now();
      traj_active_ = true;
    }
  }

  static void solveProfile(Segment& s){
    const double a = s.a, vmax = s.v_max, D = s.D, v0 = s.v_in, v1 = s.v_out;
    // Trial trapezoid
    double t1 = std::max(0.0, (vmax - v0)/a);
    double t3 = std::max(0.0, (vmax - v1)/a);
    double d1 = (v0+vmax)*0.5*t1;
    double d3 = (v1+vmax)*0.5*t3;
    if (d1 + d3 <= D) {
      // trapezoid
      double d2 = D - d1 - d3;
      double t2 = d2 / vmax;
      s.t1=t1; s.t2=t2; s.t3=t3; s.T=t1+t2+t3; s.triangular=false; s.v_peak=vmax;
    } else {
      // triangle with peak v* < vmax
      double vpk2 = a*D + 0.5*(v0*v0 + v1*v1);
      double vpk  = std::sqrt(std::max(0.0, vpk2));
      double t1p = (vpk - v0)/a;
      double t3p = (vpk - v1)/a;
      s.t1=t1p; s.t2=0.0; s.t3=t3p; s.T=t1p+t3p; s.triangular=true; s.v_peak=vpk;
    }
  }

  // Sample s(t) progress (distance along path) at time t since seg start
  static double sampleS(const Segment& s, double t, double& sdot){
    t = std::clamp(t, 0.0, s.T);
    if (!s.triangular){
      if (t < s.t1){
        // accel from v0
        double v0 = s.v_in, a = s.a;
        sdot = v0 + a*t;
        return v0*t + 0.5*a*t*t;
      } else if (t < s.t1 + s.t2){
        sdot = s.v_peak;
        double d1 = (s.v_in + s.v_peak)*0.5*s.t1;
        return d1 + s.v_peak*(t - s.t1);
      } else {
        double tau = t - (s.t1 + s.t2);
        double a = s.a;
        sdot = s.v_peak - a*tau;
        double d1 = (s.v_in + s.v_peak)*0.5*s.t1;
        double d2 = s.v_peak * s.t2;
        return d1 + d2 + s.v_peak*tau - 0.5*a*tau*tau;
      }
    } else {
      if (t < s.t1){
        double v0 = s.v_in, a = s.a;
        sdot = v0 + a*t;
        return v0*t + 0.5*a*t*t;
      } else {
        double tau = t - s.t1;
        double a = s.a;
        sdot = s.v_peak - a*tau;
        double d1 = (s.v_in + s.v_peak)*0.5*s.t1;
        return d1 + s.v_peak*tau - 0.5*a*tau*tau;
      }
    }
  }

  // ---------- Main loop ----------
  void loop(){
    // dt
    rclcpp::Time nowt = now();
    double dt = (nowt - last_tick_).seconds();
    last_tick_ = nowt;
    if (dt <= 0) dt = hp_.dt_nominal;

    // Bulk read
    int comm = gbr_->txRxPacket();
    if (comm != COMM_SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, "Bulk read fail");
      return;
    }

    std_msgs::msg::Float32MultiArray msg_cur, msg_vel;
    std_msgs::msg::Int32MultiArray msg_pos;

    std::vector<float> pos(5,0), vel_raw(5,0);

    for (size_t i=0;i<dxl_ids_.size();++i) {
      auto id = dxl_ids_[i];

      uint32_t raw_c = gbr_->getData(id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
      float cur_raw = (float)((int16_t)(raw_c & 0xFFFF));
      msg_cur.data.push_back(cur_raw);

      uint32_t raw_v = gbr_->getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY);
      int32_t sv = (int32_t)raw_v; if (sv & 0x80000000) sv -= 0x100000000;
      float v = (float)sv;
      msg_vel.data.push_back(v);
      vel_raw[i] = v;

      uint32_t raw_p = gbr_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
      int32_t sp = (int32_t)raw_p; if (sp & 0x80000000) sp -= 0x100000000;
      msg_pos.data.push_back(sp);
      pos[i] = (float)sp;
    }

    pub_cur_->publish(msg_cur);
    pub_vel_->publish(msg_vel);
    pub_pos_->publish(msg_pos);

    // Low-pass filter velocities (1~4 for joint control)
    for(int i=0;i<4;++i){
      double a = hp_.vel_lpf_a;
      double v_est = (pos[i]-last_pos_meas_[i]) / dt;
      vel_filt_[i] = (float)(a*vel_filt_[i] + (1.0-a)*v_est);
      last_pos_meas_[i] = pos[i];
    }

    // ---- Reference generation from trajectory ----
    if (traj_active_) {
      // if no active segment, try to pop next
      if (!active_seg_.has_value() && !segs_.empty()){
        active_seg_ = segs_.front(); segs_.pop_front(); active_seg_->t0 = nowt;
      }
      if (active_seg_.has_value()){
        double t = (nowt - active_seg_->t0).seconds();
        double sdot=0.0;
        double s  = sampleS(*active_seg_, t, sdot); // progress in counts along path
        for(int i=0;i<4;++i){
          qref_[i] = (float)(active_seg_->q0[i] + active_seg_->dir_ratio[i] * s);
        }
        // segment done?
        if (t >= active_seg_->T - 1e-6){
          // Snap to end
          for(int i=0;i<4;++i) qref_[i] = active_seg_->q1[i];
          // advance
          if (!segs_.empty()){ active_seg_ = segs_.front(); segs_.pop_front(); active_seg_->t0 = nowt; }
          else { active_seg_.reset(); traj_active_ = false; }
        }
      }
    } else {
      // hold last reference
    }

    // Store `desired_pos_slew_` as qref_
    // Also compute ref speed for FSM by finite difference
    std::array<float,4> qd_ref{};
    for(int i=0;i<4;++i){
      desired_pos_[i] = qref_[i];
      qd_ref[i] = (qref_[i] - qref_prev_[i]) / (float)dt;
      qref_prev_[i] = qref_[i];
    }

    // ---- FSM ----
    bool moving_ref=false, moving_meas=false;
    for(int i=0;i<4;++i){
      moving_ref  |= std::fabs(qd_ref[i]) > hp_.v_ref_eps;
      moving_meas |= std::fabs(vel_filt_[i]) > hp_.v_meas_eps;
    }
    CtrlMode new_mode = (traj_active_ || moving_ref || moving_meas) ? MOVE : HOLD;
    bool mode_changed = (new_mode != mode_);
    if (mode_changed){
      mode_ = new_mode;
      for(int i=0;i<4;++i) i_freeze_t_[i] = hp_.i_freeze_sec;
    }

    // ---- Gravity torque (1..4) ----
    std::array<double,4> qrad = {
      sign_[0]*((int32_t)pos[0] - zero_count_[0]) * COUNT2RAD_,
      sign_[1]*((int32_t)pos[1] - zero_count_[1]) * COUNT2RAD_,
      sign_[2]*((int32_t)pos[2] - zero_count_[2]) * COUNT2RAD_,
      sign_[3]*((int32_t)pos[3] - zero_count_[3]) * COUNT2RAD_
    };
    Vector4d tau_g = computeGravityTorqueNm(qrad);
    for(int i=0;i<4;++i) tau_g[i] *= hp_.K_gff[i];
    std::array<int32_t,4> tau_raw = {
      torqueNm_to_currentRaw(tau_g[0], 0),
      torqueNm_to_currentRaw(tau_g[1], 1),
      torqueNm_to_currentRaw(tau_g[2], 2),
      torqueNm_to_currentRaw(tau_g[3], 3)
    };

    // ---- Control and output ----
    gsw_->clearParam();

    for(int i=0;i<4;++i){
      float e = desired_pos_[i] - pos[i];

      // HOLD-specific shaping
      if (mode_==HOLD && std::fabs(e) <= hp_.e_hold_db) e = 0.0f;

      // D term with LPF velocity
      float dterm = -hp_.Kd[i] * vel_filt_[i];
      dterm = std::clamp(dterm, -hp_.d_clamp, hp_.d_clamp);

      // Integrator freeze window after mode switch
      if (i_freeze_t_[i] > 0.0f) {
        i_freeze_t_[i] = std::max(0.0f, i_freeze_t_[i] - (float)dt);
      } else {
        // Anti-windup + optional HOLD leak
        I_[i] += e * (float)dt;
        if (mode_==HOLD) I_[i] *= std::max(0.0f, 1.0f - hp_.i_leak*(float)dt);
        I_[i] = std::clamp(I_[i], -hp_.I_max, hp_.I_max);
      }

      float Ki_eff = (mode_==HOLD) ? (hp_.Ki[i] * hp_.ki_hold_scale) : hp_.Ki[i];

      // Slow bias learning only in HOLD
      if (mode_==HOLD){
        bias_ff_[i] += hp_.bias_alpha * e * (float)dt;
        bias_ff_[i] = std::clamp(bias_ff_[i], -hp_.bias_max, hp_.bias_max);
      }

      float ff_raw = (float)tau_raw[i];

      float u = hp_.Kp[i]*e + Ki_eff*I_[i] + dterm + ff_raw + bias_ff_[i];

      // per-axis current limit
      float limit = (i==3) ? 700.0f : (float)hp_.limit_joint_raw[i];
      // but hardware caps differ: 1-3:1188, 4:689
      float hwmax = (i<=2) ? 1188.0f : 689.0f;
      limit = std::min(limit, hwmax);

      u = std::clamp(u, -limit, +limit);

      int32_t goal = (int32_t)std::lround(u);
      goal_bufs_[i][0] = (uint8_t)(goal & 0xFF);
      goal_bufs_[i][1] = (uint8_t)((goal>>8) & 0xFF);
      if (!gsw_->addParam(dxl_ids_[i], goal_bufs_[i].data())) {
        RCLCPP_ERROR(get_logger(), "addParam fail id %u", dxl_ids_[i]);
      }
    }

    // Joint 5 simple position PID in current domain
    {
      float e5 = desired_dxl5_pos_ - pos[4];
      // naive PD(I=0) in LSB units (reuse Kp[3], Kd[3] scale)
      float v5 = (pos[4]-last_pos5_) / (float)dt;
      last_pos5_ = pos[4];

      float u5 = 0.5f*e5 - 0.012f*v5; // from your prior gains
      float limit5 = std::min(700.0f, 689.0f); // per request but capped to HW
      u5 = std::clamp(u5, -limit5, +limit5);

      int32_t goal5 = (int32_t)std::lround(u5);
      goal_bufs_[4][0] = (uint8_t)(goal5 & 0xFF);
      goal_bufs_[4][1] = (uint8_t)((goal5>>8) & 0xFF);
      if (!gsw_->addParam(5, goal_bufs_[4].data())) {
        RCLCPP_ERROR(get_logger(), "addParam fail id 5");
      }
    }

    int wres = gsw_->txPacket();
    if (wres != COMM_SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 500, "SyncWrite fail: %s", ph_->getTxRxResult(wres));
    }
  }

  // ---------- Gravity torque ----------
  Eigen::Vector4d computeGravityTorqueNm(const std::array<double,4>& q) const {
    Matrix4d T01 = dh(q[0], dh_.d1, 0.0, dh_.alpha1);
    Matrix4d T12 = dh(q[1] + dh_.theta2_offset, 0.0, dh_.a2, 0.0);
    Matrix4d T23 = dh(q[2], 0.0, dh_.a3, 0.0);
    Matrix4d T34 = dh(q[3], 0.0, dh_.a4, 0.0);
    Matrix4d T02 = T01*T12;
    Matrix4d T03 = T02*T23;
    Matrix4d T04 = T03*T34;

    Vector3d p0=Vector3d::Zero(), z0(0,0,1);
    Vector3d p1=T01.block<3,1>(0,3), z1=T01.block<3,1>(0,2);
    Vector3d p2=T02.block<3,1>(0,3), z2=T02.block<3,1>(0,2);
    Vector3d p3=T03.block<3,1>(0,3), z3=T03.block<3,1>(0,2);

    Vector4d c1(link_[0].com.x(), link_[0].com.y(), link_[0].com.z(), 1.0);
    Vector4d c2(link_[1].com.x(), link_[1].com.y(), link_[1].com.z(), 1.0);
    Vector4d c3(link_[2].com.x(), link_[2].com.y(), link_[2].com.z(), 1.0);
    Vector4d c4(link_[3].com.x(), link_[3].com.y(), link_[3].com.z(), 1.0);
    Vector3d pc1=(T01*c1).head<3>(), pc2=(T02*c2).head<3>(), pc3=(T03*c3).head<3>(), pc4=(T04*c4).head<3>();

    Eigen::Matrix<double,3,4> Jv1=Eigen::Matrix<double,3,4>::Zero(); Jv1.col(0)=z0.cross(pc1-p0);
    Eigen::Matrix<double,3,4> Jv2=Eigen::Matrix<double,3,4>::Zero(); Jv2.col(0)=z0.cross(pc2-p0); Jv2.col(1)=z1.cross(pc2-p1);
    Eigen::Matrix<double,3,4> Jv3=Eigen::Matrix<double,3,4>::Zero(); Jv3.col(0)=z0.cross(pc3-p0); Jv3.col(1)=z1.cross(pc3-p1); Jv3.col(2)=z2.cross(pc3-p2);
    Eigen::Matrix<double,3,4> Jv4=Eigen::Matrix<double,3,4>::Zero(); Jv4.col(0)=z0.cross(pc4-p0); Jv4.col(1)=z1.cross(pc4-p1); Jv4.col(2)=z2.cross(pc4-p2); Jv4.col(3)=z3.cross(pc4-p3);

    Vector4d tau = Vector4d::Zero();
    Vector3d g = dh_.gravity_vector;
    tau += Jv1.transpose() * (link_[0].m * g);
    tau += Jv2.transpose() * (link_[1].m * g);
    tau += Jv3.transpose() * (link_[2].m * g);
    tau += Jv4.transpose() * (link_[3].m * g);
    return tau;
  }

  inline int32_t torqueNm_to_currentRaw(double tau, int axis) const {
    // tau[Nm] -> motor shaft current [A] -> driver LSB
    // tau = Kt * N * eta * I  => I = tau / (Kt*N*eta)
    const double I = tau / (Kt_[axis] * N_[axis] * eta_[axis]);
    const double raw = I / A_per_count_[axis]; // axis 3 uses 0.00269, axis 4 uses 0.00134
    return (int32_t)std::lround(raw);
  }

  // ---------- Params callback ----------
  rcl_interfaces::msg::SetParametersResult onParamSet(const std::vector<rclcpp::Parameter>& ps){
    for(const auto& p: ps){
      auto nm = p.get_name();
      if (nm.rfind("joint_sign_",0)==0){
        int idx = std::stoi(nm.substr(std::string("joint_sign_").size())) - 1;
        if (0<=idx && idx<4) sign_[idx] = p.as_int();
      }
      if (nm.rfind("joint_zero_",0)==0){
        int idx = std::stoi(nm.substr(std::string("joint_zero_").size())) - 1;
        if (0<=idx && idx<4) zero_count_[idx] = p.as_int();
      }
    }
    rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
  }

  // ---------- Members ----------
  // DH / links
  DHParameters dh_;
  struct LinkInertial { double m; Vector3d com; };
  std::array<LinkInertial,4> link_;

  // DXL
  std::string device_name_;
  double protocol_version_;
  std::vector<uint8_t> dxl_ids_;
  PortHandler* port_ = nullptr;
  PacketHandler* ph_ = nullptr;
  std::shared_ptr<dynamixel::GroupBulkRead>  gbr_;
  std::shared_ptr<dynamixel::GroupSyncWrite> gsw_;

  // Control table
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

  // Publishers / Subscribers
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_cur_, pub_vel_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_pos_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_path_, sub_pos_, sub_dxl5_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  enum CtrlMode { HOLD, MOVE };
  CtrlMode mode_{HOLD};
  bool traj_active_{false};
  rclcpp::Time last_tick_;
  rclcpp::Time t_ref_start_;
  std::deque<std::array<float,4>> path_;
  std::deque<Segment> segs_;
  std::optional<Segment> active_seg_;

  // References and feedback
  std::vector<float> desired_pos_, qref_, qref_prev_, vel_filt_, last_pos_meas_, I_, bias_ff_;
  std::vector<std::array<uint8_t,2>> goal_bufs_;
  float desired_dxl5_pos_{0.f}, last_pos5_{0.f};

  // Encoders to radians
  double POS_COUNT_PER_REV_{4096.0};
  double COUNT2RAD_{2.0 * M_PI / 4096.0};
  std::array<int,4>     sign_{ {+1,+1,+1,+1} };
  std::array<int32_t,4> zero_count_{ {0,0,0,0} };

  // Motor params
  std::array<double,4> N_, eta_;
  std::array<double,4> Kt_;
  std::array<double,4> A_per_count_;

  // FSM helpers
  std::vector<float> i_freeze_t_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

// ---------- Signal safe ----------
std::shared_ptr<GravityArmNode> g_node;
void signalHandler(int signum){
  RCLCPP_INFO(rclcpp::get_logger("signal"), "signal %d received, shutting down...", signum);
  if (g_node) rclcpp::shutdown();
}

int main(int argc, char** argv){
  signal(SIGINT,  signalHandler);
  signal(SIGTERM, signalHandler);
  rclcpp::init(argc, argv);
  try {
    g_node = std::make_shared<GravityArmNode>();
    rclcpp::spin(g_node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception: %s", e.what());
  }
  g_node.reset();
  rclcpp::shutdown();
  return 0;
}
