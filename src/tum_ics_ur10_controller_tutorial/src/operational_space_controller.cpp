#include <tum_ics_ur10_controller_tutorial/operational_space_controller.h>

#include <ros/ros.h>
#include <control_core/math/error_functions.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/SVD>

namespace tum_ics_ur_robot_lli
{
namespace RobotControllers
{

OperationalSpaceControl::OperationalSpaceControl(double weight, const QString &name)
  : ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
    model_("ur10_model"),
    model_ready_(false),
    enable_safe_(true),
    enable_move_(true),
    enable_draw_(true),
    Kp_q6_(Matrix6d::Zero()),
    Kd_safe6_(Matrix6d::Zero()),
    q_safe6_(Vector6d::Zero()),
    safe_tol_(0.01),
    Kp_p_(Matrix3d::Zero()),
    Kp_o_(Matrix3d::Zero()),
    Kd6_(Matrix6d::Zero()),
    move_time_(20.0),
    move_length_(0.5),
    move_initialized_(false),
    t_move0_(0.0),
    R_move_(cc::Rotation3::Identity()),
    draw_time_(20.0),
    draw_length_(0.5),
    draw_initialized_(false),
    t_draw0_(0.0),
    X_start_(cc::Vector3::Zero()),
    R_draw_(cc::Rotation3::Identity()),
    adaptive_enabled_(true),
    adapt_gamma_(0.5),
    adapt_sigma_(0.02),
    theta_hat_max_(200.0),
    theta_hat_(ur::URModel::Parameters::Zero(45, 1)),
    theta_hat_initialized_(false),
    qdot_r_prev6_(Vector6d::Zero()),
    qdot_r_prev_valid_(false),
    prev_time_sec_(0.0),
    phase_(PHASE_SAFE),
    safe_done_(false),
    t0_(-1.0),
    tau_max_(120.0),
    qdot_r_max_(2.0),
    xdot_r_max_(0.25),
    wdot_r_max_(0.80)
{
}

OperationalSpaceControl::~OperationalSpaceControl() {}

void OperationalSpaceControl::setQInit(const JointState &q_init) { q_init_ = q_init; }
void OperationalSpaceControl::setQHome(const JointState &q_home) { q_home_ = q_home; }
void OperationalSpaceControl::setQPark(const JointState &q_park) { q_park_ = q_park; }

bool OperationalSpaceControl::init()
{
  ros::NodeHandle nh_private("~");
  const std::string ns = "~operational_space_ctrl";

  // switches
  ros::param::get(ns + "/enable_safe", enable_safe_);
  ros::param::get(ns + "/enable_move", enable_move_);
  ros::param::get(ns + "/enable_draw", enable_draw_);

  // safe
  ros::param::get(ns + "/safe_tol", safe_tol_);

  std::vector<double> v;

  // q_safe
  if (ros::param::get(ns + "/q_safe", v) && v.size() >= 6)
  {
    for (int i = 0; i < 6; ++i) q_safe6_(i) = v[(size_t)i];
  }
  else
  {
    q_safe6_.setZero();
    q_safe6_(2) = M_PI / 2.0;
  }

  // Kp_safe (diag 6)
  if (ros::param::get(ns + "/Kp_safe", v) && v.size() >= 3)
  {
    Kp_q6_.setZero();
    for (int i = 0; i < 6; ++i)
    {
      const double val = (i < (int)v.size()) ? v[(size_t)i] : v.back();
      Kp_q6_(i, i) = val;
    }
  }
  else
  {
    Kp_q6_.diagonal() << 30, 30, 20, 10, 8, 6;
  }

  // Kd (diag 6) used in MOVE/DRAW damping term too
  if (ros::param::get(ns + "/Kd", v) && v.size() >= 3)
  {
    Kd6_.setZero();
    for (int i = 0; i < 6; ++i)
    {
      const double val = (i < (int)v.size()) ? v[(size_t)i] : v.back();
      Kd6_(i, i) = val;
    }
  }
  else
  {
    Kd6_.diagonal() << 6, 6, 5, 3, 2, 2;
  }

  // Kd_safe (optional; fallback to Kd)
  if (ros::param::get(ns + "/Kd_safe", v) && v.size() >= 3)
  {
    Kd_safe6_.setZero();
    for (int i = 0; i < 6; ++i)
    {
      const double val = (i < (int)v.size()) ? v[(size_t)i] : v.back();
      Kd_safe6_(i, i) = val;
    }
  }
  else
  {
    Kd_safe6_ = Kd6_;
  }

  // MOVE/DRAW gains
  if (ros::param::get(ns + "/Kp_pos", v) && v.size() >= 3)
  {
    Kp_p_.setZero();
    for (int i = 0; i < 3; ++i) Kp_p_(i, i) = v[(size_t)i];
  }
  else
  {
    Kp_p_.diagonal() << 6, 6, 6;
  }

  if (ros::param::get(ns + "/Kp_ori", v) && v.size() >= 3)
  {
    Kp_o_.setZero();
    for (int i = 0; i < 3; ++i) Kp_o_(i, i) = v[(size_t)i];
  }
  else
  {
    Kp_o_.diagonal() << 3, 3, 3;
  }

  // move params
  ros::param::get(ns + "/move/time", move_time_);
  ros::param::get(ns + "/move/length", move_length_);
  if (move_time_ <= 1e-3) move_time_ = 20.0;

  // draw params
  ros::param::get(ns + "/draw/time", draw_time_);
  ros::param::get(ns + "/draw/length", draw_length_);
  if (draw_time_ <= 1e-3) draw_time_ = 20.0;

  // adaptive
  ros::param::get(ns + "/adaptive/enabled", adaptive_enabled_);
  ros::param::get(ns + "/adaptive/gamma", adapt_gamma_);
  ros::param::get(ns + "/adaptive/sigma", adapt_sigma_);
  ros::param::get(ns + "/adaptive/theta_hat_max", theta_hat_max_);

  // saturation
  ros::param::get(ns + "/tau_max", tau_max_);
  ros::param::get(ns + "/qdot_r_max", qdot_r_max_);
  ros::param::get(ns + "/xdot_r_max", xdot_r_max_);
  ros::param::get(ns + "/wdot_r_max", wdot_r_max_);

  // init model
  model_ready_ = model_.initRequest(nh_private);
  if (!model_ready_)
  {
    ROS_ERROR_STREAM("[OperationalSpaceControl] init(): failed to init URModel.");
    m_error = true;
    return false;
  }

  // desired orientation in MOVE:
  // tool z-axis -> +X (world), tool y-axis -> +Y, tool x-axis -> -Z (right-handed)
  R_move_.setIdentity();
  R_move_ << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

  // desired orientation in DRAW:
  // tool z-axis -> +X (world), tool y-axis -> +Y, tool x-axis -> -Z (right-handed)
  R_draw_.setIdentity();
  R_draw_ << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

  // init theta_hat from model initial guess
  theta_hat_ = model_.parameterInitalGuess();
  theta_hat_initialized_ = true;

  // (optional) publishers for debug/rviz
  traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/ur10/desired_trajectory", 1);
  actual_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/ur10/actual_trajectory", 1);
  task_error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur10/task_space_error", 1);
  effort_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur10/joint_effort_debug", 1);
  effort_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/ur10/joint_effort_state", 1);

  publishDeleteAllMarkers();

  traj_marker_.header.frame_id = "world";
  traj_marker_.ns = "traj_desired";
  traj_marker_.id = 0;
  traj_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  traj_marker_.action = visualization_msgs::Marker::ADD;
  traj_marker_.scale.x = 0.01;
  traj_marker_.color.a = 1.0;
  traj_marker_.pose.orientation.w = 1.0;
  traj_marker_.color.r = 0.0;
  traj_marker_.color.g = 0.0;
  traj_marker_.color.b = 1.0;

  actual_traj_marker_.header.frame_id = "world";
  actual_traj_marker_.ns = "traj_actual";
  actual_traj_marker_.id = 0;
  actual_traj_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  actual_traj_marker_.action = visualization_msgs::Marker::ADD;
  actual_traj_marker_.scale.x = 0.01;
  actual_traj_marker_.color.a = 1.0;
  actual_traj_marker_.pose.orientation.w = 1.0;
  actual_traj_marker_.color.r = 1.0;
  actual_traj_marker_.color.g = 0.0;
  actual_traj_marker_.color.b = 0.0;

  ROS_INFO_STREAM("[OperationalSpaceControl] init ok"
                  << " enable_safe=" << (enable_safe_ ? 1 : 0)
                  << " enable_move=" << (enable_move_ ? 1 : 0)
                  << " move_time=" << move_time_
                  << " move_length=" << move_length_
                  << " enable_draw=" << (enable_draw_ ? 1 : 0)
                  << " draw_time=" << draw_time_
                  << " draw_length=" << draw_length_);

  return true;
}

bool OperationalSpaceControl::start()
{
  qdot_r_prev_valid_ = false;
  prev_time_sec_ = 0.0;

  t0_ = -1.0;
  safe_done_ = false;
  move_initialized_ = false;
  t_move0_ = 0.0;
  draw_initialized_ = false;
  t_draw0_ = 0.0;

  // phase select at start
  if (enable_safe_)
    phase_ = PHASE_SAFE;
  else if (enable_move_)
    phase_ = PHASE_MOVE;
  else if (enable_draw_)
    phase_ = PHASE_DRAW;
  else
    phase_ = PHASE_SAFE;

  resetMarkerNewSegment();
  return true;
}

bool OperationalSpaceControl::stop() { return true; }

// -------------------------
// SAFE/MOVE/DRAW phase selection
// -------------------------
bool OperationalSpaceControl::inSafePhase(const cc::JointPosition &q6) const
{
  if (!enable_safe_) return false;
  const double e = (q6 - q_safe6_).norm();
  return (e > safe_tol_);
}

void OperationalSpaceControl::ensureMoveInit(double t_sec, const cc::JointPosition &q6)
{
  if (move_initialized_) return;

  // Use current EE position as start if safe disabled or if user jumps to move
  X_start_ = fkPos(q6);
  R_move_  = fkOri(q6);
  t_move0_ = t_sec;
  move_initialized_ = true;

  ROS_WARN_STREAM("[OperationalSpaceControl] MOVE init: X_start=" << X_start_.transpose()
                  << " t_move0=" << t_move0_);
}

void OperationalSpaceControl::ensureDrawInit(double t_sec, const cc::JointPosition &q6)
{
  if (draw_initialized_) return;

  // Use current EE position as start if safe disabled or if user jumps to draw
  X_start_ = fkPos(q6);
  R_draw_  = fkOri(q6);
  t_draw0_ = t_sec;
  draw_initialized_ = true;

  ROS_WARN_STREAM("[OperationalSpaceControl] DRAW init: X_start=" << X_start_.transpose()
                  << " t_draw0=" << t_draw0_);
}

// -------------------------
// Desired trajectory (MOVE)
// -------------------------
void OperationalSpaceControl::cartesianDesiredMove(
  double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d,
  cc::Rotation3 &Rd, cc::Vector3 &Wd)
{
  const double t = std::max(0.0, t_sec - t_move0_);

  const double T = std::max(1e-3, move_time_);
  const double L = move_length_;   // 直线总长度

  // 归一化时间 0~1
  double s = t / T;
  if (s > 1.0) s = 1.0;

  Xd = X_start_;

  // ===== X 方向直线 =====
  Xd(0) = X_start_(0) - L * s;
  Xd(1) = X_start_(1);
  Xd(2) = X_start_(2);

  // ===== 速度 =====
  Xdot_d.setZero();

  if (t < T)
    Xdot_d(0) = L / T;   // 匀速
  else
    Xdot_d(0) = 0.0;     // 到终点停止

  // 姿态保持不变
  Rd = R_move_;
  Wd.setZero();
}

// -------------------------
// Desired trajectory (DRAW)
// -------------------------
void OperationalSpaceControl::cartesianDesiredDraw(
  double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d,
  cc::Rotation3 &Rd, cc::Vector3 &Wd)
{
  const double t = std::max(0.0, t_sec - t_draw0_);

  double T = draw_time_;
  double R = draw_length_;   // 用 length 当半径

  double omega = 2.0 * M_PI / std::max(1e-3, T);

  Xd = X_start_;

  // ===== XY 平面圆 =====
  Xd(0) = X_start_(0) + R * std::cos(omega * t);
  Xd(1) = X_start_(1) + R * std::sin(omega * t);
  Xd(2) = X_start_(2);

  // 速度
  Xdot_d.setZero();
  Xdot_d(0) = -R * omega * std::sin(omega * t);
  Xdot_d(1) =  R * omega * std::cos(omega * t);

  Rd = R_draw_;
  Wd.setZero();
}

// -------------------------
// Model helpers (FK/J)
// -------------------------
cc::Vector3 OperationalSpaceControl::fkPos(const cc::JointPosition &q6) const
{
  const cc::HomogeneousTransformation T_0_B = model_.T_0_B();
  const cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(q6);
  const cc::HomogeneousTransformation T = T_0_B * T_tool_0;
  cc::Vector3 X;
  X << T.pos()(0), T.pos()(1), T.pos()(2);
  return X;
}

cc::Rotation3 OperationalSpaceControl::fkOri(const cc::JointPosition &q6) const
{
  const cc::HomogeneousTransformation T_0_B = model_.T_0_B();
  const cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(q6);
  const cc::HomogeneousTransformation T = T_0_B * T_tool_0;
  return T.orientation();
}

OperationalSpaceControl::Matrix6d OperationalSpaceControl::jacobian6(const cc::JointPosition &q6) const
{
  const cc::Jacobian J_tool_0 = model_.J_tool_0(q6);
  const cc::HomogeneousTransformation T_0_B = model_.T_0_B();

  Matrix3d R_0_B;
  R_0_B << T_0_B(0,0), T_0_B(0,1), T_0_B(0,2),
           T_0_B(1,0), T_0_B(1,1), T_0_B(1,2),
           T_0_B(2,0), T_0_B(2,1), T_0_B(2,2);

  Matrix6d J6 = Matrix6d::Zero();
  J6.block<3,6>(0,0) = R_0_B * J_tool_0.block<3,6>(0,0);
  J6.block<3,6>(3,0) = R_0_B * J_tool_0.block<3,6>(3,0);
  return J6;
}

// -------------------------
// DLS
// -------------------------
OperationalSpaceControl::Vector6d
OperationalSpaceControl::dlsSolve6(const Matrix6d &J, const Vector6d &xdot, double lambda)
{
  const Matrix6d A = (J * J.transpose() + (lambda * lambda) * Matrix6d::Identity());
  const Vector6d y = A.inverse() * xdot;
  return J.transpose() * y;
}

OperationalSpaceControl::Vector6d
OperationalSpaceControl::dlsSolve3(const Matrix3x6d &Jp, const cc::Vector3 &xdot_p, double lambda)
{
  const Matrix3d A = (Jp * Jp.transpose() + (lambda * lambda) * Matrix3d::Identity());
  const cc::Vector3 y = A.inverse() * xdot_p;
  return Jp.transpose() * y;
}

// -------------------------
// MOVE/DRAW reference qdot_r
// -------------------------
OperationalSpaceControl::Vector6d
OperationalSpaceControl::computeQdotR_MOVE_and_DRAW(double t_sec, const cc::JointPosition &q6)
{
  cc::Vector3 Xd, Xdot_d, Wd;
  cc::Rotation3 Rd;
  if (phase_ == PHASE_MOVE)
  {
      cartesianDesiredMove(t_sec, Xd, Xdot_d, Rd, Wd);
  }
  else if (phase_ == PHASE_DRAW)
  {
      cartesianDesiredDraw(t_sec, Xd, Xdot_d, Rd, Wd);
  }

  // current
  const cc::HomogeneousTransformation T_0_B = model_.T_0_B();
  const cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(q6);
  const cc::HomogeneousTransformation T = T_0_B * T_tool_0;

  // desired transform
  cc::HomogeneousTransformation Td = cc::HomogeneousTransformation::Identity();
  Td.pos() = Xd;
  Td.orientation() = Rd;

  // 6D cartesian error in world
  const cc::CartesianVector e = cc::cartesianErrorWorld(Td, T);

  // reference task velocity
  Vector6d xdot_d6 = Vector6d::Zero();
  xdot_d6.head<3>() = Xdot_d;
  xdot_d6.tail<3>() = Wd;

  // Kp (pos + ori)
  Matrix6d Kp6 = Matrix6d::Zero();
  Kp6.topLeftCorner<3,3>() = Kp_p_;
  Kp6.bottomRightCorner<3,3>() = Kp_o_;

  Vector6d xdot_r6 = xdot_d6 + Kp6 * e;

  // clamp (optional safety)
  for (int i = 0; i < 3; ++i)
    xdot_r6(i) = std::max(-xdot_r_max_, std::min(xdot_r6(i), xdot_r_max_));
  for (int i = 3; i < 6; ++i)
    xdot_r6(i) = std::max(-wdot_r_max_, std::min(xdot_r6(i), wdot_r_max_));

  // joint ref velocity
  const Matrix6d J6 = jacobian6(q6);
  // ================= DEBUG: Jacobian condition =================
  Eigen::JacobiSVD<Matrix6d> svd(J6);
  Eigen::VectorXd S = svd.singularValues();

  double sigma_max = S(0);
  double sigma_min = S(S.size()-1);
  double condJ = sigma_max / std::max(sigma_min, 1e-12);

  double e_pos_norm = e.head<3>().norm();
  double e_ori_norm = e.tail<3>().norm();

  double xdot_pos_norm = xdot_r6.head<3>().norm();
  double wdot_norm     = xdot_r6.tail<3>().norm();

  ROS_INFO_STREAM_THROTTLE(1.0,
    "[MOVE/DRAW DEBUG] "
    << "condJ=" << condJ
    << " sigma_min=" << sigma_min
    << " e_pos=" << e_pos_norm
    << " e_ori=" << e_ori_norm
    << " |xdot_p|=" << xdot_pos_norm
    << " |wdot|=" << wdot_norm
  );
  // =============================================================

  Vector6d qdot_r = dlsSolve6(J6, xdot_r6, 0.05);

  for (int i = 0; i < 6; ++i)
    qdot_r(i) = std::max(-qdot_r_max_, std::min(qdot_r(i), qdot_r_max_));

  // publish error
  std_msgs::Float64MultiArray msg;
  msg.data = {e(0), e(1), e(2), e(3), e(4), e(5)};
  task_error_pub_.publish(msg);

  return qdot_r;
}

// -------------------------
// main update
// -------------------------
OperationalSpaceControl::Vector6d
OperationalSpaceControl::update(const RobotTime &time, const JointState &state)
{
  Vector6d tau = Vector6d::Zero();
  if (!model_ready_) return tau;

  const double t_sec = time.tD();
  if (t0_ < 0.0) t0_ = t_sec;

  const cc::JointPosition q6 = state.q;
  const cc::JointVelocity qP6 = state.qp;

  double dt = t_sec - prev_time_sec_;
  if (prev_time_sec_ <= 0.0 || !std::isfinite(dt) || dt <= 0.0) dt = 1e-3;
  prev_time_sec_ = t_sec;

  // phase routing
  if (!enable_safe_ && enable_move_ && !enable_draw_) phase_ = PHASE_MOVE;
  else if (!enable_safe_ && !enable_move_ && enable_draw_) phase_ = PHASE_DRAW;
  else if (!enable_safe_ && !enable_move_ && !enable_draw_) return tau; // all off
  else if (enable_safe_ && !safe_done_) phase_ = PHASE_SAFE;

  // -------------------------
  // SAFE
  // -------------------------
  if (phase_ == PHASE_SAFE)
  {
    const double e_norm = (q6 - q_safe6_).norm();
    if (e_norm <= safe_tol_)
    {
      safe_done_ = true;
      if (enable_move_) phase_ = PHASE_MOVE;
      move_initialized_ = false;
      qdot_r_prev_valid_ = false; // avoid qddot spike on transition
      resetMarkerNewSegment();
    }

    // qrP = -Kp * (q-q_safe)
    const Vector6d e = (q6 - q_safe6_);

    // debug: print joint error every 1s for tuning
    ROS_INFO_STREAM_THROTTLE(1.0, "[SAFE] e(q-q_safe)= "
      << e.transpose().format(Eigen::IOFormat(4, 0, " ", " ", "", "", "[", "]"))
      << "  |e|=" << e.norm());

    const Vector6d qrP = (-Kp_q6_) * e;

    // s = qP - qrP
    const Vector6d s = qP6 - qrP;

    // qrPP = 0
    cc::JointAcceleration qrPP = cc::JointAcceleration::Zero();

    // regressor & adaptive
    const ur::URModel::Regressor &Y = model_.regressor(q6, qP6, qrP, qrPP);

    if (!theta_hat_initialized_ || theta_hat_.size() != Y.cols())
    {
      theta_hat_ = model_.parameterInitalGuess();
      if (theta_hat_.size() != Y.cols()) theta_hat_.setZero(Y.cols(), 1);
      theta_hat_initialized_ = true;
    }

    Vector6d tau_ff = Vector6d::Zero();
    if (adaptive_enabled_)
    {
      tau_ff = Y * theta_hat_;

      ur::URModel::Parameters theta_dot =
        (-adapt_gamma_) * (Y.transpose() * s) - adapt_sigma_ * theta_hat_;
      theta_hat_ += theta_dot * dt;

      if (theta_hat_max_ > 0.0)
      {
        for (int i = 0; i < theta_hat_.size(); ++i)
          theta_hat_(i) = std::max(-theta_hat_max_, std::min(theta_hat_(i), theta_hat_max_));
      }
    }

    tau = (-Kd_safe6_) * s + tau_ff;

    // saturate
    for (int i = 0; i < 6; ++i)
      tau(i) = std::max(-tau_max_, std::min(tau(i), tau_max_));

    publishControlDebug(t_sec, "SAFE", state, tau);
    return tau;
  }

  // -------------------------
  // MOVE
  // -------------------------
  if (phase_ == PHASE_MOVE)
  {
    if (!enable_move_) return tau;

    ensureMoveInit(t_sec, q6);

    // ===== MOVE 完成判定 =====
    double t_move = t_sec - t_move0_;
    if (t_move >= move_time_)
    {
      ROS_WARN_STREAM("[PHASE SWITCH] MOVE -> DRAW");

      phase_ = PHASE_DRAW;
      draw_initialized_ = false;
      qdot_r_prev_valid_ = false;   // 避免 qddot_r 突变
      resetMarkerNewSegment();
      return Vector6d::Zero();      // 当前周期先不输出旧控制

    }

    // qdot_r from task ref (pos + ori lock)
    const Vector6d qdot_r = computeQdotR_MOVE_and_DRAW(t_sec, q6);

    ROS_INFO_STREAM_THROTTLE(1.0,
      "[MOVE DEBUG] |qdot_r|=" << qdot_r.norm()
      << " |qdot|=" << qP6.norm());


    // qddot_r by numeric diff
    Vector6d qddot_r = Vector6d::Zero();
    if (qdot_r_prev_valid_ && dt > 1e-6)
      qddot_r = (qdot_r - qdot_r_prev6_) / dt;
    qdot_r_prev6_ = qdot_r;
    qdot_r_prev_valid_ = true;

    // sliding var
    const Vector6d s = qP6 - qdot_r;

    // regressor
    const ur::URModel::Regressor &Y = model_.regressor(q6, qP6, qdot_r, qddot_r);

    if (!theta_hat_initialized_ || theta_hat_.size() != Y.cols())
    {
      theta_hat_ = model_.parameterInitalGuess();
      if (theta_hat_.size() != Y.cols()) theta_hat_.setZero(Y.cols(), 1);
      theta_hat_initialized_ = true;
    }

    Vector6d tau_ff = Vector6d::Zero();
    if (adaptive_enabled_)
    {
      tau_ff = Y * theta_hat_;

      ur::URModel::Parameters theta_dot =
        (-adapt_gamma_) * (Y.transpose() * s) - adapt_sigma_ * theta_hat_;
      theta_hat_ += theta_dot * dt;

      if (theta_hat_max_ > 0.0)
      {
        for (int i = 0; i < theta_hat_.size(); ++i)
          theta_hat_(i) = std::max(-theta_hat_max_, std::min(theta_hat_(i), theta_hat_max_));
      }
    }

    tau = (-Kd6_) * s + tau_ff;

    for (int i = 0; i < 6; ++i)
      tau(i) = std::max(-tau_max_, std::min(tau(i), tau_max_));

    publishControlDebug(t_sec, "MOVE", state, tau);

    // trajectory markers (optional)
    {
      cc::Vector3 Xd, Xdot_d, Wd;
      cc::Rotation3 Rd;
      cartesianDesiredMove(t_sec, Xd, Xdot_d, Rd, Wd);

      geometry_msgs::Point pd;
      pd.x = Xd(0); pd.y = Xd(1); pd.z = Xd(2);
      traj_points_.push_back(pd);

      if (traj_points_.size() >= 2)
      {
        traj_marker_.points = traj_points_;
        traj_marker_.header.stamp = ros::Time::now();
        traj_pub_.publish(traj_marker_);
      }

      const cc::Vector3 X = fkPos(q6);
      geometry_msgs::Point pa;
      pa.x = X(0); pa.y = X(1); pa.z = X(2);
      actual_traj_points_.push_back(pa);

      if (actual_traj_points_.size() >= 2)
      {
        actual_traj_marker_.points = actual_traj_points_;
        actual_traj_marker_.header.stamp = ros::Time::now();
        actual_traj_pub_.publish(actual_traj_marker_);
      }
    }

    return tau;
  }

  // -------------------------
  // DRAW
  // -------------------------
  if (phase_ == PHASE_DRAW)
  {
    if (!enable_draw_) return tau;

    ensureDrawInit(t_sec, q6);

    // ===== DRAW 完成判定 =====
    double t_draw = t_sec - t_draw0_;
    if (t_draw >= draw_time_)
    {
      ROS_WARN_STREAM("[PHASE SWITCH] DRAW -> SAFE");

      phase_ = PHASE_SAFE;
      qdot_r_prev_valid_ = false;   // 避免 qddot_r 突变
      resetMarkerNewSegment();


    }


    // qdot_r from task ref (pos + ori lock)
    const Vector6d qdot_r = computeQdotR_MOVE_and_DRAW(t_sec, q6);

    ROS_INFO_STREAM_THROTTLE(1.0,
      "[DRAW DEBUG] |qdot_r|=" << qdot_r.norm()
      << " |qdot|=" << qP6.norm());


    // qddot_r by numeric diff
    Vector6d qddot_r = Vector6d::Zero();
    if (qdot_r_prev_valid_ && dt > 1e-6)
      qddot_r = (qdot_r - qdot_r_prev6_) / dt;
    qdot_r_prev6_ = qdot_r;
    qdot_r_prev_valid_ = true;

    // sliding var
    const Vector6d s = qP6 - qdot_r;

    // regressor
    const ur::URModel::Regressor &Y = model_.regressor(q6, qP6, qdot_r, qddot_r);

    if (!theta_hat_initialized_ || theta_hat_.size() != Y.cols())
    {
      theta_hat_ = model_.parameterInitalGuess();
      if (theta_hat_.size() != Y.cols()) theta_hat_.setZero(Y.cols(), 1);
      theta_hat_initialized_ = true;
    }

    Vector6d tau_ff = Vector6d::Zero();
    if (adaptive_enabled_)
    {
      tau_ff = Y * theta_hat_;

      ur::URModel::Parameters theta_dot =
        (-adapt_gamma_) * (Y.transpose() * s) - adapt_sigma_ * theta_hat_;
      theta_hat_ += theta_dot * dt;

      if (theta_hat_max_ > 0.0)
      {
        for (int i = 0; i < theta_hat_.size(); ++i)
          theta_hat_(i) = std::max(-theta_hat_max_, std::min(theta_hat_(i), theta_hat_max_));
      }
    }

    tau = (-Kd6_) * s + tau_ff;

    for (int i = 0; i < 6; ++i)
      tau(i) = std::max(-tau_max_, std::min(tau(i), tau_max_));

    publishControlDebug(t_sec, "DRAW", state, tau);

    // trajectory markers (optional)
    {
      cc::Vector3 Xd, Xdot_d, Wd;
      cc::Rotation3 Rd;
      cartesianDesiredDraw(t_sec, Xd, Xdot_d, Rd, Wd);

      geometry_msgs::Point pd;
      pd.x = Xd(0); pd.y = Xd(1); pd.z = Xd(2);
      traj_points_.push_back(pd);

      if (traj_points_.size() >= 2)
      {
        traj_marker_.points = traj_points_;
        traj_marker_.header.stamp = ros::Time::now();
        traj_pub_.publish(traj_marker_);
      }

      const cc::Vector3 X = fkPos(q6);
      geometry_msgs::Point pa;
      pa.x = X(0); pa.y = X(1); pa.z = X(2);
      actual_traj_points_.push_back(pa);

      if (actual_traj_points_.size() >= 2)
      {
        actual_traj_marker_.points = actual_traj_points_;
        actual_traj_marker_.header.stamp = ros::Time::now();
        actual_traj_pub_.publish(actual_traj_marker_);
      }
    }

    return tau;
  }

  return tau;
}

// -------------------------
// RViz/debug helpers
// -------------------------
void OperationalSpaceControl::publishDeleteAllMarkers()
{
  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  clear.header.frame_id = "world";
  for (int i = 0; i < 2; ++i)
  {
    clear.header.stamp = ros::Time::now();
    traj_pub_.publish(clear);
    actual_traj_pub_.publish(clear);
    ros::Duration(0.03).sleep();
  }
}

void OperationalSpaceControl::resetMarkerNewSegment()
{
  traj_marker_.id += 1;
  traj_points_.clear();
  traj_marker_.points.clear();
  traj_marker_.header.stamp = ros::Time::now();

  actual_traj_marker_.id += 1;
  actual_traj_points_.clear();
  actual_traj_marker_.points.clear();
  actual_traj_marker_.header.stamp = ros::Time::now();
}

void OperationalSpaceControl::publishControlDebug(
  double t_sec, const std::string &phase, const JointState &state, const Vector6d &tau_cmd)
{
  double phase_id = -1.0;
  if (phase == "SAFE") phase_id = 0.0;
  else if (phase == "MOVE") phase_id = 1.0;
  else if (phase == "DRAW") phase_id = 2.0;

  std_msgs::Float64MultiArray dbg;
  dbg.data.reserve(2 + 6 + 6 + 6 + 6);
  dbg.data.push_back(t_sec);
  dbg.data.push_back(phase_id);
  for (int i = 0; i < 6; ++i) dbg.data.push_back(tau_cmd(i));
  for (int i = 0; i < 6; ++i) dbg.data.push_back(state.tau(i));
  for (int i = 0; i < 6; ++i) dbg.data.push_back(state.q(i));
  for (int i = 0; i < 6; ++i) dbg.data.push_back(state.qp(i));
  effort_debug_pub_.publish(dbg);

  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.name.resize(6);
  js.position.resize(6);
  js.velocity.resize(6);
  js.effort.resize(6);
  for (int i = 0; i < 6; ++i)
  {
    js.name[i] = "joint_" + std::to_string(i + 1);
    js.position[i] = state.q(i);
    js.velocity[i] = state.qp(i);
    js.effort[i] = tau_cmd(i);
  }
  effort_joint_state_pub_.publish(js);
}

} // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli