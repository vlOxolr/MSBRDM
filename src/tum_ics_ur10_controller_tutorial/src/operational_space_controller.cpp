#include <tum_ics_ur10_controller_tutorial/operational_space_controller.h>

#include <ros/ros.h>
#include <control_core/math/error_functions.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
struct FittedTrajConfig
{
  bool loaded = false;
  bool enabled = true;

  bool fixed_z = true;
  double z_paper = -0.40;

  bool fixed_x = false;
  double x_paper = 0.0;

  bool start_with_safe = true;
  double safe_time = 3.0;
  double safe_tol = 0.05;

  bool use_move_to_start = true;
  double move_time = 20.0;
  double move_tol = 0.01;

  bool clamp_time = true;
  double T_end = 10.0;

  double t0 = -1.0;
  double t_draw0 = -1.0;

  bool moving = false;
  bool draw_started = false;

  bool have_start = false;
  cc::Vector3 X_start = cc::Vector3::Zero();

  std::vector<double> ax{0,0,0,0,0,0};
  std::vector<double> ay{0,0,0,0,0,0};
  std::vector<double> az{0,0,0,0,0,0};

  bool use_sin_x = false, use_sin_y = false, use_sin_z = false;
  double Ax = 0.0, wx = 0.0, phix = 0.0;
  double Ay = 0.0, wy = 0.0, phiy = 0.0;
  double Az = 0.0, wz = 0.0, phiz = 0.0;

  void resetTime()
  {
    t0 = -1.0;
    t_draw0 = -1.0;
    draw_started = false;
    moving = false;
    have_start = false;
    X_start.setZero();
  }

  static double polyN(const std::vector<double> &c, double t)
  {
    if (c.empty()) return 0.0;
    double acc = c.back();
    for (int i = (int)c.size() - 2; i >= 0; --i)
      acc = acc * t + c[(size_t)i];
    return acc;
  }

  static double dpolyN(const std::vector<double> &c, double t)
  {
    if (c.size() <= 1) return 0.0;
    double acc = (double)(c.size() - 1) * c.back();
    for (int i = (int)c.size() - 2; i >= 1; --i)
      acc = acc * t + (double)i * c[(size_t)i];
    return acc;
  }

  void loadFromParams(const std::string &ns)
  {
    ros::param::get(ns + "/traj/enabled", enabled);
    ros::param::get(ns + "/traj/fixed_z", fixed_z);
    ros::param::get(ns + "/traj/z_paper", z_paper);
    ros::param::get(ns + "/traj/fixed_x", fixed_x);
    ros::param::get(ns + "/traj/x_paper", x_paper);

    ros::param::get(ns + "/traj/start_with_safe", start_with_safe);
    ros::param::get(ns + "/traj/safe_time", safe_time);
    ros::param::get(ns + "/traj/safe_tol", safe_tol);

    ros::param::get(ns + "/traj/use_move_to_start", use_move_to_start);
    ros::param::get(ns + "/traj/move_time", move_time);
    ros::param::get(ns + "/traj/move_tol", move_tol);

    ros::param::get(ns + "/traj/clamp_time", clamp_time);
    ros::param::get(ns + "/traj/T_end", T_end);

    std::vector<double> v;
    auto loadPoly = [&](const std::string &key, std::vector<double> &c)
    {
      if (ros::param::get(ns + key, v) && !v.empty())
      {
        c = v;
        return true;
      }
      return false;
    };

    loadPoly("/traj/ax", ax);
    loadPoly("/traj/ay", ay);
    loadPoly("/traj/az", az);

    ros::param::get(ns + "/traj/use_sin_x", use_sin_x);
    ros::param::get(ns + "/traj/use_sin_y", use_sin_y);
    ros::param::get(ns + "/traj/use_sin_z", use_sin_z);

    ros::param::get(ns + "/traj/sin_x/A", Ax);
    ros::param::get(ns + "/traj/sin_x/w", wx);
    ros::param::get(ns + "/traj/sin_x/phi", phix);

    ros::param::get(ns + "/traj/sin_y/A", Ay);
    ros::param::get(ns + "/traj/sin_y/w", wy);
    ros::param::get(ns + "/traj/sin_y/phi", phiy);

    ros::param::get(ns + "/traj/sin_z/A", Az);
    ros::param::get(ns + "/traj/sin_z/w", wz);
    ros::param::get(ns + "/traj/sin_z/phi", phiz);

    loaded = true;
  }

  void eval(double t_local, double &x, double &y, double &z, double &xd, double &yd, double &zd) const
  {
    double tt = t_local;

    if (clamp_time)
    {
      if (tt < 0.0) tt = 0.0;
      if (tt > T_end) tt = T_end;
    }

    x  = polyN(ax, tt);
    y  = polyN(ay, tt);
    z  = polyN(az, tt);

    xd = dpolyN(ax, tt);
    yd = dpolyN(ay, tt);
    zd = dpolyN(az, tt);

    if (use_sin_x) { x += Ax * std::sin(wx * tt + phix); xd += Ax * wx * std::cos(wx * tt + phix); }
    if (use_sin_y) { y += Ay * std::sin(wy * tt + phiy); yd += Ay * wy * std::cos(wy * tt + phiy); }
    if (use_sin_z) { z += Az * std::sin(wz * tt + phiz); zd += Az * wz * std::cos(wz * tt + phiz); }

    if (fixed_z) { z = z_paper; zd = 0.0; }
    if (fixed_x) { x = x_paper; xd = 0.0; }
  }

  void computeStartIfNeeded()
  {
    if (have_start) return;
    double x0, y0, z0, xd0, yd0, zd0;
    eval(0.0, x0, y0, z0, xd0, yd0, zd0);
    X_start << x0, y0, z0;
    have_start = true;
  }
};

static FittedTrajConfig g_traj;
} // namespace

namespace tum_ics_ur_robot_lli
{
namespace RobotControllers
{

OperationalSpaceControl::OperationalSpaceControl(double weight, const QString &name)
  : ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
    model_("ur10_model"),
    model_ready_(false),
    Kd6_(Matrix6d::Zero()),
    Kp_p_(Matrix3d::Zero()),
    Ki_p_(Matrix3d::Zero()),
    Kp_o_(Matrix3d::Zero()),
    Ki_o_(Matrix3d::Zero()),
    Kp_q6_(Matrix6d::Zero()),
    int_task_(Vector6d::Zero()),
    int_p_max_(0.05),
    int_o_max_(0.20),
    q_safe6_(Vector6d::Zero()),
    R_safe_(cc::Rotation3::RPY(0.0, 0.0, 0.0)),
    qdot_r_prev6_(Vector6d::Zero()),
    qdot_r_prev_valid_(false),
    prev_time_sec_(0.0),
    tau_max_(120.0),
    xdot_r_max_(0.25),
    wdot_r_max_(0.80),
    qdot_r_max_(2.0),
    controller_type_(OP_PID),
    actual_traj_frame_("world"),
    actual_traj_ee_link_("ursa_ee_link")
{
}

OperationalSpaceControl::~OperationalSpaceControl() {}

void OperationalSpaceControl::setQInit(const JointState &q_init) { q_init_ = q_init; }
void OperationalSpaceControl::setQHome(const JointState &q_home) { q_home_ = q_home; }
void OperationalSpaceControl::setQPark(const JointState &q_park) { q_park_ = q_park; }

bool OperationalSpaceControl::init()
{
  std::string ns = "~operational_space_ctrl";
  std::vector<double> v;

  // Kd (6 or >=3 acceptable; fill diag)
  if (ros::param::get(ns + "/Kd", v) && v.size() >= 3)
  {
    Kd6_.setZero();
    for (int i = 0; i < 6; ++i)
    {
      double val = (i < (int)v.size()) ? v[(size_t)i] : v.back();
      Kd6_(i, i) = val;
    }
  }
  else
  {
    Kd6_.diagonal() << 4,4,3,1,1,1;
  }

  if (ros::param::get(ns + "/Kp_x", v) && v.size() >= 3)
  {
    Kp_p_.setZero();
    for (int i = 0; i < 3; ++i) Kp_p_(i, i) = v[(size_t)i];
  }
  else
  {
    Kp_p_.diagonal() << 7, 7, 6;
  }

  if (ros::param::get(ns + "/Ki_x", v) && v.size() >= 3)
  {
    Ki_p_.setZero();
    for (int i = 0; i < 3; ++i) Ki_p_(i, i) = v[(size_t)i];
  }
  else
  {
    Ki_p_.diagonal() << 4, 4, 4;
  }

  if (ros::param::get(ns + "/Kp_o", v) && v.size() >= 3)
  {
    Kp_o_.setZero();
    for (int i = 0; i < 3; ++i) Kp_o_(i, i) = v[(size_t)i];
  }
  else
  {
    Kp_o_.diagonal() << 3, 3, 3;
  }

  if (ros::param::get(ns + "/Ki_o", v) && v.size() >= 3)
  {
    Ki_o_.setZero();
    for (int i = 0; i < 3; ++i) Ki_o_(i, i) = v[(size_t)i];
  }
  else
  {
    Ki_o_.setZero();
  }

  // Kp_q (for SAFE torque PD, diag 6)
  if (ros::param::get(ns + "/Kp_q", v) && v.size() >= 3)
  {
    Kp_q6_.setZero();
    for (int i = 0; i < 6; ++i)
    {
      double val = (i < (int)v.size()) ? v[(size_t)i] : v.back();
      Kp_q6_(i, i) = val;
    }
  }
  else
  {
    Kp_q6_.diagonal() << 25,25,15,8,6,4;
  }

  ros::param::get(ns + "/int_x_max", int_p_max_);
  ros::param::get(ns + "/int_o_max", int_o_max_);

  // q_safe (allow 3 or 6; if only 3, tail stays 0)
  if (ros::param::get(ns + "/q_safe", v) && v.size() >= 3)
  {
    q_safe6_.setZero();
    for (int i = 0; i < std::min(6, (int)v.size()); ++i)
      q_safe6_(i) = v[(size_t)i];
  }
  else
  {
    q_safe6_.setZero();
    q_safe6_(2) = M_PI / 2.0;
  }

  ros::param::get(ns + "/tau_max", tau_max_);
  ros::param::get(ns + "/xdot_r_max", xdot_r_max_);
  ros::param::get(ns + "/wdot_r_max", wdot_r_max_);
  ros::param::get(ns + "/qdot_r_max", qdot_r_max_);
  ros::param::param<std::string>(ns + "/actual_traj/frame_id", actual_traj_frame_, "world");
  ros::param::param<std::string>(ns + "/actual_traj/ee_link", actual_traj_ee_link_, "ursa_ee_link");

  int ctrl_type = (int)controller_type_;
  ros::param::get(ns + "/controller_type", ctrl_type);
  controller_type_ = (ctrl_type == (int)OP_PD) ? OP_PD : OP_PID;

  g_traj.loadFromParams(ns);

  // Model init
  model_ready_ = model_.initRequest(nh_);
  if (!model_ready_)
  {
    ROS_ERROR_STREAM("OperationalSpaceControl init(): failed to init URModel.");
    m_error = true;
    return false;
  }

  cc::JointPosition q_safe_cc = cc::JointPosition::Zero();
  q_safe_cc.head<6>() = q_safe6_;
  R_safe_ = fkOri(q_safe_cc);

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

  actual_traj_marker_.header.frame_id = actual_traj_frame_;
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

  ROS_INFO_STREAM("[OperationalSpaceControl] actual trajectory TF: "
                  << actual_traj_frame_ << " -> " << actual_traj_ee_link_);

  return true;
}

bool OperationalSpaceControl::start()
{
  qdot_r_prev_valid_ = false;
  prev_time_sec_ = 0.0;
  resetIntegrators();
  g_traj.resetTime();
  resetMarkerNewSegment();

  if (model_ready_)
  {
    cc::JointPosition q6 = cc::JointPosition::Zero();
    q6.head<6>() = q_safe6_;
    const cc::Vector3 Xs = fkPos(q6);
    ROS_INFO_STREAM("[OperationalSpaceControl] X_safe (world) = " << Xs.transpose());
  }
  return true;
}

bool OperationalSpaceControl::stop() { return true; }

void OperationalSpaceControl::resetIntegrators() { int_task_.setZero(); }

bool OperationalSpaceControl::lookupActualEePositionTf(cc::Vector3 &X)
{
  try
  {
    tf::StampedTransform T_frame_ee;
    tf_listener_.lookupTransform(actual_traj_frame_, actual_traj_ee_link_, ros::Time(0), T_frame_ee);
    X << T_frame_ee.getOrigin().x(), T_frame_ee.getOrigin().y(), T_frame_ee.getOrigin().z();
    return true;
  }
  catch (const tf::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(1.0,
      "[OperationalSpaceControl] TF lookup failed for actual trajectory ("
      << actual_traj_frame_ << " -> " << actual_traj_ee_link_ << "): " << ex.what());
    return false;
  }
}

void OperationalSpaceControl::publishControlDebug(double t_sec, const std::string &phase, const JointState &state, const Vector6d &tau_cmd)
{
  double phase_id = -1.0;
  if (phase == "SAFE")
    phase_id = 0.0;
  else if (phase == "MOVE")
    phase_id = 1.0;
  else if (phase == "DRAW")
    phase_id = 2.0;
  else if (phase == "DISABLED")
    phase_id = 3.0;

  std_msgs::Float64MultiArray dbg;
  dbg.data.reserve(2 + 6 + 6 + 6 + 6);
  dbg.data.push_back(t_sec);
  dbg.data.push_back(phase_id);
  for (int i = 0; i < 6; ++i)
    dbg.data.push_back(tau_cmd(i));
  for (int i = 0; i < 6; ++i)
    dbg.data.push_back(state.tau(i));
  for (int i = 0; i < 6; ++i)
    dbg.data.push_back(state.q(i));
  for (int i = 0; i < 6; ++i)
    dbg.data.push_back(state.qp(i));
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

// ===== SAFE phase check =====
bool OperationalSpaceControl::inSafePhase(double t_sec, const cc::JointPosition &q6) const
{
  if (!g_traj.enabled || !g_traj.start_with_safe || g_traj.t0 < 0.0)
    return false;

  // Once MOVE/DRAW starts, never re-enter SAFE to avoid phase chattering.
  if (g_traj.moving || g_traj.draw_started)
    return false;

  const double safe_elapsed = t_sec - g_traj.t0;
  if (safe_elapsed < g_traj.safe_time)
    return true;

  const Vector6d dq6 = q6 - q_safe6_;
  return (dq6.head<3>().norm() > g_traj.safe_tol);
}


// ===== FK =====
cc::Vector3 OperationalSpaceControl::fkPos(const cc::JointPosition &q6) const
{
  const cc::HomogeneousTransformation T = model_.T_ef_0(q6);
  cc::Vector3 X;
  X << T.pos()(0), T.pos()(1), T.pos()(2);
  return X;
}

cc::Rotation3 OperationalSpaceControl::fkOri(const cc::JointPosition &q6) const
{
  const cc::HomogeneousTransformation T = model_.T_ef_0(q6);
  cc::Rotation3 R;
  R = T.orientation();
  return R;
}

// ===== Jacobian (6x6) =====
OperationalSpaceControl::Matrix6d
OperationalSpaceControl::jacobian6(const cc::JointPosition &q6) const
{
  const cc::Jacobian J = model_.J_ef_0(q6);
  Matrix6d J6 = J;
  return J6;
}

// ===== DLS solve =====
OperationalSpaceControl::Vector6d
OperationalSpaceControl::dlsSolve6(const Matrix6d &J, const Vector6d &xdot, double lambda)
{
  Matrix6d A = (J * J.transpose() + (lambda * lambda) * Matrix6d::Identity());
  Vector6d y = A.inverse() * xdot;
  Vector6d qdot = J.transpose() * y;
  return qdot;
}

OperationalSpaceControl::Vector6d
OperationalSpaceControl::dlsSolve3(const Matrix3x6d &Jp, const cc::Vector3 &xdot_p, double lambda)
{
  Matrix3d A = (Jp * Jp.transpose() + (lambda * lambda) * Matrix3d::Identity());
  cc::Vector3 y = A.inverse() * xdot_p;
  Vector6d qdot = Jp.transpose() * y;
  return qdot;
}

// ===== Operational xdot_r =====
OperationalSpaceControl::Vector6d OperationalSpaceControl::xdotROperational(double t_sec, const cc::JointPosition &q6, double dt)
{
  cc::Vector3 Xd, Xdot_d, Wd;
  cc::Rotation3 Rd;
  cartesianDesired(t_sec, Xd, Xdot_d, Rd, Wd);

  cc::HomogeneousTransformation Td = cc::HomogeneousTransformation::Identity();
  Td.pos() = Xd;
  Td.orientation() = Rd;
  const cc::HomogeneousTransformation T = model_.T_ef_0(q6);
  const cc::CartesianVector e = cc::cartesianErrorWorld(Td, T);

  Matrix6d Kp6 = Matrix6d::Zero();
  Kp6.topLeftCorner<3, 3>() = Kp_p_;
  Kp6.bottomRightCorner<3, 3>() = Kp_o_;

  Matrix6d Ki6 = Matrix6d::Zero();
  Ki6.topLeftCorner<3, 3>() = Ki_p_;
  Ki6.bottomRightCorner<3, 3>() = Ki_o_;

  Vector6d xdot_d6 = Vector6d::Zero();
  xdot_d6.head<3>() = Xdot_d;
  xdot_d6.tail<3>() = Wd;

  Vector6d xdot_r = xdot_d6 + Kp6 * e;

  if (controller_type_ == OP_PID)
  {
    int_task_ += e * dt;
    for (int i = 0; i < 3; ++i)
      int_task_(i) = std::max(-int_p_max_, std::min(int_task_(i), int_p_max_));
    for (int i = 3; i < 6; ++i)
      int_task_(i) = std::max(-int_o_max_, std::min(int_task_(i), int_o_max_));
    xdot_r += Ki6 * int_task_;
  }

  for (int i = 0; i < 3; ++i)
    xdot_r(i) = std::max(-xdot_r_max_, std::min(xdot_r(i), xdot_r_max_));
  for (int i = 3; i < 6; ++i)
    xdot_r(i) = std::max(-wdot_r_max_, std::min(xdot_r(i), wdot_r_max_));

  return xdot_r;
}

// ===== Desired trajectory generator =====
void OperationalSpaceControl::cartesianDesired(double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d, cc::Rotation3 &Rd, cc::Vector3 &Wd)
{
  Rd = R_safe_;
  Wd.setZero();

  if (g_traj.enabled && g_traj.use_move_to_start && g_traj.moving && g_traj.have_start)
  {
    // During MOVE, visualization/error should target trajectory start, not safe point.
    Xd = g_traj.X_start;
    Xdot_d.setZero();
    return;
  }

  if (!g_traj.enabled || !g_traj.draw_started || g_traj.t_draw0 < 0.0)
  {
    // show safe point before drawing starts
    cc::JointPosition q6 = cc::JointPosition::Zero();
    q6.head<6>() = q_safe6_;
    Xd = fkPos(q6);
    Xdot_d.setZero();
    return;
  }

  const double t_local = t_sec - g_traj.t_draw0;
  double x, y, z, xd, yd, zd;
  g_traj.eval(t_local, x, y, z, xd, yd, zd);

  Xd << x, y, z;
  Xdot_d << xd, yd, zd;
}

// ===== qdot_r generator (6-DoF): SAFE handled in update(), here only MOVE/DRAW =====
OperationalSpaceControl::Vector6d
OperationalSpaceControl::computeQdotR6(double t_sec, const cc::JointPosition &q6, double dt)
{
  static double last_debug_t = -1.0;
  const bool do_debug = (last_debug_t < 0.0) || ((t_sec - last_debug_t) >= 1.0);
  if (do_debug) last_debug_t = t_sec;

  if (!g_traj.enabled)
  {
    if (do_debug) ROS_INFO_STREAM("[OperationalSpaceControl] phase=DISABLED dq_norm=nan dX_norm=nan");
    return Vector6d::Zero();
  }

  // For logging
  const double dq_norm = (q6 - q_safe6_).norm();

  g_traj.computeStartIfNeeded();
  if (g_traj.have_start)
{
  ROS_WARN_STREAM_ONCE("[MOVE DEBUG] X_start (world) = "
                       << g_traj.X_start.transpose());
}

  // 1) MOVE-TO-START
  if (g_traj.use_move_to_start && !g_traj.draw_started)
  {
    if (!g_traj.moving)
    {
      g_traj.moving = true;
      resetIntegrators();
      resetMarkerNewSegment();
      qdot_r_prev_valid_ = false; // avoid qddot spike
    }

    const double elapsed = t_sec - g_traj.t0;

    const cc::Vector3 X = fkPos(q6);
    const cc::Vector3 dX = X - g_traj.X_start;
    ROS_WARN_STREAM_THROTTLE(1.0,
  "[MOVE DEBUG] X_now=" << X.transpose()
  << "  X_start=" << g_traj.X_start.transpose()
  << "  dX=" << (X - g_traj.X_start).transpose()
  << "  dX_norm=" << (X - g_traj.X_start).norm());

    const double dX_norm = dX.norm();

    const bool reached = (dX_norm <= g_traj.move_tol);
    const bool timeout = (elapsed >= (g_traj.safe_time + g_traj.move_time + 0.5));

    if (reached || timeout)
    {
      g_traj.moving = false;
      g_traj.draw_started = true;
      g_traj.t_draw0 = t_sec;
      resetIntegrators();
      resetMarkerNewSegment();
      qdot_r_prev_valid_ = false; // avoid qddot spike

      if (do_debug)
        ROS_INFO_STREAM("[OperationalSpaceControl] phase=MOVE->DRAW dq_norm=" << dq_norm << " dX_norm=" << dX_norm);

      return Vector6d::Zero();
    }

    if (do_debug)
      ROS_INFO_STREAM("[OperationalSpaceControl] phase=MOVE dq_norm=" << dq_norm << " dX_norm=" << dX_norm);

    // MOVE should be position-only; constraining orientation can block convergence.
    cc::Vector3 dx = g_traj.X_start - X;
    cc::Vector3 xdot_r = Kp_p_ * dx;
    for (int i = 0; i < 3; ++i)
      xdot_r(i) = std::max(-xdot_r_max_, std::min(xdot_r(i), xdot_r_max_));

    Matrix6d J = jacobian6(q6);
    Matrix3x6d Jp = J.topRows<3>();
    if (do_debug) {
      ROS_WARN_STREAM("[MOVE DEBUG] |Jp_row_z|=" << Jp.row(2).norm()
                      << " |Jp_row_y|=" << Jp.row(1).norm()
                      << " |Jp_row_x|=" << Jp.row(0).norm());
    }
    Vector6d qdot_r = dlsSolve3(Jp, xdot_r, 0.03);

    for (int i = 0; i < 6; ++i)
      qdot_r(i) = std::max(-qdot_r_max_, std::min(qdot_r(i), qdot_r_max_));

    return qdot_r;
  }

  // 2) DRAW
  if (!g_traj.draw_started)
  {
    g_traj.draw_started = true;
    g_traj.t_draw0 = t_sec;
    resetIntegrators();
    resetMarkerNewSegment();
    qdot_r_prev_valid_ = false; // avoid qddot spike
  }

  // log dX_norm w.r.t start
  if (do_debug)
  {
    const cc::Vector3 X = fkPos(q6);
    const double dX_norm = g_traj.have_start ? (X - g_traj.X_start).norm()
                                             : std::numeric_limits<double>::quiet_NaN();
    ROS_INFO_STREAM("[OperationalSpaceControl] phase=DRAW dq_norm=" << dq_norm << " dX_norm=" << dX_norm);
  }

  Vector6d xdot_r = xdotROperational(t_sec, q6, dt);

  Matrix6d J = jacobian6(q6);
  Vector6d qdot_r = dlsSolve6(J, xdot_r, 0.05);

  for (int i = 0; i < 6; ++i)
    qdot_r(i) = std::max(-qdot_r_max_, std::min(qdot_r(i), qdot_r_max_));

  return qdot_r;
}

// ===== Update (tau) =====
OperationalSpaceControl::Vector6d
OperationalSpaceControl::update(const RobotTime &time, const JointState &state)
{
  Vector6d tau = Vector6d::Zero();
  if (!model_ready_)
    return tau;

  const double t_sec = time.tD();
  const cc::JointPosition q6 = state.q;
  const cc::JointVelocity qP6 = state.qp;

  if (g_traj.t0 < 0.0)
    g_traj.t0 = t_sec;

  double dt = t_sec - prev_time_sec_;
  if (prev_time_sec_ <= 0.0 || !std::isfinite(dt) || dt <= 0.0)
    dt = 1e-3;
  prev_time_sec_ = t_sec;

  // ===== SAFE torque PD (6-DoF), criterion uses head<3>() =====
  if (inSafePhase(t_sec, q6))
  {
    // reset phase flags so we don't “half-start” move/draw
    g_traj.moving = false;
    g_traj.draw_started = false;
    g_traj.t_draw0 = -1.0;

    const Vector6d e = (q6 - q_safe6_);
    ROS_WARN_STREAM_THROTTLE(1.0,
      "[SAFE DEBUG] e(q-q_safe)=" << e.transpose()
      << "  q=" << q6.transpose()
      << "  q_safe=" << q_safe6_.transpose());
    // tau_safe = -Kp*(q-q_safe) - Kd*qdot + G
    const cc::VectorDof &G6 = model_.gravityVector(q6);

    Vector6d tau_safe = (-Kp_q6_ * e) + (-Kd6_ * qP6) + G6;

    for (int i = 0; i < 6; ++i)
      tau_safe(i) = std::max(-tau_max_, std::min(tau_safe(i), tau_max_));

    ROS_INFO_STREAM_THROTTLE(1.0, "[OperationalSpaceControl] phase=SAFE dq_norm="
                                   << (q6 - q_safe6_).norm()
                                   << " dX_norm=nan");

    // avoid qddot spike when leaving SAFE
    qdot_r_prev_valid_ = false;
    publishControlDebug(t_sec, "SAFE", state, tau_safe);
    return tau_safe;
  }

  // ===== MOVE/DRAW: operational-space -> qdot_r (6) -> tau =====
  Vector6d qdot_r = computeQdotR6(t_sec, q6, dt);

  if (!qdot_r_prev_valid_)
  {
    qdot_r_prev6_ = qdot_r;
    qdot_r_prev_valid_ = true;
  }

  Vector6d qddot_r = (qdot_r - qdot_r_prev6_) / dt;
  qdot_r_prev6_ = qdot_r;

  Vector6d Sq = qP6 - qdot_r;

  const cc::MatrixDof &M = model_.inertiaMatrix(q6);
  const cc::MatrixDof &C = model_.centripetalMatrix(q6, qP6);
  const cc::VectorDof &G = model_.gravityVector(q6);

  Vector6d Yr = M * qddot_r + C * qdot_r + G;
  tau = (-Kd6_ * Sq) + Yr;

  for (int i = 0; i < 6; ++i)
    tau(i) = std::max(-tau_max_, std::min(tau(i), tau_max_));

  std::string phase = "DRAW";
  if (!g_traj.enabled)
    phase = "DISABLED";
  else if (g_traj.use_move_to_start && !g_traj.draw_started)
    phase = "MOVE";
  publishControlDebug(t_sec, phase, state, tau);

  // ===== Visualization =====
  if (g_traj.enabled)
  {
    cc::Vector3 Xd, Xdot_d, Wd;
    cc::Rotation3 Rd;
    cartesianDesired(t_sec, Xd, Xdot_d, Rd, Wd);

    geometry_msgs::Point pd;
    pd.x = Xd(0); pd.y = Xd(1); pd.z = Xd(2);
    traj_points_.push_back(pd);

    if (traj_points_.size() >= 2)
    {
      traj_marker_.points = traj_points_;
      traj_marker_.header.stamp = ros::Time::now();
      traj_pub_.publish(traj_marker_);
    }

    cc::HomogeneousTransformation Td = cc::HomogeneousTransformation::Identity();
    Td.pos() = Xd;
    Td.orientation() = Rd;
    const cc::HomogeneousTransformation T = model_.T_ef_0(q6);
    const cc::CartesianVector e = cc::cartesianErrorWorld(Td, T);

    std_msgs::Float64MultiArray msg;
    msg.data = {e(0), e(1), e(2), e(3), e(4), e(5)};
    task_error_pub_.publish(msg);
  }

  {
    cc::Vector3 X;
    if (!lookupActualEePositionTf(X))
      X = fkPos(q6);
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

// ===== RViz helpers =====
void OperationalSpaceControl::publishDeleteAllMarkers()
{
  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  clear.header.frame_id = "world";
  for (int i = 0; i < 3; ++i)
  {
    clear.header.stamp = ros::Time::now();
    traj_pub_.publish(clear);
    ros::Duration(0.05).sleep();
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

} // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
