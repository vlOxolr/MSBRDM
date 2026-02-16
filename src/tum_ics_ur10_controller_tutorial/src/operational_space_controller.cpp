#include <tum_ics_ur10_controller_tutorial/operational_space_controller.h>

#include <ros/ros.h>
#include <control_core/math/error_functions.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <regex>
#include <sstream>
#include <Eigen/SVD>

namespace tum_ics_ur_robot_lli
{
namespace RobotControllers
{

namespace
{
constexpr double kQuatNormEps = 1e-12;
constexpr double kSmallAngleEps = 1e-8;
constexpr double kMoveHardTimeoutFactor = 2.0;
constexpr double kJointVelGuardThreshold = 35.0;
constexpr double kDrawTorqueRampTime = 0.5;

bool isFiniteVec3(const Eigen::Vector3d &v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

bool isFiniteQuat(const Eigen::Quaterniond &q)
{
  return std::isfinite(q.w()) && std::isfinite(q.x())
      && std::isfinite(q.y()) && std::isfinite(q.z());
}

double smoothStepQuintic01(double x)
{
  const double u = std::max(0.0, std::min(1.0, x));
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  return 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
}

cc::Rotation3 projectToSO3(const cc::Rotation3 &R_in)
{
  const Eigen::Matrix3d R_raw = R_in;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d U = svd.matrixU();
  const Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d R = U * V.transpose();

  if (R.determinant() < 0.0)
  {
    U.col(2) *= -1.0;
    R = U * V.transpose();
  }

  return R;
}

Eigen::Quaterniond normalizedQuatFromRotation(const cc::Rotation3 &R_in)
{
  const cc::Rotation3 R = projectToSO3(R_in);
  Eigen::Quaterniond q(R);
  if (!isFiniteQuat(q)) return Eigen::Quaterniond::Identity();

  const double n = q.norm();
  if (!std::isfinite(n) || n < kQuatNormEps) return Eigen::Quaterniond::Identity();

  q.coeffs() /= n;
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  return q;
}

double quaternionAngularDistance(const Eigen::Quaterniond &qa, const Eigen::Quaterniond &qb)
{
  double dot = std::abs(qa.dot(qb));
  dot = std::max(-1.0, std::min(1.0, dot));
  return 2.0 * std::acos(dot);
}
}  // namespace

// 功能：构造控制器并初始化全部成员变量与默认参数。
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
    move_speed_(0.0),
    draw_time_(20.0),
    draw_speed_(0.0),
    use_fixed_draw_z_(false),
    fixed_draw_z_(0.0),
    px_scale_x_(0.001),
    px_scale_y_(-0.001),
    px_offset_x_(0.0),
    px_offset_y_(0.0),
    draw_blend_ratio_(0.1),
    move_pos_tol_(0.01),
    move_ori_tol_(0.05),
    active_move_time_(20.0),
    active_draw_time_(20.0),
    active_draw_z_(0.0),
    move_start_pos_(cc::Vector3::Zero()),
    move_goal_pos_(cc::Vector3::Zero()),
    active_traj_(),
    active_traj_valid_(false),
    move_initialized_(false),
    t_move0_(0.0),
    R_move_(cc::Rotation3::Identity()),
    draw_initialized_(false),
    t_draw0_(0.0),
    X_start_(cc::Vector3::Zero()),
    R_draw_(cc::Rotation3::Identity()),
    traj_topic_("/ur10/planar_polynomial_trajectories"),
    trajectory_batch_(),
    current_traj_idx_(0),
    trajectory_reset_requested_(false),
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

// 功能：析构控制器对象。
OperationalSpaceControl::~OperationalSpaceControl() {}

// 功能：记录初始关节状态（兼容上层接口）。
void OperationalSpaceControl::setQInit(const JointState &q_init) { q_init_ = q_init; }
// 功能：记录 home 关节状态（兼容上层接口）。
void OperationalSpaceControl::setQHome(const JointState &q_home) { q_home_ = q_home; }
// 功能：记录 park 关节状态（兼容上层接口）。
void OperationalSpaceControl::setQPark(const JointState &q_park) { q_park_ = q_park; }

// 功能：读取参数、初始化模型与 ROS 发布订阅器。
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
  ros::param::get(ns + "/move/speed", move_speed_);
  if (move_time_ <= 1e-3) move_time_ = 20.0;

  // draw params
  ros::param::get(ns + "/draw/time", draw_time_);
  ros::param::get(ns + "/draw/speed", draw_speed_);
  ros::param::get(ns + "/draw/use_fixed_z", use_fixed_draw_z_);
  ros::param::get(ns + "/draw/fixed_z", fixed_draw_z_);
  if (draw_time_ <= 1e-3) draw_time_ = 20.0;
  ros::param::get(ns + "/draw/blend_ratio", draw_blend_ratio_);

  // pixel-to-world transform
  ros::param::get(ns + "/pixel_to_world/scale_x", px_scale_x_);
  ros::param::get(ns + "/pixel_to_world/scale_y", px_scale_y_);
  ros::param::get(ns + "/pixel_to_world/offset_x", px_offset_x_);
  ros::param::get(ns + "/pixel_to_world/offset_y", px_offset_y_);

  // move position tolerance
  ros::param::get(ns + "/move/pos_tol", move_pos_tol_);
  ros::param::get(ns + "/move/ori_tol", move_ori_tol_);

  // trajectory input
  ros::param::get(ns + "/trajectory/topic", traj_topic_);

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

  // desired orientation in MOVE/DRAW:
  // tool z-axis -> -Z (world, pointing down), tool x -> +X, tool y -> -Y
  // = 180° rotation about world X-axis, det=+1
  R_move_.setIdentity();
  R_move_ <<  1,  0,  0,
              0, -1,  0,
              0,  0, -1;

  R_draw_.setIdentity();
  R_draw_ <<  1,  0,  0,
              0, -1,  0,
              0,  0, -1;

  // init theta_hat from model initial guess
  theta_hat_ = model_.parameterInitalGuess();
  theta_hat_initialized_ = true;

  // (optional) publishers for debug/rviz
  traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/ur10/desired_trajectory", 1);
  actual_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/ur10/actual_trajectory", 1);
  task_error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur10/task_space_error", 1);
  effort_debug_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur10/joint_effort_debug", 1);
  effort_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/ur10/joint_effort_state", 1);
  traj_sub_ = nh_.subscribe(traj_topic_, 1, &OperationalSpaceControl::trajectoryCallback, this);

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
                  << " move_speed=" << move_speed_
                  << " enable_draw=" << (enable_draw_ ? 1 : 0)
                  << " draw_time=" << draw_time_
                  << " draw_speed=" << draw_speed_
                  << " traj_topic=" << traj_topic_);

  return true;
}

// 功能：重置运行时状态并选择初始阶段。
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
  active_traj_valid_ = false;
  active_move_time_ = move_time_;
  active_draw_time_ = draw_time_;

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

// 功能：停止控制器（当前无额外清理逻辑）。
bool OperationalSpaceControl::stop() { return true; }

// -------------------------
// SAFE/MOVE/DRAW phase selection
// -------------------------
// 功能：判断当前关节是否仍需执行 SAFE 收敛。
bool OperationalSpaceControl::inSafePhase(const cc::JointPosition &q6) const
{
  if (!enable_safe_) return false;
  const double e = (q6 - q_safe6_).norm();
  return (e > safe_tol_);
}

// 功能：检查是否还有未执行的轨迹段。
bool OperationalSpaceControl::hasPendingTrajectory() const
{
  std::lock_guard<std::mutex> lock(traj_mutex_);
  return current_traj_idx_ < trajectory_batch_.size();
}

// 功能：按序号读取缓存中的某条轨迹。
bool OperationalSpaceControl::getTrajectoryAtIndex(size_t index, PlanarPolynomialTrajectory &traj) const
{
  std::lock_guard<std::mutex> lock(traj_mutex_);
  if (index >= trajectory_batch_.size()) return false;
  traj = trajectory_batch_[index];
  return true;
}

// 功能：提取指定 XML 标签的内部文本。
bool OperationalSpaceControl::extractXmlTag(const std::string &xml, const std::string &tag, std::string &content)
{
  const std::regex re("<" + tag + "\\b[^>]*>([\\s\\S]*?)</" + tag + ">", std::regex::icase);
  std::smatch m;
  if (!std::regex_search(xml, m, re) || m.size() < 2) return false;
  content = m[1].str();
  return true;
}

// 功能：提取指定 XML 属性值。
bool OperationalSpaceControl::extractXmlAttribute(const std::string &xml, const std::string &attr, std::string &value)
{
  const std::regex re(attr + "\\s*=\\s*\"([^\"]+)\"", std::regex::icase);
  std::smatch m;
  if (!std::regex_search(xml, m, re) || m.size() < 2) return false;
  value = m[1].str();
  return true;
}

// 功能：将字符串安全转换为 double。
bool OperationalSpaceControl::toDouble(const std::string &text, double &value)
{
  try
  {
    size_t end = 0;
    value = std::stod(text, &end);
    while (end < text.size() && std::isspace(static_cast<unsigned char>(text[end]))) ++end;
    return end == text.size();
  }
  catch (const std::exception &)
  {
    return false;
  }
}

// 功能：将字符串安全转换为 int。
bool OperationalSpaceControl::toInt(const std::string &text, int &value)
{
  try
  {
    size_t end = 0;
    const long parsed = std::stol(text, &end, 10);
    while (end < text.size() && std::isspace(static_cast<unsigned char>(text[end]))) ++end;
    if (end != text.size()) return false;
    value = static_cast<int>(parsed);
    return true;
  }
  catch (const std::exception &)
  {
    return false;
  }
}

// 功能：从文本中解析多项式系数序列。
std::vector<double> OperationalSpaceControl::parseCoefficientList(const std::string &text)
{
  std::vector<double> coeffs;
  const std::regex number_re("[+-]?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][+-]?\\d+)?");
  for (std::sregex_iterator it(text.begin(), text.end(), number_re), end; it != end; ++it)
  {
    try
    {
      coeffs.push_back(std::stod((*it).str()));
    }
    catch (const std::exception &)
    {
    }
  }
  return coeffs;
}

// 功能：按升幂系数评估多项式值（a_0, a_1, ..., a_n）。
double OperationalSpaceControl::evalPoly(const std::vector<double> &coeff, double s)
{
  double value = 0.0;
  double p = 1.0;
  for (size_t i = 0; i < coeff.size(); ++i)
  {
    value += coeff[i] * p;
    p *= s;
  }
  return value;
}

// 功能：按升幂系数评估多项式一阶导数（a_0, a_1, ..., a_n）。
double OperationalSpaceControl::evalPolyDerivative(const std::vector<double> &coeff, double s)
{
  if (coeff.size() < 2) return 0.0;

  double value = 0.0;
  double p = 1.0;
  for (size_t i = 1; i < coeff.size(); ++i)
  {
    value += static_cast<double>(i) * coeff[i] * p;
    p *= s;
  }
  return value;
}

// 功能：解析 XML 轨迹批次并重置执行状态机。
void OperationalSpaceControl::trajectoryCallback(const std_msgs::String::ConstPtr &msg)
{
  const std::string xml = msg->data;
  if (xml.empty())
  {
    ROS_WARN_STREAM("[OperationalSpaceControl] trajectoryCallback: empty message.");
    return;
  }

  std::smatch strokes_match;
  const std::regex strokes_re("<strokes\\b([^>]*)>([\\s\\S]*?)</strokes>", std::regex::icase);
  if (!std::regex_search(xml, strokes_match, strokes_re) || strokes_match.size() < 3)
  {
    ROS_ERROR_STREAM("[OperationalSpaceControl] trajectoryCallback: missing <strokes> block.");
    return;
  }

  int n_traj = 0;
  {
    std::string count_text;
    if (!extractXmlAttribute(strokes_match[1].str(), "count", count_text) || !toInt(count_text, n_traj) || n_traj <= 0)
    {
      ROS_ERROR_STREAM("[OperationalSpaceControl] trajectoryCallback: invalid strokes count.");
      return;
    }
  }

  std::vector<PlanarPolynomialTrajectory> parsed(static_cast<size_t>(n_traj));
  std::vector<bool> parsed_mask(static_cast<size_t>(n_traj), false);

  const std::string strokes_body = strokes_match[2].str();
  const std::regex stroke_re("<stroke\\b([^>]*)>([\\s\\S]*?)</stroke>", std::regex::icase);
  for (std::sregex_iterator it(strokes_body.begin(), strokes_body.end(), stroke_re), end; it != end; ++it)
  {
    const std::string stroke_attr = (*it)[1].str();
    const std::string stroke_body = (*it)[2].str();

    int stroke_index = -1;
    int deg_used = -1;
    std::string index_text;
    std::string deg_text;
    if (!extractXmlAttribute(stroke_attr, "index", index_text) || !toInt(index_text, stroke_index))
    {
      ROS_ERROR_STREAM("[OperationalSpaceControl] trajectoryCallback: stroke missing valid index.");
      return;
    }
    if (!extractXmlAttribute(stroke_attr, "deg_used", deg_text) || !toInt(deg_text, deg_used) || deg_used < 0)
    {
      ROS_ERROR_STREAM("[OperationalSpaceControl] trajectoryCallback: stroke " << stroke_index
                       << " missing valid deg_used.");
      return;
    }
    if (stroke_index < 0 || stroke_index >= n_traj)
    {
      ROS_ERROR_STREAM("[OperationalSpaceControl] trajectoryCallback: stroke index out of range: "
                       << stroke_index << ", count=" << n_traj);
      return;
    }

    std::string length_text;
    double length_px = 0.0;
    if (extractXmlTag(stroke_body, "length_px", length_text)) (void)toDouble(length_text, length_px);

    std::string polyfit_block;
    std::string x_text;
    std::string y_text;
    if (!extractXmlTag(stroke_body, "polyfit", polyfit_block) ||
        !extractXmlTag(polyfit_block, "x", x_text) ||
        !extractXmlTag(polyfit_block, "y", y_text))
    {
      ROS_ERROR_STREAM("[OperationalSpaceControl] trajectoryCallback: stroke " << stroke_index
                       << " missing polyfit/x/y.");
      return;
    }

    const std::vector<double> coeff_x = parseCoefficientList(x_text);
    const std::vector<double> coeff_y = parseCoefficientList(y_text);
    const size_t needed_size = static_cast<size_t>(deg_used + 1);
    if (coeff_x.size() < needed_size || coeff_y.size() < needed_size)
    {
      ROS_ERROR_STREAM("[OperationalSpaceControl] trajectoryCallback: stroke " << stroke_index
                       << " coeff size < deg_used+1. deg_used=" << deg_used
                       << " |x|=" << coeff_x.size() << " |y|=" << coeff_y.size());
      return;
    }

    PlanarPolynomialTrajectory traj;
    traj.length = std::max(0.0, length_px);
    // XML coefficients are descending (a_n, ..., a_0) from np.polyfit;
    // evalPoly expects ascending (a_0, a_1, ..., a_n), so reverse.
    traj.coeff_x.assign(coeff_x.begin(), coeff_x.begin() + needed_size);
    std::reverse(traj.coeff_x.begin(), traj.coeff_x.end());
    traj.coeff_y.assign(coeff_y.begin(), coeff_y.begin() + needed_size);
    std::reverse(traj.coeff_y.begin(), traj.coeff_y.end());

    parsed[static_cast<size_t>(stroke_index)] = std::move(traj);
    parsed_mask[static_cast<size_t>(stroke_index)] = true;
  }

  for (int i = 0; i < n_traj; ++i)
  {
    if (!parsed_mask[static_cast<size_t>(i)])
    {
      ROS_ERROR_STREAM("[OperationalSpaceControl] trajectoryCallback: missing stroke index " << i);
      return;
    }
  }

  // Log mapped start point and mapped length (in world XY) for each trajectory.
  const int len_samples = 200;
  for (int i = 0; i < n_traj; ++i)
  {
    const PlanarPolynomialTrajectory &traj = parsed[static_cast<size_t>(i)];
    const double x0_px = evalPoly(traj.coeff_x, 0.0);
    const double y0_px = evalPoly(traj.coeff_y, 0.0);
    const double x0_w = x0_px * px_scale_x_ + px_offset_x_;
    const double y0_w = y0_px * px_scale_y_ + px_offset_y_;

    double len_w = 0.0;
    double t_prev = 0.0;
    double dxdt_prev = evalPolyDerivative(traj.coeff_x, 0.0) * px_scale_x_;
    double dydt_prev = evalPolyDerivative(traj.coeff_y, 0.0) * px_scale_y_;
    double v_prev = std::sqrt(dxdt_prev * dxdt_prev + dydt_prev * dydt_prev);
    for (int k = 1; k <= len_samples; ++k)
    {
      const double t = static_cast<double>(k) / static_cast<double>(len_samples);
      const double dt = t - t_prev;
      const double dxdt = evalPolyDerivative(traj.coeff_x, t) * px_scale_x_;
      const double dydt = evalPolyDerivative(traj.coeff_y, t) * px_scale_y_;
      const double v = std::sqrt(dxdt * dxdt + dydt * dydt);
      len_w += 0.5 * (v_prev + v) * dt;
      t_prev = t;
      v_prev = v;
    }

    ROS_INFO_STREAM("[TRAJ MAP] idx=" << i
                    << " start_world_xy=(" << x0_w << ", " << y0_w << ")"
                    << " length_world=" << len_w << " m"
                    << " length_px=" << traj.length);
  }

  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    trajectory_batch_ = std::move(parsed);
    current_traj_idx_ = 0;
    trajectory_reset_requested_ = true;
  }

  ROS_WARN_STREAM("[OperationalSpaceControl] received trajectory batch: count=" << n_traj
                  << ", order by index, state machine reset to SAFE->MOVE->DRAW.");
}

// 功能：初始化 MOVE 阶段（SAFE 点到当前轨迹起点的直线）。
void OperationalSpaceControl::ensureMoveInit(double t_sec, const cc::JointPosition &q6)
{
  if (move_initialized_) return;

  size_t traj_idx = 0;
  PlanarPolynomialTrajectory traj;
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    traj_idx = current_traj_idx_;
    if (traj_idx >= trajectory_batch_.size())
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[OperationalSpaceControl] MOVE init waiting trajectory.");
      return;
    }
    traj = trajectory_batch_[traj_idx];
  }

  move_start_pos_ = fkPos(q6);
  active_draw_z_ = use_fixed_draw_z_ ? fixed_draw_z_ : move_start_pos_(2);

  const double x0_px = evalPoly(traj.coeff_x, 0.0);
  const double y0_px = evalPoly(traj.coeff_y, 0.0);
  move_goal_pos_ << x0_px * px_scale_x_ + px_offset_x_,
                    y0_px * px_scale_y_ + px_offset_y_,
                    active_draw_z_;

  active_traj_ = traj;
  active_traj_valid_ = true;

  active_move_time_ = std::max(1e-3, move_time_);
  if (move_speed_ > 1e-6)
  {
    const double dist = (move_goal_pos_ - move_start_pos_).norm();
    active_move_time_ = std::max(1e-3, dist / move_speed_);
  }

  const double avg_scale = 0.5 * (std::abs(px_scale_x_) + std::abs(px_scale_y_));
  const double length_m = active_traj_.length * avg_scale;
  active_draw_time_ = std::max(1e-3, draw_time_);
  if (draw_speed_ > 1e-6 && length_m > 1e-6)
  {
    active_draw_time_ = std::max(1e-3, length_m / draw_speed_);
  }

  X_start_ = move_start_pos_;
  R_move_start_ = projectToSO3(fkOri(q6));   // record current orientation for SLERP
  t_move0_ = t_sec;
  move_initialized_ = true;
  draw_initialized_ = false;

  ROS_WARN_STREAM("[OperationalSpaceControl] MOVE init: idx=" << traj_idx
                  << " X_start=" << move_start_pos_.transpose()
                  << " X_goal=" << move_goal_pos_.transpose()
                  << " T_move=" << active_move_time_
                  << " T_draw=" << active_draw_time_);
}

// 功能：初始化 DRAW 阶段的起始时刻与参考状态。
void OperationalSpaceControl::ensureDrawInit(double t_sec, const cc::JointPosition &q6)
{
  if (draw_initialized_) return;
  if (!active_traj_valid_)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "[OperationalSpaceControl] DRAW init skipped: no active trajectory.");
    return;
  }

  X_start_ = fkPos(q6);
  if (!use_fixed_draw_z_) active_draw_z_ = X_start_(2);
  t_draw0_ = t_sec;
  draw_initialized_ = true;

  ROS_WARN_STREAM("[OperationalSpaceControl] DRAW init: X_start=" << X_start_.transpose()
                  << " t_draw0=" << t_draw0_);
}

// -------------------------
// Desired trajectory (MOVE)
// -------------------------
// 功能：生成 MOVE 阶段的期望笛卡尔位置/速度/姿态。
void OperationalSpaceControl::cartesianDesiredMove(
  double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d,
  cc::Rotation3 &Rd, cc::Vector3 &Wd)
{
  const double t = std::max(0.0, t_sec - t_move0_);
  const double T = std::max(1e-3, active_move_time_);
  double tau = t / T;
  if (tau > 1.0) tau = 1.0;

  // 五次最小急动度插值：s(0)=0,s(1)=1 且端点速度加速度为0
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;
  const double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;

  double sdot = 0.0;
  if (t < T) sdot = (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4) / T;

  const cc::Vector3 delta = move_goal_pos_ - move_start_pos_;
  Xd = move_start_pos_ + s * delta;
  Xdot_d = sdot * delta;

  // SLERP orientation from R_move_start_ to R_move_ using same quintic s.
  const Eigen::Quaterniond q0 = normalizedQuatFromRotation(R_move_start_);
  Eigen::Quaterniond q1 = normalizedQuatFromRotation(R_move_);
  if (q0.dot(q1) < 0.0) q1.coeffs() *= -1.0;  // shortest arc

  Eigen::Quaterniond q_interp = q0.slerp(s, q1);
  if (!isFiniteQuat(q_interp) || q_interp.norm() < kQuatNormEps)
  {
    q_interp = q0;
  }
  else
  {
    q_interp.normalize();
  }
  Rd = q_interp.toRotationMatrix();

  // angular velocity: Wd = sdot * axis * angle, with angle~0 fallback.
  Eigen::Quaterniond q_delta = q0.conjugate() * q1;
  if (!isFiniteQuat(q_delta) || q_delta.norm() < kQuatNormEps)
  {
    Wd.setZero();
    return;
  }

  q_delta.normalize();
  if (q_delta.w() < 0.0) q_delta.coeffs() *= -1.0;

  const Eigen::Vector3d v = q_delta.vec();
  const double sin_half = v.norm();
  const double cos_half = std::max(-1.0, std::min(1.0, q_delta.w()));
  if (!std::isfinite(sin_half) || !std::isfinite(cos_half) || sin_half < kSmallAngleEps)
  {
    Wd.setZero();
    return;
  }

  const double angle = 2.0 * std::atan2(sin_half, cos_half);
  const Eigen::Vector3d axis = v / sin_half;
  if (!std::isfinite(angle) || !isFiniteVec3(axis))
  {
    Wd.setZero();
    return;
  }

  Wd = sdot * angle * axis;
}

// -------------------------
// Desired trajectory (DRAW)
// -------------------------
// 功能：生成 DRAW 阶段的期望笛卡尔位置/速度/姿态。
void OperationalSpaceControl::cartesianDesiredDraw(
  double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d,
  cc::Rotation3 &Rd, cc::Vector3 &Wd)
{
  if (!active_traj_valid_)
  {
    Xd = X_start_;
    Xdot_d.setZero();
    Rd = R_draw_;
    Wd.setZero();
    return;
  }

  const double t = std::max(0.0, t_sec - t_draw0_);
  const double T_draw = std::max(1e-3, active_draw_time_);
  double t_norm = t / T_draw;
  if (t_norm > 1.0) t_norm = 1.0;

  const double x_px = evalPoly(active_traj_.coeff_x, t_norm);
  const double y_px = evalPoly(active_traj_.coeff_y, t_norm);
  const double dx_dt_norm_px = evalPolyDerivative(active_traj_.coeff_x, t_norm);
  const double dy_dt_norm_px = evalPolyDerivative(active_traj_.coeff_y, t_norm);

  cc::Vector3 X_path;
  X_path << x_px * px_scale_x_ + px_offset_x_,
            y_px * px_scale_y_ + px_offset_y_,
            active_draw_z_;

  Xdot_d.setZero();
  Xd = X_path;
  if (t < T_draw)
  {
    // smooth ramp from 0 over first draw_blend_ratio_ of T_draw.
    // During this window, blend desired position from current X_start_ to path
    // to avoid a position step right after MOVE->DRAW transition.
    double alpha = 1.0;
    double alpha_dot = 0.0;
    const double T_blend = draw_blend_ratio_ * T_draw;
    if (T_blend > 1e-6 && t < T_blend)
    {
      const double r = t / T_blend;
      const double r2 = r * r;
      const double r3 = r2 * r;
      const double r4 = r3 * r;
      alpha = 10.0 * r3 - 15.0 * r3 * r + 6.0 * r3 * r2;
      alpha_dot = (30.0 * r2 - 60.0 * r3 + 30.0 * r4) / T_blend;
      Xd = (1.0 - alpha) * X_start_ + alpha * X_path;
    }
    Xdot_d(0) = alpha * dx_dt_norm_px * px_scale_x_ / T_draw
              + alpha_dot * (X_path(0) - X_start_(0));
    Xdot_d(1) = alpha * dy_dt_norm_px * px_scale_y_ / T_draw
              + alpha_dot * (X_path(1) - X_start_(1));
    Xdot_d(2) = alpha_dot * (X_path(2) - X_start_(2));
  }

  Rd = R_draw_;
  Wd.setZero();
}

// -------------------------
// Model helpers (FK/J)
// -------------------------
// 功能：计算末端在世界系下的位置。
cc::Vector3 OperationalSpaceControl::fkPos(const cc::JointPosition &q6) const
{
  const cc::HomogeneousTransformation T_0_B = model_.T_0_B();
  const cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(q6);
  const cc::HomogeneousTransformation T = T_0_B * T_tool_0;
  cc::Vector3 X;
  X << T.pos()(0), T.pos()(1), T.pos()(2);
  return X;
}

// 功能：计算末端在世界系下的姿态。
cc::Rotation3 OperationalSpaceControl::fkOri(const cc::JointPosition &q6) const
{
  const cc::HomogeneousTransformation T_0_B = model_.T_0_B();
  const cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(q6);
  const cc::HomogeneousTransformation T = T_0_B * T_tool_0;
  return projectToSO3(T.orientation());
}

// 功能：计算并转换到世界系的 6x6 雅可比矩阵。
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
// 功能：用阻尼最小二乘求解 6 维任务到关节速度映射。
OperationalSpaceControl::Vector6d
OperationalSpaceControl::dlsSolve6(const Matrix6d &J, const Vector6d &xdot, double lambda)
{
  const Matrix6d A = (J * J.transpose() + (lambda * lambda) * Matrix6d::Identity());
  const Vector6d y = A.inverse() * xdot;
  return J.transpose() * y;
}

// 功能：用阻尼最小二乘求解 3 维任务到关节速度映射。
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
// 功能：在 MOVE/DRAW 阶段生成关节参考速度 qdot_r。
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
  cc::HomogeneousTransformation T = T_0_B * T_tool_0;
  T.orientation() = projectToSO3(T.orientation());

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
// 功能：主控制循环，执行 SAFE/MOVE/DRAW 状态机并输出关节力矩。
OperationalSpaceControl::Vector6d
OperationalSpaceControl::update(const RobotTime &time, const JointState &state)
{
  Vector6d tau = Vector6d::Zero();
  if (!model_ready_) return tau;

  const double t_sec = time.tD();
  if (t0_ < 0.0) t0_ = t_sec;

  const cc::JointPosition q6 = state.q;
  const cc::JointVelocity qP6 = state.qp;

  auto applyVelocityGuard = [&](Vector6d &tau_cmd, const char *phase_name)
  {
    const double qp_abs_max = qP6.cwiseAbs().maxCoeff();
    if (!std::isfinite(qp_abs_max) || qp_abs_max <= kJointVelGuardThreshold) return;

    tau_cmd = (-Kd_safe6_) * qP6;
    for (int i = 0; i < 6; ++i)
      tau_cmd(i) = std::max(-tau_max_, std::min(tau_cmd(i), tau_max_));

    qdot_r_prev_valid_ = false;
    ROS_ERROR_STREAM_THROTTLE(
      0.5,
      "[VEL GUARD][" << phase_name << "] |qp|max=" << qp_abs_max
      << " > " << kJointVelGuardThreshold
      << ", override tau with damping.");
  };

  double dt = t_sec - prev_time_sec_;
  if (prev_time_sec_ <= 0.0 || !std::isfinite(dt) || dt <= 0.0) dt = 1e-3;
  prev_time_sec_ = t_sec;

  bool trajectory_reset = false;
  {
    std::lock_guard<std::mutex> lock(traj_mutex_);
    if (trajectory_reset_requested_)
    {
      trajectory_reset_requested_ = false;
      trajectory_reset = true;
    }
  }
  if (trajectory_reset)
  {
    safe_done_ = false;
    move_initialized_ = false;
    draw_initialized_ = false;
    active_traj_valid_ = false;
    qdot_r_prev_valid_ = false;
    phase_ = enable_safe_ ? PHASE_SAFE : PHASE_MOVE;
    resetMarkerNewSegment();
  }

  if (!enable_safe_ && !enable_move_ && !enable_draw_) return tau;

  // -------------------------
  // SAFE
  // -------------------------
  if (phase_ == PHASE_SAFE)
  {
    const double e_norm = (q6 - q_safe6_).norm();
    if (e_norm <= safe_tol_)
    {
      safe_done_ = true;
      if (hasPendingTrajectory() && enable_move_)
      {
        ROS_WARN_STREAM("[PHASE SWITCH] SAFE -> MOVE");
        phase_ = PHASE_MOVE;
        move_initialized_ = false;
        draw_initialized_ = false;
        active_traj_valid_ = false;
        qdot_r_prev_valid_ = false; // avoid qddot spike on transition
        resetMarkerNewSegment();

        Vector6d tau_bridge = (-Kd_safe6_) * qP6;
        for (int i = 0; i < 6; ++i)
          tau_bridge(i) = std::max(-tau_max_, std::min(tau_bridge(i), tau_max_));
        return tau_bridge;
      }
    }
    else
    {
      safe_done_ = false;
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

    applyVelocityGuard(tau, "SAFE");
    publishControlDebug(t_sec, "SAFE", state, tau);
    return tau;
  }

  // -------------------------
  // MOVE
  // -------------------------
  if (phase_ == PHASE_MOVE)
  {
    if (!enable_move_) return tau;
    if (!hasPendingTrajectory())
    {
      phase_ = enable_safe_ ? PHASE_SAFE : PHASE_MOVE;
      return Vector6d::Zero();
    }

    ensureMoveInit(t_sec, q6);
    if (!move_initialized_) return Vector6d::Zero();

    // ===== MOVE 完成判定 =====
    const double t_move = t_sec - t_move0_;
    const double move_pos_err = (fkPos(q6) - move_goal_pos_).norm();

    // orientation error: angle between current and desired R_move_
    const cc::Rotation3 R_cur = fkOri(q6);
    const Eigen::Quaterniond q_des = normalizedQuatFromRotation(R_move_);
    const Eigen::Quaterniond q_cur = normalizedQuatFromRotation(R_cur);
    const double move_ori_err = quaternionAngularDistance(q_des, q_cur);

    const bool pose_converged = (move_pos_err < move_pos_tol_ && move_ori_err < move_ori_tol_);
    const bool time_up = (t_move >= active_move_time_);
    const bool converged_early = (t_move > 0.5 * active_move_time_ && pose_converged);
    const bool hard_timeout = (t_move >= kMoveHardTimeoutFactor * active_move_time_);

    ROS_INFO_STREAM_THROTTLE(2.0,
      "[MOVE CONV] e_pos=" << move_pos_err << " e_ori=" << move_ori_err
      << " pos_tol=" << move_pos_tol_ << " ori_tol=" << move_ori_tol_);

    if (time_up && !pose_converged)
    {
      ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[MOVE HOLD] time_up but pose not converged: e_pos=" << move_pos_err
        << " e_ori=" << move_ori_err << ", holding MOVE.");
    }

    if (converged_early || hard_timeout)
    {
      if (hard_timeout && !pose_converged)
      {
        ROS_WARN_STREAM("[MOVE FORCE SWITCH] hard timeout reached at t=" << t_move
                        << "s, e_pos=" << move_pos_err << " e_ori=" << move_ori_err);
      }

      ROS_WARN_STREAM("[PHASE SWITCH] MOVE -> DRAW (e_pos=" << move_pos_err
                      << " e_ori=" << move_ori_err << ")");

      phase_ = PHASE_DRAW;
      draw_initialized_ = false;
      qdot_r_prev_valid_ = false;   // 避免 qddot_r 突变
      resetMarkerNewSegment();

      // 过渡周期：用纯阻尼减速，而非零力矩（避免自由漂移一个周期）
      Vector6d tau_bridge = (-Kd6_) * qP6;
      for (int i = 0; i < 6; ++i)
        tau_bridge(i) = std::max(-tau_max_, std::min(tau_bridge(i), tau_max_));
      return tau_bridge;
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

    applyVelocityGuard(tau, "MOVE");

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
    if (!hasPendingTrajectory())
    {
      phase_ = enable_safe_ ? PHASE_SAFE : PHASE_DRAW;
      return Vector6d::Zero();
    }

    ensureDrawInit(t_sec, q6);
    if (!draw_initialized_) return Vector6d::Zero();

    // ===== DRAW 完成判定 =====
    double t_draw = t_sec - t_draw0_;
    if (t_draw >= active_draw_time_)
    {
      {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (current_traj_idx_ < trajectory_batch_.size())
          ++current_traj_idx_;
      }
      ROS_WARN_STREAM("[PHASE SWITCH] DRAW -> SAFE");
      phase_ = enable_safe_ ? PHASE_SAFE : PHASE_MOVE;
      safe_done_ = false;
      move_initialized_ = false;
      draw_initialized_ = false;
      active_traj_valid_ = false;
      qdot_r_prev_valid_ = false;
      resetMarkerNewSegment();

      Vector6d tau_bridge = (-Kd6_) * qP6;
      for (int i = 0; i < 6; ++i)
        tau_bridge(i) = std::max(-tau_max_, std::min(tau_bridge(i), tau_max_));
      return tau_bridge;
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

    // Soften MOVE->DRAW transition: blend from pure damping to full DRAW torque.
    if (t_draw < kDrawTorqueRampTime)
    {
      const double beta = smoothStepQuintic01(t_draw / kDrawTorqueRampTime);
      Vector6d tau_damp = (-Kd_safe6_) * qP6;
      for (int i = 0; i < 6; ++i)
        tau_damp(i) = std::max(-tau_max_, std::min(tau_damp(i), tau_max_));
      tau = (1.0 - beta) * tau_damp + beta * tau;
    }

    applyVelocityGuard(tau, "DRAW");

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
// 功能：清除 RViz 中已发布的轨迹 Marker。
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

// 功能：开启新的轨迹段并重置 Marker 缓冲。
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

// 功能：发布控制调试信息与关节力矩状态。
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
