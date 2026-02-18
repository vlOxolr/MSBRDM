#ifndef UR_ROBOT_LLI_OPERATIONALSPACECONTROL_H
#define UR_ROBOT_LLI_OPERATIONALSPACECONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>

#include <ur_model/ur_model.h>
#include <control_core/types.h>

#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <string>
#include <vector>
#include <geometry_msgs/WrenchStamped.h> 
#include <ros/ros.h>

namespace tum_ics_ur_robot_lli
{
namespace RobotControllers
{

class OperationalSpaceControl : public ControlEffort
{
public:
  enum ControllerType
  {
    OP_PD = 1,
    OP_PID = 2
  };

private:
  // ===== 6-DoF types =====
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  using Matrix3x6d = Eigen::Matrix<double, 3, 6>;

  struct PlanarPolynomialTrajectory
  {
    std::vector<double> coeff_x;
    std::vector<double> coeff_y;
    double length = 0.0;
  };

  enum Phase
  {
    PHASE_SAFE = 0,
    PHASE_MOVE = 1,
    PHASE_DRAW = 2
  };

private:
  ros::NodeHandle nh_;
  ur::URModel model_;
  bool model_ready_;

  // ===== YAML switches =====
  bool enable_safe_;
  bool enable_move_;
  bool enable_draw_;

  // ===== SAFE =====
  Matrix6d Kp_q6_;      // joint gain (diag) used to build qrP in SAFE
  Matrix6d Kd_safe6_;   // damping in SAFE (diag)
  Vector6d q_safe6_;
  double safe_tol_;

  // ===== MOVE/DRAW =====
  Matrix3d Kp_p_;       // position feedback (diag)
  Matrix3d Kp_o_;       // orientation feedback (diag) (MOVE/DRAW only)
  Matrix6d Kd6_;        // damping on sliding variable in MOVE/DRAW (diag)
  double move_time_;
  double move_speed_;
  double draw_time_;
  double draw_speed_;
  bool use_fixed_draw_z_;
  double fixed_draw_z_;

  // ===== pixel-to-world transform =====
  double px_scale_x_;     // meters per pixel-col (negative to flip X)
  double px_scale_y_;     // meters per pixel-row (negative to flip Y)
  double px_offset_x_;    // world X offset (meters)
  double px_offset_y_;    // world Y offset (meters)
  double px_rotation_deg_;  // rotation about z-axis in degrees (CW positive)

  // ===== MOVE→DRAW blend =====
  double draw_blend_ratio_;

  // ===== MOVE convergence =====
  double move_pos_tol_;
  double move_ori_tol_;

  // active segment timing/path
  double active_move_time_;
  double active_draw_time_;
  double active_draw_z_;
  cc::Vector3 move_start_pos_;
  cc::Vector3 move_goal_pos_;
  PlanarPolynomialTrajectory active_traj_;
  bool active_traj_valid_;

  // move timing & start pose
  bool move_initialized_;
  double t_move0_;
  cc::Vector3 X_start_;     // fixed x,z and y start
  cc::Rotation3 R_move_;    // desired orientation in MOVE (tool z -> +X)
  cc::Rotation3 R_move_start_;  // orientation at MOVE init (for SLERP interpolation)

  // draw timing & start pose
  bool draw_initialized_;
  double t_draw0_;
  cc::Rotation3 R_draw_;    // desired orientation in DRAW (tool z -> +X)

  // ===== trajectory input =====
  ros::Subscriber traj_sub_;
  std::string traj_topic_;
  mutable std::mutex traj_mutex_;
  std::vector<PlanarPolynomialTrajectory> trajectory_batch_;
  size_t current_traj_idx_;
  bool trajectory_reset_requested_;

  // ===== Adaptive =====
  bool adaptive_enabled_;
  double adapt_gamma_;
  double adapt_sigma_;
  double theta_hat_max_;
  ur::URModel::Parameters theta_hat_;
  bool theta_hat_initialized_;

  // Differentiate qdot_r
  Vector6d qdot_r_prev6_;
  bool qdot_r_prev_valid_;
  double prev_time_sec_;

  // State machine
  Phase phase_;
  bool safe_done_;
  double t0_;

  // (kept for compatibility with your existing codebase)
  JointState q_init_;
  JointState q_home_;
  JointState q_park_;

  // Saturations
  double tau_max_;
  double qdot_r_max_;
  double xdot_r_max_;
  double wdot_r_max_;

  double force_target_z_;      // desired contact force [N]
  double force_z_sign_;        // effective Fz = force_z_sign_ * raw sensor Fz
  double force_k_z_;           // integral gain [m/(N*s)]
  double current_force_z_;     // latest measured force (raw)
  double draw_z_offset_;       // accumulated z offset [m]
  double force_prev_time_sec_; // timestamp for force integration
  
  ros::Subscriber wrench_sub_;

  // RViz visualization (kept; you can remove later)
  ros::Publisher traj_pub_;
  ros::Publisher actual_traj_pub_;
  ros::Publisher task_error_pub_;
  ros::Publisher effort_debug_pub_;
  ros::Publisher effort_joint_state_pub_;

  visualization_msgs::Marker traj_marker_;
  visualization_msgs::Marker actual_traj_marker_;
  std::vector<geometry_msgs::Point> traj_points_;
  std::vector<geometry_msgs::Point> actual_traj_points_;

public:
  OperationalSpaceControl(double weight = 1.0, const QString &name = "OperationalSpaceCtrl");
  virtual ~OperationalSpaceControl();

  void setQInit(const JointState &q_init) override;
  void setQHome(const JointState &q_home) override;
  void setQPark(const JointState &q_park) override;

private:
  bool init();
  bool start();
  Vector6d update(const RobotTime &time, const JointState &state);
  bool stop();

  // RViz marker helpers
  void publishDeleteAllMarkers();
  void resetMarkerNewSegment();
  void publishControlDebug(double t_sec, const std::string &phase, const JointState &state, const Vector6d &tau_cmd);

  // ===== phase helpers =====
  bool inSafePhase(const cc::JointPosition &q6) const;
  void ensureMoveInit(double t_sec, const cc::JointPosition &q6);
  void ensureDrawInit(double t_sec, const cc::JointPosition &q6);
  void trajectoryCallback(const std_msgs::String::ConstPtr &msg);
  bool hasPendingTrajectory() const;
  bool getTrajectoryAtIndex(size_t index, PlanarPolynomialTrajectory &traj) const;
  static bool extractXmlTag(const std::string &xml, const std::string &tag, std::string &content);
  static bool extractXmlAttribute(const std::string &xml, const std::string &attr, std::string &value);
  static bool toDouble(const std::string &text, double &value);
  static bool toInt(const std::string &text, int &value);
  static std::vector<double> parseCoefficientList(const std::string &text);
  static double evalPoly(const std::vector<double> &coeff, double s);
  static double evalPolyDerivative(const std::vector<double> &coeff, double s);

  // ===== reference generators =====
  void cartesianDesiredMove(double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d, cc::Rotation3 &Rd, cc::Vector3 &Wd);
  void cartesianDesiredDraw(double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d, cc::Rotation3 &Rd, cc::Vector3 &Wd);
  Vector6d computeQdotR_MOVE_and_DRAW(double t_sec, const cc::JointPosition &q6);

  // ===== model helpers =====
  cc::Vector3 fkPos(const cc::JointPosition &q6) const;
  cc::Rotation3 fkOri(const cc::JointPosition &q6) const;
  Matrix6d jacobian6(const cc::JointPosition &q6) const;

  // pixel-to-world conversion (with 90° CW rotation about z)
  void pixelToWorld(double x_px, double y_px, double &x_w, double &y_w) const;
  void pixelVelToWorld(double dx_px, double dy_px, double &dx_w, double &dy_w) const;

  // Damped Least Squares
  static Vector6d dlsSolve6(const Matrix6d &J, const Vector6d &xdot, double lambda = 0.05);
  static Vector6d dlsSolve3(const Matrix3x6d &Jp, const cc::Vector3 &xdot_p, double lambda = 0.03);

  void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
};

} // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif
