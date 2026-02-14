#ifndef UR_ROBOT_LLI_OPERATIONALSPACECONTROL_H
#define UR_ROBOT_LLI_OPERATIONALSPACECONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>

#include <ur_model/ur_model.h>
#include <control_core/types.h>

#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

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

  enum Phase
  {
    PHASE_SAFE = 0,
    PHASE_DRAW = 1
  };

private:
  ros::NodeHandle nh_;
  ur::URModel model_;
  bool model_ready_;

  // ===== YAML switches =====
  bool enable_safe_;
  bool enable_draw_;

  // ===== SAFE =====
  Matrix6d Kp_q6_;      // joint gain (diag) used to build qrP in SAFE
  Matrix6d Kd_safe6_;   // damping in SAFE (diag)
  Vector6d q_safe6_;
  double safe_tol_;

  // ===== DRAW =====
  Matrix3d Kp_p_;       // position feedback (diag)
  Matrix3d Kp_o_;       // orientation feedback (diag) (DRAW only)
  Matrix6d Kd6_;        // damping on sliding variable in DRAW (diag)
  double draw_time_;    // fixed 20s
  double draw_length_;  // 0.5m along +Y

  // draw timing & start pose
  bool draw_initialized_;
  double t_draw0_;
  cc::Vector3 X_start_;     // fixed x,z and y start
  cc::Rotation3 R_draw_;    // desired orientation in DRAW (tool z -> +X)

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
  void ensureDrawInit(double t_sec, const cc::JointPosition &q6);

  // ===== reference generators =====
  void cartesianDesired(double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d, cc::Rotation3 &Rd, cc::Vector3 &Wd);
  Vector6d computeQdotR_DRAW(double t_sec, const cc::JointPosition &q6);

  // ===== model helpers =====
  cc::Vector3 fkPos(const cc::JointPosition &q6) const;
  cc::Rotation3 fkOri(const cc::JointPosition &q6) const;
  Matrix6d jacobian6(const cc::JointPosition &q6) const;

  // Damped Least Squares
  static Vector6d dlsSolve6(const Matrix6d &J, const Vector6d &xdot, double lambda = 0.05);
  static Vector6d dlsSolve3(const Matrix3x6d &Jp, const cc::Vector3 &xdot_p, double lambda = 0.03);
};

} // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif
