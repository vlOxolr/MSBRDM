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

private:
  ros::NodeHandle nh_;
  ur::URModel model_;
  bool model_ready_;

  // Gains
  Matrix6d Kd6_;    // damping on Sq (6x6 diag)
  Matrix3d Kp_p_;   // cartesian position feedback (3x3 diag)
  Matrix3d Ki_p_;   // cartesian position integral gain (3x3 diag)
  Matrix3d Kp_o_;   // cartesian orientation feedback (3x3 diag)
  Matrix3d Ki_o_;   // cartesian orientation integral gain (3x3 diag)
  Matrix6d Kp_q6_;  // joint ref gain (6x6 diag) (used for SAFE torque PD)

  // Task-space integral term (anti-windup)
  Vector6d int_task_;
  double int_p_max_;
  double int_o_max_;

  // Safe pose in joint/task space
  Vector6d q_safe6_;
  cc::Rotation3 R_safe_;

  // Differentiate qdot_r (6-DoF)
  Vector6d qdot_r_prev6_;
  bool qdot_r_prev_valid_;
  double prev_time_sec_;

  JointState q_init_;
  JointState q_home_;
  JointState q_park_;

  // Saturations
  double tau_max_;
  double xdot_r_max_;
  double wdot_r_max_;
  double qdot_r_max_;

  ControllerType controller_type_;

  // RViz visualization
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
  void resetIntegrators();
  void publishControlDebug(double t_sec, const std::string &phase, const JointState &state, const Vector6d &tau_cmd);

  // ===== Control core =====
  Vector6d computeQdotR6(double t_sec, const cc::JointPosition &q6, double dt);

  Vector6d xdotROperational(double t_sec, const cc::JointPosition &q6, double dt);
  void cartesianDesired(double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d, cc::Rotation3 &Rd, cc::Vector3 &Wd);
  bool inSafePhase(double t_sec, const cc::JointPosition &q6) const;

  // Model interfaces
  cc::Vector3 fkPos(const cc::JointPosition &q6) const;
  cc::Rotation3 fkOri(const cc::JointPosition &q6) const;

  // 6x6 geometric Jacobian
  Matrix6d jacobian6(const cc::JointPosition &q6) const;

  // Damped Least Squares: qdot = J^T (J J^T + λ^2 I)^-1 xdot
  static Vector6d dlsSolve6(const Matrix6d &J, const Vector6d &xdot, double lambda = 0.05);
};

} // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif
