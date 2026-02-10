#ifndef UR_ROBOT_LLI_OPERATIONALSPACECONTROL_H
#define UR_ROBOT_LLI_OPERATIONALSPACECONTROL_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>

#include <ur_model/ur_model.h>
#include <control_core/types.h>

#include <Eigen/Core>  // ✅ added: ensure Eigen::Matrix types compile reliably

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class OperationalSpaceControl : public ControlEffort
    {
    public:
      // Controller type: OP_PD or OP_PID
      enum ControllerType
      {
        OP_PD = 1,
        OP_PID = 2
      };

    private:
      ros::NodeHandle nh_;
      ur::URModel model_;
      bool model_ready_;

      // Gains: same meaning as the Python version
      Matrix6d Kd_;        // joint space damping on Sq
      cc::Matrix3 Kp_x_;   // cartesian position feedback
      cc::Matrix3 Ki_x_;   // cartesian integral gain
      Matrix6d Kp_q_;      // joint reference gain

      // Integral term (anti-windup)
      cc::Vector3 int_x_;
      double int_x_max_;

      Vector6d q_safe_;
      JointState q_init_;
      JointState q_home_;
      JointState q_park_;

      // Differentiate qdot_r to get qddot_r
      Vector6d qdot_r_prev_;
      bool qdot_r_prev_valid_;
      double prev_time_sec_;
      Vector6d last_q_;
      bool last_q_valid_;

      // Saturations
      double tau_max_;
      double xdot_r_max_;
      double qdot_r_max_;

      ControllerType controller_type_;

      // RViz visualization: desired trajectory + error
      ros::Publisher traj_pub_;
      ros::Publisher actual_traj_pub_;
      ros::Publisher task_error_pub_;
      visualization_msgs::Marker traj_marker_;
      visualization_msgs::Marker actual_traj_marker_;
      std::vector<geometry_msgs::Point> traj_points_;
      std::vector<geometry_msgs::Point> actual_traj_points_;

    public:
      OperationalSpaceControl(double weight = 1.0, const QString &name = "OperationalSpaceCtrl");
      virtual ~OperationalSpaceControl();

      void setQInit(const JointState &q_init);
      void setQHome(const JointState &q_home);
      void setQPark(const JointState &q_park);

    private:
      bool init();
      bool start();
      // Control core: compute tau
      Vector6d update(const RobotTime &time, const JointState &state);
      bool stop();

      // RViz marker helpers
      void publishDeleteAllMarkers();
      void resetMarkerNewSegment();
      void setMarkerColorForCurrentRun();

      void resetIntegrators();

      // Reference velocity: compute qdot_r from state
      Vector6d computeQdotR(double t_sec, const cc::JointPosition &q, double dt);
      Vector6d qdotRJointToTarget(const cc::JointPosition &q, const Vector6d &q_target);

      // Operational-space reference: compute xdot_r, then map to qdot_r
      cc::Vector3 xdotROperational(double t_sec, const cc::JointPosition &q, double dt);
      void cartesianDesired(double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d);
      bool inSafePhase(double t_sec, const cc::JointPosition &q) const;

      // Model interfaces
      cc::Vector3 fkPos(const cc::JointPosition &q) const;
      Eigen::Matrix<double, 3, 6> jacobianLinear(const cc::JointPosition &q) const;

      // 3x6 pseudo-inverse: for qdot_r = pinv(Jv) * xdot_r
      // changed eps default: 1e-6 -> 1e-3 (more robust near singularities)
      static Eigen::Matrix<double, 6, 3> pinv3x6(const Eigen::Matrix<double, 3, 6> &J, double eps = 1e-3);
    };

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_OPERATIONALSPACECONTROL_H
