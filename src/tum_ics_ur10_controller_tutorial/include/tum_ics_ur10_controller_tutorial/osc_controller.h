#ifndef UR_ROBOT_LLI_OSCCONTROLLER_H
#define UR_ROBOT_LLI_OSCCONTROLLER_H

#include <tum_ics_ur_robot_lli/RobotControllers/ControlEffort.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ur_model/ur_model.h>

// Debug message (same style as the example controller)
#include <tum_ics_ur_robot_msgs/ControlData.h>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    class OscController : public ControlEffort
    {
    private:
      bool is_first_iter_;

      ros::NodeHandle nh_;
      ros::Publisher control_data_pub_;

      // Task-space gains
      Matrix3d Kp_pos_;
      Matrix3d Kd_pos_;
      Matrix3d Kp_ori_;
      Matrix3d Kd_ori_;

      // Nullspace posture gains
      Matrix6d Kp_null_;
      Matrix6d Kd_null_;

      // Damping for pseudo-inverse
      double damp_lambda_;

      // Circle trajectory parameters
      double circle_radius_;
      double circle_center_x_;
      double circle_center_y_;
      double circle_plane_z_;
      double circle_period_;

      // Orientation constraint: tool z-axis should align with board normal (+Z or -Z)
      // Default is -1 (tool z-axis aligns with -Z, i.e., pointing towards the board)
      double tool_z_direction_;

      // Soft-start / safety limits
      double ramp_time_;
      double max_f_pos_;
      double max_f_ori_;
      double max_tau_;

      // Stored initial state
      Vector6d q_start_;
      JointState q_init_;
      JointState q_home_;
      JointState q_park_;
      ur::URModel model_;

      // Stored initial tool pose for ramp
      Vector3d p_start_;
      bool have_p_start_;

      // Debug
      Vector6d tau_cmd_;

    public:
      OscController(double weight = 1.0, const QString &name = "osc_controller");

      ~OscController();

      void setQInit(const JointState &q_init);

      void setQHome(const JointState &q_home);

      void setQPark(const JointState &q_park);

    private:
      bool init();

      bool start();

      Vector6d update(const RobotTime &time, const JointState &state);

      bool stop();

    private:
      // Compute desired circle trajectory in task space (position/vel/acc)
      void getCircleRef(double t,
                        Vector3d &pd,
                        Vector3d &vd,
                        Vector3d &ad) const;

      // Damped pseudo-inverse for a 6x6 Jacobian (square case, but we keep damping for robustness)
      Matrix6d dampedPseudoInverse6x6(const Matrix6d &J) const;

      // Build a torque-level nullspace projector N = I - J^T * (J^+)^T
      Matrix6d torqueNullspaceProjector(const Matrix6d &J, const Matrix6d &J_pinv) const;

      // Utility: clamp vector norm to a maximum
      Vector3d clampVectorNorm(const Vector3d &v, double max_norm) const;

      // Utility: clamp each element of a 6D torque vector
      Vector6d clampTauAbs(const Vector6d &tau, double max_abs) const;

      // Smoothstep (quintic) for ramp: s in [0,1], with zero vel/acc at ends
      double smoothStep5(double x) const;

      // Derivative of smoothstep5 with respect to x (used for velocity blending)
      double smoothStep5Dot(double x) const;
    };

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli

#endif // UR_ROBOT_LLI_OSCCONTROLLER_H
