#include <tum_ics_ur10_controller_tutorial/osc_controller.h>

#include <cmath>
#include <Eigen/Dense>

namespace tum_ics_ur_robot_lli
{
  namespace RobotControllers
  {

    OscController::OscController(double weight, const QString &name) :
      ControlEffort(name, SPLINE_TYPE, JOINT_SPACE, weight),
      is_first_iter_(true),
      ee_path_computed_(),
      ee_path_desired_(),
      path_decim_(1),
      path_count_(0),
      path_max_len_(5000),
      Kp_pos_(Matrix3d::Zero()),
      Kd_pos_(Matrix3d::Zero()),
      Kp_ori_(Matrix3d::Zero()),
      Kd_ori_(Matrix3d::Zero()),
      Kp_null_(Matrix6d::Zero()),
      Kd_null_(Matrix6d::Zero()),
      damp_lambda_(0.02),
      circle_radius_(0.2),
      circle_center_x_(0.5),
      circle_center_y_(0.0),
      circle_plane_z_(0.0),
      circle_period_(8.0),
      tool_z_direction_(-1.0),
      ramp_time_(1.5),
      max_f_pos_(80.0),
      max_f_ori_(8.0),
      max_tau_(120.0),
      model_("ur10_model"),
      p_start_(Vector3d::Zero()),
      have_p_start_(false),
      tau_cmd_(Vector6d::Zero())
    {
      control_data_pub_ = nh_.advertise<tum_ics_ur_robot_msgs::ControlData>("osc_controller_data", 1);

      // Debug publishers (Path)
      ee_path_computed_pub_ = nh_.advertise<nav_msgs::Path>("osc_controller/ee_path_computed", 1);
      ee_path_desired_pub_ = nh_.advertise<nav_msgs::Path>("osc_controller/ee_path_desired", 1);

      // Debug publishers (PointStamped)
      ee_pos_computed_pub_ = nh_.advertise<geometry_msgs::PointStamped>("osc_controller/ee_pos_computed", 1);
      ee_pos_desired_pub_ = nh_.advertise<geometry_msgs::PointStamped>("osc_controller/ee_pos_desired", 1);
    }

    OscController::~OscController()
    {
    }

    void OscController::setQInit(const JointState &q_init)
    {
      q_init_ = q_init;
    }

    void OscController::setQHome(const JointState &q_home)
    {
      q_home_ = q_home;
    }

    void OscController::setQPark(const JointState &q_park)
    {
      q_park_ = q_park;
    }

    bool OscController::init()
    {
      ROS_WARN_STREAM("OscController::init");
      std::vector<double> vec;
      ros::NodeHandle nh_private("~");

      if (!model_.initRequest(nh_private))
      {
        ROS_ERROR_STREAM("OscController init(): failed to initialize ur10 model namespace 'ur10_model'.");
        m_error = true;
        return false;
      }

      // check namespace
      std::string ns = "~osc_controller";
      if (!ros::param::has(ns))
      {
        ROS_ERROR_STREAM("OscController init(): parameters not defined in: " << ns);
        m_error = true;
        return false;
      }

      // --- Position gains ---
      ros::param::get(ns + "/gains_pos_p", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("gains_pos_p: wrong number of dimensions: " << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < 3; i++) Kp_pos_(i, i) = vec[i];

      ros::param::get(ns + "/gains_pos_d", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("gains_pos_d: wrong number of dimensions: " << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < 3; i++) Kd_pos_(i, i) = vec[i];

      // --- Orientation gains (only used to align tool z-axis with board normal) ---
      ros::param::get(ns + "/gains_ori_p", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("gains_ori_p: wrong number of dimensions: " << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < 3; i++) Kp_ori_(i, i) = vec[i];

      ros::param::get(ns + "/gains_ori_d", vec);
      if (vec.size() < 3)
      {
        ROS_ERROR_STREAM("gains_ori_d: wrong number of dimensions: " << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < 3; i++) Kd_ori_(i, i) = vec[i];

      // --- Nullspace gains ---
      ros::param::get(ns + "/gains_null_p", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_null_p: wrong number of dimensions: " << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++) Kp_null_(i, i) = vec[i];

      ros::param::get(ns + "/gains_null_d", vec);
      if (vec.size() < STD_DOF)
      {
        ROS_ERROR_STREAM("gains_null_d: wrong number of dimensions: " << vec.size());
        m_error = true;
        return false;
      }
      for (int i = 0; i < STD_DOF; i++) Kd_null_(i, i) = vec[i];

      // --- Trajectory parameters ---
      ros::param::get(ns + "/circle_radius", circle_radius_);
      ros::param::get(ns + "/circle_center_x", circle_center_x_);
      ros::param::get(ns + "/circle_center_y", circle_center_y_);
      ros::param::get(ns + "/circle_plane_z", circle_plane_z_);
      ros::param::get(ns + "/circle_period", circle_period_);

      if (!(circle_period_ > 0.0))
      {
        ROS_ERROR_STREAM("circle_period is invalid: " << circle_period_ << " -> fallback to 8.0");
        circle_period_ = 8.0;
      }

      // --- Damping and orientation direction ---
      ros::param::get(ns + "/damp_lambda", damp_lambda_);
      ros::param::get(ns + "/tool_z_direction", tool_z_direction_);

      if (tool_z_direction_ >= 0.0)
        tool_z_direction_ = 1.0;
      else
        tool_z_direction_ = -1.0;

      // --- Soft-start / limits (optional params) ---
      ros::param::get(ns + "/ramp_time", ramp_time_);
      ros::param::get(ns + "/max_f_pos", max_f_pos_);
      ros::param::get(ns + "/max_f_ori", max_f_ori_);
      ros::param::get(ns + "/max_tau", max_tau_);

      if (ramp_time_ < 0.0) ramp_time_ = 0.0;
      if (max_f_pos_ <= 0.0) max_f_pos_ = 80.0;
      if (max_f_ori_ <= 0.0) max_f_ori_ = 8.0;
      if (max_tau_ <= 0.0) max_tau_ = 120.0;

      // --- Path debug params (optional) ---
      ros::param::get(ns + "/path_decim", path_decim_);
      ros::param::get(ns + "/path_max_len", path_max_len_);
      if (path_decim_ < 1) path_decim_ = 1;
      if (path_max_len_ < 10) path_max_len_ = 10;

      ROS_WARN_STREAM("Kp_pos:\n" << Kp_pos_);
      ROS_WARN_STREAM("Kd_pos:\n" << Kd_pos_);
      ROS_WARN_STREAM("Kp_ori:\n" << Kp_ori_);
      ROS_WARN_STREAM("Kd_ori:\n" << Kd_ori_);
      ROS_WARN_STREAM("Kp_null:\n" << Kp_null_);
      ROS_WARN_STREAM("Kd_null:\n" << Kd_null_);
      ROS_WARN_STREAM("circle: r=" << circle_radius_
                                  << " center=(" << circle_center_x_ << "," << circle_center_y_ << ")"
                                  << " z=" << circle_plane_z_
                                  << " period=" << circle_period_);
      ROS_WARN_STREAM("damp_lambda=" << damp_lambda_ << ", tool_z_direction=" << tool_z_direction_);
      ROS_WARN_STREAM("ramp_time=" << ramp_time_
                                   << ", max_f_pos=" << max_f_pos_
                                   << ", max_f_ori=" << max_f_ori_
                                   << ", max_tau=" << max_tau_);
      ROS_WARN_STREAM("path_decim=" << path_decim_ << ", path_max_len=" << path_max_len_);

      return true;
    }

    bool OscController::start()
    {
      ROS_WARN_STREAM("OscController::start");
      is_first_iter_ = true;
      have_p_start_ = false;

      // Reset debug paths
      ee_path_computed_.poses.clear();
      ee_path_desired_.poses.clear();
      ee_path_computed_.header.frame_id = "world";
      ee_path_desired_.header.frame_id = "world";
      path_count_ = 0;

      return true;
    }

    void OscController::getCircleRef(double t,
                                     Vector3d &pd,
                                     Vector3d &vd,
                                     Vector3d &ad) const
    {
      // Circle in XY plane, z = constant
      // pd = [cx + r cos(wt), cy + r sin(wt), z]
      // vd = derivative
      // ad = second derivative
      const double w = 2.0 * M_PI / circle_period_;
      const double c = std::cos(w * t);
      const double s = std::sin(w * t);

      pd(0) = circle_center_x_ + circle_radius_ * c;
      pd(1) = circle_center_y_ + circle_radius_ * s;
      pd(2) = circle_plane_z_;

      vd(0) = -circle_radius_ * w * s;
      vd(1) =  circle_radius_ * w * c;
      vd(2) = 0.0;

      ad(0) = -circle_radius_ * w * w * c;
      ad(1) = -circle_radius_ * w * w * s;
      ad(2) = 0.0;
    }

    Matrix6d OscController::dampedPseudoInverse6x6(const Matrix6d &J) const
    {
      // J^+ = J^T (J J^T + lambda^2 I)^-1
      Matrix6d I = Matrix6d::Identity();
      Matrix6d JJt = J * J.transpose();
      Matrix6d inv = (JJt + (damp_lambda_ * damp_lambda_) * I).inverse();
      Matrix6d J_pinv = J.transpose() * inv;
      return J_pinv;
    }

    Matrix6d OscController::torqueNullspaceProjector(const Matrix6d &J, const Matrix6d &J_pinv) const
    {
      // Torque-level projector:
      // N = I - J^T * (J^+)^T
      Matrix6d I = Matrix6d::Identity();
      Matrix6d N = I - J.transpose() * J_pinv.transpose();
      return N;
    }

    Vector3d OscController::clampVectorNorm(const Vector3d &v, double max_norm) const
    {
      double n = v.norm();
      if (n <= 1e-12) return v;
      if (n > max_norm) return (max_norm / n) * v;
      return v;
    }

    Vector6d OscController::clampTauAbs(const Vector6d &tau, double max_abs) const
    {
      Vector6d out = tau;
      for (int i = 0; i < STD_DOF; i++)
      {
        if (out(i) > max_abs) out(i) = max_abs;
        if (out(i) < -max_abs) out(i) = -max_abs;
      }
      return out;
    }

    double OscController::smoothStep5(double x) const
    {
      // Quintic smoothstep: 6x^5 - 15x^4 + 10x^3
      if (x <= 0.0) return 0.0;
      if (x >= 1.0) return 1.0;
      double x2 = x * x;
      double x3 = x2 * x;
      double x4 = x3 * x;
      double x5 = x4 * x;
      return 6.0 * x5 - 15.0 * x4 + 10.0 * x3;
    }

    double OscController::smoothStep5Dot(double x) const
    {
      // Derivative: 30x^4 - 60x^3 + 30x^2
      if (x <= 0.0) return 0.0;
      if (x >= 1.0) return 0.0;
      double x2 = x * x;
      double x3 = x2 * x;
      double x4 = x3 * x;
      return 30.0 * x4 - 60.0 * x3 + 30.0 * x2;
    }

    Vector6d OscController::update(const RobotTime &time, const JointState &state)
    {
      if (is_first_iter_)
      {
        q_start_ = state.q;
        ROS_WARN_STREAM("OSC START q [RAD]: " << q_start_.transpose());
        is_first_iter_ = false;
      }

      // --- Get current tool pose ---
      // Use existing model functions (do not re-implement kinematics/jacobian)
      // T_tool_0 is tool w.r.t. robot base frame (0)
      cc::HomogeneousTransformation T_tool_0 = model_.T_tool_0(state.q);
      cc::Jacobian J_tool_0_cc = model_.J_tool_0(state.q);

      Matrix6d J = J_tool_0_cc;

      Vector3d p;
      p << T_tool_0(0, 3), T_tool_0(1, 3), T_tool_0(2, 3);

      if (!have_p_start_)
      {
        p_start_ = p;
        have_p_start_ = true;
      }

      Matrix3d R;
      R << T_tool_0(0, 0), T_tool_0(0, 1), T_tool_0(0, 2),
           T_tool_0(1, 0), T_tool_0(1, 1), T_tool_0(1, 2),
           T_tool_0(2, 0), T_tool_0(2, 1), T_tool_0(2, 2);

      // Current twist xdot = J * qdot
      Vector6d xdot = J * state.qp;
      Vector3d v = xdot.segment<3>(0);
      Vector3d omega = xdot.segment<3>(3);

      // --- Desired circle reference ---
      Vector3d pd_c(Vector3d::Zero()), vd_c(Vector3d::Zero()), ad_c(Vector3d::Zero());
      getCircleRef(time.tD(), pd_c, vd_c, ad_c);

      // --- Soft-start blending (prevents huge initial jump) ---
      Vector3d pd = pd_c;
      Vector3d vd = vd_c;
      Vector3d ad = ad_c;

      if (ramp_time_ > 1e-6)
      {
        double x = time.tD() / ramp_time_;
        if (x < 1.0)
        {
          double s = smoothStep5(x);
          double sdot = smoothStep5Dot(x) / ramp_time_;

          // Blend position: from current start position to circle reference
          // pd = p_start + s*(pd_c - p_start)
          // vd = s*vd_c + sdot*(pd_c - p_start)
          Vector3d dp = (pd_c - p_start_);
          pd = p_start_ + s * dp;
          vd = s * vd_c + sdot * dp;
          ad = Vector3d::Zero(); // keep it simple and safe in ramp phase
        }
      }

      // --- Publish debug trajectories (PointStamped) ---
      // NOTE: These are computed inside controller (not from external topics)
      {
        ros::Time stamp = ros::Time::now();

        geometry_msgs::PointStamped msg_p;
        msg_p.header.stamp = stamp;
        msg_p.header.frame_id = "world";
        msg_p.point.x = p(0);
        msg_p.point.y = p(1);
        msg_p.point.z = p(2);
        ee_pos_computed_pub_.publish(msg_p);

        geometry_msgs::PointStamped msg_pd;
        msg_pd.header.stamp = stamp;
        msg_pd.header.frame_id = "world";
        msg_pd.point.x = pd(0);
        msg_pd.point.y = pd(1);
        msg_pd.point.z = pd(2);
        ee_pos_desired_pub_.publish(msg_pd);
      }

      // --- Publish debug trajectories as Path ---
      path_count_++;
      if ((path_count_ % path_decim_) == 0)
      {
        ros::Time stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose_p;
        pose_p.header.stamp = stamp;
        pose_p.header.frame_id = "world";
        pose_p.pose.position.x = p(0);
        pose_p.pose.position.y = p(1);
        pose_p.pose.position.z = p(2);
        pose_p.pose.orientation.w = 1.0;
        pose_p.pose.orientation.x = 0.0;
        pose_p.pose.orientation.y = 0.0;
        pose_p.pose.orientation.z = 0.0;

        geometry_msgs::PoseStamped pose_pd;
        pose_pd.header.stamp = stamp;
        pose_pd.header.frame_id = "world";
        pose_pd.pose.position.x = pd(0);
        pose_pd.pose.position.y = pd(1);
        pose_pd.pose.position.z = pd(2);
        pose_pd.pose.orientation.w = 1.0;
        pose_pd.pose.orientation.x = 0.0;
        pose_pd.pose.orientation.y = 0.0;
        pose_pd.pose.orientation.z = 0.0;

        ee_path_computed_.header.stamp = stamp;
        ee_path_desired_.header.stamp = stamp;

        // Force valid frame_id each publish (avoid empty frame issues)
        ee_path_computed_.header.frame_id = "world";
        ee_path_desired_.header.frame_id = "world";

        ee_path_computed_.poses.push_back(pose_p);
        ee_path_desired_.poses.push_back(pose_pd);

        // Keep path bounded
        if ((int)ee_path_computed_.poses.size() > path_max_len_)
        {
          int extra = (int)ee_path_computed_.poses.size() - path_max_len_;
          ee_path_computed_.poses.erase(ee_path_computed_.poses.begin(),
                                        ee_path_computed_.poses.begin() + extra);
        }
        if ((int)ee_path_desired_.poses.size() > path_max_len_)
        {
          int extra = (int)ee_path_desired_.poses.size() - path_max_len_;
          ee_path_desired_.poses.erase(ee_path_desired_.poses.begin(),
                                       ee_path_desired_.poses.begin() + extra);
        }

        ee_path_computed_pub_.publish(ee_path_computed_);
        ee_path_desired_pub_.publish(ee_path_desired_);
      }

      // --- Position task (impedance in task space) ---
      Vector3d e_p = pd - p;
      Vector3d e_v = vd - v;

      Vector3d f_pos = Kp_pos_ * e_p + Kd_pos_ * e_v;
      f_pos = clampVectorNorm(f_pos, max_f_pos_);

      // --- Orientation task: keep tool z-axis aligned with board normal (+/-Z) ---
      // Board plane is z=0 => normal is +Z in base frame
      Vector3d n_board;
      n_board << 0.0, 0.0, 1.0;

      // Tool z-axis in base frame
      Vector3d z_tool = R.col(2);

      // Desired tool z-axis
      Vector3d z_des = tool_z_direction_ * n_board;

      // Orientation error for aligning one axis: e_o = z_tool x z_des
      // This is a standard small-angle approximation error vector
      Vector3d e_o = z_tool.cross(z_des);

      // Desired angular velocity is zero
      Vector3d e_omega = -omega;

      Vector3d f_ori = Kp_ori_ * e_o + Kd_ori_ * e_omega;
      f_ori = clampVectorNorm(f_ori, max_f_ori_);

      // --- Build 6D wrench ---
      Vector6d wrench;
      wrench.segment<3>(0) = f_pos;
      wrench.segment<3>(3) = f_ori;

      // --- OSC torque: tau_task = J^T * wrench ---
      Vector6d tau_task = J.transpose() * wrench;

      // --- Nullspace posture (to keep joints near q_home if provided, otherwise near start) ---
      Vector6d q_null = q_start_;
      if (q_home_.q.size() == STD_DOF)
      {
        q_null = q_home_.q;
      }

      Vector6d tau_null;
      tau_null.setZero();

      // Simple PD in joint space for nullspace
      Vector6d e_q = q_null - state.q;
      Vector6d e_qp = -state.qp;

      tau_null = Kp_null_ * e_q + Kd_null_ * e_qp;

      // --- Project nullspace torque ---
      Matrix6d J_pinv = dampedPseudoInverse6x6(J);
      Matrix6d N = torqueNullspaceProjector(J, J_pinv);
      Vector6d tau_ns = N * tau_null;

      // --- Gravity compensation ---
      Vector6d g = model_.gravityVector(state.q);

      // --- Final command ---
      tau_cmd_ = tau_task + tau_ns + g;

      // Hard safety clamp to avoid triggering the simulator velocity limiter
      tau_cmd_ = clampTauAbs(tau_cmd_, max_tau_);

      // --- Publish debug data (same message type as tutorial) ---
      tum_ics_ur_robot_msgs::ControlData msg;
      msg.header.stamp = ros::Time::now();
      msg.time = time.tD();

      for (int i = 0; i < STD_DOF; i++)
      {
        msg.q[i] = state.q(i);
        msg.qp[i] = state.qp(i);
        msg.qpp[i] = state.qpp(i);

        // For compatibility with existing fields:
        // Put a "virtual desired joint" as q_null (informative only)
        msg.qd[i] = q_null(i);
        msg.qpd[i] = 0.0;

        // Use Dq/Dqp to expose joint errors to the plotter
        msg.Dq[i] = e_q(i);
        msg.Dqp[i] = e_qp(i);

        // Raw measured torques
        msg.torques[i] = state.tau(i);
      }
      control_data_pub_.publish(msg);

      return tau_cmd_;
    }

    bool OscController::stop()
    {
      return true;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
