#include <tum_ics_ur10_controller_tutorial/operational_space_controller.h>

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

namespace
{
  // =========================================================
  // Trajectory: polynomial (any order) + optional sinusoid
  // Two-stage flow: move-to-start, then draw
  // =========================================================
  struct FittedTrajConfig
  {
    bool loaded = false;
    bool enabled = true;

    // Drawing plane: fix z for xy-plane drawing
    bool fixed_z = true;
    double z_paper = -0.40;

    // Drawing plane: fix x for yz-plane drawing
    bool fixed_x = false;
    double x_paper = 0.0;

    // Go to a safe joint pose before drawing
    bool start_with_safe = true;
    double safe_time = 3.0; // seconds
    double safe_tol = 0.05; // rad (norm),  error threshold of joint state to confirm if SAFE is finished

    // Move-to-start before drawing (task space)
    bool use_move_to_start = true;
    double move_time = 20.0; // seconds (timeout)
    double move_tol = 0.01;  // meters
    bool moving = false;

    // Time handling
    bool clamp_time = true;
    double T_end = 10.0; // seconds

    // t0: controller start time origin (for safe/move timing)
    double t0 = -1.0;

    // Draw time origin (t_local = t_sec - t_draw0)
    double t_draw0 = -1.0;

    bool draw_started = false;

    // Cached start point Xd(t=0)
    bool have_start = false;
    cc::Vector3 X_start = cc::Vector3::Zero();

    // Polynomial coefficients: x(t)=a0+a1 t+...+aN t^N
    std::vector<double> ax{0, 0, 0, 0, 0, 0};
    std::vector<double> ay{0, 0, 0, 0, 0, 0};
    std::vector<double> az{0, 0, 0, 0, 0, 0};

    // Optional sinusoid A*sin(w t + phi)
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

    // Horner evaluation: polyN(a, t)=a0+a1 t+...+aN t^N
    static double polyN(const std::vector<double> &c, double t)
    {
      if (c.empty())
        return 0.0;
      double acc = c.back();
      for (int i = static_cast<int>(c.size()) - 2; i >= 0; --i)
        acc = acc * t + c[static_cast<size_t>(i)];
      return acc;
    }

    // First derivative: d/dt polyN
    static double dpolyN(const std::vector<double> &c, double t)
    {
      if (c.size() <= 1)
        return 0.0;

      double acc = static_cast<double>(c.size() - 1) * c.back();
      for (int i = static_cast<int>(c.size()) - 2; i >= 1; --i)
        acc = acc * t + static_cast<double>(i) * c[static_cast<size_t>(i)];
      return acc;
    }

    // ===== NEW =====
    // Second derivative: d^2/dt^2 polyN
    static double ddpolyN(const std::vector<double> &c, double t)
    {
      if (c.size() <= 2)
        return 0.0;

      // For aN t^N, second derivative is N*(N-1)*aN t^(N-2)
      double N = static_cast<double>(c.size() - 1);
      double acc = N * (N - 1.0) * c.back();

      // iterate from power (N-1) down to 2
      for (int i = static_cast<int>(c.size()) - 2; i >= 2; --i)
      {
        double ii = static_cast<double>(i);
        acc = acc * t + ii * (ii - 1.0) * c[static_cast<size_t>(i)];
      }
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

    // eval: returns position and velocity (as before)
    void eval(double t_local,
              double &x, double &y, double &z,
              double &xd, double &yd, double &zd) const
    {
      double tt = t_local;

      if (clamp_time)
      {
        if (tt < 0.0)
          tt = 0.0;
        if (tt > T_end)
          tt = T_end;
      }

      x = polyN(ax, tt);
      y = polyN(ay, tt);
      z = polyN(az, tt);

      xd = dpolyN(ax, tt);
      yd = dpolyN(ay, tt);
      zd = dpolyN(az, tt);

      if (use_sin_x)
      {
        x += Ax * std::sin(wx * tt + phix);
        xd += Ax * wx * std::cos(wx * tt + phix);
      }
      if (use_sin_y)
      {
        y += Ay * std::sin(wy * tt + phiy);
        yd += Ay * wy * std::cos(wy * tt + phiy);
      }
      if (use_sin_z)
      {
        z += Az * std::sin(wz * tt + phiz);
        zd += Az * wz * std::cos(wz * tt + phiz);
      }

      if (fixed_z)
      {
        z = z_paper;
        zd = 0.0;
      }
      if (fixed_x)
      {
        x = x_paper;
        xd = 0.0;
      }
    }

    // ===== NEW =====
    // evalAcc: return position, velocity, acceleration
    void evalAcc(double t_local,
                 double &x, double &y, double &z,
                 double &xd, double &yd, double &zd,
                 double &xdd, double &ydd, double &zdd) const
    {
      double tt = t_local;

      if (clamp_time)
      {
        if (tt < 0.0)
          tt = 0.0;
        if (tt > T_end)
          tt = T_end;
      }

      x = polyN(ax, tt);
      y = polyN(ay, tt);
      z = polyN(az, tt);

      xd = dpolyN(ax, tt);
      yd = dpolyN(ay, tt);
      zd = dpolyN(az, tt);

      xdd = ddpolyN(ax, tt);
      ydd = ddpolyN(ay, tt);
      zdd = ddpolyN(az, tt);

      if (use_sin_x)
      {
        x += Ax * std::sin(wx * tt + phix);
        xd += Ax * wx * std::cos(wx * tt + phix);
        xdd += -Ax * wx * wx * std::sin(wx * tt + phix);
      }
      if (use_sin_y)
      {
        y += Ay * std::sin(wy * tt + phiy);
        yd += Ay * wy * std::cos(wy * tt + phiy);
        ydd += -Ay * wy * wy * std::sin(wy * tt + phiy);
      }
      if (use_sin_z)
      {
        z += Az * std::sin(wz * tt + phiz);
        zd += Az * wz * std::cos(wz * tt + phiz);
        zdd += -Az * wz * wz * std::sin(wz * tt + phiz);
      }

      if (fixed_z)
      {
        z = z_paper;
        zd = 0.0;
        zdd = 0.0;
      }
      if (fixed_x)
      {
        x = x_paper;
        xd = 0.0;
        xdd = 0.0;
      }
    }

    void computeStartIfNeeded()
    {
      if (have_start)
        return;

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
          Kd_(Matrix6d::Zero()),
          Kp_x_(cc::Matrix3::Zero()),
          Ki_x_(cc::Matrix3::Zero()),
          // ===== NEW =====
          Kd_x_(cc::Matrix3::Zero()),
          Kp_q_(Matrix6d::Zero()),
          int_x_(cc::Vector3::Zero()),
          int_x_max_(0.05),
          q_safe_(Vector6d::Zero()),
          qdot_r_prev_(Vector6d::Zero()),
          qdot_r_prev_valid_(false),
          prev_time_sec_(0.0),
          last_q_(Vector6d::Zero()),
          last_q_valid_(false),
          tau_max_(120.0),
          xdot_r_max_(0.25),
          qdot_r_max_(2.0),
          controller_type_(OP_PID),
          // ===== NEW =====
          Jv_prev_(Eigen::Matrix<double, 3, 6>::Zero()),
          Jv_prev_valid_(false)
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

      // Joint damping Kd (inner loop)
      if (ros::param::get(ns + "/Kd", v) && v.size() >= 6)
      {
        Kd_.setZero();
        for (int i = 0; i < 6; ++i)
          Kd_(i, i) = v[static_cast<size_t>(i)];
      }
      else
      {
        Kd_.diagonal() << 4.0, 4.0, 3.0, 4.0, 4.0, 3.0;
      }

      // Task-space proportional (position)
      if (ros::param::get(ns + "/Kp_x", v) && v.size() >= 3)
      {
        Kp_x_.setZero();
        for (int i = 0; i < 3; ++i)
          Kp_x_(i, i) = v[static_cast<size_t>(i)];
      }
      else
      {
        Kp_x_.diagonal() << 7.0, 7.0, 6.0;
      }

      // Task-space integral
      if (ros::param::get(ns + "/Ki_x", v) && v.size() >= 3)
      {
        Ki_x_.setZero();
        for (int i = 0; i < 3; ++i)
          Ki_x_(i, i) = v[static_cast<size_t>(i)];
      }
      else
      {
        Ki_x_.diagonal() << 4.0, 4.0, 4.0;
      }

      // ===== NEW =====
      // Task-space derivative (velocity) for Λ-based DRAW control
      // If not provided, use a reasonable default (critically damped-ish)
      if (ros::param::get(ns + "/Kd_x", v) && v.size() >= 3)
      {
        Kd_x_.setZero();
        for (int i = 0; i < 3; ++i)
          Kd_x_(i, i) = v[static_cast<size_t>(i)];
      }
      else
      {
        // heuristic: ~ 2*sqrt(Kp)
        Kd_x_.diagonal() << 4.5, 4.5, 4.0;
      }

      // SAFE joint-space proportional
      if (ros::param::get(ns + "/Kp_q", v) && v.size() >= 6)
      {
        Kp_q_.setZero();
        for (int i = 0; i < 6; ++i)
          Kp_q_(i, i) = v[static_cast<size_t>(i)];
      }
      else
      {
        Kp_q_.diagonal() << 2.0, 2.0, 1.5, 1.0, 1.0, 1.0;
      }

      ros::param::get(ns + "/int_x_max", int_x_max_);

      if (ros::param::get(ns + "/q_safe", v) && v.size() >= 6)
        for (int i = 0; i < 6; ++i)
          q_safe_(i) = v[static_cast<size_t>(i)];
      else
      {
        q_safe_.setZero();
        q_safe_(2) = M_PI / 2.0;
      }

      // Saturations
      ros::param::get(ns + "/tau_max", tau_max_);
      ros::param::get(ns + "/xdot_r_max", xdot_r_max_);
      ros::param::get(ns + "/qdot_r_max", qdot_r_max_);

      // Controller type (still used for MOVE and legacy xdot PID)
      int ctrl_type = static_cast<int>(controller_type_);
      ros::param::get(ns + "/controller_type", ctrl_type);
      controller_type_ = (ctrl_type == static_cast<int>(OP_PD)) ? OP_PD : OP_PID;

      // Load fitted trajectory
      g_traj.loadFromParams(ns);

      // Model namespace
      std::string model_ns;
      if (ros::param::get(ns + "/model_ns", model_ns) && !model_ns.empty())
      {
        model_ = ur::URModel(model_ns);
      }

      model_ready_ = model_.initRequest(nh_);
      if (!model_ready_)
      {
        ROS_ERROR_STREAM("OperationalSpaceControl init(): failed to init URModel. Check parameters under " << model_.name());
        m_error = true;
        return false;
      }

      // RViz + error topics
      traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/ur10/desired_trajectory", 1);
      actual_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/ur10/actual_trajectory", 1);
      task_error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur10/task_space_error", 1);

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

      return true;
    }

    bool OperationalSpaceControl::start()
    {
      qdot_r_prev_valid_ = false;
      prev_time_sec_ = 0.0;
      resetIntegrators();

      g_traj.resetTime();
      resetMarkerNewSegment();

      // ===== NEW =====
      Jv_prev_valid_ = false;

      if (model_ready_)
      {
        const cc::Vector3 X_safe = fkPos(q_safe_);
        ROS_INFO_STREAM("[OperationalSpaceControl] X_safe (world) = " << X_safe.transpose());
      }
      return true;
    }

    bool OperationalSpaceControl::stop() { return true; }

    // ===== NEW =====
    OperationalSpaceControl::Phase
    OperationalSpaceControl::currentPhase(double t_sec,
                                          const cc::JointPosition &q) const
    {
      if (!g_traj.enabled)
        return PHASE_DISABLED;

      if (inSafePhase(t_sec, q))
        return PHASE_SAFE;

      if (g_traj.use_move_to_start && !g_traj.draw_started)
        return PHASE_MOVE;

      return PHASE_DRAW;
    }

    // ============================
    // Main update: compute tau
    // SAFE & MOVE: keep old inner loop
    // DRAW: use Λ-based OSC (position only)
    // ============================
    Vector6d OperationalSpaceControl::update(const RobotTime &time, const JointState &state)
    {
      Vector6d tau = Vector6d::Zero();
      if (!model_ready_)
        return tau;

      const double t_sec = time.tD();

      const cc::JointPosition q = state.q;
      const cc::JointVelocity qP = state.qp;
      last_q_ = q;
      last_q_valid_ = true;

      // Initialize controller time origin once
      if (g_traj.t0 < 0.0)
        g_traj.t0 = t_sec;

      // Time step dt
      double dt = t_sec - prev_time_sec_;
      if (prev_time_sec_ <= 0.0 || !std::isfinite(dt) || dt <= 0.0)
        dt = 1e-3;
      prev_time_sec_ = t_sec;

      const Phase phase = currentPhase(t_sec, q);

      // ======================================================
      // 1) SAFE/MOVE: keep your original "qdot_r + computed torque"
      // ======================================================
      if (phase == PHASE_DISABLED || phase == PHASE_SAFE || phase == PHASE_MOVE)
      {
        // Reference joint velocity qdot_r (state machine unchanged)
        Vector6d qdot_r = computeQdotR(t_sec, q, dt);

        // Avoid a qddot spike on the first step
        if (!qdot_r_prev_valid_)
        {
          qdot_r_prev_ = qdot_r;
          qdot_r_prev_valid_ = true;
        }

        Vector6d qddot_r = (qdot_r - qdot_r_prev_) / dt;
        qdot_r_prev_ = qdot_r;

        // Sq = qdot - qdot_r (velocity error)
        Vector6d Sq = qP - qdot_r;

        // Dynamics terms
        const cc::MatrixDof &M = model_.inertiaMatrix(q);
        const cc::MatrixDof &C = model_.centripetalMatrix(q, qP);
        const cc::VectorDof &G = model_.gravityVector(q);

        Vector6d Yr_theta = M * qddot_r + C * qdot_r + G;

        // tau = -Kd*Sq + Yr_theta
        tau = (-Kd_ * Sq) + Yr_theta;

        for (int i = 0; i < 6; ++i)
          tau(i) = std::max(-tau_max_, std::min(tau(i), tau_max_));
      }
      // ======================================================
      // 2) DRAW: Λ-based Operational Space Controller
      // ======================================================
      else
      {
        // --- Desired trajectory (position, velocity, acceleration)
        cc::Vector3 Xd, Xdot_d;
        cartesianDesired(t_sec, Xd, Xdot_d);

        // Compute desired acceleration from trajectory (using g_traj directly)
        cc::Vector3 Xddot_d = cc::Vector3::Zero();
        if (g_traj.draw_started && g_traj.t_draw0 >= 0.0)
        {
          const double t_local = t_sec - g_traj.t_draw0;
          double x, y, z, xd, yd, zd, xdd, ydd, zdd;
          g_traj.evalAcc(t_local, x, y, z, xd, yd, zd, xdd, ydd, zdd);
          Xddot_d << xdd, ydd, zdd;
        }

        // --- Current end-effector state
        const cc::Vector3 X = fkPos(q);
        const Eigen::Matrix<double, 3, 6> Jv = jacobianLinear(q);
        const cc::Vector3 Xdot = Jv * qP;

        // --- Errors (note: e = Xd - X, edot = Xdot_d - Xdot)
        const cc::Vector3 e = Xd - X;
        const cc::Vector3 edot = Xdot_d - Xdot;

        // --- Integral for steady-state bias (optional)
        if (controller_type_ == OP_PID)
        {
          int_x_ += e * dt; // integrate the "desired - actual"
          for (int i = 0; i < 3; ++i)
            int_x_(i) = std::max(-int_x_max_, std::min(int_x_(i), int_x_max_));
        }
        else
        {
          int_x_.setZero();
        }

        // --- Task-space desired acceleration (2nd-order)
        // xddot* = xddot_d + Kd_x*edot + Kp_x*e + Ki_x*int
        cc::Vector3 Xddot_star = Xddot_d + (Kd_x_ * edot) + (Kp_x_ * e) + (Ki_x_ * int_x_);

        // --- Approximate Jdot*qdot by finite-differencing Jv
        cc::Vector3 Jdot_qdot = cc::Vector3::Zero();
        if (Jv_prev_valid_)
        {
          const Eigen::Matrix<double, 3, 6> Jdot = (Jv - Jv_prev_) / dt;
          Jdot_qdot = Jdot * qP;
        }
        Jv_prev_ = Jv;
        Jv_prev_valid_ = true;

        // --- Dynamics terms
        const cc::MatrixDof &M = model_.inertiaMatrix(q);
        const cc::MatrixDof &C = model_.centripetalMatrix(q, qP);
        const cc::VectorDof &G = model_.gravityVector(q);

        // Compute Minv (6x6)
        Eigen::Matrix<double, 6, 6> Minv = M.inverse();

        // Λ = (J M^{-1} J^T)^{-1}   (3x3)
        Eigen::Matrix3d A = (Jv * Minv * Jv.transpose());
        // Robust inverse via LDLT
        Eigen::LDLT<Eigen::Matrix3d> ldlt(A);
        Eigen::Matrix3d Lambda = ldlt.solve(Eigen::Matrix3d::Identity());

        // Dynamically consistent pseudo-inverse: J# = M^{-1} J^T Λ  (6x3)
        Eigen::Matrix<double, 6, 3> Jbar = Minv * Jv.transpose() * Lambda;

        // Joint acceleration command:
        // qddot_cmd = J# * (xddot* - Jdot*qdot)
        Vector6d qddot_cmd = Jbar * (Xddot_star - Jdot_qdot);

        // Convert to torque via computed torque form:
        // tau = M*qddot_cmd + C*qdot + G
        tau = M * qddot_cmd + C * qP + G;

        // Saturate torque
        for (int i = 0; i < 6; ++i)
          tau(i) = std::max(-tau_max_, std::min(tau(i), tau_max_));
      }

      // ======================================================
      // Publish desired trajectory and task-space error
      // (keep exactly the same as before)
      // ======================================================
      if (g_traj.enabled)
      {
        cc::Vector3 Xd, Xdot_d;
        cartesianDesired(t_sec, Xd, Xdot_d);

        geometry_msgs::Point p;
        p.x = Xd(0);
        p.y = Xd(1);
        p.z = Xd(2);
        traj_points_.push_back(p);

        if (traj_points_.size() >= 2)
        {
          traj_marker_.points = traj_points_;
          traj_marker_.header.stamp = ros::Time::now();
          traj_pub_.publish(traj_marker_);
        }

        cc::Vector3 X = fkPos(q);
        cc::Vector3 dX = X - Xd;

        std_msgs::Float64MultiArray msg;
        msg.data = {dX(0), dX(1), dX(2)};
        task_error_pub_.publish(msg);
      }

      // Publish actual end-effector trajectory
      {
        cc::Vector3 X = fkPos(q);
        geometry_msgs::Point p;
        p.x = X(0);
        p.y = X(1);
        p.z = X(2);
        actual_traj_points_.push_back(p);

        if (actual_traj_points_.size() >= 2)
        {
          actual_traj_marker_.points = actual_traj_points_;
          actual_traj_marker_.header.stamp = ros::Time::now();
          actual_traj_pub_.publish(actual_traj_marker_);
        }
      }

      return tau;
    }

    void OperationalSpaceControl::resetIntegrators()
    {
      int_x_.setZero();
    }

    // =========================================================
    // qdot_r generator (SAFE -> MOVE_TO_START -> DRAW)
    // (UNCHANGED: state machine stays)
    // =========================================================
    Vector6d OperationalSpaceControl::computeQdotR(double t_sec, const cc::JointPosition &q, double dt)
    {
      (void)dt;

      // Debug: phase + error norms (once per second)
      static double last_debug_t = -1.0;
      const bool do_debug = (last_debug_t < 0.0) || ((t_sec - last_debug_t) >= 1.0);
      if (do_debug)
        last_debug_t = t_sec;

      // Trajectory disabled: hold
      if (!g_traj.enabled)
      {
        if (do_debug)
          ROS_INFO_STREAM("[OperationalSpaceControl] phase=DISABLED dq_norm=nan dX_norm=nan");
        return Vector6d::Zero();
      }

      const double dq_norm = (q - q_safe_).norm();

      // 1) SAFE: joint space to q_safe
      if (inSafePhase(t_sec, q))
      {
        g_traj.moving = false;
        g_traj.draw_started = false;
        g_traj.t_draw0 = -1.0;
        if (do_debug)
          ROS_INFO_STREAM("[OperationalSpaceControl] phase=SAFE dq_norm=" << dq_norm << " dX_norm=nan");
        return qdotRJointToTarget(q, q_safe_);
      }

      // Compute start point (Xd at t=0)
      g_traj.computeStartIfNeeded();

      // 2) MOVE_TO_START: task space to X_start
      if (g_traj.use_move_to_start && !g_traj.draw_started)
      {
        if (!g_traj.moving)
        {
          g_traj.moving = true;
          resetIntegrators();
          resetMarkerNewSegment();
        }

        const double elapsed_move = t_sec - g_traj.t0;

        const cc::Vector3 X = fkPos(q);
        const cc::Vector3 dX = X - g_traj.X_start;
        const double dX_norm = dX.norm();

        const bool reached = (dX_norm <= g_traj.move_tol);
        const bool timeout = (elapsed_move >= (g_traj.safe_time + g_traj.move_time + 0.5));

        if (reached || timeout)
        {
          g_traj.moving = false;
          g_traj.draw_started = true;
          g_traj.t_draw0 = t_sec;
          resetIntegrators();
          resetMarkerNewSegment();
          if (do_debug)
            ROS_INFO_STREAM("[OperationalSpaceControl] phase=MOVE->DRAW dq_norm=" << dq_norm << " dX_norm=" << dX_norm);
        }
        else
        {
          if (do_debug)
            ROS_INFO_STREAM("[OperationalSpaceControl] phase=MOVE dq_norm=" << dq_norm << " dX_norm=" << dX_norm);

          // PD toward the start point (no integral)
          cc::Vector3 xdot_r = -(Kp_x_ * dX);

          for (int i = 0; i < 3; ++i)
            xdot_r(i) = std::max(-xdot_r_max_, std::min(xdot_r(i), xdot_r_max_));

          const Eigen::Matrix<double, 3, 6> Jv = jacobianLinear(q);
          Vector6d qdot_r = pinv3x6(Jv) * xdot_r;

          for (int i = 0; i < 6; ++i)
            qdot_r(i) = std::max(-qdot_r_max_, std::min(qdot_r(i), qdot_r_max_));

          return qdot_r;
        }
      }

      // 3) DRAW (legacy qdot_r): kept for state machine consistency / debug
      if (!g_traj.draw_started)
      {
        g_traj.draw_started = true;
        g_traj.t_draw0 = t_sec;
        resetIntegrators();
        resetMarkerNewSegment();
      }

      if (do_debug)
      {
        const cc::Vector3 X = fkPos(q);
        const double dX_norm = g_traj.have_start ? (X - g_traj.X_start).norm() : std::numeric_limits<double>::quiet_NaN();
        ROS_INFO_STREAM("[OperationalSpaceControl] phase=DRAW dq_norm=" << dq_norm << " dX_norm=" << dX_norm);
      }

      // legacy xdot-based tracking (not used to produce tau in DRAW anymore)
      const cc::Vector3 xdot_r = xdotROperational(t_sec, q, dt);
      const Eigen::Matrix<double, 3, 6> Jv = jacobianLinear(q);
      Vector6d qdot_r = pinv3x6(Jv) * xdot_r;

      for (int i = 0; i < 6; ++i)
        qdot_r(i) = std::max(-qdot_r_max_, std::min(qdot_r(i), qdot_r_max_));

      return qdot_r;
    }

    Vector6d OperationalSpaceControl::qdotRJointToTarget(const cc::JointPosition &q, const Vector6d &q_target)
    {
      Vector6d dq = q - q_target;
      Vector6d qdot_r = -(Kp_q_ * dq);

      for (int i = 0; i < 6; ++i)
        qdot_r(i) = std::max(-qdot_r_max_, std::min(qdot_r(i), qdot_r_max_));

      return qdot_r;
    }

    // =========================================================
    // Task-space PD / PID (legacy; still used in MOVE and debug)
    // xdot_r = xdot_d - Kp*(X-Xd) - Ki*∫(X-Xd) dt
    // =========================================================
    cc::Vector3 OperationalSpaceControl::xdotROperational(double t_sec, const cc::JointPosition &q, double dt)
    {
      cc::Vector3 Xd, Xdot_d;
      cartesianDesired(t_sec, Xd, Xdot_d);

      const cc::Vector3 X = fkPos(q);
      const cc::Vector3 dX = X - Xd;

      cc::Vector3 xdot_r = Xdot_d - (Kp_x_ * dX);

      if (controller_type_ == OP_PID)
      {
        int_x_ += dX * dt;
        for (int i = 0; i < 3; ++i)
          int_x_(i) = std::max(-int_x_max_, std::min(int_x_(i), int_x_max_));
        xdot_r -= (Ki_x_ * int_x_);
      }

      for (int i = 0; i < 3; ++i)
        xdot_r(i) = std::max(-xdot_r_max_, std::min(xdot_r(i), xdot_r_max_));

      return xdot_r;
    }

    void OperationalSpaceControl::cartesianDesired(double t_sec, cc::Vector3 &Xd, cc::Vector3 &Xdot_d)
    {
      // Hold during SAFE
      if (!g_traj.enabled || (last_q_valid_ && inSafePhase(t_sec, last_q_)))
      {
        Xd = fkPos(q_safe_);
        Xdot_d.setZero();
        return;
      }

      // Move-to-start phase: hold desired at start point
      if (g_traj.use_move_to_start && g_traj.moving && !g_traj.draw_started)
      {
        g_traj.computeStartIfNeeded();
        Xd = g_traj.X_start;
        Xdot_d.setZero();
        return;
      }

      if (!g_traj.draw_started || g_traj.t_draw0 < 0.0)
      {
        Xd = fkPos(q_safe_);
        Xdot_d.setZero();
        return;
      }

      const double t_local = t_sec - g_traj.t_draw0;

      double x, y, z, xd, yd, zd;
      g_traj.eval(t_local, x, y, z, xd, yd, zd);

      Xd << x, y, z;
      Xdot_d << xd, yd, zd;
    }

    bool OperationalSpaceControl::inSafePhase(double t_sec, const cc::JointPosition &q) const
    {
      if (!g_traj.enabled || !g_traj.start_with_safe || g_traj.t0 < 0.0)
        return false;

      const Vector6d dq = q - q_safe_;
      if (dq.norm() <= g_traj.safe_tol)
        return false;

      return true;
    }

    // =========================================================
    // Model interfaces
    // =========================================================
    cc::Vector3 OperationalSpaceControl::fkPos(const cc::JointPosition &q) const
    {
      const cc::HomogeneousTransformation T = model_.T_ef_0(q);
      cc::Vector3 X;
      X << T.pos()(0), T.pos()(1), T.pos()(2);
      return X;
    }

    Eigen::Matrix<double, 3, 6> OperationalSpaceControl::jacobianLinear(const cc::JointPosition &q) const
    {
      const cc::Jacobian J = model_.J_ef_0(q);
      return J.topRows<3>();
    }

    Eigen::Matrix<double, 6, 3> OperationalSpaceControl::pinv3x6(const Eigen::Matrix<double, 3, 6> &J, double eps)
    {
      Eigen::JacobiSVD<Eigen::Matrix<double, 3, 6>> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
      const auto &S = svd.singularValues();

      Eigen::Matrix<double, 6, 3> S_inv = Eigen::Matrix<double, 6, 3>::Zero();
      for (int i = 0; i < S.size(); ++i)
      {
        if (S(i) > eps)
          S_inv(i, i) = 1.0 / S(i);
      }
      return svd.matrixV() * S_inv * svd.matrixU().transpose();
    }

    // =========================================================
    // RViz helpers
    // =========================================================
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

    void OperationalSpaceControl::setMarkerColorForCurrentRun()
    {
      traj_marker_.color.r = 0.0;
      traj_marker_.color.g = 0.0;
      traj_marker_.color.b = 1.0;
    }

  } // namespace RobotControllers
} // namespace tum_ics_ur_robot_lli
