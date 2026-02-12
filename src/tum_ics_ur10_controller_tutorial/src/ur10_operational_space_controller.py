import rospy
import numpy as np
from enum import Enum

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray


class UR10OperationalSpaceController:
    class State(Enum):
        # ---------- Task 3 ----------
        A_PARK = 0
        B_NON_SINGULAR = 1
        C_LINEAR = 2
        D_CROWN_W1 = 3
        E_CROWN_W2 = 4
        DONE = 5

        # ---------- Task 4 ----------
        TASK4_WAIT = 6
        TASK4_CONTROL = 7
        TASK4_RETURN = 8

    class Controller(Enum):
        OP_PD = 1
        OP_PID = 2

    def __init__(self, model):
        self.model = model
        self.nq = int(model.nq)

        # -------------------------------------------------
        # Controller (PD-like vs PID-like)
        # -------------------------------------------------
        # self.controller = self.Controller.OP_PD
        self.controller = self.Controller.OP_PID

        # -------------------------------------------------
        # Task3 experiment loop: run_id = 0 (PD), 1 (PID)
        # -------------------------------------------------
        self.run_id = 0
        self.max_runs = 2

        # -------------------------------------------------
        # Gains (document-consistent tuning)
        # -------------------------------------------------
        self.Kd = np.diag([4.0, 4.0, 3.0])  # damping on Sq in (6.1)
        self.Kp_x = np.diag([7.0, 7.0, 6.0])  # [1/s]
        self.Ki_x = np.diag([4.0, 4.0, 4.0])  # [1/s^2], Kp^2 - 4Ki <= 0 per axis

        # Integral anti-windup for ∫ΔX
        self.int_x = np.zeros((3, 1))
        self.int_x_max = 0.05  # meters

        # -------------------------------------------------
        # State timing for Task3
        # -------------------------------------------------
        self.state = self.State.A_PARK
        self.state_start = rospy.Time.now()

        self.T_a = 2.0
        self.T_b = 2.0
        self.T_c = 4.0
        self.T_d = 15.0

        # -------------------------------------------------
        # Joint targets for states a,b
        # -------------------------------------------------
        self.q_park = np.array([[0.0], [0.0], [0.0]])
        self.q_safe = np.array([[0.0], [0.0], [np.pi / 2.0]])
        self.q_home = self.q_park.copy()

        # Joint-space reference profile (simple stable reference dynamics)
        self.Kp_q = np.diag([2.0, 2.0, 1.5])  # [1/s]

        # -------------------------------------------------
        # Cartesian trajectory parameters (Task3 c,d,e)
        # -------------------------------------------------
        self.linear_offset = np.array([[-0.4], [0.2], [-0.4]])

        self.radius = 0.20
        self.z_amp = 0.02
        self.w1 = 0.40
        self.w2 = 0.80

        self.x_b = None
        self.x_c_start = None
        self.circle_center = None
        self.theta0 = 0.0

        # -------------------------------------------------
        # qdot_r differentiation (for qddot_r)
        # -------------------------------------------------
        self.qdot_r_prev = np.zeros((self.nq, 1))
        self.qdot_r_prev_valid = False
        self.just_switched_state = True

        # -------------------------------------------------
        # Saturations
        # -------------------------------------------------
        self.tau_max = 120.0
        self.xdot_r_max = 0.25
        self.qdot_r_max = 2.0

        # Task4 requires more aggressive motion to reach singularity
        self.xdot_r_max_task4 = 1.0  # [m/s], relaxed EE velocity limit for Task4
        self.qdot_r_max_task4 = 4.0  # [rad/s], relaxed joint velocity limit for Task4

        # -------------------------------------------------
        # Task4: singularity experiment settings
        # -------------------------------------------------
        self.circle_center_task4 = None
        self.radius_singularity = 0.55
        self.cond_thresh = 30
        self.singularity_detected = False

        # Forced execution times for Task4 (per your required flow)
        self.task4_wait_time = 4.0
        self.task4_control_time = 6.0

        # Internal routing for Task4 return transitions
        self.task4_return_target = None  # "to_pid" or "to_done"

        # Task4 completion flag
        self.task4_completed = False

        # -------------------------------------------------
        # RViz trajectory visualization (Marker)
        # -------------------------------------------------
        self.traj_pub = rospy.Publisher(
            "/ur10/task3/desired_trajectory",
            Marker,
            queue_size=1,
        )

        # Clear old markers (best-effort; RViz subscription timing can vary)
        self.publish_deleteall_markers()

        self.traj_marker = Marker()
        self.traj_marker.header.frame_id = "world"
        self.traj_marker.ns = "task3_traj"
        self.traj_marker.id = 0
        self.traj_marker.type = Marker.LINE_STRIP
        self.traj_marker.action = Marker.ADD
        self.traj_marker.scale.x = 0.01
        self.traj_marker.color.a = 1.0
        self.traj_marker.pose.orientation.w = 1.0

        self.traj_points = []
        self.set_marker_color_for_current_run()

        rospy.on_shutdown(self.clear_all_markers)

        self.pub_task_space_error = rospy.Publisher(
            "/ur10/task_space_error",
            Float64MultiArray,
            queue_size=1,
        )

    # =========================================================
    # RViz helpers
    # =========================================================
    def publish_deleteall_markers(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = "world"
        for _ in range(3):
            clear.header.stamp = rospy.Time.now()
            self.traj_pub.publish(clear)
            rospy.sleep(0.05)

    def clear_all_markers(self):
        clear = Marker()
        clear.action = Marker.DELETEALL
        for _ in range(5):
            clear.header.stamp = rospy.Time.now()
            self.traj_pub.publish(clear)
            rospy.sleep(0.1)

    def publish_delete_marker(self):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = rospy.Time.now()
        m.ns = self.traj_marker.ns
        m.id = self.traj_marker.id
        m.action = Marker.DELETE
        self.traj_pub.publish(m)

    def reset_marker_new_segment(self):
        # RViz distinguishes markers by (ns, id). Bump id to start a new polyline object.
        self.traj_marker.id += 1
        self.traj_points = []
        self.traj_marker.points = self.traj_points
        self.traj_marker.header.stamp = rospy.Time.now()

    def set_marker_color_for_current_run(self):
        # Task3 run: blue
        # Task4 run: green
        if self.state in [self.State.TASK4_CONTROL]:
            self.traj_marker.color.r = 0.0
            self.traj_marker.color.g = 1.0
            self.traj_marker.color.b = 0.0
            return

        # Task3 (based on run_id/controller)
        if self.controller == self.Controller.OP_PD:
            self.traj_marker.color.r = 0.0
            self.traj_marker.color.g = 0.0
            self.traj_marker.color.b = 1.0
        else:
            self.traj_marker.color.r = 0.0
            self.traj_marker.color.g = 0.0
            self.traj_marker.color.b = 1.0

    # =========================================================
    # Debug Jacobian checker (optional, runs once)
    # =========================================================
    def debug_jacobian_full(self, q):
        eps = 1e-6
        n = self.nq

        J6 = np.array(self.model.J_ee_w_fn(*q.flatten()), dtype=float)
        J_top = J6[0:3, :]
        J_bot = J6[3:6, :]

        J_num = np.zeros((3, n))
        x0 = self.fk_pos(q)
        for i in range(n):
            q_p = q.copy()
            q_m = q.copy()
            q_p[i, 0] += eps
            q_m[i, 0] -= eps
            x_p = self.fk_pos(q_p)
            x_m = self.fk_pos(q_m)
            J_num[:, i] = ((x_p - x_m) / (2.0 * eps)).flatten()

        for blk_name, Jblk in [("top(0:3)", J_top), ("bot(3:6)", J_bot)]:
            for i in range(n):
                errs = [np.linalg.norm(J_num[:, i] - Jblk[:, j]) for j in range(n)]
                j_best = int(np.argmin(errs))

        if hasattr(self.model, "Jp_ee_w_fk_fn"):
            J_fk = np.array(self.model.Jp_ee_w_fk_fn(*q.flatten()), dtype=float)

    # =========================================================
    # Singularity check
    # =========================================================
    def check_singularity(self, q):
        J = self.jacobian_linear(q)
        cond_J = np.linalg.cond(J)

        if cond_J > self.cond_thresh:
            self.singularity_detected = True

        return cond_J

    # =========================================================
    # Main update
    # =========================================================
    def update(self, joint_states, t_now, dt_ros):
        q = joint_states[0, :].reshape(self.nq, 1)
        qdot = joint_states[1, :].reshape(self.nq, 1)

        if not hasattr(self, "jac_checked"):
            # Optional one-shot debug
            # self.debug_jacobian_full(q)
            self._jac_checked = True

        dt = float(dt_ros.to_sec())
        if dt <= 0.0 or not np.isfinite(dt):
            dt = 1e-3

        # State machine
        self.state_machine(t_now, q)

        # Compute qdot_r
        qdot_r = self.compute_qdot_r(t_now, q, dt)

        # Avoid qddot_r spike on first step and right after state switch
        if (not self.qdot_r_prev_valid) or self.just_switched_state:
            self.qdot_r_prev = qdot_r.copy()
            self.qdot_r_prev_valid = True
            self.just_switched_state = False

        qddot_r = (qdot_r - self.qdot_r_prev) / dt
        self.qdot_r_prev = qdot_r.copy()

        Sq = qdot - qdot_r

        M, C, G = self.dynamics_matrices(q, qdot)
        Yr_theta = (M @ qddot_r) + (C @ qdot_r) + G

        tau = (-self.Kd @ Sq) + Yr_theta
        tau = np.clip(tau, -self.tau_max, self.tau_max)

        # Publish RViz trajectory markers for operational-space states
        if self.state in [
            self.State.C_LINEAR,
            self.State.D_CROWN_W1,
            self.State.E_CROWN_W2,
            self.State.TASK4_CONTROL,
        ]:
            Xd, _ = self.cartesian_desired(t_now)

            p = Point()
            p.x = float(Xd[0, 0])
            p.y = float(Xd[1, 0])
            p.z = float(Xd[2, 0])

            self.traj_points.append(p)

            # LINE_STRIP requires at least 2 points
            if len(self.traj_points) >= 2:
                self.traj_marker.points = self.traj_points
                self.traj_marker.header.frame_id = "world"
                self.traj_marker.header.stamp = rospy.Time.now()
                self.traj_pub.publish(self.traj_marker)

        # Logging
        tag = (
            "[Task4]"
            if self.state in [
                self.State.TASK4_WAIT,
                self.State.TASK4_CONTROL,
                self.State.TASK4_RETURN,
            ]
            else "[Task3]"
        )

        if self.state in [
            self.State.C_LINEAR,
            self.State.D_CROWN_W1,
            self.State.E_CROWN_W2,
            self.State.TASK4_CONTROL,
        ]:
            x = self.fk_pos(q)
            xd, _ = self.cartesian_desired(t_now)
            dx = x - xd

            # -------------------------------------------------
            # Task 5: publish Cartesian steady-state / tracking error
            # e_x = x - x_d
            # -------------------------------------------------
            msg = Float64MultiArray()
            msg.data = dx.flatten().tolist()  # [ex, ey, ez]
            self.pub_task_space_error.publish(msg)
        else:
            q_target = self.current_joint_target()
            dq = q - q_target

        return tau

    # =========================================================
    # State machine logic (implements your required flow)
    # =========================================================
    def state_machine(self, t_now, q):
        elapsed = (t_now - self.state_start).to_sec()

        # -------------------------------------------------
        # Task3: A -> B -> C -> D -> E -> DONE
        # -------------------------------------------------
        if self.state == self.State.A_PARK and elapsed >= self.T_a:
            self.state = self.State.B_NON_SINGULAR
            self.state_start = t_now
            self.reset_integrators()
            self.just_switched_state = True
            return

        if self.state == self.State.B_NON_SINGULAR and elapsed >= self.T_b:
            # Cache pose using actual q
            self.x_b = self.fk_pos(q)
            self.x_c_start = self.x_b + self.linear_offset

            # Circle center so theta=0 starts at x_c_start
            self.circle_center = self.x_c_start - np.array([[-self.radius], [0.0], [0.0]])
            v0 = self.x_c_start - self.circle_center
            self.theta0 = np.arctan2(float(v0[1, 0]), float(v0[0, 0]))

            self.state = self.State.C_LINEAR
            self.state_start = t_now
            self.reset_integrators()
            self.just_switched_state = True

            # New marker segment
            self.reset_marker_new_segment()
            self.set_marker_color_for_current_run()
            return

        if self.state == self.State.C_LINEAR and elapsed >= self.T_c:
            self.state = self.State.D_CROWN_W1
            self.state_start = t_now
            self.reset_integrators()
            self.just_switched_state = True
            return

        if self.state == self.State.D_CROWN_W1 and elapsed >= self.T_d:
            self.state = self.State.E_CROWN_W2
            self.state_start = t_now
            self.reset_integrators()
            self.just_switched_state = True
            return

        if self.state == self.State.E_CROWN_W2 and elapsed >= self.T_d:
            # Task3 run completed
            self.state = self.State.DONE
            self.state_start = t_now
            self.reset_integrators()
            self.just_switched_state = True
            return

        # -------------------------------------------------
        # Task3 DONE: run switch PD -> PID -> Task4_WAIT
        # -------------------------------------------------
        if self.state == self.State.DONE:
            # If Task4 is already completed, stay in DONE forever
            if self.task4_completed:
                return

            rospy.loginfo("[Task3] Completed. Entering [Task4].")

            self.state = self.State.TASK4_WAIT
            self.state_start = t_now
            self.reset_integrators()
            self.just_switched_state = True
            self.singularity_detected = False
            self.task4_return_target = None
            return

        # -------------------------------------------------
        # Task4: WAIT -> PD/PID-like -> RETURN -> DONE
        # -------------------------------------------------
        if self.state == self.State.TASK4_WAIT:
            if elapsed >= self.task4_wait_time:
                self.state = self.State.TASK4_CONTROL
                self.state_start = t_now

                self.singularity_detected = False
                self.task4_return_target = "to_done"

                self.reset_integrators()
                self.just_switched_state = True

                self.reset_marker_new_segment()
                self.set_marker_color_for_current_run()

                # ============================
                # Task4: compute LARGE crown center
                # (must be done ONCE, when Task4 starts)
                # ============================
                k = 10.0

                c4 = self.x_c_start.copy()
                c4[0, 0] -= self.radius_singularity * np.cos(self.theta0)
                c4[1, 0] -= self.radius_singularity * np.sin(self.theta0)
                c4[2, 0] -= self.z_amp * np.sin(k * self.theta0)

                self.circle_center_task4 = c4
                return

        if self.state == self.State.TASK4_CONTROL:
            # Forced motion first
            if elapsed < self.task4_control_time:
                return

            # After forced time, check singularity; if still not detected, force transition
            self.check_singularity(q)
            if self.singularity_detected is False:
                self.singularity_detected = True

            self.state = self.State.TASK4_RETURN
            self.state_start = t_now
            self.reset_integrators()
            self.just_switched_state = True
            return

        if self.state == self.State.TASK4_RETURN:
            X = self.fk_pos(q)
            dX = X - self.x_c_start
            if np.linalg.norm(dX) < 0.03:  # meters
                if self.task4_return_target == "to_done":
                    self.task4_completed = True
                    self.state = self.State.DONE
                    self.state_start = t_now
                    return
            return

    # =========================================================
    # Integrator handling
    # =========================================================
    def reset_integrators(self):
        self.int_x[:] = 0.0

    # =========================================================
    # Reference generation
    # =========================================================
    def compute_qdot_r(self, t_now, q, dt):
        # Joint return control for Task4_RETURN (and optionally DONE to hold home)
        if self.state == self.State.TASK4_RETURN:
            return self.qdot_r_task4_return(q)

        # Task3 a,b: joint motion
        if self.state in [self.State.A_PARK, self.State.B_NON_SINGULAR]:
            qd = self.q_park if self.state == self.State.A_PARK else self.q_safe
            return self.qdot_r_joint_to_target(q, qd)

        # Operational-space states (Task3 c,d,e and Task4 PD/PID)
        xdot_r = self.xdot_r_operational(t_now, q, dt)
        J = self.jacobian_linear(q)
        qdot_r = np.linalg.pinv(J) @ xdot_r

        # Relax joint velocity limits during Task4
        if self.state in [self.State.TASK4_CONTROL]:
            qdot_r = np.clip(qdot_r, -self.qdot_r_max_task4, self.qdot_r_max_task4)
        else:
            qdot_r = np.clip(qdot_r, -self.qdot_r_max, self.qdot_r_max)

        return qdot_r

    def qdot_r_joint_to_target(self, q, q_target):
        dq = q - q_target
        qdot_r = -(self.Kp_q @ dq)
        qdot_r = np.clip(qdot_r, -self.qdot_r_max, self.qdot_r_max)
        return qdot_r

    def xdot_r_operational(self, t_now, q, dt):
        Xd, Xdot_d = self.cartesian_desired(t_now)
        X = self.fk_pos(q)

        # Document convention: ΔX = X - Xd
        dX = X - Xd

        # (4.5) Xdot_r = Xdot_d - Kp * ΔX
        if self.state in [self.State.TASK4_CONTROL]:
            # Same structure as Task3, but weaker feedback
            Xdot_r = Xdot_d - (0.5 * self.Kp_x @ dX)
        else:
            Xdot_r = Xdot_d - (self.Kp_x @ dX)

        # (4.8) PID-like: - Ki ∫ΔX (only meaningful when feedback is active)
        if self.controller == self.Controller.OP_PID:
            self.int_x += dX * dt
            self.int_x = np.clip(self.int_x, -0.02, 0.02)  # tighter bound
            Xdot_r -= (0.3 * self.Ki_x @ self.int_x)

        # Velocity saturation
        if self.state in [self.State.TASK4_CONTROL]:
            Xdot_r = np.clip(Xdot_r, -self.xdot_r_max_task4, self.xdot_r_max_task4)
        else:
            Xdot_r = np.clip(Xdot_r, -self.xdot_r_max, self.xdot_r_max)

        return Xdot_r

    def cartesian_desired(self, t_now):
        tau = (t_now - self.state_start).to_sec()

        # Safety fallback if not initialized
        if self.x_b is None or self.x_c_start is None or self.circle_center is None:
            x_hold = self.fk_pos(self.q_safe)
            return x_hold, np.zeros((3, 1))

        # Task3 c) linear segment
        if self.state == self.State.C_LINEAR:
            T = self.T_c
            s = np.clip(tau / T, 0.0, 1.0)
            Xd = (1.0 - s) * self.x_b + s * self.x_c_start
            Xdot_d = (self.x_c_start - self.x_b) / T if tau < T else np.zeros((3, 1))
            return Xd, Xdot_d

        # Task3 d,e) crown
        if self.state in [self.State.D_CROWN_W1, self.State.E_CROWN_W2]:
            w = self.w1 if self.state == self.State.D_CROWN_W1 else self.w2
            theta = self.theta0 + w * tau
            c = self.circle_center
            k = 10.0

            Xd = np.array(
                [
                    [c[0, 0] + self.radius * np.cos(theta)],
                    [c[1, 0] + self.radius * np.sin(theta)],
                    [c[2, 0] + self.z_amp * np.sin(k * theta)],
                ]
            )

            Xdot_d = np.array(
                [
                    [-self.radius * w * np.sin(theta)],
                    [self.radius * w * np.cos(theta)],
                    [self.z_amp * k * w * np.cos(k * theta)],
                ]
            )
            return Xd, Xdot_d

        # Task4 PD/PID: circle toward singularity
        if self.state in [self.State.TASK4_CONTROL]:
            # EXACT same geometry as Task3 crown, but larger radius
            w = self.w1  # same angular speed
            theta = self.theta0 + w * tau
            c = self.circle_center_task4
            k = 10.0

            Xd = np.array(
                [
                    [c[0, 0] + self.radius_singularity * np.cos(theta)],
                    [c[1, 0] + self.radius_singularity * np.sin(theta)],
                    [c[2, 0] + self.z_amp * np.sin(k * theta)],
                ]
            )

            Xdot_d = np.array(
                [
                    [-self.radius_singularity * w * np.sin(theta)],
                    [self.radius_singularity * w * np.cos(theta)],
                    [self.z_amp * k * w * np.cos(k * theta)],
                ]
            )
            return Xd, Xdot_d

        # Default: hold last known start
        return self.x_c_start.copy(), np.zeros((3, 1))

    def qdot_r_task4_return(self, q):
        X = self.fk_pos(q)
        dX = X - self.x_c_start

        xdot_r = -self.Kp_x @ dX
        J = self.jacobian_linear(q)
        qdot_r = np.linalg.pinv(J) @ xdot_r

        return np.clip(qdot_r, -self.qdot_r_max, self.qdot_r_max)

    # =========================================================
    # Model interfaces
    # =========================================================
    def fk_pos(self, q):
        T = np.array(self.model.T_ee_w_fn(*q.flatten()), dtype=float)
        return T[0:3, 3].reshape(3, 1)

    def jacobian_linear(self, q):
        if not hasattr(self.model, "Jp_ee_w_fk_fn"):
            raise RuntimeError(
                "Model does not provide Jp_ee_w_fk_fn. Check ur10_model_3dof.py jacobians()."
            )

        return np.array(self.model.Jp_ee_w_fk_fn(*q.flatten()), dtype=float)

    def dynamics_matrices(self, q, qdot):
        n = self.nq
        q_np = q.flatten().tolist()
        qdot_np = qdot.flatten().tolist()

        M = np.zeros((n, n), dtype=float)
        for i in range(n):
            for j in range(n):
                M[i, j] = float(self.model.M_fn[i][j](*q_np))

        C = np.zeros((n, n), dtype=float)
        for i in range(n):
            for j in range(n):
                C[i, j] = float(self.model.C_fn[i][j](q_np, qdot_np))

        G = np.zeros((n, 1), dtype=float)
        for i in range(n):
            G[i, 0] = float(self.model.G_fn[i](*q_np))

        return M, C, G

    # =========================================================
    # Utility
    # =========================================================
    def current_joint_target(self):
        if self.state == self.State.A_PARK:
            return self.q_park
        if self.state == self.State.B_NON_SINGULAR:
            return self.q_safe
        if self.state == self.State.TASK4_RETURN:
            return self.q_home

        # Default: for non-joint states, return something stable
        return self.q_safe
