#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt

from ur10_model.ur10_model_3dof import UR10Model3DOF
from ur10_robot.ur10_robot_3dof import UR10Robot3DOF
from ur10_controllers.ur10_operational_space_controller import (
    UR10OperationalSpaceController,
)


def main():
    rospy.loginfo(
        "Starting UR10 Operational Space Control Node "
        "(Tutorial 8 - Task 3/4 with logging)"
    )

    # ------------------------------------------------------------------
    # Initialize model, robot, controller
    # ------------------------------------------------------------------
    model = UR10Model3DOF()
    robot = UR10Robot3DOF(model)
    controller = UR10OperationalSpaceController(model)

    rate = rospy.Rate(100)
    dt_s = 1.0 / 100.0

    # ------------------------------------------------------------------
    # Data logging (same spirit as Task2)
    # ------------------------------------------------------------------
    t_hist = []

    # Task-space errors
    ex_hist = []
    ey_hist = []
    ez_hist = []
    ex_norm_hist = []

    # Joint error space (for reference / comparison)
    Sq_norm_hist = []

    # Torque norm
    tau_norm_hist = []

    t_start = rospy.Time.now()
    t_prev = t_start

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    while not rospy.is_shutdown():
        t_now = rospy.Time.now()
        dt = t_now - t_prev

        # --------------------------------------------------------------
        # 1. Read current joint states
        # --------------------------------------------------------------
        q = robot.q.reshape(-1, 1)
        qdot = robot.qdot.reshape(-1, 1)
        qddot = robot.qddot.reshape(-1, 1)

        joint_states = np.row_stack(
            (
                q.flatten(),
                qdot.flatten(),
                qddot.flatten(),
            )
        )

        # --------------------------------------------------------------
        # 2. Controller update
        # --------------------------------------------------------------
        tau = controller.update(joint_states, t_now, dt)

        # --------------------------------------------------------------
        # 3. Robot dynamics update
        # --------------------------------------------------------------
        dt_sec = dt.to_sec()
        if dt_sec <= 0.0:
            dt_sec = dt_s

        robot.update_dynamics(tau, dt_sec)

        # --------------------------------------------------------------
        # 4. Logging (Task2-style)
        # --------------------------------------------------------------
        # Time
        t_hist.append((t_now - t_start).to_sec())

        # --- Task-space error ---
        if controller.state in [
            controller.State.C_LINEAR,
            controller.State.D_CROWN_W1,
            controller.State.E_CROWN_W2,
            controller.State.TASK4_CONTROL,
        ]:
            X = controller.fk_pos(q)
            Xd, _ = controller.cartesian_desired(t_now)
            dX = X - Xd

            ex_hist.append(dX[0, 0])
            ey_hist.append(dX[1, 0])
            ez_hist.append(dX[2, 0])
            ex_norm_hist.append(np.linalg.norm(dX))
        else:
            # Not meaningful outside tracking phases → break the curve
            ex_hist.append(np.nan)
            ey_hist.append(np.nan)
            ez_hist.append(np.nan)
            ex_norm_hist.append(np.nan)

        # --- Joint error space norm ---
        if hasattr(controller, "qdot_r_prev"):
            Sq = qdot - controller.qdot_r_prev
            Sq_norm_hist.append(np.linalg.norm(Sq))
        else:
            Sq_norm_hist.append(0.0)

        # --- Torque norm ---
        tau_norm_hist.append(np.linalg.norm(tau))

        t_prev = t_now

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break

    # ------------------------------------------------------------------
    # Plot results after shutdown (same as Task2)
    # ------------------------------------------------------------------
    rospy.loginfo("Simulation finished. Plotting results...")

    plt.figure()
    plt.plot(t_hist, ex_hist, label="e_x")
    plt.plot(t_hist, ey_hist, label="e_y")
    plt.plot(t_hist, ez_hist, label="e_z")
    plt.xlabel("Time [s]")
    plt.ylabel("Task-space error [m]")
    plt.title("Operational Space Tracking Error Components")
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(t_hist, ex_norm_hist)
    plt.xlabel("Time [s]")
    plt.ylabel("||e_x|| [m]")
    plt.title("Task-space Error Norm")
    plt.grid()

    plt.figure()
    plt.plot(t_hist, Sq_norm_hist)
    plt.xlabel("Time [s]")
    plt.ylabel("||S_q||")
    plt.title("Joint Error Space Norm (Operational Control)")
    plt.grid()

    plt.figure()
    plt.plot(t_hist, tau_norm_hist)
    plt.xlabel("Time [s]")
    plt.ylabel("||tau||")
    plt.title("Torque Norm")
    plt.grid()
    plt.show()


if __name__ == "__main__":
    rospy.init_node("ur10_operational_space_control_with_plot")
    main()
