#!/usr/bin/env python3
"""
Wait for the WSG-50 driver, then grasp at the configured width.
The driver maintains grasping force automatically after the call returns.
"""
import rospy
from wsg_50_common.srv import Move


def main():
    rospy.init_node("gripper_grasp_init")

    grasp_width = rospy.get_param("~grasp_width", 15.0)  # mm
    grasp_speed = rospy.get_param("~grasp_speed", 30.0)   # mm/s

    service_name = "/wsg_50_driver/grasp"

    rospy.loginfo("[gripper_grasp_init] Waiting for service %s ...", service_name)
    rospy.wait_for_service(service_name, timeout=30.0)

    grasp = rospy.ServiceProxy(service_name, Move)

    rospy.loginfo("[gripper_grasp_init] Grasping: width=%.1f mm, speed=%.1f mm/s", grasp_width, grasp_speed)
    resp = grasp(grasp_width, grasp_speed)

    if resp.error == 0:
        rospy.loginfo("[gripper_grasp_init] Grasp OK. Gripper will hold at %.1f mm.", grasp_width)
    else:
        rospy.logwarn("[gripper_grasp_init] Grasp returned error code: %d", resp.error)

    # Node exits; driver keeps holding the object.


if __name__ == "__main__":
    main()
