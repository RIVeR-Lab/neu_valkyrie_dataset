#!/usr/bin/env python

#
# Python script that has Valkyrie move her pelvis around
#

import time
import rospy
import tf
import tf2_ros
import numpy
import sys

from geometry_msgs.msg import Quaternion
from ihmc_msgs.msg import *

LEFT = 0
RIGHT = 1

TRANSFER_TIME = 1.5
SWING_TIME = 1.5
STEP_SIZE = 0.25
DISTANCE_BETWEEN_FEET = 0.277

# Takes a small sideways step to set feet the required distance apart for walking.
def sidewayStep():

    msg = FootstepDataListRosMessage(transfer_time = TRANSFER_TIME,
                swing_time = SWING_TIME, unique_id = rospy.get_time())

    left_foot_location = createOffsetFootstep(LEFT, [0.0, DISTANCE_BETWEEN_FEET, 0.0], 0)

    # Side step
    msg.footstep_data_list.append(left_foot_location)

    footstep_list_pub.publish(msg)
    waitForFootsteps(len(msg.footstep_data_list))


# Moves the pelvis around.
def movePelvis():

    chest_home_msg = GoHomeRosMessage(body_part = 1, trajectory_time = 2, robot_side = RIGHT, unique_id = rospy.get_time())

    # Yaw
    msg = ChestTrajectoryRosMessage(taskspace_trajectory_points = [createChestTrajectory(2, 0, 0, 0.5)],
        execution_mode = 0, unique_id = rospy.get_time())
    chest_pub.publish(msg)
    time.sleep(3)
    go_home_pub.publish(chest_home_msg)
    time.sleep(3)

    msg = ChestTrajectoryRosMessage(taskspace_trajectory_points = [createChestTrajectory(2, 0, 0, -0.5)],
        execution_mode = 0, unique_id = rospy.get_time())
    chest_pub.publish(msg)
    time.sleep(3)
    go_home_pub.publish(chest_home_msg)
    time.sleep(3)

    # Pitch
    msg = ChestTrajectoryRosMessage(taskspace_trajectory_points = [createChestTrajectory(2, 0, 0.5, 0)],
        execution_mode = 0, unique_id = rospy.get_time())
    chest_pub.publish(msg)
    time.sleep(3)
    go_home_pub.publish(chest_home_msg)
    time.sleep(3)

    # Roll
    msg = ChestTrajectoryRosMessage(taskspace_trajectory_points = [createChestTrajectory(2, 0.2, 0.0, 0)],
        execution_mode = 0, unique_id = rospy.get_time())
    chest_pub.publish(msg)
    time.sleep(3)
    go_home_pub.publish(chest_home_msg)
    time.sleep(3)

    msg = ChestTrajectoryRosMessage(taskspace_trajectory_points = [createChestTrajectory(2, -0.2, 0.0, 0)],
        execution_mode = 0, unique_id = rospy.get_time())
    chest_pub.publish(msg)
    time.sleep(3)
    go_home_pub.publish(chest_home_msg)
    time.sleep(3)


# Helps create the trajectories for the chest.
def createChestTrajectory(time, roll, pitch, yaw):

    chest_world = tf_buffer.lookup_transform('world', 'pelvis', rospy.Time())
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [chest_world.transform.rotation.x, chest_world.transform.rotation.y,
        chest_world.transform.rotation.z, chest_world.transform.rotation.w])
    r += roll
    p += pitch
    y += yaw
    tfq = tf.transformations.quaternion_from_euler(r, p, y)
    desired_chest = Quaternion(tfq[0], tfq[1], tfq[2], tfq[3])

    return SO3TrajectoryPointRosMessage(orientation = desired_chest, time = 2)



# Creates footstep for the desired foot that is the given offset and
# angle in the z-axis from the stationary foot. The offset is in foot frame.
def createOffsetFootstep(step_side, offset, angle):

    footstep = createStationaryFootstep(step_side)
    if step_side == LEFT:
        stationary_foot = createStationaryFootstep(RIGHT)
    else:
        stationary_foot = createStationaryFootstep(LEFT)

    # Add in the angle
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [stationary_foot.orientation.x, stationary_foot.orientation.y,
        stationary_foot.orientation.z, stationary_foot.orientation.w])
    y += angle
    rotated_quat = tf.transformations.quaternion_from_euler(r, p, y)
    footstep.orientation.x = rotated_quat[0]
    footstep.orientation.y = rotated_quat[1]
    footstep.orientation.z = rotated_quat[2]
    footstep.orientation.w = rotated_quat[3]

    # Transform the offset to world frame
    quat = stationary_foot.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformed_offset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x = stationary_foot.location.x + transformed_offset[0]
    footstep.location.y = stationary_foot.location.y + transformed_offset[1]
    footstep.location.z = stationary_foot.location.z + transformed_offset[2]

    return footstep


 # Creates footstep with the current position and orientation of the given
 # foot.
def createStationaryFootstep(step_side):

    footstep = FootstepDataRosMessage(robot_side = step_side)

    if step_side == LEFT:
        foot_frame = 'leftFoot'
    else:
        foot_frame = 'rightFoot'

    foot_world = tf_buffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = foot_world.transform.rotation
    footstep.location = foot_world.transform.translation

    return footstep



# Helps keep track of the status of movement by waiting for current set
# of footsteps to complete.
# TODO: get rid of that global...
def waitForFootsteps(num_of_steps):
    global step_counter
    step_counter = 0
    while step_counter < num_of_steps:
        rate.sleep()


# Keeps track of number of steps.
def footstepStatusCallback(msg):
    global step_counter
    if msg.status == 1:
        step_counter += 1




if __name__ == '__main__':

    try:
        footstep_status_sub = rospy.Subscriber('/ihmc_ros/valkyrie/output/footstep_status', FootstepStatusRosMessage, footstepStatusCallback)
        footstep_list_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size = 1)
        whole_body_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/whole_body_trajectory', WholeBodyTrajectoryRosMessage, queue_size = 5)
        go_home_pub = rospy.Publisher("/ihmc_ros/valkyrie/control/go_home", GoHomeRosMessage, queue_size = 5)
        chest_pub = rospy.Publisher("/ihmc_ros/valkyrie/control/chest_trajectory", ChestTrajectoryRosMessage, queue_size = 1)

        rospy.init_node('pelvis_movement')

        # Set up TF so we know where the robot is relative to the world frame
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rate = rospy.Rate(10)
        time.sleep(1)

        if not rospy.is_shutdown():
            # Side step (helps with balance)
            sidewayStep()
            time.sleep(5)
            # Move Pelvis
            movePelvis()
            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
