#!/usr/bin/env python

#
# Python script that has Valkyrie turn 180 degrees counterclockwise
#

import time
import rospy
import tf
import tf2_ros
import numpy
import sys
import math

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

TRANSFER_TIME = 1.5
SWING_TIME = 1.5
STEP_SIZE = 0.25
DISTANCE_BETWEEN_FEET = 0.277
STEPS_FOR_TURN = 12


# Turns in the desired direction
def turn(direction):

    if direction == LEFT:
        first_foot = LEFT
        second_foot = RIGHT
        y_axis_fix = -1
        angle_direction_fix = 1
    else:
        first_foot = RIGHT
        second_foot = LEFT
        y_axis_fix = 1
        angle_direction_fix = -1

    # Generate the footsteps
    turn_angle = math.pi/STEPS_FOR_TURN
    for x in range(0, STEPS_FOR_TURN):
        msg = FootstepDataListRosMessage(transfer_time = TRANSFER_TIME,
                swing_time = SWING_TIME, unique_id = rospy.get_time())
        msg.footstep_data_list.append(createOffsetFromCurrent(first_foot, [0, 0, 0], turn_angle*angle_direction_fix))
        footstep_list_pub.publish(msg)
        waitForFootsteps(len(msg.footstep_data_list))
        msg = FootstepDataListRosMessage(transfer_time = TRANSFER_TIME,
                swing_time = SWING_TIME, unique_id = rospy.get_time())
        msg.footstep_data_list.append(createOffsetFootstep(second_foot, [0, y_axis_fix*DISTANCE_BETWEEN_FEET, 0], 0))
        footstep_list_pub.publish(msg)
        waitForFootsteps(len(msg.footstep_data_list))



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

# Creates footstep for the desired foot that is the given offset and
# angle in the z-axis from the current foot. The offset is in foot frame.
def createOffsetFromCurrent(step_side, offset, angle):

    footstep = createStationaryFootstep(step_side)
    if step_side == LEFT:
        current_foot = createStationaryFootstep(LEFT)
    else:
        current_foot = createStationaryFootstep(RIGHT)

    # Add in the angle
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [current_foot.orientation.x, current_foot.orientation.y,
        current_foot.orientation.z, current_foot.orientation.w])
    y += angle
    rotated_quat = tf.transformations.quaternion_from_euler(r, p, y)
    footstep.orientation.x = rotated_quat[0]
    footstep.orientation.y = rotated_quat[1]
    footstep.orientation.z = rotated_quat[2]
    footstep.orientation.w = rotated_quat[3]

    # Transform the offset to world frame
    quat = current_foot.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformed_offset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x = current_foot.location.x + transformed_offset[0]
    footstep.location.y = current_foot.location.y + transformed_offset[1]
    footstep.location.z = current_foot.location.z + transformed_offset[2]

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
        footstep_list_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=1)

        rospy.init_node('left_turn_180')

        # Set up TF so we can place footsteps relative to the world frame
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rate = rospy.Rate(10)
        time.sleep(1)

        # Turn
        if not rospy.is_shutdown():
            turn(LEFT)
            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
