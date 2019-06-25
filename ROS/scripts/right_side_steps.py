#!/usr/bin/env python

#
# Python script that has Valkyrie side step to the right a given number of steps
#

import time
import rospy
import tf
import tf2_ros
import numpy
import sys

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

TRANSFER_TIME = 1.5
SWING_TIME = 1.5
STEP_SIZE = 0.15
DISTANCE_BETWEEN_FEET = 0.1


# Walks the desired amount of steps in the given direction.
def sideSteps(steps, direction):

    msg = FootstepDataListRosMessage(transfer_time = TRANSFER_TIME,
                swing_time = SWING_TIME, unique_id = rospy.get_time())

    if direction == LEFT:
        step_size = STEP_SIZE
        first_foot = LEFT
        second_foot = RIGHT
    else:
        step_size = -STEP_SIZE
        first_foot = RIGHT
        second_foot = LEFT

    # Generate the footsteps
    x = step_size
    counter = 1
    while counter <= steps/2:
        msg.footstep_data_list.append(createOffsetFromCurrent(first_foot, [0, x, 0.0], 0))
        msg.footstep_data_list.append(createOffsetFromCurrent(second_foot, [0, x+DISTANCE_BETWEEN_FEET, 0.0], 0))
        x += step_size
        counter += 1

    footstep_list_pub.publish(msg)


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




if __name__ == '__main__':

    try:

        footstep_list_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=1)

        rospy.init_node('right_side_steps')

        # Set up TF so we can place footsteps relative to the world frame
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        rate = rospy.Rate(10)
        time.sleep(1)

        # Determine amount of steps to take
        steps = 10

        # Walk
        if not rospy.is_shutdown():
            sideSteps(steps, RIGHT)
            time.sleep(1)

    except rospy.ROSInterruptException:
        pass
