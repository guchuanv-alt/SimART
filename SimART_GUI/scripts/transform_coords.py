#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3


# Ground-height clamp value
GROUND_HEIGHT = 0.0


class OdomFrameTransformer(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/odom_in")
        self.output_topic = rospy.get_param("~output_topic", "/odom_out")
        self.matrix_file = rospy.get_param("~matrix_file", "")
        self.output_frame_id = rospy.get_param("~output_frame_id", "")
        self.output_child_frame_id = rospy.get_param("~output_child_frame_id", "")
        self.transform_angular_velocity = rospy.get_param("~transform_angular_velocity", False)

        if not self.matrix_file:
            rospy.logerr("Parameter ~matrix_file is not set")
            raise rospy.ROSInitException("missing ~matrix_file")

        self.T = self.load_matrix(self.matrix_file)
        self.R = self.T[:3, :3]

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback, queue_size=10)

        rospy.loginfo("odom_transform_node started.")
        rospy.loginfo("input_topic: %s", self.input_topic)
        rospy.loginfo("output_topic: %s", self.output_topic)
        rospy.loginfo("matrix_file: %s", self.matrix_file)
        rospy.loginfo("transform matrix:\n%s", self.T)
        rospy.loginfo("ground height clamp: %f", GROUND_HEIGHT)

    def load_matrix(self, matrix_file):
        if not os.path.isfile(matrix_file):
            raise rospy.ROSInitException("matrix file does not exist: {}".format(matrix_file))

        with open(matrix_file, "r") as f:
            data = json.load(f)

        mat = np.array(data, dtype=float)
        if mat.shape != (4, 4):
            raise rospy.ROSInitException(
                "transform matrix must be 4x4, got {}".format(mat.shape)
            )
        return mat

    def transform_position(self, x, y, z):
        pos_h = np.array([x, y, z, 1.0], dtype=float)
        pos_t = np.dot(self.T, pos_h)
        return pos_t[:3]

    def transform_vector(self, x, y, z):
        vec = np.array([x, y, z], dtype=float)
        vec_t = np.dot(self.R, vec)
        return vec_t

    def clamp_height(self, z_value):
        return max(z_value, GROUND_HEIGHT)

    def odom_callback(self, msg):
        out = Odometry()

        # header
        out.header = msg.header
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id

        # child frame
        out.child_frame_id = msg.child_frame_id
        if self.output_child_frame_id:
            out.child_frame_id = self.output_child_frame_id

        # Pose: transform position only; keep orientation unchanged by default.
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        p_t = self.transform_position(px, py, pz)

        # Clamp z to the ground height when it drops below the horizon.
        p_t[2] = self.clamp_height(p_t[2])

        out.pose = msg.pose
        out.pose.pose.position = Point(p_t[0], p_t[1], p_t[2])

        # Twist: transform linear velocity; angular velocity is optional.
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        v_t = self.transform_vector(vx, vy, vz)

        out.twist = msg.twist
        out.twist.twist.linear = Vector3(v_t[0], v_t[1], v_t[2])

        if self.transform_angular_velocity:
            wx = msg.twist.twist.angular.x
            wy = msg.twist.twist.angular.y
            wz = msg.twist.twist.angular.z
            w_t = self.transform_vector(wx, wy, wz)
            out.twist.twist.angular = Vector3(w_t[0], w_t[1], w_t[2])
        else:
            out.twist.twist.angular = msg.twist.twist.angular

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("odom_transform_node")
    try:
        node = OdomFrameTransformer()
        rospy.spin()
    except rospy.ROSInitException as e:
        rospy.logerr("Failed to init node: %s", str(e))
