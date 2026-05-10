#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        self.in_topic   = rospy.get_param("~in_topic", "/mavros/odometry/out")       # Odometry
        self.out_topic  = rospy.get_param("~out_topic", "/uav_path")  # Path
        self.frame_id   = rospy.get_param("~frame_id", "")            # Empty keeps odom.header.frame_id
        self.max_len    = rospy.get_param("~max_len", 0)
        self.min_dist   = rospy.get_param("~min_dist", 0.02)

        self.path_pub = rospy.Publisher(self.out_topic, Path, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(self.in_topic, Odometry, self.cb, queue_size=50)

        self.path = Path()
        self.last_xyz = None

        rospy.loginfo("[odom_to_path] subscribe=%s publish=%s", self.in_topic, self.out_topic)

    def cb(self, msg: Odometry):
        p = msg.pose.pose.position
        x, y, z = p.x, p.y, p.z

        if self.last_xyz is not None:
            dx = x - self.last_xyz[0]
            dy = y - self.last_xyz[1]
            dz = z - self.last_xyz[2]
            if (dx*dx + dy*dy + dz*dz) < (self.min_dist * self.min_dist):
                return
        self.last_xyz = (x, y, z)

        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.frame_id if self.frame_id else msg.header.frame_id
        ps.pose = msg.pose.pose

        self.path.header.stamp = ps.header.stamp
        self.path.header.frame_id = ps.header.frame_id

        self.path.poses.append(ps)

        if self.max_len and len(self.path.poses) > self.max_len:
            self.path.poses = self.path.poses[-self.max_len:]

        self.path_pub.publish(self.path)

def main():
    rospy.init_node("path_to_rviz_odom")
    OdomToPath()
    rospy.spin()

if __name__ == "__main__":
    main()
