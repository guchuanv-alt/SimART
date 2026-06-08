#!/usr/bin/env python3
import json
import math
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def main():
    rospy.init_node('airsim_gui_UErealtime_mock_publishers')
    pose_pub = rospy.Publisher('/uav_pose', PoseStamped, queue_size=10)
    path_pub = rospy.Publisher('/beam_paths_json', String, queue_size=10)

    base_stations = {
        'bs_alpha': (0.0, 0.0, 22.0),
        'bs_bravo': (55.0, -35.0, 18.0),
        'bs_charlie': (-60.0, 45.0, 20.0),
    }

    rate = rospy.Rate(20)
    t = 0.0
    while not rospy.is_shutdown():
        x = 45.0 * math.cos(t * 0.35)
        y = 30.0 * math.sin(t * 0.55)
        z = 20.0 + 8.0 * math.sin(t * 0.22)
        yaw = math.atan2(0.55 * 30.0 * math.cos(t * 0.55), -0.35 * 45.0 * math.sin(t * 0.35))

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'vins_world'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose_pub.publish(pose)

        paths = []
        for idx, (bs_id, bs) in enumerate(base_stations.items()):
            dx = bs[0] - x
            dy = bs[1] - y
            dz = bs[2] - z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            los_power = -48.0 - 0.35 * dist
            los_delay = dist / 0.299792458
            paths.append({
                'id': f'{bs_id}_los',
                'tx_id': bs_id,
                'rx_id': 'uav_ros',
                'power_db': los_power,
                'delay_ns': los_delay,
                'kind': 'los',
                'points': [[bs[0], bs[1], bs[2]], [x, y, z]],
            })
            bounce = [
                0.55 * (bs[0] + x) + 8.0 * math.cos(t + idx),
                0.55 * (bs[1] + y) - 5.0 * math.sin(t * 0.8 + idx),
                8.0 + 2.0 * idx,
            ]
            paths.append({
                'id': f'{bs_id}_reflect',
                'tx_id': bs_id,
                'rx_id': 'uav_ros',
                'power_db': los_power - 8.0 - idx * 2.0,
                'delay_ns': los_delay + 60.0 + idx * 20.0,
                'kind': 'specular',
                'points': [[bs[0], bs[1], bs[2]], bounce, [x, y, z]],
            })

        path_pub.publish(String(data=json.dumps({'paths': paths})))
        t += 0.05
        rate.sleep()


if __name__ == '__main__':
    main()
