#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path

class PoseArrayToPath:
    def __init__(self):
        rospy.init_node('pose_array_to_path')
        self.path_pub = rospy.Publisher('/trajectory_path', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"  # 确保与PoseArray的frame_id一致
        rospy.Subscriber('/kf/good_tracks_pose_array', PoseArray, self.pose_array_callback)

    def pose_array_callback(self, msg):
        # 清空历史轨迹（可选）
        # self.path_msg.poses = []

        # 将PoseArray中的每个Pose转为PoseStamped并添加到Path
        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()  # 使用当前时间戳
            pose_stamped.header.frame_id = msg.header.frame_id
            pose_stamped.pose = pose
            self.path_msg.poses.append(pose_stamped)

        # 更新Path的header时间戳
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)

if __name__ == '__main__':
    try:
        PoseArrayToPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass