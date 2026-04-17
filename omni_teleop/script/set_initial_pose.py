#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():
    rospy.init_node('set_initial_pose')
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    rospy.sleep(1.0)  # 等待发布器准备好

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    # 设置位置
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0

    # 设置朝向为无旋转（四元数 w=1）
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0
    msg.pose.pose.orientation.w = 1.0

    # 设置协方差（推荐至少给一个非零值）
    msg.pose.covariance[0] = 0.25   # x方向的协方差
    msg.pose.covariance[7] = 0.25   # y方向的协方差
    msg.pose.covariance[35] = 0.0685  # 假设角度的协方差为 ~4度（弧度）

    pub.publish(msg)
    rospy.loginfo("Initial pose published to /initialpose")

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass

