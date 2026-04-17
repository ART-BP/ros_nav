#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import threading

class CmdVelMonitor:
    def __init__(self):
        rospy.init_node('cmd_vel_timeout_node')

        self.timeout = rospy.Duration(0.22)
        self.last_cmd_time = None
        self.cmd_received = False
        self.lock = threading.Lock()

        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel_safe", Twist, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.check_timeout)

    def cmd_callback(self, msg):
        with self.lock:
            self.cmd_received = True
            self.last_cmd_time = rospy.Time.now()
            self.cmd_pub.publish(msg)

    def check_timeout(self, event):
        with self.lock:
            if not self.cmd_received:
                return

            if rospy.Time.now() - self.last_cmd_time > self.timeout:
                zero_cmd = Twist()
                self.cmd_pub.publish(zero_cmd)
                rospy.loginfo("No cmd_vel received in 0.22s, publishing zero velocity.")
                self.cmd_received = False  # 防止重复发布

if __name__ == '__main__':
    try:
        CmdVelMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

