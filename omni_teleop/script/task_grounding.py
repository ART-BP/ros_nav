#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import actionlib
import math
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class TaskToGoalActionClient:
    def __init__(self):
        self.tf_mapping_dict = {
            'base_link': 'base_link',
            '目标点': 'base_link',
            'odom': 'odom',
            '坐标点': 'odom'
        }

        rospy.init_node('task_to_goal_node', anonymous=True)

        self.sub = rospy.Subscriber('/task/stream', String, self.callback)
        self.feedback_pub = rospy.Publisher('/task/feedback', String, queue_size=10)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            task = data.get('task')
            params = data.get('params')
            task_id = data.get('task_id', 'unknown')

            # 🛑 中止任务指令
            if task == '基础动作' and params == '退出任务':
                rospy.logwarn("Received cancel request. Cancelling current goal.")
                self.client.cancel_goal()
                feedback = {
                    'task': task,
                    'params': params,
                    'task_id': task_id,
                    'llm': 'cancel@'
                }
                self.feedback_pub.publish(String(json.dumps(feedback)))
                return

            # ✅ 检查导航任务格式（现在需要 3 个参数）
            if not task or not isinstance(params, list) or len(params) != 3:
                rospy.logwarn("Invalid navigation task format (need [x, y, yaw]): %s", msg.data)
                return

            x, y, yaw = float(params[0]), float(params[1]), float(math.radians(params[2]))

            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = self.tf_mapping_dict.get(task, 'odom')

            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.position.z = 0.0

            # 将 yaw (弧度) 转换为四元数
            q = quaternion_from_euler(0, 0, yaw)
            goal.target_pose.pose.orientation = Quaternion(*q)

            rospy.loginfo("Sending goal to move_base: [%f, %f, yaw=%.2f rad]", x, y, yaw)
            self.client.send_goal(goal)
            self.client.wait_for_result()

            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                llm_status = 'success@'
                rospy.loginfo("Goal succeeded.")
            elif state in [GoalStatus.PREEMPTED, GoalStatus.ABORTED]:
                llm_status = 'fail@'
                rospy.logwarn("Goal was aborted or preempted.")
            else:
                llm_status = 'fail@'
                rospy.logwarn("Goal failed with state: %d", state)

            feedback = {
                'task': task,
                'params': params,
                'task_id': task_id,
                'llm': llm_status
            }
            self.feedback_pub.publish(String(json.dumps(feedback)))

        except Exception as e:
            rospy.logerr("Error in task processing: %s", str(e))
            try:
                data = json.loads(msg.data)
                feedback = {
                    'task': data.get('task', 'unknown'),
                    'params': data.get('params', []),
                    'task_id': data.get('task_id', 'unknown'),
                    'llm': 'fail@'
                }
                self.feedback_pub.publish(String(json.dumps(feedback)))
            except:
                rospy.logerr("Feedback failed due to malformed input.")

if __name__ == '__main__':
    try:
        TaskToGoalActionClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

