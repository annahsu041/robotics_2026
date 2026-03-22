#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np

class JointStatesPublisher:
    def __init__(self):
        rospy.init_node('joint_states_publisher', anonymous=True)

        # 創建 JointState 發布者
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # 訂閱 Joint Command（命令來時更新 joint_states）
        rospy.Subscriber('/joint_command', JointState, self.joint_command_callback)

        # 設定初始關節角度（弧度）
        self.joint_names = [
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
        ]
        self.joint_positions = np.array([0.0, 0.0, 0.0, 0.0, 0.8, -0.8, 0.8, -0.8, -0.8*2, 0.8*2, -0.8*2, 0.8*2])
        
        # 以 100Hz（10ms）頻率發布 JointState
        self.timer = rospy.Timer(rospy.Duration(0.01), self.publish_joint_states)

    def joint_command_callback(self, msg):
        """當收到新的 JointState 訊息時，更新 `joint_positions`"""
        if len(msg.position) == 12:
            self.joint_positions = np.array(msg.position)
            rospy.loginfo("更新 Joint States: {}".format(self.joint_positions))
        else:
            rospy.logerr("收到錯誤的 JointState 位置數據，應該是 12 個數值")

    def publish_joint_states(self, event):
        """定時發布 JointState"""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = self.joint_positions.tolist()  # 轉換為列表

        self.joint_states_pub.publish(msg)
        rospy.loginfo("發布 JointState: {}".format(msg.position))

if __name__ == '__main__':
    node = JointStatesPublisher()
    rospy.spin()
