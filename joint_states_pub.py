#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState

class JointStatesPublisher:
    def __init__(self, file_path):
        rospy.init_node('joint_states_publisher', anonymous=True)

        # 創建 Publisher 發布到 /joint_states
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # 讀取並儲存角度數據
        self.joint_data = self.load_joint_data(file_path)
        self.total_rows = len(self.joint_data)
        self.current_index = 0

        # 100Hz (10ms) 發布一次
        self.timer = rospy.Timer(rospy.Duration(0.01), self.publish_next_command)

        # 正確的關節名稱
        self.joint_names = [
            "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
            "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
            "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
        ]

    def load_joint_data(self, file_path):
        """讀取 txt 檔案，並確保數據格式正確"""
        try:
            data = np.loadtxt(file_path, delimiter=',')  # 讀取 TXT
            if data.shape[1] != 12:
                rospy.logerr("錯誤：文件格式不正確，應該是 12 列")
                rospy.signal_shutdown("無法繼續運行")
            return data
        except Exception as e:
            rospy.logerr(f"讀取文件錯誤: {e}")
            rospy.signal_shutdown("無法繼續運行")
            return []

    def publish_next_command(self, event):
        """發布下一筆 JointState 指令"""
        if self.current_index >= self.total_rows:
            rospy.loginfo("所有指令已發送完畢")
            rospy.signal_shutdown("完成指令發布")
            return

        # 直接使用 txt 內的順序
        joint_positions = self.joint_data[self.current_index]

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = joint_positions.tolist()

        self.publisher.publish(msg)
        rospy.loginfo(f"發布 JointState (弧度): {msg.position}")

        self.current_index += 1

if __name__ == '__main__':
    # step1: 設置 txt 檔案路徑（請依實際路徑修改）
    file_path = "/home/wang/catkin_ws/src/class2.5/quadruped/joint_states.txt"
    JointStatesPublisher(file_path)
    rospy.spin()
