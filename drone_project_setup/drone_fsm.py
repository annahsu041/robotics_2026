#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多旋翼機三階段狀態機 (FSM) 範例節點
階段一：TAKEOFF_EXPLORE (起飛與探索)
階段二：APPROACH (接近目標)
階段三：PRECISION_LAND (精準降落)
"""

import rospy
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.srv import SetMode, CommandTOL
from mavros_msgs.msg import State

class DroneFSM:
    def __init__(self):
        rospy.init_node('drone_fsm_node', anonymous=True)
        
        self.state = "TAKEOFF_EXPLORE"
        self.tag_detected = False
        self.detection_count = 0
        self.current_mavros_state = State()
        
        # Subscribers
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_cb)
        
        # Publishers
        # 用於傳送降落目標點給 PX4 (需搭配 AUTO.PRECLAND 模式)
        self.landing_target_pub = rospy.Publisher('/mavros/landing_target/pose', PoseStamped, queue_size=10)
        
        # Services
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/land')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.land_cmd = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        
        rospy.loginfo("Drone FSM Node Initialized.")

    def state_cb(self, msg):
        self.current_mavros_state = msg
        
    def tag_cb(self, msg):
        # 檢查是否偵測到 AprilTag
        if len(msg.detections) > 0:
            self.detection_count += 1
            if self.detection_count > 10: # 連續 N 幀有效
                self.tag_detected = True
                
            # 將偵測到的姿態發布為降落目標
            landing_pose = PoseStamped()
            landing_pose.header = msg.header
            landing_pose.pose = msg.detections[0].pose.pose.pose
            self.landing_target_pub.publish(landing_pose)
        else:
            # 容許短暫遮擋，連續沒看到才重置
            self.detection_count = max(0, self.detection_count - 1)
            if self.detection_count == 0:
                self.tag_detected = False

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self.state == "TAKEOFF_EXPLORE":
                # 在此階段由 ego-planner 負責規劃軌跡
                if self.tag_detected:
                    rospy.loginfo("[FSM] AprilTag 連續偵測成功！切換至 APPROACH 階段。")
                    self.state = "APPROACH"
                    
            elif self.state == "APPROACH":
                # 修改 ego-planner 的目標點為標籤上方 (需透過其他 topic 串接)
                # 當無人機夠靠近標籤時，切換至精準降落
                if self.detection_count > 30: # 假設已穩定追蹤且靠近
                    rospy.loginfo("[FSM] 目標已鎖定，切換至 PRECISION_LAND 階段。")
                    self.state = "PRECISION_LAND"
                    
            elif self.state == "PRECISION_LAND":
                # 觸發 PX4 內建的 AUTO.PRECLAND 模式
                if self.current_mavros_state.mode != "AUTO.PRECLAND":
                    rospy.loginfo("[FSM] 觸發 AUTO.PRECLAND 模式...")
                    try:
                        resp = self.set_mode(0, 'AUTO.PRECLAND')
                        if resp.mode_sent:
                            rospy.loginfo("[FSM] 成功切換至 AUTO.PRECLAND！等待降落...")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Service call failed: {e}")
                
                # 若需要強制觸發一般降落
                # self.land_cmd(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
                
                # 降落後結束狀態機循環 (簡化處理)
                rospy.sleep(5)
                break
                
            rate.sleep()

if __name__ == '__main__':
    try:
        fsm = DroneFSM()
        fsm.run()
    except rospy.ROSInterruptException:
        pass
