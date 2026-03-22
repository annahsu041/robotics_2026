#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix
import FK

class QuadrupedTFBroadcaster:
    def __init__(self):
        rospy.init_node('quadruped_tf_broadcaster', anonymous=True)

        # 創建 TF 廣播器
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 訂閱 /joint_states
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.leg_names = ['FL', 'FR', 'RL', 'RR']
        self.leg_part_name = ['hip', 'thigh', 'calf', 'foot']

        # step 4: 定義 body to hip 的 translation（單位：公尺，Unitree Go1 標準參數）
        # FR 往 -y 方向才對（PDF 說明有誤需修正）
        self.translation_dict = {
            'FL': np.array([ 0.1881,  0.04675, 0.0]),   # 左前腿
            'FR': np.array([ 0.1881, -0.04675, 0.0]),   # 右前腿（-y）
            'RL': np.array([-0.1881,  0.04675, 0.0]),   # 左後腿
            'RR': np.array([-0.1881, -0.04675, 0.0]),   # 右後腿
        }

    def get_leg_joint_positions(self, msg, leg):
        indices = {
            'FL': [0, 4, 8], 'FR': [1, 5, 9],
            'RL': [2, 6, 10], 'RR': [3, 7, 11]
        }
        return np.array([msg.position[i] for i in indices[leg]])

    def get_dh_params(self, leg, current_q):
        """
        根據不同的腿返回對應的 DH 參數表
        DH 參數順序：(theta, d, a, alpha)
          - theta : 繞 z 軸旋轉（關節角）
          - d     : 沿 z 軸平移
          - a     : 沿 x 軸連桿長度（thigh=0.213m, calf=0.213m）
          - alpha : 繞 x 軸旋轉
        hip 的側向偏移 (0.062m) 在外部 broadcast_transforms 直接修改 transform_matrix[1,3]
        """
        # step 2: 設計四隻腳的 DH table
        L_thigh = 0.213
        L_calf  = 0.213

        dh_params_dict = {
            # 左腿：alpha1 = +π/2，使腿部 z 軸向外（+y）
            'FL': [
                (current_q[0],  0,        0,       np.pi/2),
                (current_q[1],  0, -L_thigh,       0      ),
                (current_q[2],  0, -L_calf,        0      ),
            ],
            # 右腿：alpha1 = -π/2，使腿部 z 軸向內（-y）
            'FR': [
                (current_q[0],  0,        0,      -np.pi/2),
                (current_q[1],  0, -L_thigh,       0      ),
                (current_q[2],  0, -L_calf,        0      ),
            ],
            'RL': [
                (current_q[0],  0,        0,       np.pi/2),
                (current_q[1],  0, -L_thigh,       0      ),
                (current_q[2],  0, -L_calf,        0      ),
            ],
            'RR': [
                (current_q[0],  0,        0,      -np.pi/2),
                (current_q[1],  0, -L_thigh,       0      ),
                (current_q[2],  0, -L_calf,        0      ),
            ],
        }
        return dh_params_dict[leg]

    def joint_state_callback(self, msg):
        self.broadcast_transforms(msg)

    def get_body_to_hip_translation_matrix(self, leg):
        """
        step 5: 定義 body to hip 的 rotation
        所有腿都繞 y 軸轉 -90 度
        R_y(-90°) = [[0, 0, -1],
                     [0, 1,  0],
                     [1, 0,  0]]
        """
        theta = np.radians(-90)
        rotation_matrix = np.array([
            [np.cos(theta),  0, np.sin(theta), 0],
            [0,              1, 0,             0],
            [-np.sin(theta), 0, np.cos(theta), 0],
            [0,              0, 0,             1]
        ])
        # 填入平移向量
        translation_matrix = rotation_matrix.copy()
        translation_matrix[:3, 3] = self.translation_dict[leg]

        return translation_matrix

    def broadcast_transforms(self, msg):
        time = rospy.Time.now()

        for leg in self.leg_names:
            current_q = self.get_leg_joint_positions(msg, leg)
            dh_params = self.get_dh_params(leg, current_q)

            # trunk → hip
            parent_frame = "trunk"
            child_frame  = f"{leg}_{self.leg_part_name[0]}"
            transform_matrix = self.get_body_to_hip_translation_matrix(leg)
            self.publish_tf(transform_matrix, time, parent_frame, child_frame)

            # hip → thigh（側向偏移 ±0.062m 直接加在矩陣平移量上）
            parent_frame = f"{leg}_{self.leg_part_name[0]}"
            child_frame  = f"{leg}_{self.leg_part_name[1]}"
            transform_matrix = FK.forward_kinematics_translation_matrix([dh_params[0]])
            if leg in ["FL", "RL"]:
                transform_matrix[1, 3] = 0.062    # 左腿 +y
            else:
                transform_matrix[1, 3] = -0.062   # 右腿 -y
            self.publish_tf(transform_matrix, time, parent_frame, child_frame)

            # thigh → calf
            parent_frame = f"{leg}_{self.leg_part_name[1]}"
            child_frame  = f"{leg}_{self.leg_part_name[2]}"
            transform_matrix = FK.forward_kinematics_translation_matrix([dh_params[1]])
            self.publish_tf(transform_matrix, time, parent_frame, child_frame)

            # calf → foot
            parent_frame = f"{leg}_{self.leg_part_name[2]}"
            child_frame  = f"{leg}_{self.leg_part_name[3]}"
            transform_matrix = FK.forward_kinematics_translation_matrix([dh_params[2]])
            self.publish_tf(transform_matrix, time, parent_frame, child_frame)

    def publish_tf(self, transform_matrix, time, parent_frame, child_frame):
        """將轉移矩陣轉換為 TF 並發布"""
        translation = transform_matrix[:3, 3]
        quaternion  = quaternion_from_matrix(transform_matrix)

        self.tf_broadcaster.sendTransform(
            translation,
            quaternion,
            time,
            child_frame,
            parent_frame
        )

if __name__ == '__main__':
    node = QuadrupedTFBroadcaster()
    rospy.spin()
