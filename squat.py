#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np
import FK
import IK_jacobian

class QuadrupedIK:
    def __init__(self):
        rospy.init_node('quadruped_ik', anonymous=True)

        # 訂閱 /joint_states
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # 發布 /joint_command
        self.publisher = rospy.Publisher('/joint_command', JointState, queue_size=10)

        self.leg_names = ['FL', 'FR', 'RL', 'RR']
        self.joint_names = [
            'FL_hip_joint',   'FR_hip_joint',   'RL_hip_joint',   'RR_hip_joint',
            'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
            'FL_calf_joint',  'FR_calf_joint',  'RL_calf_joint',  'RR_calf_joint'
        ]

        # 運動參數
        self.A = 0.1   # 最大高度變化量（振幅，沿 x 軸）
        self.T = 5     # 週期（機器狗完整蹲站一次的時間，秒）
        self.N = 50    # 離散點數量（越大越平滑）
        self.current_step = 0      # 當前的時間步
        self.initialized  = False  # 確保只讀取初始狀態一次

        self.initial_foot_positions = {}  # 儲存初始腳的位置
        self.trajectory = []              # 預先規劃好的軌跡點

        # 100ms 更新一次（定時推進軌跡步數）
        rospy.Timer(rospy.Duration(0.1), self.update_target_positions)

    # ------------------------------------------------------------------ #
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

        與 quadruped_tf_broadcaster.py 的差異：
          此處把側向偏移 (d=±0.062m) 直接放進 DH table（hip 關節的 d 值），
          而不是事後在矩陣上手動改 [1,3]。
        """
        # step 1: 設計四隻腳的 DH table（offset 放在 DH table 裡面）
        L_thigh   = 0.213
        L_calf    = 0.213
        hip_off_L =  0.062   # 左腿側向偏移 +y
        hip_off_R = -0.062   # 右腿側向偏移 -y

        dh_params_dict = {
            # 左前腿
            'FL': [
                (current_q[0],  hip_off_L,  0,       np.pi/2),
                (current_q[1],  0,         -L_thigh, 0      ),
                (current_q[2],  0,         -L_calf,  0      ),
            ],
            # 右前腿
            'FR': [
                (current_q[0],  hip_off_R,  0,      -np.pi/2),
                (current_q[1],  0,         -L_thigh, 0      ),
                (current_q[2],  0,         -L_calf,  0      ),
            ],
            # 左後腿
            'RL': [
                (current_q[0],  hip_off_L,  0,       np.pi/2),
                (current_q[1],  0,         -L_thigh, 0      ),
                (current_q[2],  0,         -L_calf,  0      ),
            ],
            # 右後腿
            'RR': [
                (current_q[0],  hip_off_R,  0,      -np.pi/2),
                (current_q[1],  0,         -L_thigh, 0      ),
                (current_q[2],  0,         -L_calf,  0      ),
            ],
        }
        return dh_params_dict[leg]

    # ------------------------------------------------------------------ #
    def initialize_foot_positions(self, msg):
        """讀取初始腳的位置（用 FK 計算）"""
        for leg in self.leg_names:
            current_q = self.get_leg_joint_positions(msg, leg)
            dh_params = self.get_dh_params(leg, current_q)
            self.initial_foot_positions[leg] = FK.forward_kinematics(dh_params)

        x_avg = np.mean([pos[0] for pos in self.initial_foot_positions.values()])
        rospy.loginfo(f"Initialized foot positions. Avg X-height: {x_avg:.4f}")

    def plan_trajectory(self):
        """根據初始腳位置規劃完整軌跡（正弦波蹲站）"""
        self.trajectory = []
        for i in range(self.N):
            t = (i / self.N) * self.T
            x_offset = self.A * np.sin(2 * np.pi * t / self.T)   # sin 波

            step_positions = {
                leg: self.initial_foot_positions[leg] + np.array([x_offset, 0, 0])
                for leg in self.leg_names
            }
            self.trajectory.append(step_positions)

        rospy.loginfo("Trajectory planned.")

    def update_target_positions(self, event):
        """推進軌跡步數"""
        if not self.trajectory:
            return
        self.current_step = (self.current_step + 1) % self.N
        rospy.loginfo(f"Moving to trajectory step: {self.current_step}")

    # ------------------------------------------------------------------ #
    def joint_state_callback(self, msg):
        """讀取當前關節狀態，計算 IK 解，發送指令"""
        joint_positions = [0.0] * 12

        if not self.initialized:
            self.initialize_foot_positions(msg)
            self.plan_trajectory()
            self.initialized = True

        for leg in self.leg_names:
            current_q  = self.get_leg_joint_positions(msg, leg)
            target_pos = self.trajectory[self.current_step][leg]
            new_q      = self.inverse_kinematics(leg, current_q, target_pos)

            leg_indices = {'FL': 0, 'FR': 1, 'RL': 2, 'RR': 3}
            idx = leg_indices[leg]
            joint_positions[idx]     = new_q[0]   # Hip
            joint_positions[idx + 4] = new_q[1]   # Thigh
            joint_positions[idx + 8] = new_q[2]   # Calf

        self.send_joint_commands(joint_positions)

    def inverse_kinematics(self, leg, current_q, target_pos):
        """
        Jacobian 虛逆矩陣迭代法解 IK
        q_{k+1} = q_k + J† (x_desired - x_current)
        """
        max_iterations = 5
        tolerance      = 0.001

        # step 3: 用 Jacobian 虛逆解 IK
        for _ in range(max_iterations):
            dh_params   = self.get_dh_params(leg, current_q)
            current_pos = FK.forward_kinematics(dh_params)
            error       = target_pos - current_pos

            if np.linalg.norm(error) < tolerance:
                print("convergence")
                break

            jacobian    = IK_jacobian.compute_jacobian(dh_params)
            # 虛逆矩陣 J† = J^T (J J^T)^{-1}，np.linalg.pinv 自動處理
            delta_theta = np.linalg.pinv(jacobian) @ error
            current_q   = current_q + delta_theta

        return current_q

    def send_joint_commands(self, joint_positions):
        msg = JointState()
        msg.name     = self.joint_names
        msg.position = joint_positions
        print("publish")
        self.publisher.publish(msg)

if __name__ == '__main__':
    QuadrupedIK()
    rospy.spin()
