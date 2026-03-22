import numpy as np

def dh_transform(theta, d, a, alpha):
    """
    計算 Denavit-Hartenberg 變換矩陣
    T_i^{i-1} = Trans_z(d) * Rot_z(theta) * Trans_x(a) * Rot_x(alpha)
    """
    # step 3: DH 轉移矩陣
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),               d              ],
        [0,              0,                             0,                           1              ]
    ])

def forward_kinematics(dh_params):
    """
    使用 DH 參數計算末端點位置
    """
    T_total = np.eye(4)
    for params in dh_params:
        T = dh_transform(*params)
        T_total = T_total @ T
    return T_total[:3, 3]

def forward_kinematics_translation_matrix(dh_params):
    """
    使用 DH 參數計算末端點位置（返回完整 4x4 矩陣）
    """
    T_total = np.eye(4)
    for params in dh_params:
        T = dh_transform(*params)
        T_total = T_total @ T
    return T_total
