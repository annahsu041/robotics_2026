import numpy as np
import FK

def compute_jacobian(dh_params):
    """
    計算 3x3 幾何 Jacobian 矩陣（只考慮線速度部分）
    J_i = [z0 × p3  |  R1*z0 × (p3-p1)  |  R2*z0 × (p3-p2)]
    """
    T_total = np.eye(4)
    positions = []
    rotations = []

    for params in dh_params:
        T = FK.dh_transform(*params)
        T_total = T_total @ T
        positions.append(T_total[:3, 3].copy())
        rotations.append(T_total[:3, :3].copy())

    # step 2: Compute Jacobian matrix J
    p1, p2, p3 = positions       # 各關節的世界座標位置，p3 為末端點
    R1, R2, _ = rotations        # 各關節的旋轉矩陣

    # 基座 z 軸（世界座標系）
    z0 = np.array([0, 0, 1])

    # 關節 1：z0 × p3  (基座 z 軸 × 末端點位置向量)
    Jv1 = np.cross(z0, p3)

    # 關節 2：(R1 @ z0) × (p3 - p1)
    Jv2 = np.cross(R1 @ z0, (p3 - p1))

    # 關節 3：(R2 @ z0) × (p3 - p2)
    Jv3 = np.cross(R2 @ z0, (p3 - p2))

    # 組成 3×3 Jacobian
    J = np.column_stack([Jv1, Jv2, Jv3])

    return J
