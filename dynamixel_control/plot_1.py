import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline
import math
from scipy.spatial.transform import Rotation as R_quat

# === DH 파라미터 ===
DH_params = [
    [0,                 0.040,    0,        0],       # Joint 0 (Base 회전)
    [0,                 0.050,    0,  -np.pi/2],   # Joint 1 (어깨)
    [-np.pi/2,          0,        0.125,    0],       # Joint 2 (팔꿈치)
    [0,                 0,        0.13,     0],       # Joint 3 (손목 피치)
    [0,                 0,        0.10,     0],       # Joint 4 (좌우 이동 엔드 이펙터)
]
num_joints = len(DH_params)

# === 유틸리티 함수 ===
def euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """ZYX 외인성 오일러 각으로부터 회전 행렬 생성"""
    roll_rad = np.radians(roll_deg)
    pitch_rad = np.radians(pitch_deg)
    yaw_rad = np.radians(yaw_deg)

    Rx = np.array([[1, 0, 0], [0, np.cos(roll_rad), -np.sin(roll_rad)], [0, np.sin(roll_rad), np.cos(roll_rad)]])
    Ry = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)], [0, 1, 0], [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    Rz = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0], [np.sin(yaw_rad), np.cos(yaw_rad), 0], [0, 0, 1]])

    return Rz @ Ry @ Rx

def forward_kinematics(joints):
    """주어진 조인트 각도로 엔드 이펙터의 최종 변환 행렬 계산"""
    T = np.eye(4)
    for i, (theta_offset, d, a, alpha) in enumerate(DH_params):
        theta = joints[i] + theta_offset
        T_i = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
        T = T @ T_i
    return T

def rotation_matrix_to_quaternion(R):
    r = R_quat.from_matrix(R)
    return r.as_quat() # [x, y, z, w]

def quaternion_distance(q1, q2):
    """두 쿼터니언 사이의 각도 거리 계산 (더 안정적)"""
    dot_product = np.dot(q1, q2)
    return 1 - abs(dot_product) # 0 <= distance <= 2

def inverse_kinematics(target_pose_matrix, initial_guess=np.zeros(num_joints)):
    """목표 자세에 도달하기 위한 조인트 각도 계산 (최적화 사용)"""
    target_pos = target_pose_matrix[:3, 3]
    target_rot = target_pose_matrix[:3, :3]

    bounds = [
        (-np.pi, np.pi),   # Joint 0
        (-np.pi, np.pi),   # Joint 1
        (-np.deg2rad(100), np.deg2rad(100)),   # Joint 2
        (-np.deg2rad(100), np.deg2rad(100)),   # Joint 3
        (-np.pi, np.pi)    # Joint 4
    ]

    def objective_function(joints):
        current_pose_matrix = forward_kinematics(joints)
        current_pos = current_pose_matrix[:3, 3]
        current_rot = current_pose_matrix[:3, :3]

        target_quat = rotation_matrix_to_quaternion(target_rot)
        current_quat = rotation_matrix_to_quaternion(current_rot)

        position_error = np.sum((current_pos - target_pos) ** 2)
        orientation_error = quaternion_distance(target_quat, current_quat)*0.01  # 쿼터니언 거리 오차에 매우 큰 가중치 부여

        return position_error + orientation_error

    result = minimize(objective_function, initial_guess, method='L-BFGS-B', bounds=bounds, options={'maxiter': 500}) # 최대 반복 횟수 증가
    return result.x if result.success else initial_guess

def get_joint_positions(joint_angles):
    """각 조인트의 3D 위치를 계산합니다."""
    T = np.eye(4)
    joint_positions = [T[:3, 3]]
    for i, (theta_offset, d, a, alpha) in enumerate(DH_params):
        theta = joint_angles[i] + theta_offset
        T_i = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
        T = T @ T_i
        joint_positions.append(T[:3, 3])
    return np.array(joint_positions)

def cubic_spline_trajectory(joint_angles_initial, joint_angles_final, time_points):
    """조인트 공간에서 Cubic Spline 궤적 생성"""
    spline_trajectory = []
    for initial, final in zip(joint_angles_initial, joint_angles_final):
        times = [0, max(time_points)]
        angles = [initial, final]
        spline = CubicSpline(times, angles, bc_type='clamped')
        spline_trajectory.append(spline(time_points))
    return np.array(spline_trajectory).T # (time_points, num_joints) 형태로 반환

def plot_robot(joint_positions, end_effector_pose=None, target_position=None):
    """로봇 팔의 현재 자세를 3D 플롯으로 시각화하고 엔드 이펙터 및 베이스 좌표계를 표시합니다."""
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 로봇 팔 연결선 그리기
    x = joint_positions[:, 0]
    y = joint_positions[:, 1]
    z = joint_positions[:, 2]
    ax.plot(x, y, z, '-o', label='Robot Arm')

    # 베이스 좌표계 그리기
    base_origin = np.array([0, 0, 0])
    base_x = base_origin + np.array([0.05, 0, 0])
    base_y = base_origin + np.array([0, 0.05, 0])
    base_z = base_origin + np.array([0, 0, 0.05])
    ax.plot([base_origin[0], base_x[0]], [base_origin[1], base_x[1]], [base_origin[2], base_x[2]], 'r-', linewidth=2, label='Base X')
    ax.plot([base_origin[0], base_y[0]], [base_origin[1], base_y[1]], [base_origin[2], base_y[2]], 'g-', linewidth=2, label='Base Y')
    ax.plot([base_origin[0], base_z[0]], [base_origin[1], base_z[1]], [base_origin[2], base_z[2]], 'b-', linewidth=2, label='Base Z')
    ax.scatter(base_origin[0], base_origin[1], base_origin[2], color='black', marker='o', s=50, label='Base Origin')

    # 엔드 이펙터 좌표계 그리기
    if end_effector_pose is not None:
        ee_pos = end_effector_pose[:3, 3]
        ee_rot = end_effector_pose[:3, :3]

        # 각 축 방향 벡터 (길이 조절 가능)
        axis_length = 0.05
        ee_x_axis = ee_pos + ee_rot[:, 0] * axis_length
        ee_y_axis = ee_pos + ee_rot[:, 1] * axis_length
        ee_z_axis = ee_pos + ee_rot[:, 2] * axis_length

        ax.plot([ee_pos[0], ee_x_axis[0]], [ee_pos[1], ee_x_axis[1]], [ee_pos[2], ee_x_axis[2]], 'r--', linewidth=2, label='EEF X')
        ax.plot([ee_pos[0], ee_y_axis[0]], [ee_pos[1], ee_y_axis[1]], [ee_pos[2], ee_y_axis[2]], 'g--', linewidth=2, label='EEF Y')
        ax.plot([ee_pos[0], ee_z_axis[0]], [ee_pos[1], ee_z_axis[1]], [ee_pos[2], ee_z_axis[2]], 'b--', linewidth=2, label='EEF Z')

        ax.scatter(ee_pos[0], ee_pos[1], ee_pos[2], color='black', marker='s', s=50, label='EEF Origin')

    # 좌표축 레이블
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 플롯 제목
    ax.set_title('Robot Arm Visualization with Coordinate Frames')

    # 시점 설정 (보기 좋게)
    ax.view_init(elev=30, azim=45)

    # 축 비율 조정 (정확한 비율로 보이도록)
    all_points = joint_positions
    if end_effector_pose is not None:
        all_points = np.vstack([all_points, ee_pos, base_origin, base_x, base_y, base_z])
    else:
        all_points = np.vstack([all_points, base_origin, base_x, base_y, base_z])
    if target_position is not None:
        all_points = np.vstack([all_points, target_position])

    max_range = np.array([all_points[:, 0].max()-all_points[:, 0].min(),
                          all_points[:, 1].max()-all_points[:, 1].min(),
                          all_points[:, 2].max()-all_points[:, 2].min()]).max() / 2.0
    mid_x = (all_points[:, 0].max()+all_points[:, 0].min()) * 0.5
    mid_y = (all_points[:, 1].max()+all_points[:, 1].min()) * 0.5
    mid_z = (all_points[:, 2].max()+all_points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # 목표 위치 표시
    if target_position is not None:
        ax.scatter(target_position[0], target_position[1], target_position[2], color='red', marker='*', s=100, label='Target')

    ax.legend()
    plt.show()

def visualize_trajectory(trajectory_points, target_position):
    """생성된 궤적을 시각화하고 각 자세에서의 엔드 이펙터 좌표계를 표시합니다."""
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    ee_origins = []
    for joint_angles in trajectory_points:
        joint_positions = get_joint_positions(joint_angles)
        end_effector_pose = forward_kinematics(joint_angles)
        ee_pos = end_effector_pose[:3, 3]
        ee_origins.append(ee_pos)
        ax.plot(joint_positions[:, 0], joint_positions[:, 1], joint_positions[:, 2], '-', linewidth=0.5, alpha=0.3) # 로봇 팔
        # 각 자세에서의 엔드 이펙터 좌표계 (간단하게 원점만 표시)
        ee_rot = end_effector_pose[:3, :3]
        axis_length = 0.02
        ee_x_axis = ee_pos + ee_rot[:, 0] * axis_length
        ee_y_axis = ee_pos + ee_rot[:, 1] * axis_length
        ee_z_axis = ee_pos + ee_rot[:, 2] * axis_length
        ax.plot([ee_pos[0], ee_x_axis[0]], [ee_pos[1], ee_x_axis[1]], [ee_pos[2], ee_x_axis[2]], 'r-', linewidth=1, alpha=0.5)
        ax.plot([ee_pos[0], ee_y_axis[0]], [ee_pos[1], ee_y_axis[1]], [ee_pos[2], ee_y_axis[2]], 'g-', linewidth=1, alpha=0.5)
        ax.plot([ee_pos[0], ee_z_axis[0]], [ee_pos[1], ee_z_axis[1]], [ee_pos[2], ee_z_axis[2]], 'b-', linewidth=1, alpha=0.5)
        ax.scatter(ee_pos[0], ee_pos[1], ee_pos[2], color='gray', marker='.', s=10, alpha=0.5)

    ee_origins = np.array(ee_origins)
    ax.plot(ee_origins[:, 0], ee_origins[:, 1], ee_origins[:, 2], '--', color='purple', label='EEF Path')

    # 베이스 좌표계 (고정)
    base_origin = np.array([0, 0, 0])
    base_x = base_origin + np.array([0.05, 0, 0])
    base_y = base_origin + np.array([0, 0.05, 0])
    base_z = base_origin + np.array([0, 0, 0.05])
    ax.plot([base_origin[0], base_x[0]], [base_origin[1], base_x[1]], [base_origin[2], base_x[2]], 'r-', linewidth=2, label='Base X')
    ax.plot([base_origin[0], base_y[0]], [base_origin[1], base_y[1]], [base_origin[2], base_y[2]], 'g-', linewidth=2, label='Base Y')
    ax.plot([base_origin[0], base_z[0]], [base_origin[1], base_z[1]], [base_origin[2], base_z[2]], 'b-', linewidth=2, label='Base Z')
    ax.scatter(base_origin[0], base_origin[1], base_origin[2],color='black', marker='o', s=50, label='Base Origin')

    # 시작 자세와 끝 자세 강조
    start_positions = get_joint_positions(trajectory_points[0])
    end_positions = get_joint_positions(trajectory_points[-1])
    ax.plot(start_positions[:, 0], start_positions[:, 1], start_positions[:, 2], '-o', color='green', label='Start')
    ax.plot(end_positions[:, 0], end_positions[:, 1], end_positions[:, 2], '-o', color='blue', label='End')

    # 목표 위치 표시
    ax.scatter(target_position[0], target_position[1], target_position[2], color='red', marker='*', s=100, label='Target')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Robot Arm Trajectory with Coordinate Frames')
    ax.view_init(elev=30, azim=45)

    # 축 비율 조정
    joint_positions_all = np.concatenate([get_joint_positions(ja) for ja in trajectory_points], axis=0)
    min_coords = np.min(joint_positions_all, axis=0)
    max_coords = np.max(joint_positions_all, axis=0)
    overall_min = np.min([min_coords, np.min(ee_origins, axis=0), base_origin], axis=0)
    overall_max = np.max([max_coords, np.max(ee_origins, axis=0), base_origin + np.array([0.05, 0.05, 0.05])], axis=0)
    max_range = np.max(overall_max - overall_min) / 2.0
    mid = (overall_min + overall_max) / 2.0
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    ax.legend()
    plt.show()

if __name__ == '__main__':
    # 목표 위치 및 방향 설정
    target_x = 0.05
    target_y = 0.0
    target_z = 0.15
    target_roll_deg = 0
    target_pitch_deg = 0
    target_yaw_deg = 0 # 목표 방향 변경

    # 목표 자세 행렬 생성
    target_rotation_matrix = euler_to_rotation_matrix(target_roll_deg, target_pitch_deg, target_yaw_deg)
    target_pose_matrix = np.eye(4)
    target_pose_matrix[:3, :3] = target_rotation_matrix
    target_pose_matrix[:3, 3] = np.array([target_x, target_y, target_z])

    # 초기 관절 각도 추정 (모두 0으로 시작)
    initial_guess = np.zeros(num_joints)

    # 역기구학 계산
    joint_angles_final_rad = inverse_kinematics(target_pose_matrix, initial_guess)
    print("Final Joint Angles (radians):", joint_angles_final_rad)

    # 초기 관절 각도 설정 (예시)
    joint_angles_initial_rad = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    # 궤적 생성 (Cubic Spline)
    time_points = np.linspace(0, 5, 50) # 5초 동안 50개의 점
    trajectory = cubic_spline_trajectory(joint_angles_initial_rad, joint_angles_final_rad, time_points)

    # 궤적 시각화 (엔드 이펙터 경로 및 좌표계 포함)
    visualize_trajectory(trajectory, np.array([target_x, target_y, target_z]))

    # 최종 자세 시각화 (엔드 이펙터 및 베이스 좌표계 포함)
    final_joint_positions = get_joint_positions(joint_angles_final_rad)
    final_end_effector_pose = forward_kinematics(joint_angles_final_rad)
    plot_robot(final_joint_positions, final_end_effector_pose, np.array([target_x, target_y, target_z]))