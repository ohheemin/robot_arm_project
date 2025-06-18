import numpy as np
from scipy.spatial.transform import Rotation as R
from ikpy.chain import Chain
from ikpy.link import URDFLink
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation # Import FuncAnimation for animation

radian_values = [2.12082666e-04, -5.28608428e-14, 1.69739617e+00, 2.14672678e+00,
                 -7.02530298e-01, 3.83788550e-03]

degree_values = np.rad2deg(radian_values)
print(f"초기 라디안 값을 도로 변환: {degree_values}")

## IKPY 로봇 모델 생성 함수 (프리즘 조인트 포함)

def create_ikpy_chain(dh_params):
    links = [
        URDFLink(
            name="base_link",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0]  # 고정 베이스
        )
    ]

    for i, params in enumerate(dh_params):
        theta_offset, d, a, alpha = params

        if i == 0:  # Joint 0: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[0, 0, d],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.pi, np.pi)
                )
            )
        elif i == 1:  # Joint 1: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[0, 0, d],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.pi, np.pi)
                )
            )
        elif i == 2:  # Joint 2: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.pi, np.pi)
                )
            )
        elif i == 3:  # Joint 3: z축 회전 (기존 y축 회전 → z축 회전으로 맞춤)
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.radians(100), np.radians(100))
                )
            )
        elif i == 4:  # Joint 4: 프리즘 조인트 (x축 방향 슬라이더)
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 0], # 회전 없음
                    bounds=(0, 0.2)
                )
            )
        else:
            raise ValueError("Unexpected joint index")

    return Chain(name="arm", links=links)

def plot_coordinate_system(ax, origin, rotation_matrix, scale=0.05, label=""):
    x_vector = rotation_matrix[:, 0] * scale
    y_vector = rotation_matrix[:, 1] * scale
    z_vector = rotation_matrix[:, 2] * scale

    # quiver를 사용하여 3D 화살표를 그립니다.
    quivers = []
    quivers.append(ax.quiver(origin[0], origin[1], origin[2], x_vector[0], x_vector[1], x_vector[2], color='r', length=1, arrow_length_ratio=0.1))
    quivers.append(ax.quiver(origin[0], origin[1], origin[2], y_vector[0], y_vector[1], y_vector[2], color='g', length=1, arrow_length_ratio=0.1))
    quivers.append(ax.quiver(origin[0], origin[1], origin[2], z_vector[0], z_vector[1], z_vector[2], color='b', length=1, arrow_length_ratio=0.1))

    # 레이블이 있으면 텍스트를 반환합니다.
    text_obj = None
    if label:
        text_obj = ax.text(origin[0], origin[1], origin[2], label)
        quivers.append(text_obj) # 텍스트 객체도 반환하여 업데이트 가능하게 함

    return quivers # 화살표와 텍스트 객체를 리스트로 반환


def setup_robot_visualization(chain, initial_joint_angles, plot_range=0.5, title="Robot Arm Animation"):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)
    ax.set_xlabel('X axis (m)')
    ax.set_ylabel('Y axis (m)')
    ax.set_zlabel('Z axis (m)')
    ax.grid(True)
    ax.set_aspect('equal')

    # 시각화 범위 설정
    ax.set_xlim(-plot_range, plot_range)
    ax.set_ylim(-plot_range, plot_range)
    ax.set_zlim(-plot_range, plot_range)

    # 초기 로봇 팔 플롯 (업데이트될 객체)
    line, = ax.plot([], [], [], marker='o', markersize=4, color='black', linewidth=2, label='Robot Arm')
    end_effector_scatter = ax.scatter([], [], [], color='red', s=100, label='End Effector')
    base_scatter = ax.scatter(0, 0, 0, color='blue', s=100, label='Base Origin')

    ax.legend()

    # 초기 좌표계 플롯 (업데이트될 객체들)
    # 베이스 좌표계는 고정되어 있으므로 한 번만 그립니다.
    base_origin = np.array([0, 0, 0])
    base_rotation = np.identity(3)
    plot_coordinate_system(ax, base_origin, base_rotation, scale=0.05, label="Base")

    # 엔드 이펙터 좌표계는 업데이트될 것이므로 객체를 받아둡니다.
    end_effector_quivers = plot_coordinate_system(ax, [0,0,0], np.identity(3), scale=0.05, label="End Effector")

    return fig, ax, line, end_effector_scatter, end_effector_quivers

def update(frame, chain, all_joint_angles, line, end_effector_scatter, end_effector_quivers):
    current_joint_angles = all_joint_angles[frame]
    frames_kinematics = chain.forward_kinematics(current_joint_angles, full_kinematics=True)

    x_vals, y_vals, z_vals = [], [], []
    for frame_k in frames_kinematics:
        pos = frame_k[:3, 3]
        x_vals.append(pos[0])
        y_vals.append(pos[1])
        z_vals.append(pos[2])

    # 로봇 팔 라인 업데이트
    line.set_data(x_vals, y_vals)
    line.set_3d_properties(z_vals)

    # 엔드 이펙터 산점도 업데이트
    end_effector_pos = frames_kinematics[-1][:3, 3]
    end_effector_scatter._offsets3d = ([end_effector_pos[0]], [end_effector_pos[1]], [end_effector_pos[2]])

    # 엔드 이펙터 좌표축 업데이트
    end_effector_origin = frames_kinematics[-1][:3, 3]
    end_effector_rotation = frames_kinematics[-1][:3, :3]
    scale = 0.05

    # 각 화살표 및 텍스트 업데이트
    x_vector = end_effector_rotation[:, 0] * scale
    y_vector = end_effector_rotation[:, 1] * scale
    z_vector = end_effector_rotation[:, 2] * scale

    # quiver는 set_segments()로 업데이트
    # set_segments는 [x_start, y_start, z_start, x_end, y_end, z_end] 형태의 리스트를 받습니다.
    end_effector_quivers[0].set_segments([[[end_effector_origin[0], end_effector_origin[1], end_effector_origin[2]],
                                            [end_effector_origin[0] + x_vector[0], end_effector_origin[1] + x_vector[1], end_effector_origin[2] + x_vector[2]]]])
    end_effector_quivers[1].set_segments([[[end_effector_origin[0], end_effector_origin[1], end_effector_origin[2]],
                                            [end_effector_origin[0] + y_vector[0], end_effector_origin[1] + y_vector[1], end_effector_origin[2] + y_vector[2]]]])
    end_effector_quivers[2].set_segments([[[end_effector_origin[0], end_effector_origin[1], end_effector_origin[2]],
                                            [end_effector_origin[0] + z_vector[0], end_effector_origin[1] + z_vector[1], end_effector_origin[2] + z_vector[2]]]])

    # 텍스트 객체 업데이트 (만약 있다면)
    if len(end_effector_quivers) > 3 and end_effector_quivers[3]: # 텍스트 객체가 존재하면
        text_obj = end_effector_quivers[3]
        text_obj.set_position((end_effector_origin[0], end_effector_origin[1])) # 2D position
        text_obj.set_3d_properties(end_effector_origin[2], 'z') # 3D position


    return line, end_effector_scatter, *end_effector_quivers # 업데이트된 객체들을 반환

# === DH 파라미터 정의 (theta_offset, d, a, alpha) ===
# 각 링크의 기하학적 특징을 정의합니다.
DH_params = [
    [0,        0.040, 0,       0],         # Joint 0
    [0,        0.050, 0,  np.pi/2],         # Joint 1
    [np.pi/2,  0,     0.125,   0],         # Joint 2
    [0,        0,     0.13,    0],         # Joint 3
    [np.pi/2,  0,     0.08,    np.pi/2],   # Joint 4 (프리즘 조인트)
]

# 위에서 정의한 DH 파라미터로 IKPy 체인을 생성합니다.
ik_chain = create_ikpy_chain(DH_params)

# === 초기 조인트 상태 설정 ===
# IKPy는 기본적으로 첫 번째 요소(base_link)가 0이라고 가정하므로,
# 실제 조인트 값은 두 번째 요소부터 시작합니다.
# 프리즘 조인트(Joint 4)의 값은 회전 각도가 아닌 슬라이딩 길이(미터)입니다.
initial_joint_angles = np.array([0, 0, np.pi/2, 0, 0,  0.1]) # [base_joint, J0, J1, J2, J3, J4(프리즘)]

print(f"\n--- 초기 로봇 상태 ---")
print(f"초기 조인트 각도 및 프리즘 변위: {initial_joint_angles}")

# === 첫 번째 목표 위치 및 방향 ===
target_position_1 = np.array([0.095 + 0.05, 0.0, 0.15]) # Y값 변경하여 Y축 움직임 확인
target_x_axis_1 = np.array([1, 0, 0])

print(f"\n--- 첫 번째 목표 ---")
print(f"목표 위치: {target_position_1}, 목표 X축 방향: {target_x_axis_1}")

calculated_joints_1 = ik_chain.inverse_kinematics(
    target_position=target_position_1,
    initial_position=initial_joint_angles,
    target_orientation=target_x_axis_1,
    orientation_mode="X"
)
calculated_joints_1 = np.array(calculated_joints_1) # numpy 배열로 변환
print("계산된 조인트 각도 및 변위:", calculated_joints_1)

# === 두 번째 목표 위치 및 방향 ===
target_position_2 = np.array([0.095 + 0.12, 0.01, 0.010]) # Y값 다시 변경
target_x_axis_2 = np.array([0, 0, -1]) # 엔드 이펙터의 X축이 전역 Z축의 음수 방향 향함

print(f"\n--- 두 번째 목표 ---")
print(f"목표 위치: {target_position_2}, 목표 X축 방향: {target_x_axis_2}")

calculated_joints_2 = ik_chain.inverse_kinematics(
    target_position=target_position_2,
    initial_position=calculated_joints_1,
    target_orientation=target_x_axis_2,
    orientation_mode="X"
)
calculated_joints_2 = np.array(calculated_joints_2) # numpy 배열로 변환
print("계산된 조인트 각도 및 변위:", calculated_joints_2)

# === 세 번째 목표 위치 및 방향 ===
target_position_3 = np.array([0.19, -0.1, 0.08]) # Y값 음수 방향으로 변경
target_x_axis_3 = np.array([1, 0, 0]) # 엔드 이펙터의 X축이 다시 전역 X축의 양수 방향 향함

print(f"\n--- 세 번째 목표 ---")
print(f"목표 위치: {target_position_3}, 목표 X축 방향: {target_x_axis_3}")

calculated_joints_3 = ik_chain.inverse_kinematics(
    target_position=target_position_3,
    initial_position=calculated_joints_2,
    target_orientation=target_x_axis_3,
    orientation_mode="X"
)
calculated_joints_3 = np.array(calculated_joints_3) # numpy 배열로 변환
print("계산된 조인트 각도 및 변위:", calculated_joints_3)

# --- 애니메이션을 위한 조인트 각도 보간 ---
# 각 목표 포즈 사이를 부드럽게 전환하기 위해 중간 단계를 계산합니다.
num_frames_per_segment = 75 # 각 구간별 애니메이션 프레임 수 (늘리면 더 부드러워지나 시간 증가)
all_joint_angles = []

# 초기 상태 -> 첫 번째 목표
for i in range(num_frames_per_segment):
    t = i / (num_frames_per_segment - 1)
    interpolated_angles = (1 - t) * initial_joint_angles + t * calculated_joints_1
    all_joint_angles.append(interpolated_angles)

# 첫 번째 목표 -> 두 번째 목표
for i in range(num_frames_per_segment):
    t = i / (num_frames_per_segment - 1)
    interpolated_angles = (1 - t) * calculated_joints_1 + t * calculated_joints_2
    all_joint_angles.append(interpolated_angles)

# 두 번째 목표 -> 세 번째 목표
for i in range(num_frames_per_segment):
    t = i / (num_frames_per_segment - 1)
    interpolated_angles = (1 - t) * calculated_joints_2 + t * calculated_joints_3
    all_joint_angles.append(interpolated_angles)

# 마지막 목표 상태에서 잠시 멈춤 (애니메이션이 끝나고 최종 자세를 볼 수 있도록)
for _ in range(num_frames_per_segment // 5):
    all_joint_angles.append(calculated_joints_3)

# 애니메이션 초기 설정
fig, ax, line, end_effector_scatter, end_effector_quivers = setup_robot_visualization(ik_chain, initial_joint_angles, plot_range=0.5)

# FuncAnimation을 사용하여 애니메이션 생성 및 실행
# frames: 애니메이션의 총 프레임 수
# interval: 각 프레임 간의 시간 간격 (밀리초), 50ms = 20 FPS
# blit=False: 3D 애니메이션에서는 True로 설정하면 문제가 발생할 수 있으므로 False로 설정합니다.
# repeat=False: 애니메이션 반복 여부
ani = FuncAnimation(fig, update, frames=len(all_joint_angles),
                    fargs=(ik_chain, all_joint_angles, line, end_effector_scatter, end_effector_quivers),
                    interval=50, blit=False, repeat=False)

plt.show()

# --- 애니메이션 저장 (선택 사항) ---
# 이 기능을 사용하려면 'ffmpeg' 또는 'imagemagick'이 시스템에 설치되어 있어야 합니다.
# 설치 후 아래 주석을 해제하고 실행하세요.
# try:
#     print("애니메이션을 저장 중입니다... (ffmpeg 필요)")
#     ani.save('robot_arm_trajectory.mp4', writer='ffmpeg', fps=30, dpi=150)
#     print("애니메이션 저장 완료: robot_arm_trajectory.mp4")
# except Exception as e:
#     print(f"애니메이션 저장 중 오류 발생: {e}")
#     print("ffmpeg이 설치되어 있는지 확인하거나, 저장 경로를 확인해주세요.")
# === 


# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from ikpy.chain import Chain
# from ikpy.link import URDFLink

# # === IKPY 모델 생성 (수정) ===
# def create_ikpy_chain(dh_params):
#     links = [
#         URDFLink(
#             name="base_link",
#             origin_translation =[0, 0, 0],
#             origin_orientation=[0, 0, 0],
#             rotation=[0, 0, 0]  # fixed base
#         )
#     ]

#     for i, params in enumerate(dh_params):
#         theta_offset, d, a, alpha = params

#         if i == 0:  # Joint 0: z축 회전
#             links.append(
#                 URDFLink(
#                     name=f"joint_{i}",
#                     origin_translation =[0, 0, d],
#                     origin_orientation=[0, 0, 0],
#                     rotation=[0, 0, 1],
#                     bounds=(-np.pi, np.pi)
#                 )
#             )
#         elif i == 1:  # Joint 1: z축 회전
#             print(f"Joint 1: theta: {theta_offset}, a: {a}, d: {d}, alpha: {alpha}")
#             links.append(
#                 URDFLink(
#                     name=f"joint_{i}",
#                     origin_translation=[0, 0, d],
#                     origin_orientation=[alpha, 0, 0],
#                     rotation=[0, 0, 1],
#                     bounds=(-np.pi, np.pi)
#                 )
#             )
#         elif i == 2:  # Joint 2: z축 회전
#             print(f"Joint 2: theta: {theta_offset}, a: {a}, d: {d}, alpha: {alpha}")
#             links.append(
#                 URDFLink(
#                     name=f"joint_{i}",
#                     origin_translation=[a, 0, 0],
#                     origin_orientation=[0, 0, 0],
#                     rotation=[0, 0, 1],
#                     bounds=(-np.pi, np.pi)
#                 )
#             )
#         elif i == 3:  # Joint 3: y축 회전
#             links.append(
#                 URDFLink(
#                     name=f"joint_{i}",
#                     origin_translation=[a, 0, 0],
#                     origin_orientation=[alpha, 0, 0],
#                     rotation=[0, 0, 1],
#                     bounds=(-np.radians(100), np.radians(100))  
#                 )
#             )
#         elif i == 4:  # Joint 4: x축 프리즘
#             links.append(
#                 URDFLink(
#                     name=f"joint_{i}",
#                     origin_translation=[a, 0, 0],
#                     origin_orientation=[alpha, 0, 0],
#                     rotation=[0, 0, 1],
#                     bounds=(-np.pi, np.pi)
#                 )
#             )
#         else:
#             raise ValueError("Unexpected joint index")

#     return Chain(name="arm", links=links)

# # === 시각화 함수 ===
# def visualize_robot(chain, joint_angles):
#     frames = chain.forward_kinematics(joint_angles, full_kinematics=True)

#     x_vals, y_vals, z_vals = [], [], []

#     for i, frame in enumerate(frames):
#         pos = frame[:3, 3]
#         x_vals.append(pos[0])
#         y_vals.append(pos[1])
#         z_vals.append(pos[2])
#         print(f"Joint {i} 위치: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     ax.plot(x_vals, y_vals, z_vals, marker='o', markersize=6, label='Robot Arm')
#     ax.set_xlabel('X axis (m)')
#     ax.set_ylabel('Y axis (m)')
#     ax.set_zlabel('Z axis (m)')
#     ax.set_title('Robot Arm Visualization')
#     ax.legend()
#     ax.grid(True)

#     max_range = max(
#         max(x_vals) - min(x_vals),
#         max(y_vals) - min(y_vals),
#         max(z_vals) - min(z_vals)
#     ) / 2.0

#     mid_x = (max(x_vals) + min(x_vals)) * 0.5
#     mid_y = (max(y_vals) + min(y_vals)) * 0.5
#     mid_z = (max(z_vals) + min(z_vals)) * 0.5

#     ax.set_xlim(mid_x - max_range, mid_x + max_range)
#     ax.set_ylim(mid_y - max_range, mid_y + max_range)
#     ax.set_zlim(mid_z - max_range, mid_z + max_range)

#     plt.show()

# # === 예시 DH 파라미터 및 실행 ===
# # theta_offset, d, a, alpha (단위: 라디안/미터)

# DH_params = [
#     [0,                 0.040,    0,        0],       # Joint 0
#     [0,                 0.050,    0,  np.pi/2],   # Joint 1
#     [np.pi/2,          0,        0.125,    0],       # Joint 2
#     [0,                 0,        0.13,     0],       # Joint 3
#     [np.pi/2,                 0,        0.08,     0],       # Joint 4 (프리즘형)
# ]

# # 체인 생성
# ik_chain = create_ikpy_chain(DH_params)

# # 임의의 조인트 각도 입력 (프리즘 조인트는 길이 값)
# joint_angles = [0, 0, np.pi/2, 0, 0, np.pi/2]
# visualize_robot(ik_chain, joint_angles)

import numpy as np
from scipy.spatial.transform import Rotation as R
from ikpy.chain import Chain
from ikpy.link import URDFLink
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# === IKPY 모델 생성 함수 (프리즘 조인트 포함) ===
def create_ikpy_chain(dh_params):
    links = [
        URDFLink(
            name="base_link",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0]  # 고정 베이스
        )
    ]

    for i, params in enumerate(dh_params):
        theta_offset, d, a, alpha = params

        if i == 0:  # Joint 0: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[0, 0, d],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.pi, np.pi)
                )
            )
        elif i == 1:  # Joint 1: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[0, 0, d],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.pi, np.pi)
                )
            )
        elif i == 2:  # Joint 2: z축 회전
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[0, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.pi, np.pi)
                )
            )
        elif i == 3:  # Joint 3: z축 회전 (기존 y축 회전 → z축 회전으로 맞춤)
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 1],
                    bounds=(-np.radians(100), np.radians(100))
                )
            )
        elif i == 4:  # Joint 4: 프리즘 조인트 (x축 방향 슬라이더)
            links.append(
                URDFLink(
                    name=f"joint_{i}",
                    origin_translation=[a, 0, 0],
                    origin_orientation=[alpha, 0, 0],
                    rotation=[0, 0, 0], # 회전 없음
                    bounds=(0, 0.2)
                )
            )
        else:
            raise ValueError("Unexpected joint index")

    return Chain(name="arm", links=links)

# === 시각화 함수 ===
def visualize_robot(chain, joint_angles, title="Robot Arm Visualization"):
    frames = chain.forward_kinematics(joint_angles, full_kinematics=True)

    x_vals, y_vals, z_vals = [], [], []

    for i, frame in enumerate(frames):
        pos = frame[:3, 3]
        x_vals.append(pos[0])
        y_vals.append(pos[1])
        z_vals.append(pos[2])
        print(f"Joint {i} 위치: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x_vals, y_vals, z_vals, marker='o', markersize=6, label='Robot Arm')
    ax.set_xlabel('X axis (m)')
    ax.set_ylabel('Y axis (m)')
    ax.set_zlabel('Z axis (m)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True)

    max_range = max(
        max(x_vals) - min(x_vals),
        max(y_vals) - min(y_vals),
        max(z_vals) - min(z_vals)
    ) / 2.0

    mid_x = (max(x_vals) + min(x_vals)) * 0.5
    mid_y = (max(y_vals) + min(y_vals)) * 0.5
    mid_z = (max(z_vals) + min(z_vals)) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()

# === DH 파라미터 (theta_offset, d, a, alpha) ===
DH_params = [
    [0,        0.040, 0,       0],         # Joint 0
    [0,        0.050, 0,  np.pi/2],         # Joint 1
    [np.pi/2,  0,     0.125,   0],         # Joint 2
    [0,        0,     0.13,    0],         # Joint 3
    [np.pi/2,  0,     0.08,    np.pi/2],         # Joint 4 (프리즘 조인트)
]

# 체인 생성
ik_chain = create_ikpy_chain(DH_params)

# === 초기 조인트 상태 ===
initial_joint_angles = [0, 0, np.pi/2, 0, 0,  0.1]

# === 첫 번째 목표 위치 및 방향 ===
target_position_1 = [0.10, 0.0, 0.25]
target_x_axis_1 = [1, 0, 0]

# 역기구학 계산 (첫 번째 목표)
calculated_joints_1 = ik_chain.inverse_kinematics(
    target_position=target_position_1,
    initial_position=initial_joint_angles,
    target_orientation=target_x_axis_1,
    orientation_mode="X"
)

print("첫 번째 목표 - 계산된 조인트 각도 및 변위:", calculated_joints_1)

# 결과 시각화 (첫 번째 목표)
visualize_robot(ik_chain, calculated_joints_1, title="Robot Arm to First Target")

# === 두 번째 목표 위치 및 방향 (이전 결과를 초기 상태로 사용) ===
target_position_2 = [0.10, 0.01, 0.05]
target_x_axis_2 = [0, 0, -1]  # 두 번째 목표에서는 x축이 y축을 향하도록 설정

# 역기구학 계산 (두 번째 목표) - 이전 결과를 초기 상태로 사용
calculated_joints_2 = ik_chain.inverse_kinematics(
    target_position=target_position_2,
    initial_position=calculated_joints_1,  # 이전 계산 결과를 초기 상태로 사용
    target_orientation=target_x_axis_2,
    orientation_mode="X"
)

print("두 번째 목표 - 계산된 조인트 각도 및 변위:", calculated_joints_2)

# 결과 시각화 (두 번째 목표)
visualize_robot(ik_chain, calculated_joints_2, title="Robot Arm to Second Target")



# === 두 번째 목표 위치 및 방향 (이전 결과를 초기 상태로 사용) ===
target_position_2 = [0.10, 0.01, 0.05]
target_x_axis_2 = [0, 0, -1]  # 두 번째 목표에서는 x축이 y축을 향하도록 설정

target_position_3 = [0.19, 0.01, 0.05]
# 역기구학 계산 (두 번째 목표) - 이전 결과를 초기 상태로 사용
calculated_joints_3 = ik_chain.inverse_kinematics(
    target_position=target_position_3,
    initial_position=calculated_joints_2,  # 이전 계산 결과를 초기 상태로 사용
    target_orientation=target_x_axis_1,
    orientation_mode="X"
)

print("두 번째 목표 - 계산된 조인트 각도 및 변위:", calculated_joints_3)

# 결과 시각화 (두 번째 목표)
visualize_robot(ik_chain, calculated_joints_3, title="Robot Arm to Second Target")
