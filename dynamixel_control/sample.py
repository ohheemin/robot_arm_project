import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from matplotlib.animation import FuncAnimation

# DH 파라미터: [theta_offset, d, a, alpha]
DH_params = [
    [0, 0.059, 0, 0],
    [0, 0.059, 0, -np.pi/2],
    [-np.pi/2, 0, 0.13, 0],
    [0, 0, 0.13, 0],
    [0, 0.0, 0.08, 0],
]

num_joints = len(DH_params)

# 순기구학 함수
def forward_kinematics(joints):
    T = np.eye(4)
    positions = [np.array([0, 0, 0])]
    for i, (theta_offset, d, a, alpha) in enumerate(DH_params):
        theta = joints[i] + theta_offset
        T_i = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
        T = T @ T_i
        positions.append(T[:3, 3])
    return positions

# 역기구학 함수
def inverse_kinematics(target_pos, initial_guess=np.zeros(num_joints)):
    def objective_function(joints):
        pos = forward_kinematics(joints)[-1]
        return np.sum((pos - target_pos) ** 2)

    bounds = [
        (math.radians(-180), math.radians(180)),
        (math.radians(-150), math.radians(150)),
        (math.radians(-180), math.radians(180)),
        (math.radians(-150), math.radians(150)),
        (math.radians(-180), math.radians(180)),
    ]

    result = minimize(objective_function, initial_guess, method='L-BFGS-B', bounds=bounds)
    print(f"Optimization Result: {result}")
    return result.x if result.success else initial_guess

# === 애니메이션 함수 ===
def animate_arm(initial_angles, final_angles, target, steps=100):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-0.4, 0.4])
    ax.set_ylim([-0.4, 0.4])
    ax.set_zlim([0, 0.4])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(*target, color='red', label='Target', s=100)
    line, = ax.plot([], [], [], '-o', color='blue', label='Arm')
    ax.legend()

    def update(frame):
        t = frame / steps
        interpolated_angles = (1 - t) * initial_angles + t * final_angles
        positions = forward_kinematics(interpolated_angles)
        xs, ys, zs = zip(*positions)
        line.set_data(xs, ys)
        line.set_3d_properties(zs)
        return line,

    ani = FuncAnimation(fig, update, frames=steps+1, interval=50, blit=False)
    plt.tight_layout()
    plt.show()

    
# === 메인 ===
if __name__ == "__main__":
    target_position = np.array([0.2, 0.0, 0.0])
    initial_joint_angles = np.zeros(num_joints)
    final_joint_angles = inverse_kinematics(target_position, initial_joint_angles)
    print("Initial Joint Angles (degrees):", np.degrees(initial_joint_angles))
    animate_arm(initial_joint_angles, final_joint_angles, target_position)
