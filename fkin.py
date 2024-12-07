import numpy as np

dh_parameters = [
    (0, 0.4, 0.2, np.pi/2), # Adjust these parameters for your specific robot
    (0, 0.0, 0.3, 0),
    (0, 0.0, 0.3, 0),
    (0, 0.2, 0.0, np.pi/2),
    (0, 0.0, 0.0, -np.pi/2),
    (0, 0.0, 0.0, np.pi/2),
    (0, 0.1, 0.0, 0)
]


def dh_transform_matrix(theta, d, a, alpha):
    return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

def forward_kinematics(joint_angles):
    print(joint_angles)
    assert len(joint_angles) == 7
    assert len(dh_parameters) == 7
    T = np.eye(4)
    for i in range(7):
        theta, d, a, alpha = dh_parameters[i]
        theta += joint_angles[i]
        Ti = dh_transform_matrix(theta, d, a, alpha)
        T = np.dot(T, Ti)
    end_effector_position = T[:3, 3]
    return end_effector_position
