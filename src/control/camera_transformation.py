import numpy as np

## 정의 자세 : x, y축 리버스(- or 180도 회전) / z축 

def transformation_define(pose):
    _pose_matrix = np.append(pose, 1)
    pose_matrix = _pose_matrix.T

    translation_matrix = np.array([
        [1,0,0, 0],
        [0,1,0,-35],
        [0,0,1,370],
        [0,0,0,1]
    ])

    rotation_matrix_z = np.array([
        [np.cos(np.radians(180)), -np.sin(np.radians(180)), 0, 0],
        [np.sin(np.radians(180)), np.cos(np.radians(180)), 0, 0],
        [0, 0, 1, 0],
        [0,0,0,1]
    ])
    
    rotation_matrix_x = np.array([
        [1, 0, 0, 0],
        [0, np.cos(np.radians(90+39)), -np.sin(np.radians(90+39)), 0],
        [0, np.sin(np.radians(90+39)), np.cos(np.radians(90+39)), 0],
        [0,0,0,1]
    ])

    T_g_to_camera = np.dot(np.dot(translation_matrix,rotation_matrix_z), rotation_matrix_x)

    pose = np.dot(T_g_to_camera, pose_matrix)
    return pose[:3] 


# 수정 필요
def transformation_camera(pose):

    translate_vector = np.array([0, 0.2, 0.3])
    rotate_degree = -30

    vector = pose[:3]
    # rotated_vector = rotate_x(vector, rotate_degree)
    # translated_vector = translate(rotated_vector, translate_vector)

    # print(translated_vector)

    # return np.r_[translated_vector, pose[3]]
