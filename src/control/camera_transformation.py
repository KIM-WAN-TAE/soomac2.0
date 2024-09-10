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

    # rotation_matrix_z2 = np.array([
    #     [np.cos(0.065), -np.sin(0.065), 0, 0],
    #     [np.sin(0.065), np.cos(0.065), 0, 0],
    #     [0, 0, 1, 0],
    #     [0,0,0,1]
    # ])
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
def transformation_camera(camera_pose):
    pose = np.zeros(4)
    pose[0] = (camera_pose[0]/1.615-12)
    pose[1] = camera_pose[1]/1.615
    pose[2] = camera_pose[2]
    pose[3] = 1

    rotation_matrix_y = np.array([
        [np.cos(0.0325), 0, np.sin(0.0325), 0],
        [0, 1, 0, 0],
        [-np.sin(0.0325), 0,  np.cos(0.0325), 0],
        [0,0,0,1]
    ])
    pose = np.dot(rotation_matrix_y, pose.T)
    
    # pose = np.zeros(3)
    # pose[0] = camera_pose[0] - 11  +18.6 +135
    # pose[1] = -camera_pose[1] + 350 +1 -519.9 +396
    # pose[2] = 455 - camera_pose[2] -23 +27
    pose[0] = pose[0] - 15 - 15
    pose[1] = (-pose[1]+ 350)
    pose[2] = 465 - pose[2]
    return pose[:3]
    # rotated_vector = rotate_x(vector, rotate_degree)
    # translated_vector = translate(rotated_vector, translate_vector)

    # print(translated_vector)

    # return np.r_[translated_vector, pose[3]]
