import numpy as np

def rotate_x(vector, degree):
    # Degree to Radian
    rad = np.radians(degree)

    # Rotation Matrix
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(rad), -np.sin(rad)],
        [0, np.sin(rad), np.cos(rad)]
    ])

    return np.dot(rotation_matrix, vector)

def translate(vector, translation):
    return vector + translation 



def transformation_init(pose):

    translate_vector = np.array([0, -0.1057, 0.301625]) # m 기준
    rotate_degree = -34.5

    vector = pose[:3]
    rotated_vector = rotate_x(vector, rotate_degree)
    translated_vector = translate(rotated_vector, translate_vector)

    print(translated_vector)

    return np.r_[translated_vector, pose[3]]


# 수정 필요
def transformation_cam(pose):

    translate_vector = np.array([0, 0.2, 0.3])
    rotate_degree = -30

    vector = pose[:3]
    rotated_vector = rotate_x(vector, rotate_degree)
    translated_vector = translate(rotated_vector, translate_vector)

    print(translated_vector)

    return np.r_[translated_vector, pose[3]]
