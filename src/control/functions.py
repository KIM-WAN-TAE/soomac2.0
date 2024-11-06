import numpy as np
from copy import deepcopy
def grip_mm_to_value(grip_mm):
    grip_divide_mm_arr = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75]
    grip_divide_dynamixel_arr =  [2200, 2403, 2550, 2700, 2800, 2900, 3000, 3125, 3225, 3350, 3500, 3800]
    grip_value = 3800

    if grip_mm <= grip_divide_mm_arr[0]:
        grip_value = grip_divide_dynamixel_arr[0]

    elif grip_mm <= grip_divide_mm_arr[11]:
        for i in range(1, 12):
            if grip_mm <= grip_divide_mm_arr[i]:
                value_per_mm = (grip_divide_dynamixel_arr[i]-grip_divide_dynamixel_arr[i-1])/(grip_divide_mm_arr[i]-grip_divide_mm_arr[i-1])
                grip_value = grip_divide_dynamixel_arr[i-1] + (grip_mm-grip_divide_mm_arr[i-1])*value_per_mm 
                break
    else:
        grip_value = grip_divide_dynamixel_arr[11]
    return grip_value

class dynamixel_value_change():
    def __init__(self):
        self.change_para = 1024/90
        
        self.min_limits = np.array([683, 853, 565, 745, 1024])
        self.max_limits = np.array([3413, 3072, 3584, 3444, 3072])
        self.degree_change_param = [180, 90, 180, 180, 180]
        self.dof = 5

    def degree_to_dynamixel_value(self, degree, grip_mm, grip_option="mm", N=0): # input : (1,6) -> output : (1,6) // 5축 degree + gripper_value -> 6개의 모터 value
        degree = np.array(degree)
        dynamixel_value = degree

        for idx in range(self.dof):
            dynamixel_value[idx] = (degree[idx]+self.degree_change_param[idx])*self.change_para
        
        dynamixel_value = self.limit_check(dynamixel_value)

        # about gripper
        if grip_option == "mm":
            grip_value = grip_mm_to_value(grip_mm)
        else:
            grip_value = grip_mm
                        
        dynamixel_value = dynamixel_value.astype(int)
        dynamixel_value = np.append(dynamixel_value, int(grip_value))
        
        return dynamixel_value
    
    def dynamixel_value_to_degree(self, value): # value 6개 -> degree 5개 + mm 1개(보류, 추후 필요 시 작성)
        axis_value = np.array(value[:5])
        grip_value = np.array(value[5])
        
        degree = np.zeros_like(axis_value)
        grip_mm = deepcopy(grip_value)

        for idx in range(self.dof):
            degree[idx] = (axis_value[idx]/self.change_para) - self.degree_change_param[idx]

        degree = np.round(degree, 3)
        return degree, grip_mm     


    
    def limit_check(self, dynamixel_value):
        for i in range(5):
            if dynamixel_value[i] < self.min_limits[i]:
                dynamixel_value[i] = self.min_limits[i]
            elif dynamixel_value[i] > self.max_limits[i]:
                dynamixel_value[i] = self.max_limits[i]
        return dynamixel_value



def cubic_trajectory(th_i, th_f, min_n = 5, max_n = 15, N_req=0): 

    # N 계산 
    delta_value = np.max(np.abs(th_f[:5] - th_i[:5])) # 모터들의 변화량 계산 후, 모든 모터에서 발생할 수 있는 최대 변화량
    max_delta_value = (1024/90)*100                     # 모터에서 발생할 수 있는 최대 변화량
    N = min_n + (delta_value / max_delta_value) * (max_n - min_n)    
    N = int(N)
    if N > max_n:
        N = max_n
    
    if N_req != 0:
        N = N_req
    t = np.linspace(0, 1, N)
    
    # 3차 다항식 계수: 초기 속도와 최종 속도를 0으로 설정 (s_curve)
    a0 = th_i[:, np.newaxis]
    a1 = 0
    a2 = 3 * (th_f - th_i)[:, np.newaxis]
    a3 = -2 * (th_f - th_i)[:, np.newaxis]
    
    # 3차 다항식을 통해 각도를 계산 (각 행이 하나의 trajectory)
    theta = a0 + a1 * t + a2 * t**2 + a3 * t**3  
    print('trajectory N : ',N)
    return theta.T
    
def linear_trajectory(start_coord, end_coord, N=20):
    return np.linspace(start_coord, end_coord, N)


def gripper_trajectory(th_i, th_f, N = 5): 
    t = np.linspace(0, 1, N)
    # 3차 다항식 계수: 초기 속도와 최종 속도를 0으로 설정 (s_curve)
    a0 = th_i[:, np.newaxis]
    a1 = 0
    a2 = 3 * (th_f - th_i)[:, np.newaxis]
    a3 = -2 * (th_f - th_i)[:, np.newaxis]
    
    # 3차 다항식을 통해 각도를 계산 (각 행이 하나의 trajectory)
    theta = a0 + a1 * t + a2 * t**2 + a3 * t**3  
    print('trajectory N : ',N)
    return theta.T

